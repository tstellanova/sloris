
/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 *  Demo application for power-conserving remote condition monitoring
 */
#include <Particle.h>

#include "Ubidots.h"

//  Pressure Humidity Temperature Gas PHTG sensor
#include "Adafruit_BME680.h"
// Particulate sensor
#include "Adafruit_PM25AQI.h"

// Gas sensor
// #include "Air_Quality_Sensor.h"
// UV sensor
// #include "Adafruit_VEML6070.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); 

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO,
    {
      {"app", LOG_LEVEL_INFO},
      {"system.nm", LOG_LEVEL_INFO},
    });

// External status LED pins
const uint16_t EXTLED_CLOCK_PIN = (D2);
const uint16_t EXTLED_DATA_PIN = (D3);

const uint16_t USER_LED_PIN = (D7);

// const uint16_t DUST_SENSOR_PIN = (D4);
// how long to collect dust readings before recalculating
// const uint32_t DUST_SENSOR_WINDOW_MS  = 10000; 

// PM 2.5 particulate sensor
Adafruit_PM25AQI pm25_sensor;

// const uint16_t AQS_PIN = (A1);
// Sensor for volatile organic compounds, carbon monoxide, and so on.
// AirQualitySensor voc_sensor(AQS_PIN);

// Air pressure, humidity, temperature, gas sensor
Adafruit_BME680 phtg_sensor;

// in this app we use a Particle webhook to publish data and visualize with ubidots
Ubidots ubidots((char*)"webhook", UBI_PARTICLE);


// PHTG sensor readings
static float last_temp = 0;
static float last_humidity = 0;
static float last_airpressure = 0;
static int32_t last_voc_value = 0;
static uint32_t projected_phtg_done = 0;
static uint32_t last_phtg_read = 0;

// dust sensor readings
static uint32_t last_dust_recalc_ms = 0;
static uint32_t total_window_lpo = 0;
static uint32_t last_known_window_lpo = 0;
static float dust_ratio = 0;
static float dust_concentration = 0;

// // gas sensor readings
// static int32_t last_voc_qual = -1;
// static int32_t last_voc_value = 0;
// static uint32_t last_voc_read = 0;

static SystemSleepConfiguration sleep_cfg = {};


// provide gas level reading
int readGasLevel(String param) {
  return (int)last_voc_value;
}


static void park() {
	Log.error("Parking...");
	while (true) {
		delay(250);
	}
}

static bool setupDustSensor() {
  if (! pm25_sensor.begin_I2C()) {     
    Log.error("Could not find PM2.5 sensor!");
    park();
	return false;
  }

  return true;
}

static bool setupPHTGSensor() {
	if (!phtg_sensor.begin()) {
		Log.error("Could not find a valid BME680 sensor");
		park();
		return false;
	}
	// Set up oversampling and filter initialization
	phtg_sensor.setTemperatureOversampling(BME680_OS_8X);
	phtg_sensor.setHumidityOversampling(BME680_OS_2X);
	phtg_sensor.setPressureOversampling(BME680_OS_4X);
	phtg_sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
	phtg_sensor.setGasHeater(320, 150); // 320*C for 150 ms

	return true;
}

/* This function is called once at start up ----------------------------------*/
void setup() {
    Serial.begin();
	// allow some time for serial usb to start 
	delay(3000);


	ubidots.setDebug(true);

  	pinMode(USER_LED_PIN, OUTPUT);   
	setupPHTGSensor();
	setupDustSensor();

	// Configure the dust sensor pin as an input
	// pinMode(DUST_SENSOR_PIN, INPUT);

	// if (!voc_sensor.init()) {
	// 	Log.error("voc_sensor init failed");
	// }

	// register a cloud function to allow reading the gas level remotely 
	Particle.function("gasLevel", readGasLevel);

}


// Read the PM 2.5 sensor
static void read_pm25_sensor() {
	PM25_AQI_Data data;
  
	if (pm25_sensor.read(&data)) {
		last_dust_recalc_ms = millis();
		// dust_ratio = data.pm25_standard;
		last_known_window_lpo = (uint32_t)data.pm25_standard;
	}
	else {
		Log.info("PM 2.5 sensor not ready");
	}
}

// Read the gas sensor
// static void read_voc_sensor() {
// 	int gas_quality = voc_sensor.slope();
// 	if (gas_quality >= 0) {
// 		last_voc_qual = gas_quality;
// 		last_voc_value = voc_sensor.getValue();
// 		Log.info("voc qual: %ld  val: %ld", last_voc_qual, last_voc_value);
// 		last_voc_read = millis();
// 	}
// }


// Start PHTG sensor reading async
static void measure_phtg_sensor() {
	projected_phtg_done = phtg_sensor.beginReading();
	if (0 == projected_phtg_done) {
		Log.error("PHTG beginReading failed");
		park();
	}
}

// Collect readings Pressure, Humidity, Temperature, Gas sensor
static void collect_phtg_sensor() {
	uint32_t cur_millis = millis();
	if (cur_millis < projected_phtg_done) {
		return;
	}
	if (phtg_sensor.endReading()) {
		last_airpressure = phtg_sensor.pressure;
		last_humidity = phtg_sensor.humidity;
		last_temp = phtg_sensor.temperature;
		last_voc_value =  (int32_t)(phtg_sensor.gas_resistance / 1000.0);
		// TODO use readAltitude ?
		last_phtg_read = cur_millis;
	}
	else {
		Log.warn("BME endReading failed");
	}
}

// Read particulate sensor
// static void read_dust_sensor() {
// 	// measure the length of a low pulse
// 	uint32_t low_pulse_duration = 0;
	
// 	// Log.info("begin dust read");

// 	// filter out zeros
// 	for (uint8_t count = 0; count < 3; count++) {
// 		uint32_t low_pulse_duration = pulseIn(DUST_SENSOR_PIN, LOW);
// 		if (low_pulse_duration > 0) {
// 			total_window_lpo +=   low_pulse_duration;
// 			break;
// 		}
// 	}
// 	Log.info("dust pulse: %lu", low_pulse_duration);

// 	// 	periodically recalculate the particulate readings
// 	if ((total_window_lpo > 0) && ((millis() - last_dust_recalc_ms) > DUST_SENSOR_WINDOW_MS) ) {

// 		dust_ratio = total_window_lpo / (DUST_SENSOR_WINDOW_MS * 10.0);// Integer percentage 0=>100
// 		float ratio2 = dust_ratio * dust_ratio;
// 		dust_concentration = ratio2*(1.1 * dust_ratio - 3.8) + (520 * dust_ratio) + 0.62; // using spec sheet curve
// 		last_known_window_lpo = total_window_lpo;

// 		Log.info("LPO: %lu", last_known_window_lpo);
// 		Log.info("Dust Ratio: %f%%", dust_ratio);
// 		Log.info("dust_concentration: %f pcs/L", dust_concentration);

// 		total_window_lpo = 0;
// 		last_dust_recalc_ms = millis();
// 	}
// }

// control how long we sleep based on data collection and publication config
static void sleep_control(uint32_t sleep_ms) {
  sleep_cfg.mode(SystemSleepMode::ULTRA_LOW_POWER)
  	// keep network awake and able to awake MCU with eg Particle.variable calls
	.network(NETWORK_INTERFACE_CELLULAR) 
	// keep modem active while sleeping, but don't wake on data received from cloud
	// .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY) 
	// Wake on battery fuel gauge event, eg battery unplugged 
    // .gpio(LOW_BAT_UC, FALLING) 
    .duration(sleep_ms); //ms
  
  uint32_t sleep_start = millis();
  Log.info("sleep %lu ms", sleep_ms);
  SystemSleepResult sleep_res = System.sleep(sleep_cfg);
  SystemSleepWakeupReason wake_reason = sleep_res.wakeupReason();
  uint32_t sleep_actual = millis() - sleep_start;
  // allow some time for usb serial to wake from sleep
  Serial.begin();
  delay(3000);
  Log.info("sleep_actual: %lu", sleep_actual);

  switch (wake_reason) {
	case SystemSleepWakeupReason::BY_RTC:
		Log.info("wakeup on RTC");
		break;
    case SystemSleepWakeupReason::BY_GPIO:
      Log.info("GPIO wakeup pin: %u", sleep_res.wakeupPin());
      break;
	case SystemSleepWakeupReason::BY_NETWORK:
		Log.info("Network wakeup");
		break;

    case SystemSleepWakeupReason::BY_ADC: 
    default: {
      Log.info("wakeup: %u", (uint16_t)wake_reason);
    }
    break;
  }
}

// Send sensor data via Particle.publish to ubidots
static bool publish_data() {
	
	if (last_phtg_read > 0) {
		ubidots.add((char*)"press", last_airpressure); 
		ubidots.add((char*)"humidity", last_humidity);
		ubidots.add((char*)"temp", last_temp); 
		ubidots.add((char*)"gas",last_voc_value);
	}
	// ubidots.add((char*)"uv", last_uv);

	// if (last_voc_read > 0) {
	// 	ubidots.add((char*)"gas",last_voc_value);
	// }

	if (last_known_window_lpo > 0) {
      ubidots.add((char*)"dust-lpo", last_known_window_lpo);
      //ubidots.add((char*)"dust-ratio", dust_ratio);
      //ubidots.add((char*)"dust-concentration", dust_concentration);
	}

	
	// Here we use a Particle webhook to send data to Ubidots
	// This webhook name must match the webhook integration created in your Particle cloud account
 	bool ubi_res = ubidots.send((char*)"ubidota", PUBLIC | WITH_ACK); 
	 if (!ubi_res) {

	 }
	 return ubi_res;
}

/* This function loops forever --------------------------------------------*/
void loop() {
	digitalWrite(USER_LED_PIN, LOW);

	// connect if we aren't already connected
	if (!Particle.connected()) {
		Log.warn("reconnect");
		Particle.connect(); //start connection
	}

	// we use the user LED to indicate how long we spend reading sensors and publishing
	digitalWrite(USER_LED_PIN, HIGH);

	// read_uv_sensor();
    collect_phtg_sensor();
	measure_phtg_sensor();
	read_pm25_sensor();
	// read_voc_sensor();
	//read_dust_sensor();

	for (int i = 0; i < 30; i++) {
		if (Particle.connected()) { break; }
		Log.info("wait... %d",i);
		delay(1000);
	}

	if (!Particle.connected()) {
		// In this app we don't attempt to send data if we can't connect
		return;
	}

	bool pub_success = publish_data();
	digitalWrite(USER_LED_PIN, LOW);

	if (pub_success) {
		// wait 5 minutes between publications as ubidots are rate-limited
		//sleep_control(300000);
		delay(15000);
	}
	else {
		Log.warn("pub failed");
		delay(2000);
	}
	
	
}





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

#include "ChainableLED.h"
#include "Ubidots.h"

#include "Adafruit_BME280.h"

// Gas sensor
#include "Air_Quality_Sensor.h"
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
const uint8_t NUM_EXT_LEDS  = 1;


const uint16_t DUST_SENSOR_PIN = (D4);
// how long to collect dust readings before recalculating
const uint32_t DUST_SENSOR_WINDOW_MS  = 10000; 


const uint16_t AQS_PIN = (A2);
// Sensor for volatile organic compounds, carbon monoxide, and so on.
AirQualitySensor voc_sensor(AQS_PIN);

// Air pressure, temperature, humidity sensor
Adafruit_BME280 pht_sensor;

// Define the digital IO pins used for external LED
ChainableLED led_chain(EXTLED_CLOCK_PIN, EXTLED_DATA_PIN, NUM_EXT_LEDS);


// ultraviolet light sensor
// Adafruit_VEML6070 uv_sensor;

// in this app we use a Particle webhook to publish data and visualize with ubidots
Ubidots ubidots((char*)"webhook", UBI_PARTICLE);


static float last_temp = 0;
static float last_humidity = 0;
static float last_airpressure = 0;
static uint32_t last_pht_read = 0;
// static uint16_t last_uv = 0;
static uint32_t last_dust_recalc_ms = 0;
static uint32_t total_window_lpo = 0;
static uint32_t last_known_window_lpo = 0;
static float dust_ratio = 0;
static float dust_concentration = 0;

static int32_t last_voc_qual = -1;
static int32_t last_voc_value = 0;

static SystemSleepConfiguration sleep_cfg = {};


/* Function prototypes -------------------------------------------------------*/
double readTemperature();
double readHumidity();

/* This function is called once at start up ----------------------------------*/
void setup() {
    Serial.begin();
	// allow some time for serial usb to start 
	delay(3000);

	ubidots.setDebug(true);

	// Configure the dust sensor pin as an input
	pinMode(DUST_SENSOR_PIN, INPUT);

	if (!pht_sensor.begin()){ 
		Log.error("BME280 init failed");
		for (int i = 0; i < 5; i++) {
			delay(250);
			if (pht_sensor.begin()) {
				break;
			}
		}
	}

	if (!voc_sensor.init()) {
		Log.error("aq sensor init failed");
	}

	// uv_sensor.begin(VEML6070_2_T);

	led_chain.init();
	
	// Register all the Tinker functions
	Particle.variable("readTemperature", readTemperature);
	Particle.variable("readHumidity", readHumidity);

}


double readTemperature() {
	return (double)last_temp;
}

double readHumidity() {
	return (double)last_humidity;
}

// Read the gas sensor
static void read_voc_sensor() {
	int gas_quality = voc_sensor.slope();
	if (gas_quality >= 0) {
		last_voc_qual = gas_quality;
		last_voc_value = voc_sensor.getValue();
		Log.info("voc qual: %ld  val: %ld", last_voc_qual, last_voc_value);
	}
}

// Read ultraviolet light sensor
// static void read_uv_sensor() {
// 	uint16_t cur_uv = uv_sensor.readUV();
// 	if (65535 != cur_uv) { //invalid reading
// 		last_uv = cur_uv;
// 	}
// }

// Read Pressure, Temperature, Humidity sensor
static void read_pht_sensor() {
	last_temp =  pht_sensor.readTemperature();
	last_humidity = pht_sensor.readHumidity();
	last_airpressure = pht_sensor.readPressure() / 100.0F;
	last_pht_read = millis();
}

// Read particulate (dust) sensor
static void read_dust_sensor() {
	// measure the length of a low pulse
	uint32_t low_pulse_duration = 0;
	
	// Log.info("begin dust read");

	// filter out zeros
	for (uint8_t count = 0; count < 3; count++) {
		low_pulse_duration = pulseIn(DUST_SENSOR_PIN, LOW);
		if (low_pulse_duration > 0) {
			total_window_lpo +=   low_pulse_duration;
			break;
		}
	}
	Log.info("dust pulse: %lu", low_pulse_duration);

	// 	periodically recalculate the particulate readings
	if ((total_window_lpo > 0) && ((millis() - last_dust_recalc_ms) > DUST_SENSOR_WINDOW_MS) ) {

		dust_ratio = total_window_lpo / (DUST_SENSOR_WINDOW_MS * 10.0);// Integer percentage 0=>100
		float ratio2 = dust_ratio * dust_ratio;
		dust_concentration = ratio2*(1.1 * dust_ratio - 3.8) + (520 * dust_ratio) + 0.62; // using spec sheet curve
		last_known_window_lpo = total_window_lpo;

		Log.info("LPO: %lu", last_known_window_lpo);
		Log.info("Dust Ratio: %f%%", dust_ratio);
		Log.info("dust_concentration: %f pcs/L", dust_concentration);

		total_window_lpo = 0;
		last_dust_recalc_ms = millis();
	}
}



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
	
	ubidots.add((char*)"temp", last_temp); 
	ubidots.add((char*)"press", last_airpressure); 
    ubidots.add((char*)"humidity", last_humidity);
	// ubidots.add((char*)"uv", last_uv);

	ubidots.add((char*)"gas",last_voc_value);

	if (last_known_window_lpo > 0) {
      ubidots.add((char*)"dust-lpo", last_known_window_lpo);
      ubidots.add((char*)"dust-ratio", dust_ratio);
      ubidots.add((char*)"dust-concentration", dust_concentration);
	}

	

	// Here we use a Particle webhook to send data to Ubidots
	// This webhook name must match the webhook integration created in your Particle cloud account
 	return ubidots.send((char*)"ubidota", PUBLIC | WITH_ACK); 
}

/* This function loops forever --------------------------------------------*/
void loop()
{
	// temporarily change color to indicate we're recalculating 
	led_chain.setColorRGB(0, 0, 2, 0);
	// connect if we aren't already connected
	if (!Particle.connected()) {
		Log.warn("reconnect");
		Particle.connect(); //start connection
	}

	// read_uv_sensor();
    read_pht_sensor();
	read_voc_sensor();
	read_dust_sensor();

	// display a "health check" status for air quality
	uint8_t red_val = 0, green_val = 0, blue_val = 0;;

	if (last_voc_qual >= 0) {
		switch (last_voc_qual) {
			case 0:
				red_val = 255;
				break;
			case 1:
				red_val = 128;
				break;
			case 2:
				red_val = 64;
				break;
		} 
	}

	if (last_dust_recalc_ms > 0) {
		float dust_frac = dust_concentration / 500.0;
		if (dust_frac > 1.0) dust_frac = 1.0;
		Log.info("dustfrac: %f", dust_frac);
		blue_val = (uint8_t)ceilf(255.0 * dust_frac);
	}

	// pin color to all green if "bad" values are acceptable
	if ((red_val == 0) && (blue_val < 64)) {
		green_val = 128;
		blue_val = 0;
	}
		
	led_chain.setColorRGB(0, red_val, green_val, blue_val);

	for (int i = 0; i < 30; i++) {
		if (Particle.connected()) { break; }
		Log.info("wait... %d",i);
		delay(1000);
	}
	if (!Particle.connected()) {
		// In this app we don't attempt to send data if we can't connect
		return;
	}

	if (publish_data()) {
		// wait 5 minutes between publications as ubidots are rate-limited
		sleep_control(300000);
	}
	else {
		Log.warn("pub failed");
		delay(2000);
	}
}





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



//  Pressure Humidity Temperature Gas PHTG sensor
#include "Adafruit_BME680.h"
// Particulate sensor
#include "Adafruit_PM25AQI.h"

// Gas sensor
// #include "Air_Quality_Sensor.h"
// UV sensor
// #include "Adafruit_VEML6070.h"

// micro SD card logger
#include "SdCardLogHandlerRK.h"
#include "AdafruitDataLoggerRK.h"

// OLED display
#include "Adafruit_SH110X.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); 

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO,
    {
      {"app", LOG_LEVEL_INFO},
      {"system.nm", LOG_LEVEL_INFO},
    });



// RTC
RTCSynchronizer rtcSync;

// The SD card CS pin on the Adafruit AdaLogger FeatherWing is D5.
const int SDLOG_CHIP_SELECT = D5;
SdFat sd_fat;
SdCardPrintHandler printToCard(sd_fat, SDLOG_CHIP_SELECT, SPI_FULL_SPEED);


//OLED display
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// External status LED pins
const uint16_t EXTLED_CLOCK_PIN = (D2);
const uint16_t EXTLED_DATA_PIN = (D3);

// const uint16_t USER_LED_PIN = (D7);

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
// Ubidots ubidots((char*)"webhook", UBI_PARTICLE);


// PHTG sensor readings
static float last_temp = 0;
static float last_humidity = 0;
static float last_airpressure = 0;
static int32_t last_voc_value = 0;
static uint32_t projected_phtg_done = 0;
static uint32_t last_phtg_read = 0;

// dust sensor readings
static uint32_t last_dust_recalc_ms = 0;
// static uint32_t total_window_lpo = 0;
// static uint32_t last_known_window_lpo = 0;

static PM25_AQI_Data last_pm25_data;

// static float dust_ratio = 0;
// static float dust_concentration = 0;

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
	display.begin(0x3C, true); // Address 0x3C default
 	display.display();

	// allow some time for serial usb to start 
	//delay(3000);

	rtcSync.setup();

	// ubidots.setDebug(true);

  	// pinMode(USER_LED_PIN, OUTPUT);   
	setupPHTGSensor();
	setupDustSensor();

	// Configure the dust sensor pin as an input
	// pinMode(DUST_SENSOR_PIN, INPUT);

	// if (!voc_sensor.init()) {
	// 	Log.error("voc_sensor init failed");
	// }

	// register a cloud function to allow reading the gas level remotely 
	Particle.function("gasLevel", readGasLevel);

    display.clearDisplay();
  	display.setRotation(1);


}


// Read the PM 2.5 sensor
static void read_pm25_sensor() {
	PM25_AQI_Data data;
  
	if (pm25_sensor.read(&data)) {
		memcpy((void*)&last_pm25_data, (void*)&data,sizeof(last_pm25_data));
		last_dust_recalc_ms = millis();	
		Log.info("Concentration Units (standard)");
		Log.info("PM 1.0: %d" , data.pm10_standard);
		Log.info("PM 2.5: %d", data.pm25_standard);
		Log.info("PM 10: %d",data.pm100_standard);
		Log.info("Concentration Units (environmental)");
		Log.info("PM 1.0: %d", data.pm10_env);
		Log.info("PM 2.5: %d",data.pm25_env);
		Log.info("PM 10: %d", data.pm100_env);
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
		last_voc_value =  (int32_t)phtg_sensor.gas_resistance;
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
  //Log.info("sleep %lu ms", sleep_ms);
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
	char pub_buf[256] = {};

	sprintf(pub_buf,"{ \"press\": %0.2f, \"humid\": %0.2f, \"temp\": %0.2f, \"voc\": %lu, \"3um\": %u,\"5um\": %u, \"10um\": %u }",
		last_airpressure,
		last_humidity,
		last_temp,
		last_voc_value,
		last_pm25_data.particles_03um,
		last_pm25_data.particles_05um,
		last_pm25_data.particles_10um
	);

	Log.info("sending: %s", pub_buf);
	printToCard.printlnf(pub_buf);

	return Particle.publish("keen_io",pub_buf,WITH_ACK);

}

static void display_data() {
	char buf[32] = {};
    display.clearDisplay();

	display.setTextSize(1);
	display.setTextColor(SH110X_WHITE);
	display.setCursor(0,0);
	sprintf(buf,"%0.1fC %0.1f%% %d", last_temp, last_humidity, (int)last_voc_value);
	display.println(buf); //renders line and moves down one line in height
	display.println("");

	display.setTextSize(2);
	display.setTextColor(SH110X_BLACK);

	int16_t x1, y1;
	uint16_t text_w, text_h, graph_width, min_graph_width;
	// if (last_pm25_data.particles_10um > 0) {
		sprintf(buf," %d",last_pm25_data.particles_10um);
		display.getTextBounds(buf, display.getCursorX(), display.getCursorY(), &x1, &y1, &text_w, &text_h);
		min_graph_width = text_w + 16; //text width plus twice radius
		graph_width = (128 * last_pm25_data.particles_10um) / 100;
		if (graph_width < min_graph_width) graph_width = min_graph_width;
		display.fillRoundRect(x1, y1, graph_width, text_h,  8, SH110X_WHITE);
		display.println(buf);
	// }
	// if (last_pm25_data.particles_05um > 0) {
		sprintf(buf," %d",last_pm25_data.particles_05um);
		display.getTextBounds(buf, display.getCursorX(), display.getCursorY(), &x1, &y1, &text_w, &text_h);
		min_graph_width = text_w + 16; //text width plus twice radius
		graph_width = (128 * last_pm25_data.particles_10um) / 100;
		if (graph_width < min_graph_width) graph_width = min_graph_width;
		display.fillRoundRect(x1, y1, graph_width, text_h,  8, SH110X_WHITE);
		display.println(buf);
	// }
	// if (last_pm25_data.particles_03um > 0) {
		sprintf(buf," %d", last_pm25_data.particles_03um);
		display.getTextBounds(buf, display.getCursorX(), display.getCursorY(), &x1, &y1, &text_w, &text_h);
		min_graph_width = text_w + 16; //text width plus twice radius
		graph_width = (128 * last_pm25_data.particles_10um) / 100;
		if (graph_width < min_graph_width) graph_width = min_graph_width;
		display.fillRoundRect(x1, y1, graph_width, text_h,  8, SH110X_WHITE);
		display.println(buf);
	// }

	display.display(); // actually display all of the above
}

/* This function loops forever --------------------------------------------*/
void loop() {
	// digitalWrite(USER_LED_PIN, LOW);

	// connect if we aren't already connected
	if (!Particle.connected()) {
		Log.warn("reconnect");
		Particle.connect(); //start connection
	}

	// we use the user LED to indicate how long we spend reading sensors and publishing
	// digitalWrite(USER_LED_PIN, HIGH);

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
	// digitalWrite(USER_LED_PIN, LOW);
	display_data();
	
	if (pub_success) {
		// wait 5 minutes between publications 
		// sleep_control(300000);
		delay(15000);
	}
	else {
		Log.warn("pub failed");
		delay(2000);
	}
	
	rtcSync.loop();

	
}




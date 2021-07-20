/**
 *  Demo application for power-conserving remote condition monitoring
 */

#include <Particle.h>


#include "ChainableLED.h"
#include "Ubidots.h"

#include "Adafruit_BME280.h"
#include "Air_Quality_Sensor.h"
// UV sensor
#include "Adafruit_VEML6070.h"

Adafruit_VEML6070 uv_sensor;

const char *WEBHOOK_NAME = "ubidota";
constexpr char UBD_PROTO[] = "webhook";
Ubidots ubidots((char*)UBD_PROTO, UBI_PARTICLE);

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

SerialLogHandler logHandler(115200, LOG_LEVEL_WARN,
    {
      {"app", LOG_LEVEL_INFO},
      {"system.nm", LOG_LEVEL_INFO},
    });

// External status LED pins
const uint16_t EXTLED_CLOCK_PIN = (D2);
const uint16_t EXTLED_DATA_PIN = (D3);
const uint8_t NUM_EXT_LEDS  = 1;


const uint16_t AQS_PIN = (A2);
const uint16_t DUST_SENSOR_PIN = (D4);
// how long to collect dust readings before recalculating
const uint32_t DUST_SENSOR_WINDOW_MS  = 30000; 


// Sensor for volatile organic compounds, carbon monoxide, and so on.
AirQualitySensor voc_sensor(AQS_PIN);

// Air pressure, temperature, humidity sensor
Adafruit_BME280 bme;

// Define the digital IO pins used for external LED
ChainableLED led_chain(EXTLED_CLOCK_PIN, EXTLED_DATA_PIN, NUM_EXT_LEDS);


static float last_temp = 0;
static float last_humidity = 0;
static float last_airpressure = 0;
static uint32_t last_pth_read = 0;
static uint16_t last_uv = 0;
static uint32_t last_dust_recalc_ms = 0;
static uint32_t total_window_lpo = 0;
static uint32_t last_known_window_lpo = 0;
static float dust_ratio = 0;
static float dust_concentration = 0;

static SystemSleepConfiguration sleep_cfg = {};


/* Function prototypes -------------------------------------------------------*/
int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);
double readTemperature();
double readHumidity();

/* This function is called once at start up ----------------------------------*/
void setup()
{
	//Setup the Tinker application here
    Serial.begin();
    for (uint32_t count = 0; count < 5; count++) {
		if (Serial.isConnected()) {
			break;
		}
        // Particle.process();
        delay(1000);
    }
	
	Wire.begin();

	ubidots.setDebug(true);

	// Configure the dust sensor pin as an input
	pinMode(DUST_SENSOR_PIN, INPUT);

	if (!voc_sensor.init()) {
		Log.error("aq sensor init failed");
	}

	if (!bme.begin(BME280_ADDRESS)) {
		Log.error("BME280 init failed");
	}

	uv_sensor.begin(VEML6070_2_T);

	led_chain.init();
	
	//Register all the Tinker functions
	// Particle.function("digitalread", tinkerDigitalRead);
	// Particle.function("digitalwrite", tinkerDigitalWrite);
	// Particle.function("analogread", tinkerAnalogRead);
	// Particle.function("analogwrite", tinkerAnalogWrite);

	// Particle.variable("readTemperature", readTemperature);
	Particle.variable("readHumidity", readHumidity);

}


double readTemperature() {
	return (double)last_temp;
}

double readHumidity() {
	return (double)last_humidity;
}


// Read ultraviolet light sensor
static void read_uv_sensor() {
	uint16_t cur_uv = uv_sensor.readUV();
	if (65535 != cur_uv) { //invalid reading
		last_uv = cur_uv;
	}
}

// Read Pressure, Temperature, Humidity sensor
static void read_pth_sensor()
{
	last_temp =  bme.readTemperature();
	last_humidity = bme.readHumidity();
	last_airpressure = bme.readPressure() / 100.0F;
	last_pth_read = millis();
}

// Read particulate (dust) sensor
static void read_dust_sensor() {
	// measure the length of a low pulse
	uint32_t low_pulse_duration =  pulseIn(DUST_SENSOR_PIN, LOW);
	// filter out zeros
	if (low_pulse_duration > 0) {
		total_window_lpo +=   low_pulse_duration;
	}

	// 	periodically recalculate the particulate readings
	if ((total_window_lpo > 0) && ((millis() - last_dust_recalc_ms) > DUST_SENSOR_WINDOW_MS) ) {

		dust_ratio = total_window_lpo / (DUST_SENSOR_WINDOW_MS * 10.0);// Integer percentage 0=>100
		float ratio2 = dust_ratio * dust_ratio;
		dust_concentration = ratio2*(1.1 * dust_ratio - 3.8) + (520 * dust_ratio) + 0.62; // using spec sheet curve
		last_known_window_lpo = total_window_lpo;

		Log.trace("LPO: %lu", last_known_window_lpo);
		Log.trace("Dust Ratio: %f%%", dust_ratio);
		Log.trace("dust_concentration: %f pcs/L", dust_concentration);

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

constexpr char TEMPERATURE_TOPIC[] = "temp";
constexpr char AIR_PRESSURE_TOPIC[] = "press";
constexpr char HUMIDITY_TOPIC[] = "humidity";
constexpr char UV_TOPIC[] = "uv";


static bool publish_data() {
	
	ubidots.add((char*)TEMPERATURE_TOPIC, last_temp); 
	ubidots.add((char*)AIR_PRESSURE_TOPIC, last_airpressure); 
    ubidots.add((char*)HUMIDITY_TOPIC, last_humidity);
	ubidots.add((char*)UV_TOPIC, last_uv);

	if (last_known_window_lpo > 0) {
      ubidots.add((char*)"dust-lpo", last_known_window_lpo);
      ubidots.add((char*)"dust-ratio", dust_ratio);
      ubidots.add((char*)"dust-concentration", dust_concentration);
	}

	// Here we use a Particle webhook to send data to Ubidots
 	return ubidots.send(WEBHOOK_NAME, PUBLIC | WITH_ACK); 
}

/* This function loops forever --------------------------------------------*/
void loop()
{
	led_chain.setColorRGB(0, 0,8,0);
	// connect if we aren't already connected
	if (!Particle.connected()) {
		Log.warn("reconnect");
		Particle.connect(); //start connection
	}

	read_uv_sensor();
    read_pth_sensor();
	read_dust_sensor();

	if (last_pth_read > 0) {
		uint8_t red_val = (uint8_t)(last_temp*2.55f); //( temp  / 100)  * 255;
		uint8_t green_Val = 16;
		uint8_t blue_val = (uint8_t)(last_humidity*2.55f);  // (humidity/100) * 255;
		led_chain.setColorRGB(0, red_val, green_Val, blue_val);
	}
	else {
		led_chain.setColorRGB(0, 32,0,0);
	}

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
		sleep_control(10000);
	}
	else {
		Log.warn("pub failed");
		delay(2000);
	}
}

/*******************************************************************************
 * Function Name  : tinkerDigitalRead
 * Description    : Reads the digital value of a given pin
 * Input          : Pin
 * Output         : None.
 * Return         : Value of the pin (0 or 1) in INT type
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerDigitalRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT_PULLDOWN);
		return digitalRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT_PULLDOWN);
		return digitalRead(pinNumber+10);
	}
	
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalWrite
 * Description    : Sets the specified pin HIGH or LOW
 * Input          : Pin and value
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerDigitalWrite(String command)
{
	bool value = 0;
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(command.substring(3,7) == "HIGH") value = 1;
	else if(command.substring(3,6) == "LOW") value = 0;
	else return -2;

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		digitalWrite(pinNumber, value);
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		digitalWrite(pinNumber+10, value);
		return 1;
	}
	else return -3;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogRead
 * Description    : Reads the analog value of a pin
 * Input          : Pin
 * Output         : None.
 * Return         : Returns the analog value in INT type (0 to 4095)
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerAnalogRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		return -3;
	}
	else if (pin.startsWith("A"))
	{
		return analogRead(pinNumber+10);
	}
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogWrite
 * Description    : Writes an analog value (PWM) to the specified pin
 * Input          : Pin and Value (0 to 255)
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerAnalogWrite(String command)
{
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	String value = command.substring(3);

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		analogWrite(pinNumber, value.toInt());
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		analogWrite(pinNumber+10, value.toInt());
		return 1;
	}
	else return -2;
}


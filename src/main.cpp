/**
 *  Demo application for power-conserving remote condition monitoring
 */

#include <Particle.h>

#include "Adafruit_BME280.h"
#include "DHT22Gen3_RK.h"

#include "ChainableLED.h"
#include "Ubidots.h"

// #include "Adafruit_VEML6070.h"


Adafruit_VEML6070 uv_sensor;

const char* WEBHOOK_NAME = "ubidota";
constexpr char* UBD_PROTO = "webhook";
Ubidots ubidots(UBD_PROTO, UBI_PARTICLE);

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

SerialLogHandler logHandler(115200, LOG_LEVEL_WARN,
    {
      {"app", LOG_LEVEL_INFO},
      {"system.nm", LOG_LEVEL_INFO},
    });

const uint8_t DHT_INPUT_PIN = (A2);
const uint8_t EXTLED_CLOCK_PIN = (D2);
const uint8_t EXTLED_DATA_PIN = (D3);

const uint8_t NUM_LEDS  = 1;

/// Define the i2c bus used for the LEDs
ChainableLED led_chain(EXTLED_CLOCK_PIN, EXTLED_DATA_PIN, NUM_LEDS);

DHT dht(DHT_INPUT_PIN);

float last_temp = 0;
float last_humidity = 0;
uint16_t last_uv = 0;
 
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
	
	ubidots.setDebug(true);

	uv_sensor.begin(VEML6070_2_T);

	dht.begin();
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


static void read_dht() {

	float cur_temp = dht.getTempCelcius();
	float cur_hum = dht.getHumidity();

	Log.info("Temp: %f Humid: %f", cur_temp, cur_hum);
	if (!isnanf(cur_temp)) { last_temp = cur_temp; }
	if (!isnanf(cur_hum)) { last_humidity = cur_hum; }
}


static SystemSleepConfiguration sleep_cfg = {};

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

constexpr char* TEMPERATURE_TOPIC = "temp";
constexpr char* HUMIDITY_TOPIC = "humidity";
constexpr char* UV_TOPIC = "uv";

static bool publish_data() {
	
	ubidots.add(TEMPERATURE_TOPIC, last_temp); 
    ubidots.add(HUMIDITY_TOPIC, last_humidity);
	ubidots.add(UV_TOPIC, last_uv);

	// Will use Particle webhook to send data
 	return ubidots.send(WEBHOOK_NAME, PUBLIC | WITH_ACK); 
}

/* This function loops forever --------------------------------------------*/
void loop()
{
	led_chain.setColorRGB(0, 0,16,0);
	Log.info("loopstart");
	if (!Particle.connected()) {
		Log.warn("reconnect");
		Particle.connect(); //start connection
	}
	read_dht();
	uint16_t cur_uv = uv_sensor.readUV();
	if (65535 != cur_uv) { //invalid reading
		last_uv = cur_uv;
	}

	uint8_t red_val = (uint8_t)(last_temp*2.55f); //( * 255/100)
	uint8_t green_Val = 0;
	uint8_t blue_val = (uint8_t)(last_humidity*2.55f);  /// (humidity/100) * 255;
	led_chain.setColorRGB(0, red_val, green_Val, blue_val);

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
		sleep_control(30000);
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


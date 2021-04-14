#define __ESP32_MQTT_H__

#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define LED_PIN 12

//#define TIME_TO_SLEEP  4           // Time ESP32 will go to sleep (in seconds)
//#define TIME_TO_SLEEP_LOW_BAT 10 * 60
#define uS_TO_S_FACTOR 1000000ULL
#define MS_TO_S_FACTOR 1000ULL

//#define SERIAL_BLUETOOTH

#ifdef SERIAL_BLUETOOTH
# define RX_QUEUE_SIZE 512
# define TX_QUEUE_SIZE 32
# include "BluetoothSerial.h"
# if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#  error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
# endif
BluetoothSerial SerialMon;
#else
# define SerialMon Serial
#endif

HardwareSerial serialGsm(1);
#define SerialAT serialGsm

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 1024
#endif

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <ArduinoJson.h>
#include "FS.h"
#include <LITTLEFS.h>
#include "Tracker.h"
#include "ciotc_config.h" // Update this file with your configuration

TinyGsmClientSecure client(modem);
CloudIoTCoreDevice *device;
SHT3X sht30(0x44);
Adafruit_BMP280 bmp;

RTC_DATA_ATTR bool ntp_connected = false;
RTC_DATA_ATTR bool very_low_battery_check = false;
RTC_DATA_ATTR uint32_t boot_count = 0;
RTC_DATA_ATTR bool update_certs = true;
RTC_DATA_ATTR bool gps_enabled = false;
RTC_DATA_ATTR bool mqtt_connected = false;
RTC_DATA_ATTR uint32_t saved_states = 0;

String jwt;

unsigned long iat;
/* Variables to mesure speed of functions */
unsigned long connect_start;
unsigned long connect_end;
unsigned long delta;

//Tracker tracker;

Tracker *tracker = new Tracker;

void setup() {
#ifdef SERIAL_BLUETOOTH
	SerialMon.begin(MODULE_NAME);
#else
	SerialMon.begin(115200);
#endif
	SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX);
	Wire.begin(I2C_SDA, I2C_SCL);

	pinMode(PIN_ADC_BAT, INPUT);
	pinMode(PIN_ADC_SOLAR, INPUT);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	SerialMon.println(MODULE_HEADER);
	SerialMon.println("Version: 0.0.1");
	++boot_count;

	Serial.println("Mounting FS...");
	if (!LITTLEFS.begin()) {
		Serial.println("Failed to mount file system");
	}

	if (!bmp.begin(0x76)) {
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
	}

	tracker->checkPower();

	if (tracker->veryLowBattery) {
		if (very_low_battery_check) {
			very_low_battery_check = false;
			tracker->modemPowerOff();
			int32_t sleep =
				tracker->config.lowBatRefreshTime * uS_TO_S_FACTOR - micros();
			if (sleep < 0) { sleep = 1; }
			SerialMon.println(String("Going to sleep for ") +
					sleep + " microseconds");
			delay(200);
			esp_sleep_enable_timer_wakeup(sleep);
			esp_deep_sleep_start();
		} else {
			very_low_battery_check = true;
		}
	}
	tracker->modemWake();

	///////////////////////////////////////////////////////////////////////////
	int retry;
	connect_start = micros();

	if (!modem.isNetworkConnected()) {
		SerialMon.print("Initializing modem");
		tracker->modemPowerOn();
		retry = tracker->config.modemConnectAttempts + 1;
		while (!modem.init() && --retry) {
			SerialMon.println(" failed");
			SerialMon.print("retrying");
			tracker->modemRestart();
			delay(2000);
		}
		if (retry) {
			SerialMon.println(" success");
		} else {
			SerialMon.println(" failed");
			SerialMon.println("Check the sim card");
		}

		SerialMon.print("Connect to network");
		modem.setNetworkMode(38);
		modem.setPreferredMode(1);
		modem.sendAT(F("+CBANDCFG=\"CAT-M\","), 20);
		modem.waitResponse();
		retry = tracker->config.networkConnectAttempts + 1;
		while(!modem.isNetworkConnected() && --retry) {
			SerialMon.print(".");
			delay(500);
		}
		if (retry) {
			SerialMon.println(" success");
			tracker->networkConnected = true;
		} else {
			SerialMon.println(" failed");
			tracker->networkConnected = false;
		}
	} else {
		tracker->networkConnected = true;
	}

	if (!modem.isGprsConnected() && tracker->networkConnected) {
		SerialMon.print("Connecting to " + String(apn));
		retry = tracker->config.gprsConnectAttempts + 1;
		while (!modem.gprsConnect(apn) && --retry) {
			SerialMon.println(" fail");
			SerialMon.print("Retrying");
			modem.gprsDisconnect();
			tracker->gprsConnected = false;
		}
		if (retry) {
			SerialMon.println(" success");
			tracker->gprsConnected = true;
		} else {
			SerialMon.println(" failed");
			tracker->gprsConnected = true;
		}
	} else {
		if (tracker->networkConnected)
		{
			SerialMon.println("Gprs allready connected");
			tracker->gprsConnected = true;
		} else {
			SerialMon.println("No Network, continuing");
		}
	}
	connect_end = micros();
	delta = connect_end - connect_start;
	tracker->gprsConnectTime = (double)delta / MS_TO_S_FACTOR;

	tracker->modemLight(true);

	if (tracker->networkConnected) {
		while (update_certs)
		{
			if (tracker->modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			if (tracker->modem_upload_cert(
						client_cert_mosquitto, "client.pem", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			if (tracker->modem_upload_cert(
						client_key_mosquitto, "client.key", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			update_certs = false;
		}
	}

	tracker->mqttConnected = mqtt_connected;

	if (tracker->networkConnected && tracker->gprsConnected)
	{
		device = new CloudIoTCoreDevice(
				project_id, location, registry_id, device_id, private_key_str);

		retry = tracker->config.cloudConnectAttempts + 1;
		retry = 1;
		while(!tracker->connectCloudIoT() && --retry) {
			SerialMon.println("Cannot connect to google, retrying...");
		}
		tracker->cloudConnected = (retry  ? true : false);
	}

	///////////////////////////////////////////////////////////////////////////

	DynamicJsonDocument trackerConfig(1024);

	if (!tracker->loadFile(&trackerConfig, "/config.json", 1024)) {
		Serial.println("Failed to load config");
	} else {
		Serial.println("Config loaded");
	}
	if (tracker->cloudConnected) {
		if (!tracker->mqttSub("config", 1)) {
			SerialMon.println("Cannot subscribe");
		} else {
			SerialMon.println("Subscribed");
		}
		String configStr;
		if (!tracker->mqttReceive("config", &configStr, 30)) {
			SerialMon.println("Cannot receive config");
		} else {
			SerialMon.println("Config received");
		}
		configStr = configStr.substring(1, configStr.length() -2);
		deserializeJson(trackerConfig, configStr);
	}
	tracker->setConfig(&trackerConfig);

	if (!tracker->saveFile(&trackerConfig, "/config.json")) {
		Serial.println("Failed to save config");
	} else {
		Serial.println("Config saved");
	}

	DynamicJsonDocument trackerState(2048);

	tracker->getData();
	tracker->setState(&trackerState);

	serializeJsonPretty(trackerState, SerialMon);
	SerialMon.println();

	uint32_t bufSize =
		trackerState.memoryUsage() * (tracker->config.nSaveState + 2); // TODO be more precise than adding an arbitrary 2
	DynamicJsonDocument buffer(bufSize);

	if (saved_states) {
		if (!tracker->loadFile(&buffer, "/states.json", bufSize)) {
			Serial.println("Failed to load states");
		} else {
			Serial.println("Loading saved states");
		}
		if (saved_states >= tracker->config.nSaveState) {
			tracker->dataLost = true;
			buffer.remove(0);
			saved_states--;
		}
	}

	if (!buffer.add(trackerState)) {
		Serial.println("Warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	} else {
		saved_states++;
	}

	Serial.print("saved_states ");
	Serial.println(saved_states);


	for (int i = 0; i <= saved_states; i++) {
		DynamicJsonDocument tmp(1024);
		tmp = buffer[i].as<JsonObject>();

		if (!tracker->sendState(&tmp)) {
			break;
		} else {
			Serial.println(i);
			buffer.remove(0);
			saved_states--;
		}
	}
	if (saved_states == 0) {
		tracker->dataLost = false;
	}
	if (!tracker->saveFile(&buffer, "/states.json")) {
		Serial.println("Failed to save states");
	} else {
		Serial.println("Saving states");
	}

	serializeJsonPretty(buffer, Serial);
	Serial.println();
	Serial.print("saved_states ");
	Serial.println(saved_states);
	Serial.print("dataLost ");
	Serial.println(tracker->dataLost);

	tracker->mqttDisconnect();
	tracker->modemSleep();
	int32_t sleep = tracker->config.refreshTime * uS_TO_S_FACTOR - micros();
	if (sleep < 0)
		sleep = 1;
	SerialMon.println(String("Going to sleep for ") +
			sleep + " microseconds");
	esp_sleep_enable_timer_wakeup(sleep);
	esp_deep_sleep_start();
}

void loop() {
}

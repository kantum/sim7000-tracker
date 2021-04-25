#define __ESP32_MQTT_H__

#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define LED_PIN 12

#define uS_TO_S_FACTOR 1000000ULL
#define MS_TO_S_FACTOR 1000ULL

HardwareSerial serialGsm(1);
#define SerialAT serialGsm

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 1024
#endif

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG Serial

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
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
RTC_DATA_ATTR bool data_lost = 0;

String jwt;

unsigned long iat;
/* Variables to mesure speed of functions */
unsigned long connect_start;
unsigned long connect_end;
unsigned long delta;

Tracker tracker;

void setup() {
	Serial.begin(115200);
	SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX);
	Wire.begin(I2C_SDA, I2C_SCL);

	pinMode(PIN_ADC_BAT, INPUT);
	pinMode(PIN_ADC_SOLAR, INPUT);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	Serial.println(MODULE_HEADER);
	Serial.println("Version: 0.0.1");
	++boot_count;

	Serial.println("Mounting FS...");
	if (!LITTLEFS.begin()) {
		Serial.println("Failed to mount file system");
	}

	if (!bmp.begin(0x76)) {
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
	}

	tracker.checkPower();

	if (tracker.veryLowBattery) {
		if (very_low_battery_check) {
			very_low_battery_check = false;
			tracker.modemPowerOff();
			int32_t sleep =
				tracker.config.lowBatRefreshTime * uS_TO_S_FACTOR - micros();
			if (sleep < 0) { sleep = 1; }
			Serial.println(String("Going to sleep for ") +
					sleep + " microseconds");
			delay(200);
			esp_sleep_enable_timer_wakeup(sleep);
			esp_deep_sleep_start();
		} else {
			very_low_battery_check = true;
		}
	}
	tracker.modemWake();

	///////////////////////////////////////////////////////////////////////////
	// Connection to Google Cloud                                            //
	///////////////////////////////////////////////////////////////////////////
	int retry;
	connect_start = micros();

	if (!modem.isNetworkConnected()) {
		Serial.print("Initializing modem");
		tracker.modemPowerOn();
		retry = tracker.config.modemConnectAttempts + 1;
		while (!modem.init() && --retry) {
			Serial.println(" failed");
			Serial.print("retrying");
			tracker.modemRestart();
			delay(2000);
		}
		if (retry) {
			Serial.println(" success");
		} else {
			Serial.println(" failed");
			Serial.println("Check the sim card");
		}

		Serial.print("Connect to network");
		modem.setNetworkMode(38);
		modem.setPreferredMode(1);
		modem.sendAT(F("+CBANDCFG=\"CAT-M\","), 20);
		modem.waitResponse();
		retry = tracker.config.networkConnectAttempts + 1;
		while(!modem.isNetworkConnected() && --retry) {
			Serial.print(".");
			delay(500);
		}
		if (retry) {
			Serial.println(" success");
			tracker.networkConnected = true;
		} else {
			Serial.println(" failed");
			tracker.networkConnected = false;
		}
	} else {
		tracker.networkConnected = true;
	}

	if (!modem.isGprsConnected() && tracker.networkConnected) {
		Serial.print("Connecting to " + String(apn));
		retry = tracker.config.gprsConnectAttempts + 1;
		while (!modem.gprsConnect(apn) && --retry) {
			Serial.println(" fail");
			Serial.print("Retrying");
			modem.gprsDisconnect();
			tracker.gprsConnected = false;
		}
		if (retry) {
			Serial.println(" success");
			tracker.gprsConnected = true;
		} else {
			Serial.println(" failed");
			tracker.gprsConnected = true;
		}
	} else {
		if (tracker.networkConnected)
		{
			Serial.println("Gprs allready connected");
			tracker.gprsConnected = true;
		} else {
			Serial.println("No Network, continuing");
		}
	}
	connect_end = micros();
	delta = connect_end - connect_start;
	tracker.gprsConnectTime = (double)delta / MS_TO_S_FACTOR;

	tracker.modemLight(true);

	if (tracker.networkConnected) {
		while (update_certs)
		{
			if (tracker.modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
				Serial.println("Upload certificate failed");
				continue;
			}
			if (tracker.modem_upload_cert(
						client_cert_mosquitto, "client.pem", 3) < 1) {
				Serial.println("Upload certificate failed");
				continue;
			}
			if (tracker.modem_upload_cert(
						client_key_mosquitto, "client.key", 3) < 1) {
				Serial.println("Upload certificate failed");
				continue;
			}
			update_certs = false;
		}
	}

	tracker.mqttConnected = mqtt_connected;

	if (tracker.networkConnected && tracker.gprsConnected)
	{
		device = new CloudIoTCoreDevice(
				project_id, location, registry_id, device_id, private_key_str);

		retry = tracker.config.cloudConnectAttempts + 1;
		retry = 1;
		while(!tracker.connectCloudIoT() && --retry) {
			Serial.println("Cannot connect to google, retrying...");
		}
		tracker.cloudConnected = (retry  ? true : false);
	}

	///////////////////////////////////////////////////////////////////////////

	DynamicJsonDocument trackerConfig(1024);

	if (!tracker.loadFile(&trackerConfig, "/config.json", 1024)) {
		Serial.println("Failed to load config");
	} else {
		Serial.println("Config loaded");
	}
	if (tracker.cloudConnected) {
		if (!tracker.mqttSub("config", 1)) {
			Serial.println("Cannot subscribe");
		} else {
			Serial.println("Subscribed");
		}
		String configStr;
		if (!tracker.mqttReceive("config", &configStr, 30)) {
			Serial.println("Cannot receive config");
		} else {
			Serial.println("Config received");
		}
		configStr = configStr.substring(1, configStr.length() -2);
		deserializeJson(trackerConfig, configStr);

		serializeJsonPretty(trackerConfig, Serial);
		Serial.println();
	}
	tracker.setConfig(&trackerConfig);

	if (!tracker.saveFile(&trackerConfig, "/config.json")) {
		Serial.println("Failed to save config");
	} else {
		Serial.println("Config saved");
	}

	DynamicJsonDocument trackerState(2048);

	tracker.getData();
	tracker.setState(&trackerState);

	uint32_t bufSize =
		trackerState.memoryUsage() * (tracker.config.maxSavedStates + 2); // TODO be more precise than adding an arbitrary 2
	DynamicJsonDocument buffer(bufSize);

	if (saved_states) {
		if (!tracker.loadFile(&buffer, "/states.json", bufSize)) {
			Serial.println("Failed to load states");
		} else {
			Serial.println("Loading saved states");
		}
		if (saved_states >= tracker.config.maxSavedStates) {
			data_lost = true;
			trackerState["dataLost"] = true;
			buffer.remove(0);
			saved_states--;
		}
	}

	if (!buffer.add(trackerState)) {
		Serial.println("Warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	} else {
		saved_states++;
	}

	serializeJsonPretty(buffer, Serial);
	Serial.println();

	for (JsonVariant value : buffer.as<JsonArray>()) {
		DynamicJsonDocument tmp(1024);
		tmp = value.as<JsonObject>();

		if (!tracker.sendState(&tmp)) {
			break;
		} else {
			buffer.remove(0);
			saved_states--;
		}
	}
	if (saved_states == 0) {
		data_lost = false;
	}
	if (!tracker.saveFile(&buffer, "/states.json")) {
		Serial.println("Failed to save states");
	} else {
		Serial.println("Saving states");
	}

	serializeJsonPretty(buffer, Serial);
	Serial.println();

	tracker.mqttDisconnect();
	tracker.modemSleep();
	int32_t sleep = tracker.config.refreshTime * uS_TO_S_FACTOR - micros();
	if (sleep < 0)
		sleep = 1;
	Serial.println(String("Going to sleep for ") +
			sleep + " microseconds");
	esp_sleep_enable_timer_wakeup(sleep);
	esp_deep_sleep_start();
}

void loop() {
}

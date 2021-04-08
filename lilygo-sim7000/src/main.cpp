#define __ESP32_MQTT_H__

#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define LED_PIN 12

#define TIME_TO_SLEEP  4           // Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP_LOW_BAT 10 * 60
#define uS_TO_S_FACTOR 1000000ULL
#define MS_TO_S_FACTOR 1000ULL

//#define SERIAL_BLUETOOTH

#ifdef SERIAL_BLUETOOTH
#define RX_QUEUE_SIZE 512
#define TX_QUEUE_SIZE 32
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialMon;
#else
#define SerialMon Serial
#endif

HardwareSerial serialGsm(1);
#define SerialAT serialGsm

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS
//#define TINY_GSM_DEBUG SerialMon

#define CONNECT_ATTEMPTS 1

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <Client.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <CloudIoTCore.h>
#include <CloudIoTCoreMqtt.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <ArduinoJson.h>
#include "Tracker.h"
#include "ciotc_config.h" // Update this file with your configuration

TinyGsmClientSecure client(modem);
CloudIoTCoreDevice *device;
SHT3X sht30(0x44);
Adafruit_BMP280 bme;

RTC_DATA_ATTR bool ntp_connected = false;
RTC_DATA_ATTR bool veryLowBatteryCheck = false;
RTC_DATA_ATTR int boot_count = 0;
RTC_DATA_ATTR bool update_certs = true;
RTC_DATA_ATTR bool gps_enabled = false;

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

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
	}

	tracker->checkPower(3400, 3000);

	if (tracker->veryLowBattery) {
		if (veryLowBatteryCheck) {
			tracker->modemPowerOff();
			esp_sleep_enable_timer_wakeup(
					TIME_TO_SLEEP_LOW_BAT * uS_TO_S_FACTOR);
			delay(200);
			SerialMon.println(String("Going to sleep for ") +
					TIME_TO_SLEEP_LOW_BAT + " seconds");
			esp_deep_sleep_start();
		} else {
			veryLowBatteryCheck = true;
		}
	}
	tracker->modemWake();

	////////////////////////////////////////////////////////////////////////////////
	int retry;
	connect_start = micros();
	if (!modem.isNetworkConnected()) {

		SerialMon.print("Initializing modem");
		tracker->modemPowerOn();
		retry = CONNECT_ATTEMPTS + 1;
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
		retry = CONNECT_ATTEMPTS + 1;
		while(!modem.isNetworkConnected() && --retry) {
			SerialMon.print(".");
			delay(10);
		}
		if (retry) {
			SerialMon.println(" success");
			tracker->networkConnected = true;
		} else {
			SerialMon.println(" failed");
			SerialMon.println("Cannot connect, continuing without connection");
			tracker->networkConnected = false;
		}
	}

	if (!modem.isGprsConnected() && tracker->networkConnected) {
		SerialMon.print("Connecting to " + String(apn));
		retry = CONNECT_ATTEMPTS + 1;
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

	if (tracker->networkConnected && tracker->gprsConnected)
	{
		while (update_certs)
		{
			if (tracker->modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			if (tracker->modem_upload_cert(client_cert_mosquitto, "client.pem", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			if (tracker->modem_upload_cert(client_key_mosquitto, "client.key", 3) < 1) {
				SerialMon.println("Upload certificate failed");
				continue;
			}
			update_certs = false;
		}
		device = new CloudIoTCoreDevice(
				project_id, location, registry_id, device_id, private_key_str);

		int gcloudAttempts= CONNECT_ATTEMPTS + 1;
		while(!tracker->connectCloudIoT() && gcloudAttempts--) {
			SerialMon.println("Cannot connect to google, retrying...");
		}
		tracker->gcloudConnected = (gcloudAttempts ? true : false);
	}


	////////////////////////////////////////////////////////////////////////////////

	if (!tracker->mqttSub("config", 1)) {
		SerialMon.println("Cannot subscribe");
	} else {
		SerialMon.println("Subscribed");
	}
	String config;
	if (!tracker->mqttReceive("config", &config, 30)) {
		SerialMon.println("Cannot receive config");
	} else {
		SerialMon.print("Config received: ");
		SerialMon.println(config);
	}

	DynamicJsonDocument trackerConfig(1024);

	deserializeJson(trackerConfig, config);

	DynamicJsonDocument trackerState(1024);
	if (!tracker->enablePSM(true)) {
		SerialMon.println("Cannot enable PSM");
	} else {
		SerialMon.println("PSM enabled");
	}

	SerialMon.print("Data collection:");
	if (tracker->getData()) {
		SerialMon.println("All data gathered");
	} else {
		SerialMon.println("Some data missing");
	}

	tracker->setState( &trackerState);

	serializeJsonPretty(trackerState, SerialMon);
	SerialMon.println();

	if (tracker->networkConnected && tracker->gprsConnected)
		tracker->sendState(&trackerState);

	tracker->modemSleep();
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
	delay(200);
	SerialMon.println(String("Going to sleep for ") +
			TIME_TO_SLEEP + " seconds");
	esp_deep_sleep_start();
}

void loop() {
}

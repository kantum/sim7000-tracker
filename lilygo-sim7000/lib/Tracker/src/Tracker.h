#ifndef _Tracker_h
#define _Tracker_h

#include <stdint.h>
#include <time.h>
#include <sys/cdefs.h>
#include <sys/time.h>

#include <Client.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <CloudIoTCore.h>
#include <CloudIoTCoreMqtt.h>

#include <ArduinoJson.h>

#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define PIN_ADC_BAT 35
#define PIN_ADC_SOLAR 36
#define ADC_BATTERY_LEVEL_SAMPLES 100

#ifdef SERIAL_BLUETOOTH
#define RX_QUEUE_SIZE 512
#define TX_QUEUE_SIZE 32
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
extern BluetoothSerial SerialMon;
#else
#define SerialMon Serial
#endif

extern HardwareSerial serialGsm;
#define SerialAT serialGsm

extern TinyGsm modem;

extern RTC_DATA_ATTR bool ntp_connected;
extern RTC_DATA_ATTR bool gps_enabled;
extern RTC_DATA_ATTR int boot_count;

extern String jwt;
extern unsigned long iat;
extern CloudIoTCoreDevice *device;
extern const int jwt_exp_secs;

extern const char *project_id;
extern const char *location;
extern const char *registry_id;
extern const char *device_id;

#include "SHT3X.h"
extern SHT3X sht30;

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
extern Adafruit_BMP280 bme;

class Tracker
{
	public:
		Tracker(/* args */);
		~Tracker();
		void	pass_command();
		void	enableGPS(void);
		void	disableGPS(void);
		void	modemPowerOn();
		void	modemPowerOff();
		void	modemRestart();
		void	modemOff();
		bool	set_eDRX(uint8_t mode, uint8_t connType, char * eDRX_val);
		bool	enablePSM(bool onoff);
		bool	enablePSM(bool onoff, char * TAU_val, char * activeTime_val);
		void	modemSleep();
		void	modemWake();
		byte	NTPServerSync(
				String server = "pool.ntp.org",
				byte timezone = 1);
		void	setupTime();
		bool	connectCloudIoT();
		String	getJwt();
		void	read_adc_solar(uint16_t *voltage);
		void	read_adc_bat(uint16_t *voltage);
		int		modem_upload_cert(
				const char *cert, const char *name, int folder);
		bool	sendMqtt(const char *msg, const char *topic = "events",
				int qos = 1, bool retain = 1);
		bool	mqttSub(const char *topic, int qos);
		void	checkPower(void);
		bool	modemLight(bool onoff);
		bool	getData(void);
		void	setState(DynamicJsonDocument *state);
		void	sendState(DynamicJsonDocument *state);
		bool	mqttReceive(const char *topic, String *data, int timeout = 5);
		bool	mqttDisconnect(void);
		//bool	mqttConfig(void); // TODO
		//bool	mqttConnect(void); // TODO
		//bool	mqttUnSub(void); // TODO

		struct config {
			uint8_t refreshTime = 10;
			uint8_t lowBatRefreshTime = 60;
			uint8_t modemConnectAttempts = 10;
			uint8_t networkConnectAttempts = 20;
			uint8_t gprsConnectAttempts = 10;
			uint8_t psmEnableAttempts = 2;
			uint8_t cloudConnectAttempts = 10;
			uint32_t lowBatThreshold = 3400;
			uint32_t veryLowBatThreshold = 3200;
		};

		uint32_t bootCount;
		uint32_t gprsConnectTime;
		time_t timestamp;
		uint16_t batVoltage;
		uint16_t solVoltage;
		uint16_t pressure;
		int8_t temp;
		uint8_t hum;
		float lat;
		float lon;
		float alt;
		float speed;
		float accuracy;
		int vsat;
		int usat;

		bool gpsEnabled;
		bool gpsLocked;
		bool batCharging;
		bool lowBattery;
		bool veryLowBattery;
		bool networkConnected = false;
		bool gprsConnected = false;
		bool cloudConnected = false;
		bool mqttConnected = false;

		config config;

	private:
};

#endif

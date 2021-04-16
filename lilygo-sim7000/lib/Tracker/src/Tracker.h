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
#include "FS.h"
#include <LITTLEFS.h>

#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define PIN_ADC_BAT 35
#define PIN_ADC_SOLAR 36
#define ADC_BATTERY_LEVEL_SAMPLES 100

extern HardwareSerial serialGsm;
#define SerialAT serialGsm

extern TinyGsm modem;

extern RTC_DATA_ATTR bool ntp_connected;
extern RTC_DATA_ATTR bool gps_enabled;
extern RTC_DATA_ATTR uint32_t boot_count;
extern RTC_DATA_ATTR uint32_t saved_states;

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
extern Adafruit_BMP280 bmp;

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
		bool	set_eDRX(uint8_t mode, uint8_t connType, const char * eDRX_val);
		bool	enablePSM(bool onoff);
		bool	enablePSM(
				bool onoff, const char * TAU_val, const char * activeTime_val);
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
		bool	sendState(DynamicJsonDocument *state);
		bool	mqttReceive(const char *topic, String *data, int timeout = 5);
		bool	mqttDisconnect(void);
		//bool	mqttConfig(void); // TODO
		//bool	mqttConnect(void); // TODO
		//bool	mqttUnSub(void); // TODO
		void	setConfig(DynamicJsonDocument *json);
		bool	saveFile(DynamicJsonDocument *json, const char *filepath);
		bool	loadFile(DynamicJsonDocument *json,
				const char *filepath, uint32_t maxsize);

		struct config {
			uint8_t refreshTime = 10;
			uint8_t lowBatRefreshTime = 60;
			uint32_t lowBatThreshold = 3400;
			uint32_t veryLowBatThreshold = 3200;
			uint8_t modemConnectAttempts = 10;
			uint8_t networkConnectAttempts = 10;
			uint8_t gprsConnectAttempts = 10;
			uint8_t psmEnableAttempts = 2;
			uint8_t cloudConnectAttempts = 10;
			uint16_t nSaveState = 100;
		};

		uint32_t bootCount;
		uint32_t gprsConnectTime;
		time_t timestamp;
		uint16_t batVoltage;
		uint16_t solVoltage;
		float pressure;
		uint32_t savedStates;
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
		bool dataLost = false;

		config config;

	private:
};

#endif

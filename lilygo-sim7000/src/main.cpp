#define __ESP32_MQTT_H__

#define TINY_GSM_MODEM_SIM7000

#include <TinyGsmClient.h>

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define PIN_ADC_BAT 35
#define PIN_ADC_SOLAR 36
#define ADC_BATTERY_LEVEL_SAMPLES 100

#define LED_PIN 12

#define TIME_TO_SLEEP  4           // Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP_LOW_BAT 10 * 60
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for ms to seconds
#define MS_TO_S_FACTOR 1000ULL  // Conversion factor for ms to seconds

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
#define TINY_GSM_DEBUG SerialMon

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
#include "ciotc_config.h" // Update this file with your configuration

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SHT3X.h"

byte	NTPServerSync(String server, byte timezone);
void	setupTime();
String	getJwt();
void	pass_command();
void	read_adc_solar(uint16_t *voltage);
void	read_adc_bat(uint16_t *voltage);
bool	connectCloudIoT();
void	enableGPS(void);
void	disableGPS(void);
int		modem_upload_cert(const char *cert, const char *name, int folder);
void	modemPowerOn();
void	modemPowerOff();
void	modemRestart();
void	modemOff();
boolean	set_eDRX(uint8_t mode, uint8_t connType, char * eDRX_val);
boolean	enablePSM(bool onoff);
boolean	enablePSM(bool onoff, char * TAU_val, char * activeTime_val);
void	modemSleep();
void	modemWake();
bool	sendMqtt(String msg, String topic = "events");
void	checkPower(uint32_t alert = 3400, uint32_t min = 3000);
bool	modemLight(bool onoff);

TinyGsmClientSecure client(modem);
Client *netClient;
CloudIoTCoreDevice *device;
MQTTClient *mqttClient;
SHT3X sht30(0x44);
Adafruit_BMP280 bme;

RTC_DATA_ATTR bool ntp_connected  = false;
RTC_DATA_ATTR bool veryLowBatteryCheck  = false;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool update_certs = true;

String jwt;

unsigned long iat = 0;
/* Variables to mesure speed of functions */
unsigned long connect_start;
unsigned long connect_end;
unsigned long delta;
int vsat;
int usat;

struct Geo {
	float lat;
	float lon;
};

struct Data {
	unsigned long connectTime;
	time_t timestamp;
	bool battChargeState;
	uint16_t battMilliVolts;
	uint16_t solarMilliVolts;
	int pressure;
	int temp;
	int hum;
	struct Geo localisation;
	float lat;
	float lon;
	float alt;
	float speed;
	float accuracy;
	bool lowBattery;
	bool veryLowBattery;
};

struct Data tracker;

void setup() {
	checkPower(3400, 3300);
	if (tracker.veryLowBattery) {
		if (veryLowBatteryCheck) {
			modemPowerOff();
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
#ifdef SERIAL_BLUETOOTH
	SerialMon.begin(MODULE_NAME);
#else
	SerialMon.begin(115200);
#endif
	SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX);
	Wire.begin(I2C_SDA, I2C_SCL);

	pinMode(PIN_ADC_BAT, INPUT);
	pinMode(PIN_ADC_SOLAR, INPUT);

	SerialMon.println(MODULE_HEADER);
	SerialMon.println("Version: 0.0.1");
	++bootCount;

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
	}

	modemWake();

	connect_start = micros();
	if (!modem.isNetworkConnected()) {

		pinMode(LED_PIN, OUTPUT);
		digitalWrite(LED_PIN, HIGH);

		SerialMon.print("Initializing modem");

		modemPowerOn();

		if (!modem.init()) {
			modemRestart();
			delay(2000);
		}

		modem.setNetworkMode(38);
		modem.setPreferredMode(1);
		modem.sendAT(F("+CBANDCFG=\"CAT-M\","), 20);
		modem.waitResponse();

		while(!modem.isNetworkConnected()) {
			Serial.print(".");
			delay(10);
		}
	}

	if (!modem.isGprsConnected()) {
		SerialMon.print("Connecting to " + String(apn));
		while (!modem.gprsConnect(apn)) {
			SerialMon.println(" fail");
			SerialMon.print("Retrying");
			modem.gprsDisconnect();
		}
		SerialMon.println(" success");
	} else {
		SerialMon.println("Gprs allready connected");
	}
	connect_end = micros();
	delta = connect_end - connect_start;
	tracker.connectTime = (double)delta / MS_TO_S_FACTOR;

	modemLight(true);

	while (update_certs)
	{
		if (modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
			SerialMon.println("Upload certificate failed");
			continue;
		}
		if (modem_upload_cert(client_cert_mosquitto, "client.pem", 3) < 1) {
			SerialMon.println("Upload certificate failed");
			continue;
		}
		if (modem_upload_cert(client_key_mosquitto, "client.key", 3) < 1) {
			SerialMon.println("Upload certificate failed");
			continue;
		}
		update_certs = false;
	}

	device = new CloudIoTCoreDevice(
			project_id, location, registry_id, device_id, private_key_str);

	while(!connectCloudIoT()) {
		SerialMon.println("Cannot connect to google, retrying...");
		//pass_command();
	}

/*//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Get data  /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////*/

	tracker.timestamp = iat;

	read_adc_bat(&tracker.battMilliVolts);
	read_adc_solar(&tracker.solarMilliVolts);

	tracker.battChargeState = tracker.battMilliVolts == 0 ? true : false;
	if (tracker.solarMilliVolts > 4000)
		tracker.battChargeState = true;

	tracker.pressure = bme.readPressure();
	if (sht30.get() == 0) {
		tracker.temp = sht30.cTemp;
		tracker.hum = sht30.humidity;
	}

	enableGPS();
	tracker.lat = 0.0000;
	tracker.lon = 0.0000;
	tracker.alt = 0.0000;
	tracker.speed = 0.0000;
	tracker.accuracy = 0.0000;

	Serial.println("Checking GPS");
	if (modem.getGPS(&tracker.lat, &tracker.lon, &tracker.speed, &tracker.alt,
				&vsat, &usat, &tracker.accuracy)) {
		Serial.println("The location has been locked:");
	} else {
		Serial.println("No GPS data yet");
	}

	SerialMon.println(String("ts:                 ") + tracker.timestamp);
	SerialMon.println(String("boot count:         ") + bootCount);
	SerialMon.println(String("connect time:       ") + tracker.connectTime);
	SerialMon.println(String("battery charging:   ") + tracker.battChargeState);
	SerialMon.println(String("battery voltage:    ") + tracker.battMilliVolts);
	SerialMon.println(String("solar voltage:      ") + tracker.solarMilliVolts);
	SerialMon.println(String("low battery:        ") + tracker.lowBattery ? "true" : "false");
	SerialMon.println(String("very low battery:   ") + tracker.veryLowBattery);
	SerialMon.println(String("temperature:        ") + tracker.temp);
	SerialMon.println(String("pressure:           ") + tracker.pressure);
	SerialMon.println(String("humidity:           ") + tracker.hum);
	SerialMon.println(String("lat:                ") + String(tracker.lat, 7));
	SerialMon.println(String("lon:                ") + String(tracker.lon, 7));
	SerialMon.println(String("alt:                ") + tracker.alt);
	SerialMon.println(String("speed:              ") + tracker.speed);
	SerialMon.println(String("accuracy:           ") + tracker.accuracy);
	SerialMon.println(String("vsat:               ") + vsat);
	SerialMon.println(String("usat:               ") + usat);

	digitalWrite(LED_PIN, LOW);
	if (!sendMqtt(
				String("{\"ts\":") + tracker.timestamp +
				",\"bootCount\":" + bootCount +
				",\"gprsConnectTime\":" + tracker.connectTime +
				",\"batCharging\":" + (tracker.battChargeState ? "true" : "false") +
				",\"batVoltage\":" + tracker.battMilliVolts +
				",\"solVoltage\":" + tracker.solarMilliVolts +
				",\"lowBattery\":" + (tracker.lowBattery ? "true" : "false") +
				",\"veryLowBattery\":" + (tracker.veryLowBattery ? "true" : "false") +
				",\"temperature\":" + tracker.temp +
				",\"pressure\":" + tracker.pressure +
				",\"humidity\":" + tracker.hum +
				",\"lat\":" + String(tracker.lat, 7) +
				",\"lon\":" + String(tracker.lon, 7) +
				",\"alt\":" + tracker.alt +
				",\"speed\":" + tracker.speed +
				",\"accuracy\":" + tracker.accuracy +
				"}"))
	{
		SerialMon.println("Cannot send MQTT");
	}
	digitalWrite(LED_PIN, HIGH);

	modemSleep();
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
	delay(200);
	SerialMon.println(String("Going to sleep for ") +
			TIME_TO_SLEEP + " seconds");
	esp_deep_sleep_start();
}

void loop() {
}
/**
 * @brief Setup the internal clock with modem and NTP
 */
void setupTime(){
	SerialMon.print("Synchronize with ntp server");
	if (NTPServerSync("pool.ntp.org", 2) < 0) {
		SerialMon.println(" fail");
	} else {
		SerialMon.println(" success");
	}

	struct tm tm;
	float timezone;

	modem.getNetworkTime(&tm.tm_year, &tm.tm_mon, &tm.tm_mday,
			&tm.tm_hour, &tm.tm_min, &tm.tm_sec, &timezone);

	tm.tm_year -= 1900;
	tm.tm_mon -= 1;
	tm.tm_hour -= 2;

	time_t t = mktime(&tm);

	struct timeval tv = { .tv_sec = t };

	settimeofday(&tv, NULL);
}

/**
 * @brief Handle the SSL settings and connect to google iot core
 */
bool connectCloudIoT() {

	setupTime();
	jwt = getJwt();

	String client_id = device->getClientId();
	String url = CLOUD_IOT_CORE_MQTT_HOST_LTS;
	String port = "8883";
	String keep_time = "60";

	SerialMon.println("Setting SSL");

	modem.sendAT(String("+CSSLCFG=\"sslversion\",0,3"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+CSSLCFG=\"convert\",2,\"ca.pem\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+CSSLCFG=\"convert\",1,\"client.pem\",\"client.key\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMSSL=1,\"ca.pem\",\"client.pem\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	SerialMon.print(String("Connecting to ") + url + " ");

	modem.sendAT(GF("+SMDISC"));
	modem.waitResponse();

	modem.sendAT(String("+SMCONF=\"URL\",\"") +
			url +
			GF("\",\"") +
			port +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF=\"KEEPTIME\",\"") +
			keep_time +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF=\"CLIENTID\",\"") +
			client_id +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(String("+SMCONF=\"USERNAME\",\"") +
			"unused" +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF=\"PASSWORD\",\"") +
			jwt +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF=\"CLEANSS\",1"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF=\"QOS\",1"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(String("+SMCONF=\"RETAIN\",1"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONF?"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	modem.sendAT(GF("+SMCONN"));
	if (modem.waitResponse(30000L) != 1) {SerialMon.println("ERROR"); return 0;}

	SerialMon.println("OK");
	return 1;
}

/**
 * @brief Enable SIM7000's gps
 */
void enableGPS(void)
{
	// Set SIM7000G GPIO4 LOW ,turn on GPS power
	// CMD:AT+SGPIO=0,4,1,1
	// Only in version 20200415 is there a function to control GPS power
	modem.sendAT("+SGPIO=0,4,1,1");
	modem.enableGPS();
}

/**
 * @brief Disable SIM7000's gps
 */
void disableGPS(void)
{
	// Set SIM7000G GPIO4 LOW ,turn off GPS power
	// CMD:AT+SGPIO=0,4,1,0
	// Only in version 20200415 is there a function to control GPS power
	modem.sendAT("+SGPIO=0,4,1,0");
	modem.disableGPS();
}

/**
 * @brief
 * @param cert The String containing the certificate to upload
 * @param name The name for the uploaded file
 * @param folder 0 "/custapp/" 1 "/fota/" 2 "/datatx/" 3 "/customer/"
 * @retval 0 success
 * @retval -1 failed to init buffer
 * @retval -2 failed to send write file command
 * @retval -3 failed to send the string
 * @retval -4 failed to free buffer
 */
int modem_upload_cert(const char *cert, const char *name, int folder)
{
	int len = strlen(cert);

	SerialMon.println(String("Upload certificate: ") + name);

	// Init buffer
	modem.sendAT(F("+CFSINIT"));
	if (modem.waitResponse(10000L) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -1;
	}

	int timeout = 10000;

	// Write file
	modem.sendAT(String("+CFSWFILE=") +
			folder +
			",\"" +
			name +
			"\",0," +
			len +
			"," +
			timeout);
	if (modem.waitResponse(10000L, GF("DOWNLOAD")) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -2;
	}

	for (int i = 0; i < len; i++)
		SerialAT.print(cert[i]);

	if (modem.waitResponse(10000L, GF("OK")) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -3;
	}

	// Free buffer
	modem.sendAT(F("+CFSTERM"));
	if (modem.waitResponse(10000L) != 1) {
		return -4;
	}

	return 1;
}

/**
 * @brief Switch modem on with hardware pins
 */
void modemPowerOn() {
	pinMode(MODEM_PWKEY, OUTPUT);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1000);    //Datasheet Ton mintues = 1S
	digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Switch modem off with hardware pins
 */
void modemPowerOff() {
	pinMode(MODEM_PWKEY, OUTPUT);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1500);    //Datasheet Ton mintues = 1.2S
	digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Restarts modem with hardware pins
 */
void modemRestart() {
	modemPowerOff();
	delay(1000);
	modemPowerOn();
}

void modemOff() {
	//if you turn modem off while activating the fancy sleep modes it takes ~20sec, else its immediate
	Serial.println("Going to sleep now with modem turned off");
	//modem.gprsDisconnect();
	//modem.radioOff();
	modem.sleepEnable(false); // required in case sleep was activated and will apply after reboot
	modem.poweroff();
}

/* @brief Set e-RX parameters
 ** Mode options:
 ** 0  Disable the use of eDRX
 ** 1  Enable the use of eDRX
 ** 2  Enable the use of eDRX and auto report
 ** 3  Disable the use of eDRX(Reserved)
 **
 ** Connection type options:
 ** 4 - CAT-M
 ** 5 - NB-IoT
 **
 ** See AT command manual for eDRX values (options 0-15)
 ** NOTE: Network must support eDRX mode
 */
boolean set_eDRX(uint8_t mode, uint8_t connType, char * eDRX_val) {
	if (strlen(eDRX_val) > 4) return false;

	char auxStr[21];

	sprintf(auxStr, "AT+CEDRXS=%i,%i,\"%s\"", mode, connType, eDRX_val);

	modem.stream.println(auxStr);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

/*
 ** NOTE: Network must support PSM and modem needs to restart before it takes effect
 */
boolean enablePSM(bool onoff) {
	modem.stream.println(String("AT+CSCLK=1"));
	modem.stream.println(String("AT+CPSMS=") + onoff);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}
// Set PSM with custom TAU and active time
// For both TAU and Active time, leftmost 3 bits represent the multiplier and rightmost 5 bits represent the value in bits.

// For TAU, left 3 bits:
// 000 10min
// 001 1hr
// 010 10hr
// 011 2s
// 100 30s
// 101 1min
//
// For Active time, left 3 bits:
// 000 2s
// 001 1min
// 010 6min
// 111 disabled

// Note: Network decides the final value of the TAU and active time. 
boolean enablePSM(bool onoff, char * TAU_val, char * activeTime_val) { // AT+CPSMS command
	if (strlen(activeTime_val) > 8) return false;
	if (strlen(TAU_val) > 8) return false;

	modem.stream.println(String("AT+CSCLK=1"));

	char auxStr[35];
	sprintf(auxStr, "AT+CPSMS=%i,,,\"%s\",\"%s\"", onoff, TAU_val, activeTime_val);

	modem.stream.println(auxStr);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

// fancy low power mode - while connected
// will have an effect after reboot and will replace normal power down
void modemSleep() {
	Serial.println("Going to sleep now with modem in power save mode");
	// needs reboot to activa and takes ~20sec to sleep
	while(!enablePSM(1, "01100010", "00000001")) {
		Serial.println("Cannot enable PSM, retrying");
		delay(100);
	}
	// https://github.com/botletics/SIM7000-LTE-Shield/wiki/Current-Consumption#e-drx-mode
	//	while(!set_eDRX(2, 4, "1111")) {
	//		Serial.println("Cannot enable eDRX, retrying");
	//		delay(100);
	//	}
	modem.sleepEnable(); //will sleep (1.7mA), needs DTR or PWRKEY to wake
	pinMode(MODEM_DTR, OUTPUT);
	digitalWrite(MODEM_DTR, HIGH);
}

/**
 * @brief Wake modem from sleep with DTR pin
 */
void modemWake() {
	Serial.println("Wake up modem from sleep");
	pinMode(MODEM_DTR, OUTPUT);
	digitalWrite(MODEM_DTR, LOW);
	delay(50);
}

/**
 * @brief Sync ntp time with SIM7000 internal clock
 * @param server The NTP server address
 * @param timezone The time zone you need to get when you call 
 */
byte NTPServerSync(String server = "pool.ntp.org", byte timezone = 1) {
	// Set GPRS bearer profile to associate with NTP sync
	modem.sendAT(GF("+CNTPCID=1"));
	if (modem.waitResponse(10000L) != 1) { return -1; }

	//// Set NTP server and timezone
	//modem.sendAT(GF("+CNTP="), server, ',', String(timezone));
	//if (modem.waitResponse(10000L) != 1) { return -1; }
	return 1;
}

/**
 * @brief Create JWT token
 */
String getJwt() {
	iat = time(nullptr);
	Serial.println("Refreshing JWT");
	jwt = device->createJWT(iat, jwt_exp_secs);
	//Serial.println(jwt);

	return jwt;
}

/**
 * @brief Send MQTT message
 * @param msg The payload to send
 * @param topic The MQTT topic
 */
bool sendMqtt(String msg, String topic) {
	SerialMon.print("Sending message: ");
	SerialMon.println(msg);

	int len = msg.length();

	modem.sendAT(String("+SMPUB=") +
			"\"/devices/" +
			GCLOUD_DEVICE_ID +
			"/" +
			topic +
			"\"," +
			len +
			",1,1");
	if (modem.waitResponse(GF(">")) != 1) { return 0; }
	modem.stream.write(msg.c_str(), len);
	if (modem.waitResponse(GF("OK")) != 1) { return 0; }
	modem.stream.flush();

	return 1;
}

/**
 * @brief UART passthrough
 */
void pass_command() {
	SerialAT.println("ate");
	while (true) {
		if (SerialAT.available())
			Serial.write(SerialAT.read());
		if (Serial.available())
			SerialAT.write(Serial.read());
		//delay(1);
	}
}

void read_adc_bat(uint16_t *voltage) {
	uint32_t in = 0;
	for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++) {
		in += (uint32_t)analogRead(PIN_ADC_BAT);
	}
	in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

	uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

	*voltage = bat_mv;
}

void read_adc_solar(uint16_t *voltage) {
	uint32_t in = 0;
	for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++) {
		in += (uint32_t)analogRead(PIN_ADC_SOLAR);
	}
	in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

	uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

	*voltage = bat_mv;
}

void checkPower(uint32_t alert, uint32_t min) {
	pinMode(PIN_ADC_BAT, INPUT);
	read_adc_bat(&tracker.battMilliVolts);
	if (tracker.battMilliVolts <= alert && tracker.battMilliVolts > 0)
	{
		tracker.lowBattery = true;
		if (tracker.battMilliVolts <= min)
			tracker.veryLowBattery = true;
	} else {
		tracker.lowBattery = false;
		tracker.veryLowBattery = false;
	}

}

bool modemLight(bool onoff) {
	modem.sendAT(F("+CSGS="), onoff);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

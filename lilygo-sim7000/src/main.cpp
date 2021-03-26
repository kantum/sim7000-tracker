#define __ESP32_MQTT_H__

////////////////////////////////////////////////////////////////////////////////
// From TinyGsm examples
////////////////////////////////////////////////////////////////////////////////

#define TINY_GSM_MODEM_SIM7000

#include <TinyGsmClient.h>

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define MODEM_TX 26
#define MODEM_RX 27

#define LED_PIN 12

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for ms to seconds

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
#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClientSecure client(modem);

time_t epoch;
bool ntp_connected = false;

////////////////////////////////////////////////////////////////////////////////
// From esp32-mqtt.h
////////////////////////////////////////////////////////////////////////////////
#include <Client.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <CloudIoTCore.h>
#include <CloudIoTCoreMqtt.h>

#include "ciotc_config.h" // Update this file with your configuration

void messageReceived(String &topic, String &payload){
	Serial.println("incoming: " + topic + " - " + payload);
}

Client *netClient;
CloudIoTCoreDevice *device;
//CloudIoTCoreMqtt *mqtt;
MQTTClient *mqttClient;
unsigned long iat = 0;
String jwt;

String getDefaultSensor(){
	return "TEST DATA";
}

void setupTime(){
	configTime(0, 0, ntp_primary, ntp_secondary);
	Serial.println("Waiting on time sync...");
	while (time(nullptr) < 1510644967){
		delay(10);
	}
}

//bool publishTelemetry(String data){
//	return mqtt->publishTelemetry(data);
//}
//
//bool publishTelemetry(const char *data, int length){
//	return mqtt->publishTelemetry(data, length);
//}
//
//bool publishTelemetry(String subfolder, String data){
//	return mqtt->publishTelemetry(subfolder, data);
//}
//
//bool publishTelemetry(String subfolder, const char *data, int length){
//	return mqtt->publishTelemetry(subfolder, data, length);
//}

//void connect(){
//	mqtt->mqttConnect();
//}

void setupCloudIoT(){
	device = new CloudIoTCoreDevice(
			project_id, location, registry_id, device_id,
			private_key_str);


	//netClient = (Client*) &client;
	//mqttClient = new MQTTClient(512);
	////mqttClient->setOptions(180, true, 1000); // keepAlive, cleanSession, timeout
	//mqttClient->setOptions(180, true, 1000); // keepAlive, cleanSession, timeout
	//mqtt = new CloudIoTCoreMqtt(mqttClient, netClient, device);
	//mqtt->setUseLts(true);
	//mqtt->startMQTT();
}

////////////////////////////////////////////////////////////////////////////////
// From Kantum
////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SHT3X.h"

SHT3X sht30(0x44);
Adafruit_BMP280 bme;

RTC_DATA_ATTR int update_certs = 1;

/* Variables to mesure speed of functions */
unsigned long start;
unsigned long end;
unsigned long delta;

int battpercent;
int pressure;
int temp;
int hum;
float lat;
float lon;
String getJwt();
void pass_command();

/**
 * @brief Handle the SSL settings and connect to google iot core
 */
bool connectCloudIoT() {
	jwt = getJwt();

	String client_id = device->getClientId();
	String url = CLOUD_IOT_CORE_MQTT_HOST_LTS;
	String port = "8883";
	String keep_time = "60";

	Serial.println(client_id);
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
int modem_upload_cert(const char *cert, const char *name, int folder = 0)
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
void modemPowerOn()
{
    pinMode(MODEM_PWKEY, OUTPUT);
    digitalWrite(MODEM_PWKEY, LOW);
    delay(1000);    //Datasheet Ton mintues = 1S
    digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Switch modem off with hardware pins
 */
void modemPowerOff()
{
    pinMode(MODEM_PWKEY, OUTPUT);
    digitalWrite(MODEM_PWKEY, LOW);
    delay(1500);    //Datasheet Ton mintues = 1.2S
    digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Restarts modem with hardware pins
 */
void modemRestart()
{
    modemPowerOff();
    delay(1000);
    modemPowerOn();
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

	// Set NTP server and timezone
	modem.sendAT(GF("+CNTP="), server, ',', String(timezone));
	if (modem.waitResponse(10000L) != 1) { return -1; }
	return 1;
}

/**
 * @brief Retrieves epoch from SIM7000
 */
time_t getEpoch() {
	if (!ntp_connected)
	{
		if (NTPServerSync("pool.ntp.org", 1) < 0) {
			SerialMon.println("NTP Failed");
			ntp_connected = false;
			return 0;
		}
		ntp_connected = true;
	}

	const char *timestr = modem.getGSMDateTime(DATE_FULL).c_str();
	struct tm t;
	//SerialMon.println(timestr);
	strptime(timestr, "%y/%m/%d,%H:%M:%S%z", &t);
	t.tm_hour -=1; // TODO Find a better to way to get good timezone (%z doesn't work)
	return mktime(&t);
}

/**
 * @brief Create JWT token
 */
String getJwt(){
	iat = getEpoch();
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
bool sendMqtt(String msg, String topic="events")
{
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
    modem.stream.flush();
    if (modem.waitResponse(GF("OK")) != 1) { return 0; }

	return 1;
}

/**
 * @brief UART passthrough
 */
void pass_command()
{
	SerialAT.println("ate");
	while (true)
	{
		if (SerialAT.available())
			Serial.write(SerialAT.read());
		if (Serial.available())
			SerialAT.write(Serial.read());
		//delay(1);
	}
}
////////////////////////////////////////////////////////////////////////////////

void setup() {
#ifdef SERIAL_BLUETOOTH
	SerialMon.begin(MODULE_NAME);
#else
	SerialMon.begin(115200);
#endif
	SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX);
	Wire.begin(21, 22); // I2C on Lilygo SIM7000

	SerialMon.println("Checking temperature sensor");
	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
	}

	start = micros();

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	SerialMon.println("Initializing modem...");

	modemPowerOn();

	if (!modem.init()) {
		modemRestart();
		delay(2000);
	}

	modem.setNetworkMode(38);
	modem.setPreferredMode(1);
	modem.sendAT(F("+CBANDCFG=\"CAT-M\","), 20);

	while(!modem.isNetworkConnected()) {
		Serial.print(".");
	}
	Serial.println();

	SerialMon.print("Connecting to " + String(apn) + " ");
	while (!modem.gprsConnect(apn)) {
		SerialMon.println(" fail");
		SerialMon.print("Retrying");
		modem.gprsDisconnect();
	}
	SerialMon.println(" success");

	digitalWrite(LED_PIN, HIGH);

	end = micros();
	delta = end - start;
	double connect_time = (double)delta / uS_TO_S_FACTOR;

	Serial.println("");
	Serial.print("Time to connect: ");
	Serial.print(connect_time);
	Serial.println(" seconds");

	while (update_certs)
	{
		if (modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
			SerialMon.println("Upload Certificate failed");
			continue;
		}
		if (modem_upload_cert(client_cert_mosquitto, "client.pem", 3) < 1) {
			SerialMon.println("Upload Certificate failed");
			continue;
		}
		if (modem_upload_cert(client_key_mosquitto, "client.key", 3) < 1) {
			SerialMon.println("Upload Certificate failed");
			continue;
		}
		update_certs = 0;
	}
	setupCloudIoT();

	while(!connectCloudIoT()) {
		SerialMon.println("Cannot connect to google, retrying...");
		pass_command();
	}

	battpercent = modem.getBattPercent();
	pressure = bme.readPressure();
	epoch = getEpoch();
	if (sht30.get() == 0) {
		temp = sht30.cTemp;
		hum = sht30.humidity;
	}

	sendMqtt(String("{\"ts\":\"") +
			String(epoch).c_str() +
			"\",bat\":\"" +
			String(battpercent).c_str() +
			"\",\"temp\":\"" +
			String(temp).c_str() +
			"\",\"hum\":\"" +
			String(hum).c_str() +
			"\"}");

	//modem.gprsDisconnect();
	modem.poweroff();

	//modem.enableGPS();

	//int gps_checks = 0;
	//while (gps_checks < 30) {
	//    if (modem.getGPS(&lat, &lon)) {
	//        Serial.println("The location has been locked, the latitude and longitude are:");
	//        Serial.print("latitude:"); Serial.println(lat);
	//        Serial.print("longitude:"); Serial.println(lon);
	//		gps_checks = 0;
	//        break;
	//    }
	//	gps_checks++;
	//	delay(1000);
	//}

	//disableGPS();

	ESP.restart();

	//	pass_command();

	//mqtt->loop();T
	//delay(10);  // <- fixes some issues with WiFi stability

	//if (!mqttClient->connected()) {
	//	connect();
	//}

	//publishTelemetry(getDefaultSensor());
}


void loop() {
}

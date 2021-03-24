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

#define SerialMon Serial
HardwareSerial serialGsm(1);
#define SerialAT serialGsm

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS
//#define TINY_GSM_DEBUG SerialMon

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
 * @brief
 * @param cert The String containing the certificate to upload
 * @param name The name for the uploaded file
 * @param folder 0 "/custapp/" 1 "/fota/" 2 "/datatx/" 3 "/customer/"
 */
int modem_upload_cert(const char *cert, const char *name, int folder = 0)
{
	int len = strlen(cert);

	SerialMon.println(String("Upload certificate: ") + name);

	// Init buffer
	modem.sendAT(F("+CFSINIT"));
	if (modem.waitResponse(10000L) != 1) { return -1; }

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
	if (modem.waitResponse(10000L, GF("DOWNLOAD")) != 1) { return -1; }

	for (int i = 0; i < len; i++)
		SerialAT.print(cert[i]);
	if (modem.waitResponse(30000L) != 1) { return -1; }

	// Free buffer
	modem.sendAT(F("+CFSTERM"));
	if (modem.waitResponse(30000L) != 1) { return -1; }
	return 1;
}

/**
 * @brief Switch modem on with hardware pins
 */
void modem_on()
{
	// Set-up modem  power pin
	pinMode(MODEM_PWKEY, OUTPUT);
	digitalWrite(MODEM_PWKEY, HIGH);
	delay(10);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1010); //Ton 1sec
	digitalWrite(MODEM_PWKEY, HIGH);

	//wait_till_ready();
	Serial.println("Waiting till modem ready...");
	delay(4510); //Ton uart 4.5sec but seems to need ~7sec after hard (button) reset
	//On soft-reset serial replies immediately.
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
			while(1);
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

	SerialMon.begin(115200);
	delay(10);
	SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX); //reversing them
	delay(6000);

	SerialMon.println("Initializing modem...");
	modem_on();
	//modem.restart();
	modem.init();

	SerialMon.print(F("Connecting to "));
	SerialMon.print(apn);
	while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
		SerialMon.println(" fail");
		delay(1000);
		SerialMon.print("Retrying");
		modem.restart();
	}
	SerialMon.println(" success");

	if (modem.isGprsConnected()) {
		SerialMon.println("GPRS connected");
	}

	Serial.println("configuring LTE-M mode");
	modem.setNetworkMode(38);
	modem.setPreferredMode(1);
	modem.sendAT(F("+CBANDCFG=\"CAT-M\","), 20);
	if (modem.waitResponse(30000L) != 1) { SerialMon.println("ERROR"); pass_command();}

	if (modem_upload_cert(root_cert, "ca.pem", 3) < 1) {
		SerialMon.println("Upload Certificate failed");
		while (1);
	}
	if (modem_upload_cert(client_cert_mosquitto, "client.pem", 3) < 1) {
		SerialMon.println("Upload Certificate failed");
		while (1);
	}

	if (modem_upload_cert(client_key_mosquitto, "client.key", 3) < 1) {
		SerialMon.println("Upload Certificate failed");
		while (1);
	}
	setupCloudIoT();
	if (!connectCloudIoT()) {
		SerialMon.println("Cannot connect to google");
		while(1);
	}

	//sendMqtt("Bonjour Roger", "events");
	sendMqtt(modem.getGSMDateTime(DATE_FULL).c_str(), "events");
	pass_command();

	//mqtt->loop();T
	//delay(10);  // <- fixes some issues with WiFi stability

	//if (!mqttClient->connected()) {
	//	connect();
	//}

	//publishTelemetry(getDefaultSensor());
}


void loop() {
}

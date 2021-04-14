#include "Tracker.h"

Tracker::Tracker(/* args */)
{
}

Tracker::~Tracker()
{
}

/**
 * @brief Enable SIM7000's gps
 */
void Tracker::enableGPS(void)
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
void Tracker::disableGPS(void)
{
	// Set SIM7000G GPIO4 LOW ,turn off GPS power
	// CMD:AT+SGPIO=0,4,1,0
	// Only in version 20200415 is there a function to control GPS power
	modem.sendAT("+SGPIO=0,4,1,0");
	modem.disableGPS();
}

/**
 * @brief Switch modem on with hardware pins
 */
void Tracker::modemPowerOn() {
	pinMode(MODEM_PWKEY, OUTPUT);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1000);    //Datasheet Ton mintues = 1S
	digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Switch modem off with hardware pins
 */
void Tracker::modemPowerOff() {
	pinMode(MODEM_PWKEY, OUTPUT);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1500);    //Datasheet Ton mintues = 1.2S
	digitalWrite(MODEM_PWKEY, HIGH);
}

/**
 * @brief Restarts modem with hardware pins
 */
void Tracker::modemRestart() {
	modemPowerOff();
	delay(1000);
	modemPowerOn();
}

/**
 * @brief Turn modem off
 */
void Tracker::modemOff() {
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
boolean Tracker::set_eDRX(uint8_t mode, uint8_t connType, const char * eDRX_val) {
	if (strlen(eDRX_val) > 4) return false;

	char auxStr[21];

	sprintf(auxStr, "AT+CEDRXS=%i,%i,\"%s\"", mode, connType, eDRX_val);

	modem.stream.println(auxStr);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

/**
 * @brief Enable Power Saving Mode
 * NOTE: Network must support PSM and modem needs to restart before it takes effect
 */
boolean Tracker::enablePSM(bool onoff) {
	modem.stream.println(String("AT+CSCLK=1"));
	modem.stream.println(String("AT+CPSMS=") + onoff);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

/**
 * @brief Set PSM with custom TAU and active time
 * For both TAU and Active time, leftmost 3 bits represent the multiplier and rightmost 5 bits represent the value in bits.
 *
 * For TAU, left 3 bits:
 * 000 10min
 * 001 1hr
 * 010 10hr
 * 011 2s
 * 100 30s
 * 101 1min
 *
 * For Active time, left 3 bits:
 * 000 2s
 * 001 1min
 * 010 6min
 * 111 disabled
 *
 * Note: Network decides the final value of the TAU and active time. 
 */
boolean Tracker::enablePSM(bool onoff, const char * TAU_val, const char * activeTime_val) {
	if (strlen(activeTime_val) > 8) return false;
	if (strlen(TAU_val) > 8) return false;

	modem.stream.println(String("AT+CSCLK=1"));

	char auxStr[35];
	sprintf(auxStr, "AT+CPSMS=%i,,,\"%s\",\"%s\"",
			onoff, TAU_val, activeTime_val);

	modem.stream.println(auxStr);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

/**
 * @brief Low power mode - while connected
 * will have an effect after reboot and will replace normal power down
 */
void Tracker::modemSleep(void) {
	Serial.println("Going to sleep now with modem in power save mode");
	// needs reboot to activa and takes ~20sec to sleep
	Serial.print("Enabling PSM");
	int retry = config.psmEnableAttempts + 1;
	while(!enablePSM(1, "01100010", "00000001") && --retry) {
		Serial.println(" failed");
		Serial.println("Retrying");
		delay(100);
	}
	if (retry) {
		SerialMon.println(" success");
	} else {
		SerialMon.println(" failed");
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
void Tracker::modemWake(void) {
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
byte Tracker::NTPServerSync(String server, byte timezone) {
	// Set GPRS bearer profile to associate with NTP sync
	modem.sendAT(GF("+CNTPCID=1"));
	if (modem.waitResponse(10000L) != 1) { return -1; }

	// Set NTP server and timezone
	modem.sendAT(GF("+CNTP="), server, ',', String(timezone));
	if (modem.waitResponse(GF("OK")) != 1) { return -1; }
	return 1;
}

/**
 * @brief Setup the internal clock with modem and NTP
 */
void Tracker::setupTime(){
	struct tm tm;
	float timezone;

	SerialMon.print("Synchronize with ntp server");

	if (NTPServerSync("pool.ntp.org", 2) < 0) {
		SerialMon.println(" failed");
		ntp_connected = false;
	} else {
		if (!modem.getNetworkTime(&tm.tm_year, &tm.tm_mon, &tm.tm_mday,
					&tm.tm_hour, &tm.tm_min, &tm.tm_sec, &timezone)) {
			SerialMon.println(" failed");
			ntp_connected = false;
		} else {
			if (tm.tm_year < 120) {
				SerialMon.println(" failed");
				ntp_connected = false;
			} else {
				SerialMon.println(" success");
				ntp_connected = true;
			}
		}
	}
	if (ntp_connected) {
		tm.tm_year -= 1900;
		tm.tm_mon -= 1;
		tm.tm_hour -= 2;

		time_t t = mktime(&tm);

		struct timeval tv = { .tv_sec = t };

		settimeofday(&tv, NULL);
	}
}

/**
 * @brief Handle the SSL settings and connect to google iot core
 */
bool Tracker::connectCloudIoT() { // TODO Add timeout
	setupTime();
	jwt = getJwt();

	String client_id = device->getClientId();
	String url = CLOUD_IOT_CORE_MQTT_HOST_LTS;
	String port = "8883";
	String keep_time = "60";
	bool cleanSession = 0;

	SerialMon.print(String("Connecting to ") + url + ":" + port + " ");

	modem.sendAT(String("+CSSLCFG=\"sslversion\",0,3"));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+CSSLCFG=\"convert\",2,\"ca.pem\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+CSSLCFG=\"convert\",1,\"client.pem\",\"client.key\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMSSL=1,\"ca.pem\",\"client.pem\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(String("+SMCONF=\"URL\",\"") +
			url +
			GF("\",\"") +
			port +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONF=\"KEEPTIME\",\"") +
			keep_time +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONF=\"CLIENTID\",\"") +
			client_id +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(String("+SMCONF=\"USERNAME\",\"") +
			"unused" +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONF=\"PASSWORD\",\"") +
			jwt +
			GF("\""));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(String("+SMCONF=\"CLEANSS\",") +
			cleanSession
			);
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONF=\"QOS\",1"));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(String("+SMCONF=\"RETAIN\",1"));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONF?"));
	if (modem.waitResponse(30000L) != 1) {return 0;}

	modem.sendAT(GF("+SMCONN"));
	if (modem.waitResponse(5000L) != 1) {return 0;}

	SerialMon.println("OK");
	return 1;
}


/**
 * @brief Create JWT token
 */
String Tracker::getJwt(void) {
	iat = time(nullptr);
	Serial.println("Refreshing JWT");
	jwt = device->createJWT(iat, jwt_exp_secs);
	//Serial.println(jwt);

	return jwt;
}

/**
 * @brief UART passthrough
 */
void Tracker::pass_command(void) {
	SerialAT.println("ate");
	while (true) {
		if (SerialAT.available())
			Serial.write(SerialAT.read());
		if (Serial.available())
			SerialAT.write(Serial.read());
		//delay(1);
	}
}

/**
 * @brief Read ADC of battery pin
 * @param voltage Pointer to where to write the value 
 */
void Tracker::read_adc_bat(uint16_t *voltage) {
	uint32_t in = 0;
	for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++) {
		in += (uint32_t)analogRead(PIN_ADC_BAT);
	}
	in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

	uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

	*voltage = bat_mv;
}

/**
 * @brief Read ADC of solar pin
 * @param voltage Pointer to where to write the value 
 */
void Tracker::read_adc_solar(uint16_t *voltage) {
	uint32_t in = 0;
	for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++) {
		in += (uint32_t)analogRead(PIN_ADC_SOLAR);
	}
	in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

	uint16_t bat_mv = ((float)in / 4096) * 3600 * 2;

	*voltage = bat_mv;
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
int Tracker::modem_upload_cert(const char *cert, const char *name, int folder)
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
	if (modem.waitResponse(timeout * 1L, GF("DOWNLOAD")) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -2;
	}

	for (int i = 0; i < len; i++)
		SerialAT.print(cert[i]);

	if (modem.waitResponse(timeout * 1L, GF("OK")) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -3;
	}

	// Free buffer
	modem.sendAT(F("+CFSTERM"));
	if (modem.waitResponse(timeout * 1L, GF("OK")) != 1) {
		modem.sendAT(F("+CFSTERM"));
		modem.waitResponse();
		return -4;
	}

	return 1;
}

//// TODO Implemente this
///**
// * @brief Config MQTT connection
// */
//bool Tracker::mqttConfig(void) {
//	return false;
//}
//
///**
// * @brief Connect to MQTT broker
// */
//bool Tracker::mqttConnect(void) {
//	return false;
//}
/**
// * @brief Unsubscribe from topic
// */
//bool Tracker::mqttUnSub(void) {
//	return false;
//}

/**
 * @brief Subscript MQTT topic
 * @param topic The MQTT topic
 * @param qos 0, 1, 2
 */
bool Tracker::mqttSub(const char *topic, int qos) {
	SerialMon.print("Subscribing topic: ");
	SerialMon.println(topic);

	int len = 21 + strlen(device_id) + strlen(topic) + 1;
	char auxStr[len];
	sprintf(auxStr, "+SMSUB=\"/devices/%s/%s\",%i", device_id, topic, qos);
	modem.sendAT(auxStr);
	if (modem.waitResponse(2000L, GF("OK")) != 1) { return 0; }
	return 1;
}

/**
 * @brief Receive MQTT data
 * @param data Pointer where to right data
 */
bool Tracker::mqttReceive(const char *topic, String *data, int timeout) {
	int len = 21 + strlen(device_id) + strlen(topic) + 1;
	char auxStr[len];
	sprintf(auxStr, "+SMSUB: \"/devices/%s/%s\",", device_id, topic);
	if (modem.waitResponse(timeout * 1000L, auxStr) != 1) { return false; }
	*data = modem.stream.readStringUntil('\n');
	return true;
}

/**
 * @brief Stop MQTT connection
 */
bool Tracker::mqttDisconnect(void) {
	modem.sendAT(GF("+SMDISC"));
	modem.waitResponse();
	return true;
}

/**
 * @brief Send MQTT message
 * @param msg The payload to send
 * @param topic The MQTT topic
 */
bool Tracker::sendMqtt(
		const char *msg, const char *topic, int qos, bool retain) {
	SerialMon.print("Sending message: ");
	SerialMon.println(msg);

	int len = strlen(msg);
	if (len > 512) {
		SerialMon.println("Message too long");
		return false;
	}
	int slen = 24 + strlen(device_id) + strlen(topic) + 1;
	char auxStr[slen];
	sprintf(auxStr, "+SMPUB=\"/devices/%s/%s\",%i,%i,%i",
			device_id, topic, len, qos, retain);
	modem.sendAT(auxStr);
	if (modem.waitResponse(GF(">")) != 1) { return false; }
	modem.stream.write(msg, len);
	if (modem.waitResponse(10000L, GF("OK")) != 1) { return false; }
	modem.stream.flush();
	return true;
}

/**
 * @brief Check if power is high enough and set the values accordingly
 * @param alert Voltage when we might want to start lower the power consuption
 * @param min Voltage when the module should sleep a very long time
 */
void Tracker::checkPower(void) {
	pinMode(PIN_ADC_BAT, INPUT);
	read_adc_bat(&batVoltage);
	if (batVoltage <= config.lowBatThreshold && batVoltage > 0)
	{
		lowBattery = true;
		if (batVoltage <= config.veryLowBatThreshold)
			veryLowBattery = true;
	} else {
		lowBattery = false;
		veryLowBattery = false;
	}
}


/**
 * @brief Set SIM7000 network lights off
 * @param onoff 
 */
bool Tracker::modemLight(bool onoff) {
	modem.sendAT(F("+CSGS="), onoff);
	return (modem.waitResponse(GF("OK")) != 1 ? 0 : 1 );
}

/**
 * @brief Get tracker's data
 * @param tracker Pointer to data
 */
bool Tracker::getData(void) {
	bool ret = true;
	timestamp = time(nullptr);
	if (timestamp < 1617808583) {
		SerialMon.println("Timestamp error");
		ret = false;
	}
	bootCount = boot_count;
	savedStates = saved_states;
	//read_adc_bat(&batVoltage);
	batCharging = batVoltage == 0 ? true : false;
	read_adc_solar(&solVoltage);
	if (solVoltage > 4000)
		batCharging = true;
	if (sht30.get() == 0) {
		temp = sht30.cTemp;
		hum = sht30.humidity;
	} else {
		SerialMon.println("Env error");
		ret = false;
	}
	pressure = bmp.readPressure();
	//if (!gps_enabled) {
	enableGPS();
	gps_enabled = true;
	//}
	gpsEnabled = gps_enabled;
	if (modem.getGPS(&lat, &lon, &speed,
				&alt, &vsat, &usat, &accuracy)) {
		gpsLocked = true;
	} else {
		gpsLocked = false;
		lat = 0;
		lon = 0;
		alt = 0;
		speed = 0;
		accuracy = 0;
		SerialMon.println("GPS error");
		ret = false;
	}
	return ret;
}

/**
 * @brief Save actual config to littlefs
 * @param json Pointer to json document
 * @param filepath File's path on the filesystem
 */
bool Tracker::saveFile(DynamicJsonDocument *json, const char *filepath) {
	File file = LITTLEFS.open(filepath, "w");
	if (!file) {
		Serial.println("Failed to open file for writing");
		return false;
	}
	if (serializeJson(json[0], file) == 0) {
		Serial.println(F("Failed to write to file"));
	}
	file.close();
	return true;
}

/**
 * @brief Load config from filesystem
 * @param json Pointer to json document
 * @param filepath File's path on the filesystem
 */
bool Tracker::loadFile(DynamicJsonDocument *json, const char *filepath, uint32_t maxsize) {
	File file = LITTLEFS.open(filepath, "r");
	if (!file) {
		Serial.println("Failed to open file");
		return false;
	}
	auto error = deserializeJson(json[0], file);
	if (error) {
		Serial.println("Failed to parse config file");
		return false;
	}
	return true;
}

/**
 * @brief Set configuration variables from json document
 * @param json Pointer to json document
 */
void Tracker::setConfig(DynamicJsonDocument *json) {
	if (json[0].containsKey("refreshTime"))
		config.refreshTime = json[0]["refreshTime"];
	if (json[0].containsKey("lowBatRefreshTime"))
		config.lowBatRefreshTime = json[0]["lowBatRefreshTime"];
	if (json[0].containsKey("lowBatThreshold"))
		config.lowBatThreshold = json[0]["lowBatThreshold"];
	if (json[0].containsKey("veryLowBatThreshold"))
		config.veryLowBatThreshold = json[0]["veryLowBatThreshold"];
	if (json[0].containsKey("modemConnectAttempts"))
		config.modemConnectAttempts = json[0]["modemConnectAttempts"];
	if (json[0].containsKey("networkConnectAttempts"))
		config.networkConnectAttempts = json[0]["networkConnectAttempts"];
	if (json[0].containsKey("gprsConnectAttempts"))
		config.gprsConnectAttempts = json[0]["gprsConnectAttempts"];
	if (json[0].containsKey("psmEnableAttempts"))
		config.psmEnableAttempts = json[0]["psmEnableAttempts"];
	if (json[0].containsKey("cloudConnectAttempts"))
		config.cloudConnectAttempts = json[0]["cloudConnectAttempts"];
}

/**
 * @brief Make a Json with data
 * @param state Pointer to json document
 */
void Tracker::setState(DynamicJsonDocument *state) {
	state[0]["ts"] = timestamp;
	state[0]["bootCount"] = bootCount;
	state[0]["batVoltage"] = batVoltage;
	state[0]["solVoltage"] = solVoltage;
	state[0]["batCharging"] = batCharging;
	state[0]["gprsConnectTime"] = gprsConnectTime;
	state[0]["temperature"] = temp;
	state[0]["humidity"] = hum;
	state[0]["pressure"] = pressure;
	state[0]["lat"] = lat;
	state[0]["lon"] = lon;
	state[0]["alt"] = alt;
	state[0]["speed"] = speed;
	state[0]["accuracy"] = accuracy;
	state[0]["gpsLocked"] = gpsLocked;
	state[0]["gpsEnabled"] = gpsEnabled;
	state[0]["lowBattery"] = lowBattery;
	state[0]["veryLowBattery"] = veryLowBattery;
}

/**
 * @brief Send Tracker's data
 * @param state Pointer to json document
 */
bool	Tracker::sendState(DynamicJsonDocument *state) {
	String s;
	serializeJson(*state, s);
	if (!sendMqtt(s.c_str(),"events")) {
		SerialMon.println("Cannot send MQTT");
		return false;
	}
	//if (!sendMqtt(s.c_str(),"state")) {
	//	SerialMon.println("Cannot send MQTT");
	//	return false;
	//}
	return true;
}

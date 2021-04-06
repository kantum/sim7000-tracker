#ifndef _Tracker_h
#define _Tracker_h

#include <stdint.h>
#include <time.h>

class Tracker
{
	public:
		Tracker(/* args */);
		~Tracker();

		int bootCount;
		unsigned long connectTime;
		time_t timestamp;
		bool batCharging;
		uint16_t batVoltage;
		uint16_t solVoltage;
		int pressure;
		int temp;
		int hum;
		bool gpsEnabled;
		bool gpsLocked;
		float lat;
		float lon;
		float alt;
		float speed;
		float accuracy;
		bool lowBattery;
		bool veryLowBattery;
	private:
};

#endif

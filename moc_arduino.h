#ifndef MOC_ARDUINO_H
#define MOC_ARDUINO_H

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <cassert> 

#include <string>

#ifdef WIN32
#include <windows.h> 
#else
#include <unistd.h>
#endif

#define WIFI_STA 0
#define LED_BUILTIN 0
#define OUTPUT 0
#define LOW 0
#define HIGH 1

#define DEBUG(t) printf(t); printf("\n");



void pinMode(int i, int out) {};

void digitalWrite(int i, int v) {
	printf("Digital write : %i, %i \n", i, v);
}
void delay(int msec) {

#ifdef WIN32
	Sleep(msec);
#else
	usleep(static_cast<useconds_t>(msec)*1000); //or use nanosleep on platforms where it's needed
#endif

}

time_t millis() {
	time_t timer;
	time(&timer);
	return timer;

}

int random(int low, int high) {
	return rand() % (high-low) + low;
}

struct String : public std::string {
	
	String(const char * s) : std::string(s) 
	{}
	
	bool equals(const String & s) const {
		return *this == s;
	}
};


const int N_WIFI = 4;
const String BSSIDs[N_WIFI] = {
  "CC:61:E5:CC:14:D4", 
  "BC:EE:B8:FA:38:46",
  "7B:8F:7E:1A:FE:34",
  "F4:CA:E5:BF:E4:68" 
};

const String SSIDs[N_WIFI] = {
	"B0",
	"B1",
	"B2",
	"B3"
};

const float RSSIs[N_WIFI] = {
	-150.0, 
	-141.0,
	-140.0,
	-148.0
};


struct _Wifi {
	
	void mode(int mode) {};
	void disconnect() {};
	int scanNetworks() {return N_WIFI; }
	
	String SSID(int i) {
		assert(i < N_WIFI && i >=0  && "Index should be less than N_WIFI");
		return SSIDs[i];
	}
	
	int channel(int i) {
		return 0;
	}
	
	int encryptionType(int i) {
		return 0;
	}
	
	float RSSI(int i) {
		assert(i < N_WIFI && i >=0 && "Index should be less than N_WIFI");
		return RSSIs[i];
	}	
	
	String BSSIDstr(int i) {
		assert(i < N_WIFI && i >=0 && "Index should be less than N_WIFI");
		return BSSIDs[i];		
	}
};

struct _Serial {
	void begin(int baud) {};
	
	void print(char* c) {
		printf("%s", c);
	}
	void print(int c) {
		printf("%i", c);
	}
	void print(float c) {
		printf("%f", c);
	}
	void print(String s) {
		printf("%s", s.c_str());
	}
	template<typename T>
	void println(T c) {
		print(c);
		printf("\n");
	}
};

_Wifi WiFi = _Wifi();
_Serial Serial = _Serial();

#endif // MOC_ARDUINO_H

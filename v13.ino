// wawawa temp control
// version 13
#define VERSION "13" 

#include "DodgeBrewingUtils.hpp"
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire one_wire(ONE_WIRE_BUS);
DallasTemperature temp_sensors(&one_wire);
DeviceAddress temp_sensor_addresses{};

DodgeBrewing::TFTDisplay tft_display;

void setup () {
	Serial.begin(9600);
	Serial.println("BEGIN NEW RUN");

	pinMode(RELAYPIN, OUTPUT);
	digitalWrite(RELAYPIN, LOW);
	pinMode(ONE_WIRE_PWR, OUTPUT);
	digitalWrite(ONE_WIRE_PWR, HIGH);

	temp_sensors.begin();
	temp_sensors.setResolution(temp_sensor_addresses, 12);
	temp_sensors.setWaitForConversion(false);

	tft_display.init();
}

void loop () {
	Serial.println("loop() executing");

	tft_display.update();
}
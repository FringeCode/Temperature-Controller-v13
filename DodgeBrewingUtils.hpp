// Dodge Brewing Systems utilities
#pragma once

// Definitions lole
#define MINPRESSURE 10
#define MAXPRESSURE 100

#define RELAYPIN 49
#define ONE_WIRE_BUS 35
#define ONE_WIRE_PWR 31

#define XP 8
#define XM 56 // = A2 = 0x38
#define YP 57 // = A3 = 0x39
#define YM 9
#define TS_LEFT 116
#define TS_RIGHT 905
#define TS_TOP 936
#define TS_BOTTOM 93

#define TFT_BLUE 0x001F


#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>


namespace DodgeBrewing {

	class TFTDisplay {
	public:
		TFTDisplay (uint16_t disp_w = 320, uint16_t disp_h = 480, uint8_t r = 0, uint16_t x_resistance = 300)
				   : touch_screen(XP, YP, XM, YM, x_resistance),
				     width(disp_w), height(disp_h) {

			display.setRotation(r);

		}

		void init () {
			display.begin(display.readID());
			display.fillScreen(TFT_BLUE);
		}

		void update () {
			display.setCursor(300, 300);
			display.setTextSize(3);
			display.setTextColor(0x0000, 0x5555);
			display.print("hi :3");
		}

	private:
		uint32_t width, height;
		MCUFRIEND_kbv display;
		TouchScreen touch_screen;
	};

	enum class TempControlState {
		OFF, RAMPUP, RAMPOFF, STRIKEPID, MASHPID
	};

}



// SCREEN LAYOUTS
//   Home - static text
//     "DODGE BREWING SYSTEMS        v13" (100, 10) @1.75
//     Horizontal Line (0, 30) w320
//     "HOME" (130, 40) @2
//     "Mash Tun State:" (10, 70) @2
//     "Mash Tun Step:" (10, 103) @2
//     "MT Set Point:" (10, 133) @2
//     "MT Actual Temp:" (10, 163) @2
//     "MT Strike Temp:" (10, 193) @2
//     "Mash Timer:" (10, 223) @2
//
//   Settings
//  
// screendraw.h
// Dodge Temperature Controller / PID Libary
// Screen Display Functions here

#ifndef SCREEN_DRAW_H
#define SCREEN_DRAW_H

#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

#define MINPRESSURE 10
#define MAXPRESSURE 1000
#define RelayPin 49
#define ONE_WIRE_BUS 37
#define ONE_WIRE_PWR 39
#define BuzzerPin 43
#define MTLEDPin 33
#define BKLEDPin 29
#define BKFirstAddLEDPin 27
#define BKSecondAddLEDPin 25
#define BKThirdAddLEDPin 23

inline int pixel_x, pixel_y;
const int TS_LEFT = 116, TS_RT = 905, TS_TOP = 936, TS_BOT = 93;
inline const int XP = 8, XM = A2, YP = A3, YM = 9; //320x480 ID=0x9486

bool Touch_getXY(TouchScreen& ts, MCUFRIEND_kbv& tft) {

  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);

  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (pressed) {
    // most apps use Portrait or Landscape. No need for all 4 cases
    switch (tft.getRotation()) {
      case 0:
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
        break;
      case 1:
        pixel_x = map(p.y, TS_TOP, TS_BOT, 0, tft.width());
        pixel_y = map(p.x, TS_RT, TS_LEFT, 0, tft.height());
        break;
      case 2:
        pixel_x = map(p.x, TS_RT, TS_LEFT, 0, tft.width());
        pixel_y = map(p.y, TS_BOT, TS_TOP, 0, tft.height());
        break;
      case 3:
        pixel_x = map(p.y, TS_BOT, TS_TOP, 0, tft.width());
        pixel_y = map(p.y, TS_LEFT, TS_RT, 0, tft.height());
        break;
    }
  }

  return pressed;
}

// GetClockTime
// Return a string in the format "HH:MM:SS" for timer displays
String GetClockTime (unsigned long timer_ms) {

  // Hours
  // NO LEADING 0
  uint8_t h = timer_ms  / 3600000;
  String hour_string(h);

  // Minutes
  String minute_string = "";
  if (h > 0)
    timer_ms -= h * 3600000;
  uint8_t m = timer_ms / 60000;
  if (m < 10)
    minute_string += String(0);
  minute_string += String(m);

  // Seconds
  String second_string = "";
  if (m > 0)
    timer_ms -= m * 60000;
  uint8_t s = timer_ms / 1000;
  if (s < 10)
    second_string += String(0);
  second_string += String(s);

  String colon = ":";

  return hour_string + colon + minute_string + colon + second_string;

}

static inline String Version = "V12X";
static inline bool DrawHomeScreenStaticText_flag;
static inline bool DrawSettingsScreenStaticText_flag;

enum class stateList { OFF = 0, RAMPUP, RAMPOFF, STRIKEPID, MASHPID };

extern stateList globalState = stateList::OFF;
extern double MTStrikeTemp;

// SEPTEMBER 19 2022
// timer class encapsulating its own state?
template<typename T>
struct LoopingTimerStateList {
  // ?
};

// using typename lint = long unsigned int;
typedef long unsigned int lint;

struct Timer {
  // tTimer ( lint l , lint s , int i = 0 ) : tLengthS(l) , tStartMS(s), tStateNow(i) {}
  lint tLengthS; // seconds? ms?
  lint tStartMS , tCurrentMS; //MS: milliseconds , CU: COUNTING UP
  lint tPauseStart; //-1 means UNPAUSED, RUNNING

  lint tCurrentCU () { return millis() - tPauseLength() - tStartMS; }
  // int tState {beginningPaused = 0, beginningRunning = 1, middleRunning = 2, middlePaused = 3, endRunning = 4, endPaused = 5};
  // six states
  int tStates[6] = { 0 , 1 , 2 , 3 , 4 , 5 };
  int tStateNow = 0;

  // UPDATE SEPT 20 2022
  bool doneNotReset = false;

  bool reset_timer () {
    tStateNow = 0;
    doneNotReset = false;
  }

  bool add_time ( lint t ) {
    tLengthS += t;
  }
  
  lint tPauseLength () {
    if ( tStateNow == 3 )
      return millis() - tPauseStart;
    else
      return 0;
  }

  lint pause () {
    // tPauseStart = millis();
    if ( tStateNow == 2 || tStateNow == 1 )
      tStateNow = 3;
    if ( tStateNow == 5 )
      tStateNow = 0;
    return tPauseStart = millis(); // assignments
  }

  lint get_time_count_down () {
    switch(tStateNow) {
      case 0:
        doneNotReset = false;
        return tLengthS;
        break;
      case 1:
        tStateNow = 2; // middleRunning
        doneNotReset = false;
        return tLengthS - tCurrentCU(); //? is state 1 needed? beginningRunning
        break;
      case 2:
        if ( tLengthS <= tCurrentCU() ) {
          tStateNow = 5;
          return 0;
        }
        return tLengthS - tCurrentCU();
        break;
      case 3:
        return tLengthS - tCurrentCU();
        break;
      case 4:
        tStateNow = 5;
        doneNotReset = true;
        return tCurrentCU();
        break;
      case 5:
        doneNotReset = true;
        return 0;
        break;
      
    }
    return tCurrentCU();
  }

  lint start_from_beginning () {
    // tCurrentCU = 0;
    doneNotReset = false;
    
    tStateNow = 1;
    tStartMS = millis();
    tCurrentMS = millis();
    tPauseStart = -1;
  }
};

// UPDATE 4/14/22
struct StateSystem {

    stateList getState () const { // does it matter that the first const is there?
      return opState;             // 3/6/22 removed initial const in case that affects mutability of returned stateList
    }
    void doState();
    
    // SEPT 19 2022
    Timer MashTimerObj;
    Timer BoilTimerObj;
    Timer AddTimerObj1, AddTimerObj2, AddTimerObj3;
    
    stateList checkState();
  private:
    // stateList opState = stateList::OFF; // initial value
    // 3/13/22 Setting opState to be a reference to globalState - hopefully this doesn't break anything
    stateList& opState = globalState;

    void Off();
    void RampUp();
    void RampOff();
    void StrikePID();
    void MashPID();

    const int MASHTimer = 10000; // 1 minute in milliseconds
    int MASHStartTime = -1; // Set to current time when beginning MASHPID, set to -1 when out of state
    double MaxRampOffTemp = 0; // JULY 17 2022 keep track of max temp and dips during Ramp Off
};



/*********************
  UPDATE 3/28
  Took color #define statements from adafruit pdf page 6 of 26
**********************/
#define TFT_BLACK 0x0000
#define TFT_LIGHTGREY 0x5555
#define TFT_BLUE 0x001F
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_DARKGREEN 0x0320
#define TFT_CYAN 0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW 0xFFE0
#define TFT_WHITE 0xFFFF
#define TFT_ORANGE 0xFD20
#define TFT_GREENYELLOW 0xAFE5

// V12i 4/5/22 SettingVariable Indices
#define MTSTKpIndex 0
#define MTSTKiIndex 1
#define MTSTKdIndex 2

#define MTKpIndex 3
#define MTKiIndex 4
#define MTKdIndex 5

#define MTSpIndex 6 //MT Temperature Set Point;
#define MTWaterToGrainRatioIndex 7
#define MTGrainTempIndex 8
#define MTInitialWaterTempIndex 9
#define MTRampOffTempIndex 10
#define MTPIDStartTempIndex 11

#define MashTimeIndex	12
#define BoilTimeIndex	13
#define FirstAddTimeIndex	14
#define SecondAddTimeIndex	15
#define ThirdAddTimeIndex	16

void DrawSettingsScreenStaticText (MCUFRIEND_kbv& tft) {

  tft.setRotation(0);
  tft.setCursor(100, 10);
  tft.setTextSize(1.75);
  tft.print("DODGE BREWING SYSTEMS        ");
  tft.print(Version);
	
  tft.drawFastHLine(0, 30, 320, TFT_BLACK);
  tft.setCursor(90, 40);
  tft.setTextSize(2);
  tft.print("PID SETTINGS");
  tft.drawFastHLine(0, 60, 320, TFT_BLACK);
  
  tft.setCursor(15, 70);
  tft.setTextSize(1);
  tft.print("STRIKE TEMP");

  tft.setCursor(90, 70);
  tft.setTextSize(1);
  tft.print("P:");

  tft.setCursor(180, 70);
  tft.setTextSize(1);
  tft.print("I:");

  tft.setCursor(250, 70);
  tft.setTextSize(1);
  tft.print("D:");

  tft.setCursor(15, 90);
  tft.setTextSize(1);
  tft.print("MASH TEMP");

  tft.setCursor(90, 90);
  tft.setTextSize(1);
  tft.print("P:");

  tft.setCursor(180, 90);
  tft.setTextSize(1);
  tft.print("I:");

  tft.setCursor(250, 90);
  tft.setTextSize(1);
  tft.print("D:");

  tft.drawFastHLine(0, 105, 320, TFT_BLACK);
  tft.setCursor(80, 115);
  tft.setTextSize(2);
  tft.print("MASH TUN SETTINGS");
  tft.drawFastHLine(0, 135, 320, TFT_BLACK);

  tft.setCursor(15, 142);
  tft.setTextSize(1);
  tft.print("MASH TEMP SETPOINT:");

  tft.setCursor(15, 162);
  tft.setTextSize(1);
  tft.print("WATER TO GRAIN RATIO:");

  tft.setCursor(15, 182);
  tft.setTextSize(1);
  tft.print("INITIAL GRAIN TEMP:");

  tft.setCursor(15, 202);
  tft.setTextSize(1);
  tft.print("INITIAL WATER TEMP:");

  tft.setCursor(15, 222);
  tft.print("MASH TUN RAMP OFF TEMP:");

  tft.setCursor(15, 242);
  tft.print("MASH TUN PID START TEMP:");

  tft.drawFastHLine(0, 283, 320, TFT_BLACK);
  tft.setCursor(80, 293);
  tft.setTextSize(2);
  tft.print("TIMER SETTINGS");
  tft.drawFastHLine(0, 313, 320, TFT_BLACK);

  tft.setCursor(15, 327);
  tft.setTextSize(1);
  tft.print("MASH TIME:");

  tft.setCursor(170, 327);
  tft.setTextSize(1);
  tft.print("BOIL TIME:");

  tft.setCursor(170, 347);
  tft.print("1st Add:");
  
  tft.setCursor(170, 367);
  tft.print("2nd Add:");
      
  tft.setCursor(170, 387);
  tft.print("3rd Add:");
  
  tft.drawFastHLine(0, 440, 320, TFT_BLACK);

}

void DrawHomeScreenStaticText (MCUFRIEND_kbv& tft) {

  tft.setRotation(0);
  tft.setCursor(100, 10);
  tft.setTextSize(1.75);
  tft.print("DODGE BREWING SYSTEMS           ");
  tft.print(Version);

  tft.drawFastHLine(0, 30, 320, TFT_BLACK);
  tft.setCursor(130, 40);
  tft.setTextSize(2);
  tft.print("HOME");
  tft.drawFastHLine(0, 60, 320, TFT_BLACK);

  tft.setCursor(10, 70);
  tft.setTextSize(2);
  tft.print("Mash Tun State:");

  tft.setCursor(10, 103);
  tft.setTextSize(2);
  tft.print("Mash Tun Step:");

  tft.setCursor(10, 133);
  tft.setTextSize(2);
  tft.print("MT Set Point:");

  tft.setCursor(10, 163);
  tft.setTextSize(2);
  tft.print("MT Actual Temp:");

  tft.setCursor(10, 193);
  tft.setTextSize(2);
  tft.print("MT Strike Temp:");

  tft.setCursor(10, 223);
  tft.setTextSize(2);
  tft.print("Mash Timer:");

  tft.setCursor(10, 253);
  tft.setTextSize(2);
  tft.print("STATUS:");
  tft.drawFastHLine(0, 245, 320, TFT_BLACK);
  tft.drawFastHLine(0, 275, 320, TFT_BLACK);

  tft.setCursor(10, 313);
  tft.setTextSize(2);
  tft.print("Boil Timer:");

  tft.setCursor(10, 343);
  tft.setTextSize(2);
  tft.print("1st Addition:");

  tft.setCursor(10, 373);
  tft.setTextSize(2);
  tft.print("2nd Addition:");

  tft.setCursor(10, 403);
  tft.setTextSize(2);
  tft.print("3rd Addition:");

}

// Draw the Home Screen (screen == 0)
void DrawHomeScreen ( MCUFRIEND_kbv& tft , double* SettingVariable , StateSystem& sys , double& Input , double& Output, long unsigned int& MTActTime ,
                      long unsigned int& BoilActTime , long unsigned int& FirstAddActTime , long unsigned int& SecondAddActTime , long unsigned int& ThirdAddActTime ) {
  
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  if (DrawHomeScreenStaticText_flag) {
    DrawHomeScreenStaticText(tft);
    DrawHomeScreenStaticText_flag = false;
  }

  tft.setCursor(200, 133);
  tft.print(SettingVariable[MTSpIndex], 1);

  tft.setCursor(200, 163);
  tft.print(Input, 1);

  tft.setCursor(200, 193);
  tft.print(MTStrikeTemp, 1);

  tft.setCursor(150, 223);
  MTActTime = sys.MashTimerObj.get_time_count_down();
  tft.print(GetClockTime(MTActTime));

  if ( sys.getState() == stateList::MASHPID ) {
    tft.setCursor(100, 253);
    tft.print("MASH PID STEP  ");
  } else if ( sys.getState() == stateList::STRIKEPID ) {
    tft.setCursor(100, 253);
    tft.print("STRIKE PID STEP");
  } else if ( sys.getState() == stateList::RAMPUP ) {
    tft.setCursor(100, 253);
    tft.print("RAMP UP STEP   ");
  } else if ( sys.getState() == stateList::RAMPOFF ) {
    tft.setCursor(100, 253);
    tft.print("RAMP OFF STEP");
  } else {
    tft.setCursor(100, 253);
    tft.print("OFF            ");
  }

  tft.setCursor(142, 313);
  BoilActTime = sys.BoilTimerObj.get_time_count_down();
  tft.print(GetClockTime(BoilActTime));

  FirstAddActTime = sys.AddTimerObj1.get_time_count_down();
  tft.setCursor(175, 343);
  tft.print(GetClockTime(FirstAddActTime));
  
  SecondAddActTime = sys.AddTimerObj2.get_time_count_down();
  tft.setCursor(175, 373);
  tft.print(GetClockTime(SecondAddActTime));

  ThirdAddActTime = sys.AddTimerObj3.get_time_count_down();
  tft.setCursor(175, 403);
  tft.print(GetClockTime(ThirdAddActTime));

}

// Draw the Settings Screen (screen == 1)
void DrawSettingsScreen ( MCUFRIEND_kbv& tft , double* SettingVariable , StateSystem& sys , double& Input , double& Output ) {

  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  if (DrawSettingsScreenStaticText_flag) {
    DrawSettingsScreenStaticText(tft);
    DrawSettingsScreenStaticText_flag = false;
  }

  tft.setTextSize(1);
  
  tft.setCursor(105, 70);
  tft.print(SettingVariable[MTSTKpIndex], 1);

  tft.setCursor(195, 70);
  tft.print(SettingVariable[MTSTKiIndex], 1);

  tft.setCursor(265, 70);
  tft.print(SettingVariable[MTSTKdIndex], 1);

  tft.setCursor(105, 90);
  tft.print(SettingVariable[MTKpIndex], 1);

  tft.setCursor(195, 90);
  tft.print(SettingVariable[MTKiIndex], 1);

  tft.setCursor(265, 90);
  tft.print(SettingVariable[MTKdIndex], 1);

  tft.setCursor(170, 142);
  tft.print(SettingVariable[MTSpIndex], 1);

  tft.setCursor(170, 162);
  tft.print(SettingVariable[MTWaterToGrainRatioIndex], 1);

  tft.setCursor(170, 182);
  tft.print(SettingVariable[MTGrainTempIndex], 1);

  tft.setCursor(170, 202);
  tft.print(SettingVariable[MTInitialWaterTempIndex], 1);

  tft.setCursor(170, 222);
  tft.print(SettingVariable[MTRampOffTempIndex], 1);

  tft.setCursor(170, 242);
  tft.print(SettingVariable[MTPIDStartTempIndex], 1);

  tft.setCursor(75, 327);
  tft.print(GetClockTime(SettingVariable[MashTimeIndex]));

  tft.setCursor(230, 327);
  tft.print(GetClockTime(SettingVariable[BoilTimeIndex]));

  tft.setCursor(230, 347);
  tft.print(GetClockTime(SettingVariable[FirstAddTimeIndex]));

  tft.setCursor(230, 367);
  tft.print(GetClockTime(SettingVariable[SecondAddTimeIndex]));

  tft.setCursor(230, 387);
  tft.print(GetClockTime(SettingVariable[ThirdAddTimeIndex]));

}

#endif

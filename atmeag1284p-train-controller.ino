/* Serial1ly managed model train throttle and automatic station stop controller

Version: 1.0
   Author: quarterturn
   Date: 5/30/2024
   MCU Hardware: ATMEGA 1284p

   A DC motor controller using Adafruit DRV8871 and train detection via TMAG5231C1DQDBZR hall-effect on/off bipolar sensors (active low)

   The code supports two terminus stations and a middle station.

   Sensors are placed as follows on the train line:
   
   left station               middle station                right station                 
   |==<stop>====<slow>====<slow>====<stop>====slow>====<slow>====stop>==|

   The code uses a state machine to track what the train is doing

   |->sleeping: the train is not running because the schedule says it isn't
   |  |
   |  v
   |  startup: if no sensor is already covered by the train, and the train is not at the last sensor, move the train to the leftmost stop
   |  |
   |  v  
   |>-accelerating: the train is ramping up to running speed
   |  |
   |  v  
   |  running: the train is at running speed
   |  |
   |  v  
   |  deaccelerating: the train is ramping down to minimum speed
   |  |
   |  v  
   |  stop: the train is stops
   |  |
   |  v  
   |--waiting: the train is waiting at a station
   
The code tries to account for the instances where the train overshoots a sensor while stopping,
and will not run the train if it can not find it at one of the stop sensors. In this case, manually move the train to the sensor.

Also, should a train somehow miss a "stop" sensor, the code limits the total run time between stops,
to prevent the train from damaging itself by running against a stop for a long period of time.

The middle station stop sensor can be omitted. It's really only needed for very short trains (1-2 cars), otherwise it
looks better to use the opposing slow sensor as "stop", as it allows the train to line up with the platform better.
*/

#include <Adafruit_SH110X.h>
#include <splash.h>
#include <EEPROM.h>
#include <Time.h>
#include <Wire.h>
#include <DS3232RTC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "PWM.h"
////// PWM note frequencies
//const int32_t NOTE1 = 392;
const int32_t PWM_SLOW = 392;
const int32_t PWM_FAST = 300;
int32_t current_pwm = PWM_SLOW;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, OLED_RESET);

// memory size of the internal EEPROM in bytes
#define EEPROM_SIZE 4096

// pin used to put HC-05 bluetooth module into AT mode
#define BT_AT_PIN 12

// Serial1 baudrate (bluetooth serial module)
#define Serial1_BAUD 9600

// compare this to what is stored in the eeprom to see if it has been cleared
#define EE_MAGIC_NUMBER 0xBBDC
#define EE_MAGIC_ADDRESS 2048

// number of text fornatting commands

// for tracking time in delay loops
// global so it can be used in any function
unsigned long previousMillis;
// track display time to prevent burn-in
unsigned long displayMillis;
// track how long the train runs to stop it if a sensor is missed
unsigned long runMillis;

// a global to contain Serial1 menu input
char menuChar;

// global to track main menu display
byte mainMenuCount = 0;

// RAM copy of eeprom magic number
uint16_t magicNumber;

// store the last second
// so we can update the display as soon as we get a new second
static uint8_t prevSecond = 61;

// temporary time variable
time_t t;
tmElements_t tm;

// tracks the EEPROM address
const int eeprom_addr = 0;
const int eeprom_addr2 = 128;

// the number of stops on the line, including terminus stations
// right now just three since we're using I/O pins
// future plan is to use i2c expanders so more stops makes sense later
#define NUM_STOPS 3

// array of wait times
int station_wait_times[NUM_STOPS];

// array of on and off times
// stored as: on hour, on minute, off hour, off minute
int on_off_times[5];
// track current state
bool is_running = 1;

// speed up and slow down time
// five seconds
#define UP_DOWN_TIME 5000
// about 39ms per tick if there are 127 pwm change steps
int deaccel_tick_time = 1000;
#define ACCEL_TICK_TIME 150000

// stopping time
// used when the train isn't yet at the minimum speed when it reaches a stop sensor
// one second
#define QUICK_STOP_TIME 1000

// how long train sits at either terminus with lights on
#define WAIT_PAUSE_TIMER 5000

// how long the train will stay in slow speed
// mainly to prevent it missing a stop sensor and then hitting the stop at the end of the line
#define SLOW_MAX_TIMER 15000
// how long the train will stay in run speed in case it misses a slow sensor
#define RUN_MAX_TIMER 30000

// message string size
#define MAX_SIZE 80
char currentString[MAX_SIZE] = {0};

// state machine
// the train is not running as the scheduele indicates it does not run
#define STATE_SLEEPING 0
// the train system is either waking up or staring up
// the train will find the leftmost terminus stop if it is not already there
#define STATE_STARTUP 1
// the train is waiting at a stop
#define STATE_WAITING 2
// the train is acclerating
#define STATE_ACCEL 3
// the train is at full speed
#define STATE_RUNNING 4
// the train is slowing down
#define STATE_DEACCEL 5
// the train is going slowly
#define STATE_SLOW 6
// the train has stopped
#define STATE_STOPPED 7

// track the program state
int program_state = 1;
// track if we're starting up so we can override the first wait time
bool is_startup = 1;


// lazy method of pin tracking
// yes there's a struct but there are only enough pins for three stations so...
#define L_STOP_PIN 24
#define L_SLOW_PIN 25
#define C_L_SLOW_PIN 26
#define C_STOP_PIN 27
#define C_R_SLOW_PIN 28
#define R_SLOW_PIN 29
#define R_STOP_PIN 30

// motor control PWM pins
#define MOTOR_LEFT_PIN 13
#define MOTOR_RIGHT_PIN 14

// min and max motor speeds
// Greenmax without LED lighting
//#define START_SPEED 10
//#define MIN_SPEED 12
//#define MAX_SPEED 25
//#define IDLE_SPEED 5
// Kato with LED ligting
//#define START_SPEED 18
//#define MIN_SPEED 15
//#define MAX_SPEED 41
//#define IDLE_SPEED 10
// Greenmax with LED lighting
#define START_SPEED 10
#define MIN_SPEED 15
#define MAX_SPEED 41
#define IDLE_SPEED 8

// track motor speed
float motor_speed = 0;
// track the last speed so we don't constantly try to change it for no reason
int prev_motor_speed = 0;

// motor direction
bool direction_left = 0;
// signal flags
bool left_stop = 0;
int temp_station = 0;
// set to an out of range index because the train will seek the left station
// at start up time
// and we want to trigger on any sensor to figure out the position
int current_station = 3;
// sensor check result
int check_result = 9;

//function prototypes
void testdrawchar(void);
int check_slow(void);
int check_stop(void);
bool check_schedule(void);
void printI00(int val, char delim);
void printDate(time_t t);
void printTime(time_t t);
void printDateTime(time_t t);
void showOnOffTime(void);
void setOnOffTime(void);
void setTheTime(void);
void initEeprom(void);
void getSerial1String(void);
int getSerial1Int(void);
void editStations(void);
void showStations(void);
void clearAndHome(void);
void display_speed(void);
void display_direction(void);

//---------------------------------------------------------------------------------------------//
// setup
//---------------------------------------------------------------------------------------------//
void setup()
{
  // set up the Serial ports
  Serial1.begin(Serial1_BAUD);
  Serial.begin(115200);

  //initialize all timers except for 0, to save time keeping functions

  //InitTimersSafe(); 
  delay(250);
  if(!display.begin(SCREEN_ADDRESS, true)) {
    Serial.println(F("sh1106 allocation failed"));
  }

  testdrawchar();

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SH110X_WHITE);

  // startup message
  display.setCursor(0,0);
  display.clearDisplay();
  display.println(F("DC MOTOR"));
  display.println(F("TRAIN"));
  display.println(F("CONTROLLER"));
  display.println(F("STARTING"));
  display.display();
  delay(2000);
  display.clearDisplay();
  displayMillis = millis();

  // configure the sensor input pins
  // note: the TI sensors used do not need a pull-up, but other kinds might
  pinMode(L_STOP_PIN, INPUT);
  digitalWrite(L_STOP_PIN, HIGH);
  pinMode(L_SLOW_PIN, INPUT);
  digitalWrite(L_SLOW_PIN, HIGH);
  pinMode(C_L_SLOW_PIN, INPUT);
  digitalWrite(C_L_SLOW_PIN, HIGH);
  pinMode(C_STOP_PIN, INPUT);
  digitalWrite(C_STOP_PIN, HIGH);
  pinMode(C_R_SLOW_PIN, INPUT);
  digitalWrite(C_R_SLOW_PIN, HIGH);
  pinMode(R_SLOW_PIN, INPUT);
  digitalWrite(R_SLOW_PIN, HIGH);
  pinMode(R_STOP_PIN, INPUT);
  digitalWrite(R_STOP_PIN, HIGH);

  // configure the PWM pins
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);

  // see if the eeprom has been cleared
  // if it has, initialize it with default values
  EEPROM.get(EE_MAGIC_ADDRESS, magicNumber);
  if (magicNumber != EE_MAGIC_NUMBER)
  {
    initEeprom();
  // otherwise load the values from the EEPROM
  } else {
    EEPROM.get(eeprom_addr, station_wait_times);
    EEPROM.get(eeprom_addr2, on_off_times);
  }
  
  // set the local time provider
  setSyncProvider(RTC.get);

  // set the start PWM speed
//  current_pwm = PWM_SLOW;
//  bool result1 = SetPinFrequencySafe(MOTOR_LEFT_PIN, current_pwm);
//  bool result2 = SetPinFrequencySafe(MOTOR_RIGHT_PIN, current_pwm);

} // end setup

//---------------------------------------------------------------------------------------------//
// main loop
//---------------------------------------------------------------------------------------------//
void loop()
{
  
  // if there is something in the Serial1 buffer read it
  if (Serial1.available() >  0)
  {
    // print the main menu once
    if (mainMenuCount == 0)
    {
      clearAndHome();
      Serial1.println(F("Train Controller - Main Menu"));
      Serial1.println(F("----------------------"));
      Serial1.println(F("1     Edit Station"));
      Serial1.println(F("2     Show Station"));
      Serial1.println(F("3     Edit On/Off Time"));
      Serial1.println(F("4     Display On/Off Time"));
      Serial1.println(F("5     Set Time"));
      Serial1.println(F("6     Show Time"));
      Serial1.println(F("7     Leave station now"));
      Serial1.println(F("x     Exit setup"));
      Serial1.println();
      mainMenuCount = 1;
    }

    menuChar = Serial1.read();
    if (menuChar == '1')
    {
      // edit a station
      editStations();

    }
    else if (menuChar == '2')
    {
      // show all the stations
      showStations();

    }
    else if (menuChar == '3')
    {
      // change the on/off times
      setOnOffTime();

    }
    else if (menuChar == '4')
    {
      // show the on/off times
      showOnOffTime();

    }
    else if (menuChar == '5')
    {
      // set the time
      setTheTime();

    }
    else if (menuChar == '6')
    {
      // set the time
      showTheTime();

    }
    else if (menuChar == '7')
    {
      // leave the station now
      motor_speed = START_SPEED;
      program_state = STATE_ACCEL;
    }
    else if (menuChar == 'x')
    {
      mainMenuCount = 1;
    }
  }
  // reset the main menu if the Serial1 connection drops
  mainMenuCount = 0;

  Serial1.flush();

  // do main loop stuff here
  switch (program_state)
  {
    // see if it's time to wake up
    case STATE_SLEEPING:
      is_running = check_schedule();
      if (is_running) {
        // startup
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("STARTUP"));
        display.display();
        displayMillis = millis();        
        program_state = STATE_STARTUP;
      }
      break;
    // move the train to the leftmost station
    case STATE_STARTUP:
      // unset startup
      is_startup = 0;
      // check if we're already in position
      temp_station = check_stop();
      current_station = temp_station;
      // make the train move regardless of schedule as a 'do something' check at startup
      if (current_station == 0) {
        current_station = 0;
        // set direction to the right
        direction_left = 1;
        // wait to depart
        program_state = STATE_ACCEL;           
      }
      if (current_station >= 1 ){
        // set direction to the left
        direction_left = 0;
        // set initial speed
        motor_speed = MIN_SPEED;
        // accelerate
        program_state = STATE_ACCEL;
      }
      if (current_station == -1) {
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("TRAIN"));
        display.println(F("NOT FOUND"));
        display.println(F("MOVE"));
        display.println(F("MANUALLY"));
        display.display();
        displayMillis = millis();      
      }
      while (current_station == -1) {
        temp_station = check_stop();
        current_station = temp_station;
        if (current_station == 0) {
          current_station = 0;
          // set direction to the right
          direction_left = 0;
          // wait to depart
          program_state = STATE_ACCEL;           
        }
        if (current_station >= 1 ){
          // set direction to the left
          direction_left = 1;
          // set initial speed
          motor_speed = MIN_SPEED;
          // accelerate
          program_state = STATE_ACCEL;
        }
      }
      program_state = STATE_ACCEL;
      // start timing
      previousMillis = millis();
      displayMillis = millis();
      break;

    case STATE_WAITING:
      // keep the lights on at the center station with a little bit of power
      if (current_station == 1 ) {
        motor_speed = IDLE_SPEED;
      } else {
        motor_speed = IDLE_SPEED;
      }
      
      // is it time to turn off the lights?
      // only applies at terminus stops
      if ((current_station != 1) && ((millis() - previousMillis) > WAIT_PAUSE_TIMER))
        motor_speed = 0;
      
      // is it time to go to sleep?
      is_running = check_schedule();
      if (!is_running) {
        // cut power to motor fully
        motor_speed = 0;
        program_state = STATE_SLEEPING;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("SLEEPING"));
        display.print(F("STA: "));
        display.println(current_station);
        display_direction();
        display.setCursor(65,0);
        break;
      }
      // is it time to leave yet?
      if (((current_station == 1) && ((millis() - previousMillis) > (long(station_wait_times[current_station]) * 1000))) || ((current_station != 1) && (depart_time()))) {
      //if ((millis() - previousMillis) >= 5000) {
        // accelerate
        program_state = STATE_ACCEL;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("ACCEL"));
        display.print(F("STA: "));
        display.println(current_station);
        display_direction();
        displayMillis = millis();
        // set motor speed to start speed to get the train moving faster
        motor_speed = START_SPEED;        
        // time the next tick period
        previousMillis = micros();     
      }
      break;

    case STATE_ACCEL:
      // check if we hit a slow sensor
      // except the center station, where the train stops at a slow sensor
      check_result = check_slow();
      //if ((check_result > -1) && (check_result != 1)) {
      if (check_result > -1 ) {
        // disregard sensors at the same station
        if (current_station == check_result) {
          // do nothing
        } else {
          current_station = temp_station = check_result;
          // slow down
          program_state = STATE_DEACCEL;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("DEACCEL"));
          display.print(F("STA: "));
          display.println(current_station);
          display_direction();
          displayMillis = millis();
          // time the next tick period
          previousMillis = micros();
          break;
        }
      }
      // check if we hit a stop sensor
      check_result = check_stop();
      if ((check_result > -1) && (check_result != 1)) {
        // disgregard sensors at the same station
        if (current_station == check_result) {
          // do nothing
        // dont stop at the middle stop sensor
        // stop at the oher slow sensor
        } else {
          current_station = temp_station = check_result;
          // stop
          program_state = STATE_STOPPED;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("STOPPED"));
          display.print(F("STA: "));
          display.println(current_station);
          display_direction();
          display.setCursor(65,0);
          Serial.print(F("Station: "));
          Serial.println(current_station);
          Serial.print(F("Speed: "));
          Serial.println(motor_speed);
          Serial.print(F("Wait time: "));
          Serial.println(station_wait_times[current_station]);
          displayMillis = millis();
          previousMillis = micros();
        }
      }
      // are we at max speed?
      if (motor_speed >= MAX_SPEED) {
        // run at max speed
        program_state = STATE_RUNNING;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("FULLSPEED"));
        display.print(F("STA: "));
        display.println(current_station);
        display_direction();
        displayMillis = millis();
        runMillis = millis();
        break;
      }
      // is it time to change the speed?
      if ((micros() - previousMillis) >= ACCEL_TICK_TIME) {
        // increase speed
        motor_speed = motor_speed + 0.25;

        // display the speed
        display_speed();
        // display the direction
        display_direction();
        // time the next tick period
        previousMillis = micros();
        displayMillis = millis();
      }
      break;

    case STATE_RUNNING:
      // check if we have been running too long
      if (((millis() - runMillis) >= RUN_MAX_TIMER)) {
        // stop
        program_state = STATE_STOPPED;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("E-STOP!"));
        display.print(F("STA: "));
        display.println(current_station);
        display.setCursor(65,0);
        Serial.print(F("Station: "));
        Serial.println(current_station);
        Serial.print(F("Speed: "));
        Serial.println(motor_speed);
        Serial.print(F("Wait time: "));
        Serial.println(station_wait_times[current_station]);
        // reverse direction
        direction_left = ~direction_left;
        display_direction();
        displayMillis = millis();
        break;
      }
      
      // check if we hit a slow sensor
      check_result = check_slow();
      if (check_result > -1) {
        // disgregard sensors at the same station
        if (current_station == check_result) {
          // do nothing
        } else {
          current_station = temp_station = check_result;
          // slow down
          program_state = STATE_DEACCEL;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("DEACCEL"));
          display.print(F("STA: "));
          display.println(current_station);
          display_direction();
          displayMillis = millis();
          previousMillis = micros();
        }
      }
      // check if we hit a stop sensor
      check_result = check_stop();
      if ((check_result > -1) && (check_result != 1)) {
        current_station = check_result;
        // stop
        program_state = STATE_STOPPED;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("STOPPED"));
        display.print(F("STA: "));
        display.println(current_station);
        display_direction();
        display.setCursor(65,0);
        Serial.print(F("Station: "));
        Serial.println(current_station);
        Serial.print(F("Speed: "));
        Serial.println(motor_speed);
        Serial.print(F("Wait time: "));
        Serial.println(station_wait_times[current_station]);
        displayMillis = millis();
        previousMillis = micros();
        break;
      }
      break;

    case STATE_DEACCEL:
      // check if we hit a stop sensor
      check_result = check_stop();
      if ((check_result > -1) && (check_result != 1)) { 
        current_station = temp_station = check_result;
        // stop
        program_state = STATE_STOPPED;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("STOPPED"));
        display.print(F("STA: "));
        display.println(current_station);
        display.setCursor(65,0);
        Serial.print(F("Station: "));
        Serial.println(current_station);
        Serial.print(F("Speed: "));
        Serial.println(motor_speed);
        Serial.print(F("Wait time: "));
        Serial.println(station_wait_times[current_station]);
        display_direction();
        displayMillis = millis();
        previousMillis = micros();
        break;
      }
      // ignore the stop sensor at the center station and instead stop at the other slow sensor
      if (check_result == 1) {
        check_result = check_center_slow();        
        if (((check_result == 0) && (direction_left)) || ((check_result == 2) && (!direction_left))) {
          current_station = temp_station = 1;
          // stop
          program_state = STATE_STOPPED;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("STOPPED"));
          display.print(F("STA: "));
          display.println(current_station);
          display.setCursor(65,0);
          Serial.print(F("Station: "));
          Serial.println(current_station);
          Serial.print(F("Speed: "));
          Serial.println(motor_speed);
          Serial.print(F("Wait time: "));
          Serial.println(station_wait_times[current_station]);
          display_direction();
          displayMillis = millis();
          previousMillis = micros();
          break;
        }
      }
      
      // are we at minimum speed?
      if (motor_speed <= MIN_SPEED) {
        // run at minimum speed
        program_state = STATE_SLOW;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("ARRIVING"));
        display.print(F("STA: "));
        display.println(current_station);
        display_direction();
        displayMillis = millis();
        runMillis = millis();
        break;
      }
      if (current_station == 1) {
        deaccel_tick_time = 19000; 
      } else {
        deaccel_tick_time = 1;
      }
      // is it time to change the speed?
      if (((micros() - previousMillis) >= deaccel_tick_time) && (motor_speed > MIN_SPEED)) {
        // decrease speed
        motor_speed--;
        // display the speed
        display_speed();
        // display the direction
        display_direction();
        // time the next tick period
        previousMillis = micros();
        displayMillis = millis();
      }
      break;

    case STATE_SLOW:
      // check if we someone missed a stop sensor
      // to avoid making the train work against the end of line stop
      if (((millis() - runMillis) >= SLOW_MAX_TIMER)) {
        // stop
        program_state = STATE_STOPPED;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("E-STOP!"));
        display.print(F("STA: "));
        display.println(current_station);
        display.setCursor(65,0);
        Serial.print(F("Station: "));
        Serial.println(current_station);
        Serial.print(F("Speed: "));
        Serial.println(motor_speed);
        Serial.print(F("Wait time: "));
        Serial.println(station_wait_times[current_station]);
        // reverse direction
        direction_left = ~direction_left;
        display_direction();
        displayMillis = millis();
        break;
      }
      
      // check if we hit a stop sensor
      check_result = check_stop();
      if ((check_result > -1) && (check_result != 1)) { 
        current_station = temp_station = check_result;
        // stop
        program_state = STATE_STOPPED;
        display.setCursor(0,0);
        display.clearDisplay();
        display.println(F("STOPPED"));
        display.print(F("STA: "));
        display.println(current_station);
        display.setCursor(65,0);
        Serial.print(F("Station: "));
        Serial.println(current_station);
        Serial.print(F("Speed: "));
        Serial.println(motor_speed);
        Serial.print(F("Wait time: "));
        Serial.println(station_wait_times[current_station]);
        display_direction();
        displayMillis = millis();
        break;
      }
      // ignore the stop sensor at the center station and instead stop at the other slow sensor
      if (current_station == 1) {
        check_result = check_center_slow();
        if (((check_result == 0) && (direction_left)) || ((check_result == 2) && (!direction_left))) {
          current_station = temp_station = 1;
          // stop
          program_state = STATE_STOPPED;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("STOPPED"));
          display.print(F("STA: "));
          display.println(current_station);
          display.setCursor(65,0);
          Serial.print(F("Station: "));
          Serial.println(current_station);
          Serial.print(F("Speed: "));
          Serial.println(motor_speed);
          Serial.print(F("Wait time: "));
          Serial.println(station_wait_times[current_station]);
          display_direction();
          displayMillis = millis();
          previousMillis = micros();
          break;
        }
      }
      break;

    case STATE_STOPPED:
      // keep the lights on at the center station with a little bit of power
      if (current_station == 1 ) {
        motor_speed = IDLE_SPEED;
      } else {
        motor_speed = IDLE_SPEED;
      }
      if (current_station == 2)
        // change direction
        direction_left = 1;
      // note: we only sleep the train at the leftmost station
      // to avoid just stopping it randomly which looks wrong
      if (current_station == 0) {
        // change direction
        direction_left = 0;
        // time to sleep?
        is_running = check_schedule();
        if (is_running == 0) {
          program_state = STATE_SLEEPING;
          display.setCursor(0,0);
          display.clearDisplay();
          display.println(F("SLEEPING"));
          display.print(F("STA: "));
          display.println(current_station);
          display_direction();
          displayMillis = millis();
          // start timing
          previousMillis = millis();
          break;
        }
      }
      // move to the waiting state
      program_state = STATE_WAITING;
      display.setCursor(0,0);
      display.clearDisplay();
      display.println(F("WAITING"));
      display_direction();
      display.setCursor(65,0);
      Serial.println(station_wait_times[current_station]);
      displayMillis = millis();
      // start timing
      previousMillis = millis();    
      break;

    // default case
    default:
      break;
  }

  // adjust the motor speed and direction if necessary
  if (motor_speed != prev_motor_speed) {
    if (direction_left) {
      analogWrite(MOTOR_LEFT_PIN, motor_speed);
      analogWrite(MOTOR_RIGHT_PIN, 0);
    } else {
      analogWrite(MOTOR_RIGHT_PIN, motor_speed);
      analogWrite(MOTOR_LEFT_PIN, 0);
    }
    prev_motor_speed = motor_speed;
  }

  // clear the display to prevent OLED burn-in
  if ((millis() - displayMillis) >= 10000) {
    display.clearDisplay();
    display.display();
  }
}

//---------------------------------------------------------------------------------------------//
// function clearAndHome
// returns the cursor to the home position and clears the screen
//---------------------------------------------------------------------------------------------//
void clearAndHome(void)
{
  Serial1.write(27); // ESC
  Serial1.print("[2J"); // clear screen
  Serial1.write(27); // ESC
  Serial1.print("[H"); // cursor to home
}

//---------------------------------------------------------------------------------------------//
// function showStations
// displays the station wait times
//---------------------------------------------------------------------------------------------//
void showStations(void)
{
  byte station_count = 0;

  // clear the terminal
  clearAndHome();

  Serial1.flush();

  // clear the string in RAM
  sprintf(currentString, "");

  // display the menu on entry
  Serial1.println(F("Here are the ststion stop times"));
  Serial1.println(F("-------------------------"));
  Serial1.println();
  // display the stations from the struct data
  // the plain array is only for storing the stop times in and out of EEPROM
  while (station_count < NUM_STOPS)
  {
    Serial1.print("Station ");
    Serial1.print(station_count, DEC);
    Serial1.print(": ");
    Serial1.println(station_wait_times[station_count]);
    station_count++;
  }

  Serial1.println();
  Serial1.println(F("x    return to Main Menu"));

  // poll Serial1 until exit
  while (1)
  {
    // if there is something in the Serial1 buffer read it
    if (Serial1.available() >  0)
    {
      menuChar = Serial1.read();
      if (menuChar == 'x')
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // return to main menu and return the mode
        return;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function editStations
// edits the display program stored in the EEPROM
//---------------------------------------------------------------------------------------------//
void editStations(void)
{
  // clear the terminal
  clearAndHome();
  // track menu display
  byte displayEditProgramMenu = 0;

  // message values
  int slot = 0;
  int stopTime;

  // input char
  char menuChar;

  // input valid flag
  byte inputBad = 1;

  // loop flag
  byte loopFlag = 1;

  Serial1.flush();

  // display the menu on entry
  if (displayEditProgramMenu == 0)
  {
    Serial1.println(F("Edit Station Stop Times"));
    showStations();
    Serial1.println(F("Set the middle station to zero to skip it"));
    Serial1.println(F("---------------------------"));
    while (slot < NUM_STOPS)
    {
      Serial1.print("Station ");
      Serial1.print(slot, DEC);
      Serial1.print(": ");
      Serial1.println(station_wait_times[slot]);
      slot++;
    }
    displayEditProgramMenu = 1;
  }

  // loop until we return from the function
  while (1)
  {
    Serial1.println(F("Enter the number of the slot to edit: "));
    // loop until the input is acceptable
    while (inputBad)
    {
      slot = getSerial1Int();
      if ((slot >= 0) && (slot < NUM_STOPS))
      {
        inputBad = 0;
      }
      else
      {
        Serial1.println(F("Error: station "));
        Serial1.print(slot);
        Serial1.println(F(" is out of range. Try again."));

      }
    }
    // reset the input test flag for the next time around
    inputBad = 1;

    // show the choice since no echo
    Serial1.println(F("Station: "));
    Serial1.println(slot);
    
    Serial1.println(F("Enter the wait time: "));
    // loop until the input is acceptable
    while (inputBad)
    {
      stopTime = getSerial1Int();
      if (stopTime >= 0)
      {
        inputBad = 0;
      }
      else
      {
        Serial1.println(F("Error: Stop time "));
        Serial1.print(stopTime);
        Serial1.println(F(" is out of range. Try again."));

      }
    }
    // reset the input test flag for the next time around
    inputBad = 1;
    
    // write the data to array in RAM and the struct array
    Serial1.println(F("Writing data to eeprom..."));
    station_wait_times[slot] = stopTime;

    Serial1.println(F("Edit another? (y/n): "));
    
    while (loopFlag)
    {
      if (Serial1.available() > 0)
      {
        menuChar = Serial1.read();
        if (menuChar == 'n')
        {
          Serial1.println(menuChar);

          // write the program to the EEPROM before we return
          EEPROM.put(eeprom_addr, station_wait_times);
          return;
        }
        if (menuChar == 'y')
        {
          loopFlag = 0;
          Serial1.println(menuChar);
          Serial1.flush();
        }
      }
    }
    loopFlag = 1;
  }
  return;
}

//---------------------------------------------------------------------------------------------//
// function getSerial1Int
// uses Serial1 input to get an integer
//---------------------------------------------------------------------------------------------//
int getSerial1Int(void)
{
  char inChar;
  int in;
  int input = 0;

  Serial1.flush();
  do
  // at least once
  {
    while (Serial1.available() > 0)
    {
      inChar = Serial1.read();
      // echo the input
      Serial1.print(inChar);
      // convert 0-9 character to 0-9 int
      in = inChar - '0';
      if ((in >= 0) && (in <= 9))
      {
        // since numbers are entered left to right
        // the current number can be shifted to the left
        // to make room for the new digit by multiplying by ten
        input = (input * 10) + in;
      }
    }
  }
  // stop looping when an ^M is received
  while (inChar != 13);
  // return the number
  return input;
}

//---------------------------------------------------------------------------------------------//
// function getSerial1String
// uses Serial1 input to get a string
//---------------------------------------------------------------------------------------------//
void getSerial1String(void)
{
  // track how many characters entered
  byte cCount = 0;
  // input valid flag
  byte inputBad = 1;
  // track when string is done
  byte stringDone = 0;

  // clear the string in RAM
  sprintf(currentString, "");

  Serial1.flush();
  // loop until done
  while (stringDone == 0)
  {
    // if there is something in the Serial1 buffer read it
    while (Serial1.available() >  0)
    {
      // grab a character
      menuChar = Serial1.read();
      // echo the input
      Serial1.print(menuChar);

      // do stuff until we reach MAX_SIZE
      if (cCount < (MAX_SIZE - 1))
      {
        // pressed <ENTER> (either one)
        if ((menuChar == 3) || (menuChar == 13))
        {
          // set flag to redraw menu
          mainMenuCount = 0;
          // make the last character a null
          // cCount++;
          currentString[cCount] = 0;
          // mark the string done
          stringDone = 1;
        }
        // if we are not at the end of the string and <delete> or <backspace> not pressed
        else if ((menuChar != 127) || (menuChar != 8))
        {
          currentString[cCount] = menuChar;
          cCount++;
        }
        // if index is between start and end and delete or backspace is pressed
        // clear the current character and go back one in the index
        else if ((cCount > 0) && ((menuChar == 127) || (menuChar == 8)))
        {
          currentString[cCount] = 0;
          cCount--;
          // print a backspace to the screen so things get deleted
          Serial1.write(8);
        }
      }
      // we reached MAX_SIZE
      else
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // set the current character to null
        currentString[cCount] = 0;
        // mark the string as done
        stringDone = 1;
      }
    }
  } // end of the string input loop

  return;
}

//---------------------------------------------------------------------------------------------//
// function initEeprom
// loads default values into the eeprom if nothing is present based on ee_magicNumber
// the defaults are to have the train wait 10 seconds at stops the first and last stops
// which are the end stop, and no stops in between
//---------------------------------------------------------------------------------------------//
void initEeprom(void)
{

  int temp_addr = 0;

  // set the magic number
  EEPROM.put(EE_MAGIC_ADDRESS, EE_MAGIC_NUMBER);

  // fill the station variables
  for (int slot = 0; slot < (NUM_STOPS - 1); slot++)
  {
    // first and last stops
    if ((slot == 0) | (slot == (NUM_STOPS - 1))) {
      station_wait_times[slot] = 10;
    } else {
      station_wait_times[slot] = 10;
    }
  }
  // write the station wait times to EEPROM
  EEPROM.put(eeprom_addr, station_wait_times);

  // fill the on-off time variables
  // default is on at 0600 and off at 2200 and enabled
  on_off_times[0] = 6;
  on_off_times[1] = 0;
  on_off_times[2] = 22;
  on_off_times[3] = 0;
  on_off_times[4] = 1;

  // write the on-off times to EEPROM
  EEPROM.put(eeprom_addr2, on_off_times);
  
}

//---------------------------------------------------------------------------------------------//
// function setTheTime()
// sets the time by Serial1 data entry
// expects nothing
// returns nothing
//---------------------------------------------------------------------------------------------//
void setTheTime(void)
{

  uint16_t y;
  
  // year
  Serial1.println("Enter the 4-digit year:");
  y = getSerial1Int();
  if (y >= 100 && y < 1000)
    tm.Year = 2000;

  // month
  Serial1.println("Enter the month:");
  tm.Month = getSerial1Int();
  // day
  Serial1.println("Enter the day:");
  tm.Day = getSerial1Int();
  // hour
  Serial1.println("Enter the hour:");
  tm.Hour = getSerial1Int();
  // minute
  Serial1.println("Enter the minute:");
  tm.Minute = getSerial1Int();
  // second
  Serial1.println("Enter the second:");
  tm.Second = getSerial1Int();
  t = makeTime(tm);
  setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, y);
  RTC.set(now());  
  Serial1.print(F("RTC set to: "));
  printDateTime(now());
  Serial1.println();
  while (1)
  {
    // if there is something in the Serial1 buffer read it
    if (Serial1.available() >  0)
    {
      menuChar = Serial1.read();
      if (menuChar == 27)
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // return to main menu
        return;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function showTheTime()
// displays the time
// expects nothing
// returns nothing
//---------------------------------------------------------------------------------------------//
void showTheTime(void)
{
  Serial1.print(F("RTC set to: "));
  printDateTime(now());
  Serial1.println();
  while (1)
  {
    // if there is something in the Serial1 buffer read it
    if (Serial1.available() >  0)
    {
      menuChar = Serial1.read();
      if (menuChar == 27)
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // return to main menu
        return;
      }
    }
  }
}


//---------------------------------------------------------------------------------------------//
// function setOnOffTime()
// sets the train operation on and off times by Serial1 data entry
// expects nothing
// returns nothing
//---------------------------------------------------------------------------------------------//
void setOnOffTime(void)
{

  Serial1.println("Note: Times are expected in 24-hour format");
  // on hour
  Serial1.println("Enter the on hour:");
  on_off_times[0] = getSerial1Int();
  // on minute
  Serial1.println("Enter the on minute:");
  on_off_times[1] = getSerial1Int();
  // off hour
  Serial1.println("Enter the off hour:");
  on_off_times[2] = getSerial1Int();
  // minute
  Serial1.println("Enter the off minute:");
  on_off_times[3] = getSerial1Int();
  // second
  Serial1.println("Enter 1 for enbaled or 0 for disabled:");
  on_off_times[4] = getSerial1Int();
  while ((on_off_times[4] < 0) || (on_off_times[4] > 1)) {
    Serial1.println("Invalid input: Enter 1 for enbaled or 0 for disabled:");
    on_off_times[4] = getSerial1Int();
  }

  Serial1.println(F("The on-off schedule is now: "));
  Serial1.println(F("On at: "));
  Serial1.print(on_off_times[0]);
  Serial1.print(":");
  Serial1.print(on_off_times[1]);
  Serial1.println();
  Serial1.println(F("Off at: "));
  Serial1.print(on_off_times[2]);
  Serial1.print(":");
  Serial1.print(on_off_times[3]);
  Serial1.println();
  if (on_off_times[4] == 1) {
    Serial1.println(F("Schedule enabled"));
  } else {
    Serial1.println(F("Schedule disabled"));
  }
  while (1)
  {
    // if there is something in the Serial1 buffer read it
    if (Serial1.available() >  0)
    {
      menuChar = Serial1.read();
      if (menuChar == 27)
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // write the on-off times to EEPROM
        EEPROM.put(eeprom_addr2, on_off_times);
        // return to main menu
        return;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function showOnOffTime()
// shows the train on-off times
// expects nothing
// returns nothing
//---------------------------------------------------------------------------------------------/
void showOnOffTime(void)
{  
  
  Serial1.println(F("The on-off schedule is now: "));
  Serial1.println(F("On at: "));
  Serial1.print(on_off_times[0]);
  Serial1.print(":");
  Serial1.print(on_off_times[1]);
  Serial1.println();
  Serial1.println(F("Off at: "));
  Serial1.print(on_off_times[2]);
  Serial1.print(":");
  Serial1.print(on_off_times[3]);
  Serial1.println();
  if (on_off_times[4] == 1) {
    Serial1.println("Schedule enabled");
  } else {
    Serial1.println("Schedule disabled");
  }
  while (1)
  {
    // if there is something in the Serial1 buffer read it
    if (Serial1.available() >  0)
    {
      menuChar = Serial1.read();
      if (menuChar == 27)
      {
        // set flag to redraw menu
        mainMenuCount = 0;
        // return to main menu
        return;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function printDateTime
//---------------------------------------------------------------------------------------------//
//print date and time to Serial1
void printDateTime(time_t t)
{
    printDate(t);
    Serial1.print(" ");
    printTime(t);
}

//---------------------------------------------------------------------------------------------//
// function printTime
//---------------------------------------------------------------------------------------------//
void printTime(time_t t)
{
    printI00(hour(t), ':');
    printI00(minute(t), ':');
    printI00(second(t), ' ');
}

//---------------------------------------------------------------------------------------------//
// function printDate
//---------------------------------------------------------------------------------------------//
void printDate(time_t t)
{
    printI00(day(t), 0);
    Serial1.print(monthShortStr(month(t)));
    Serial1.print(year(t), DEC);
}

//---------------------------------------------------------------------------------------------//
// function printI00
//Print an integer in "00" format (with leading zero),
//followed by a delimiter character to Serial1.
//Input value assumed to be between 0 and 99.
//---------------------------------------------------------------------------------------------//
void printI00(int val, char delim)
{
    if (val < 10) Serial1.print('0');
    Serial1.print(val, DEC);
    if (delim > 0) Serial1.print(delim);
    return;
}

//---------------------------------------------------------------------------------------------//
// function check_schedule()
// checks if the train is supposed to be running
// based on on_off_times
//---------------------------------------------------------------------------------------------//
bool check_schedule(void)
{ 
  // is it enabled?
  if (on_off_times[4] == 0) {
    // the schedule is disabled so it is always running
    return 1;
  }
  // check schedule
  // convert times to minutes for easier comparison
  int now_minutes = hour() * 60 + minute();
  int on_minutes = on_off_times[0] * 60 + on_off_times[1];
  int off_minutes = on_off_times[2] * 60 + on_off_times[3];
  if ((now_minutes >= on_minutes) && (now_minutes < off_minutes)) {
    return 1;
  } else {
    return 0;
  }
}

//---------------------------------------------------------------------------------------------//
// function depart_time()
// checks if it is on the quarter hour for train departure
//---------------------------------------------------------------------------------------------//
bool depart_time(void)
{
  if (((minute() == 15) || (minute() == 30) || (minute() == 45) || (minute() == 0)) && (second() == 0))
  {
    return 1;
  } else {
    return 0;
  }
}

//---------------------------------------------------------------------------------------------//
// function check_slow()
// checks if a slow sensor is high
// returns the station number a sensor is high
// otherwise returns -1
//---------------------------------------------------------------------------------------------//
int check_slow(void)
{
  if (!digitalRead(L_SLOW_PIN))
    return 0;
  if (!digitalRead(C_L_SLOW_PIN) || !digitalRead(C_R_SLOW_PIN))
    return 1;
  if (!digitalRead(R_SLOW_PIN))
    return 2;
  return -1;
}

//---------------------------------------------------------------------------------------------//
// function check_center_slow()
// checks if a center slow sensor is high
// returns 0 if the left slow
// returns 2 if the right slow
// otherwise returns -1
//---------------------------------------------------------------------------------------------//
int check_center_slow(void)
{
  if (!digitalRead(C_L_SLOW_PIN))
    return 0;
  if (!digitalRead(C_R_SLOW_PIN))
    return 2;
  return -1;
}

//---------------------------------------------------------------------------------------------//
// function check_stop()
// checks if a stop sensor is high
// returns the station number a sensor is high
// otherwise returns -1
//---------------------------------------------------------------------------------------------//
int check_stop(void)
{
  // magnetic sensor is active low
  if (!digitalRead(L_STOP_PIN))
    return 0;
  if (!digitalRead(C_STOP_PIN))
    return 1;
  if (!digitalRead(R_STOP_PIN))
    return 2;
  return -1;
}

//---------------------------------------------------------------------------------------------//
// function testdrawchar
// fills the OLED display with text to test it
//---------------------------------------------------------------------------------------------//
void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

//---------------------------------------------------------------------------------------------//
// function display_speed
// prints the speed on the display
//---------------------------------------------------------------------------------------------//
void display_speed(void) {
  display.clearDisplay();
  display.setCursor(0,32);
  display.print(motor_speed);
}

//---------------------------------------------------------------------------------------------//
// function display_direction
// prints the direction and current station on the display
//---------------------------------------------------------------------------------------------//
void display_direction(void) {
  // show the direction on the display
  display.setCursor(65,32);
  if (direction_left) {
    display.print(F("LEFT"));
  } else {
    display.print(F("RIGHT"));
  }
  // show the current station on the display
  display.setCursor(0,48);
  display.print(F("CURR: "));
  display.print(current_station);
  // send it to the display
  display.display();
}

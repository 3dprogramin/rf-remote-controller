#include <Servo.h>
#include <RCSwitch.h>
//#include "LowPower.h"

#define COMMON_ANODE

RCSwitch receiver = RCSwitch();     // rf receiver
Servo servo;  // create servo object to control a servo

// DEBUGGING VARIABLES
const boolean DEBUG = false;

// ALIVE variables (actions are made only if alive_connection is true)
// ------------------------------------------------------
boolean alive_connection = false;        // alive only when A is pressed, this variable keeps track of connection/alive state
int alive_time = 0;           // last alive time
const int ALIVE_TIME_DIFF = 30000;    // 30 secs now

// RF signals
// ------------------------------------------------------
// REMOTE
// A - 21952
// B - 21808
// C - 21772
// D - 21763

const int ON_CON_REMOTE = 21763;
const int OFF_CON_REMOTE = 21808;
const int ON_REMOTE = 21772;
const int OFF_REMOTE = 21952;

// ------------------------------------------------------
// LED STATUS
// ------------------------------------------------------
// blink counter, keeps track of 'alive' green led blink
int blink_counter = 0;
boolean led_status = false;
// ------------------------------------------------------

// SERVO POSITION
const int SERVO_ON_POSITION = 53;
const int SERVO_OFF_POSITION = 105;
const int SERVO_STANDBY_POSITION = 80;

// servo 'state' delay
const int SERVO_DELAY = 500;

// ------------------------------------------------------
// PINS
// ---------------------------------
const int SERVO_PIN = 9;  // servo pin 9 on both uno and pro mini 
const int RF_PIN = 0;     // pin 2, but interrupt 0 for uno and ProMini
// ------------------------------------------------------
// RGB LED PINS
// PWM PINS
// ------------------------------------------------------
const int redPin = 3;     // both uno & pro mini
const int greenPin = 5;   // -=-
const int bluePin = 6;    // -=-
// ------------------------------------------------------
// LED STATUS BUTTON PIN
// ------------------------------------------------------
const int LED_STATUS_BTN_PIN = 4;       // button PIN for controlling the LED (either make LED enabled or disabled)


void setup() {
  if(DEBUG)
  {
    Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
    Serial.println("Started");
  }
  set_servo(SERVO_STANDBY_POSITION, false);    // set servo standby position
  receiver.enableReceive(RF_PIN);  // Receiver on interrupt 0 => that is pin #2
  // RGB led pins
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
  pinMode(13, OUTPUT);        // make sure LED 13 is off
  
  pinMode(LED_STATUS_BTN_PIN, INPUT);          // led status button
  digitalWrite(13, LOW);      // make sure it's off, on uno it's not
  
  update_led_status(true);      // update led status
  setColor(0,0,0);          // disable LED (for now)
  boot();                   // 'boot' leds showing different colors
}

void loop() {
  // ATmega328P, ATmega168
  // LOW POWER CONSUMTION
  // ------------------------------------------------------------------
  //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
//                SPI_OFF, USART0_OFF, TWI_OFF);
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  // ------------------------------------------------------------------
  
  //int servo_position = read_serial();       // read from serial
  // read with RF receiver
  // ---------------------------------------------------------------
  // UPDATE - In this function, the connection status (alive) is
  // set as well, and returns 0 when it does that
  // ---------------------------------------------------------------
  int servo_position = read_rf();
  
  // check if we have a new good value (ONLY 60 or 101, based on our 'specs')
  if((servo_position == SERVO_ON_POSITION) && (alive_connection))
  {
    setColor(0, 0, 255);        // make led blue
    if(DEBUG)
    {
      Serial.println("ON");
    }
    // got a value, change 
    set_servo(servo_position);
    setColor(0, 0, 0);          // clear led
    blink_counter = 0;          // reset blink counter
    delay(2000);                // sleep for 2 seconds, for remote
  }
  else if((servo_position == SERVO_OFF_POSITION) && (alive_connection))
  {
    setColor(0, 0, 255);       // make led blue
    if(DEBUG)
    {
      Serial.println("OFF");
    }
    set_servo(servo_position);
    // save last position
    setColor(0, 0, 0);        // clear led
    blink_counter = 0;          // reset blink counter
    delay(2000);                // sleep for 2 seconds, for remote
  }

  // every 10 seconds
  if(blink_counter == 100)
  {
    setColor(0, 255, 0);
    // sleep for 100 millis
    delay(100);           
    // ------------------------------------
    // NO WORRIES ABOUT THE RF RECEIVER
    // SINCE IT'S SET ON AN INTERRUPT, YOU CAN SLEEP EVEN FOR 5 MINUTES
    // IT WILL STILL GET THE DATA WHILE SLEEPING, BUT ONLY PROCESS
    // IT WHEN IT GETS OUT OF SLEEP
    // ! TESTED MYSELF
    // ------------------------------------
    setColor(0, 0, 0);      // clear led
    blink_counter = 0;      // reset blink counter
  }
  else
  {
    delay(100);             // sleep for 100 millis
    blink_counter += 1;     // increase blink counter
  }

  update_led_status();      // update led status
  handle_alive_con();     // takes care of connection 'alive'
}

// read from RF receiver
int read_rf()
{
   int incoming_int = 0;
   if (receiver.available()) {
    // get the available value
    int value = receiver.getReceivedValue();

    if(DEBUG)
    {
      Serial.print("Received RF: ");
      Serial.println(value);
    }

    // check for message
    // --------------------------------------
    // ON/OFF normal VALUES
    // --------------------------------------
    if(value == ON_REMOTE)
    {
      // ON was pressed on remote control
      incoming_int = SERVO_ON_POSITION;
    }
    else if(value == OFF_REMOTE)
    {
      // OFF was pressed on remote control
      incoming_int = SERVO_OFF_POSITION;
    }
    // --------------------------------------
    // ON/OFF alive connection VALUES
    // --------------------------------------
    else if(value == ON_CON_REMOTE)
    {
      // make it 'alive'
      alive_connection = true;
      alive_time = millis();        // save time
      if(DEBUG)
      {
        Serial.println("alive up");
      }

      setColorRaw(255, 255, 255);
      delay(100);
      setColor(0, 0, 0);

      
      // DEPRECATED
      // WAS USED BEFORE, FOR BLINKING LED ONLY IF BUTTON IS ENABLED
      // I WANT IT TO BLINK EVERY TIME I PRESS THE REMOTE THOUGH
      /*
      // led status
      if(led_status)
      {
        setColor(255, 255, 255);
        delay(100);
        setColor(0, 0, 0);
      }*/
    }
    else if(value == OFF_CON_REMOTE)
    {
      // go to sleep
      alive_connection = false;
      if(DEBUG)
      {
        Serial.println("alive down");
      }

      // led status
      if(led_status)
      {
        setColor(128, 128, 128);
        delay(100);
        setColor(0, 0, 0);
      }
    }

    receiver.resetAvailable();      // reset
   }

   return incoming_int;    // return position
}

// set servo position
void set_servo(int pos)
{
  servo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  servo.write(pos);     // write new position (ON/OFF)
  delay(SERVO_DELAY);           // sleep for half a second
  servo.write(SERVO_STANDBY_POSITION);      // go to stand by position 
  delay(SERVO_DELAY);
  servo.detach();  // attaches the servo on pin 9 to the servo object
}
// set servo position
void set_servo(int pos, boolean delay_on)
{
  servo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  servo.write(pos);     // write new position (ON/OFF)
  // ---------------------------------------
  // basically takes care of delay NOT
  // being on
  // ---------------------------------------
  if(delay_on == true)
  {
    delay(SERVO_DELAY);           // sleep for half a second
  }
  servo.write(SERVO_STANDBY_POSITION);      // go to stand by position 
  delay(SERVO_DELAY);
  servo.detach();  // attaches the servo on pin 9 to the servo object
}

/*
 * // read from serial - debugging purposes
int read_serial()
{
  int incoming_int = 0;
  String incoming;
  // send data only when you receive data:
  if (Serial.available() > 0) {
      // read the incoming byte:
      incoming = Serial.readString();

      incoming_int = incoming.toInt();
      
      Serial.print("Int: ");
      if(incoming_int == 0)
      {
        Serial.println("not an int");
      }
      else
      {
        Serial.println(incoming_int);
      }
  }

  return incoming_int;
}
*/

// set let color
void setColor(int red, int green, int blue)
{
  if(!led_status)      // return if led status is off
  {
    return;
  }
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

// set let color raw
void setColorRaw(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

// set let color - new argument
void setColor(int red, int green, int blue, bool led_status)
{
  if(!led_status)      // return if led status is off
  {
    return;
  }
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

// 'boot' leds showing multiple colors
void boot()
{
  if(!led_status)     // return if led status is off
  {
    return;
  }
  for(int x = 0; x <= 255; x++)
  {
    setColor(255 - x, x, x + 2, true);
    delay(10);
  }
  delay(180);
  setColor(0, 255, 0, true);
  delay(180);
  setColor(0, 0, 0, true);
  delay(180);
  setColor(0, 255, 0, true);
  delay(180);
  setColor(0, 0, 0, true);
  delay(180);
  setColor(0, 255, 0, true);
  // make sure it's ON
  set_servo(SERVO_ON_POSITION);     
  // while led is green
  setColor(0, 0, 0, true);
}

// update the led status
void update_led_status()
{
  // LED status disabled
  // -----------------------------------------
  if(digitalRead(LED_STATUS_BTN_PIN) == HIGH)
  {
    led_status = false;
    setColor(0, 0, 0, true);    // disable led
  }
  else
  {
    // if the status was false, make a blink
    if(!led_status)
    {
      setColor(0, 255, 0, true);
      delay(180);
      setColor(0, 0, 0, true);
      delay(180);
      setColor(0, 255, 0, true);
      delay(180);
      setColor(0, 0, 0, true);     
    }
    led_status = true;
  }
}

// update the led status - argument before boot
void update_led_status(boolean before_boot)
{
  if(digitalRead(LED_STATUS_BTN_PIN) == HIGH)
  {
    led_status = false;
    setColor(0, 0, 0, true);    // disable led
  }
  else
  {
    // if the status was false, make a blink
    if((!led_status) && (!before_boot))
    {
      setColor(0, 255, 0, true);
      delay(180);
      setColor(0, 0, 0, true);
      delay(180);
      setColor(0, 255, 0, true);
      delay(180);
      setColor(0, 0, 0, true);     
    }
    led_status = true;
  }
}

// handle connection alive variable
void handle_alive_con()
{
  // if connection is alive, do the check
  if(alive_connection)
  {
    // if connection is 'alive' check time
    int diff = millis() - alive_time;
    
    // check time difference
    if(diff > ALIVE_TIME_DIFF)
    {
      // if diff is too big, stop the connection
      alive_connection = false;   // disabled
      if(DEBUG)
      {
        Serial.println("alive disabled");
      }
    }
  }
}

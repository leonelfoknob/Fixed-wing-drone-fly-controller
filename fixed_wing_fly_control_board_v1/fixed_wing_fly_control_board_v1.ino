#include <IBusBM.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

#define throtle_pin 11
#define aileron_left_pin 10
#define aileron_right_pin 9
#define ruder_pin 6
#define elevator_pin 5

Servo throtle; // create servo object to control a servo
Servo aileron_left;
Servo aileron_right;
Servo ruder;
Servo elevator;

int elevator_val = 0;
int throttle_val = 0;
int ruder_val = 0;
int aileron_val = 0;

int safestate_func(){ // dont forget to give state safe value for all when signal i lose
elevator_val = 0;
throttle_val = 0;
ruder_val = 0;
aileron_val = 0;
}

IBusBM IBus; // IBus object

void setup() {
  // Serial.begin(115200);   // remove comment from this line if you change the Serial port in the next line

  IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required

  throtle.attach(throtle_pin);  // attaches the servo on pin 9 to the servo object
  aileron_left.attach(aileron_left_pin);
  aileron_right.attach(aileron_right_pin);
  ruder.attach(ruder_pin);
  elevator.attach(elevator_pin);

  Serial.println("Start IBus2PWM");

  lcd.init();
  lcd.backlight();
  lcd.begin (16, 2);
}

int saveval=0;

void loop() {
  int val;
  ruder_val = IBus.readChannel(0); // get latest value for servo channel 1
  elevator_val = IBus.readChannel(1);
  throttle_val = IBus.readChannel(2); 
  aileron_val = IBus.readChannel(3); 

  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.setCursor(2, 0);
  lcd.print(throttle_val);

  lcd.setCursor(7, 0);
  lcd.print("E: ");
  lcd.setCursor(9, 0);
  lcd.print(elevator_val);

  lcd.setCursor(0, 1);
  lcd.print("R: ");
  lcd.setCursor(2, 1);
  lcd.print(ruder_val);

  lcd.setCursor(7, 1);
  lcd.print("A: ");
  lcd.setCursor(9, 1);
  lcd.print(aileron_val);

  throtle.writeMicroseconds(throttle_val);
  aileron_left.writeMicroseconds(aileron_val);
  aileron_right.writeMicroseconds(aileron_val);
  ruder.writeMicroseconds(ruder_val);
  elevator.writeMicroseconds(elevator_val);
  

  /*if (saveval != val) {
    Serial.println(val); // display new value in microseconds on PC
    saveval = val;    
    myservo.writeMicroseconds(val);   // sets the servo position 
  }*/
  
  delay(1);
}
 /***********library : https://github.com/bmellink/IBusBM *******/

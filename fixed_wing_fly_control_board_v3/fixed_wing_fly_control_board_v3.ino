/*
 * This is our third version of fixed wing drone's control board where the flysky command allow us to 
 * control it and with conbine with gyroscop it it capable to stabilisite itself in roll (X-axis) and pitch (y-axis) 
 * angle. using arduino nano ang mpu6050 6DOF GYROSCOP/ACCELEROMETER this version of code conbine kalman filter and 
 * pid algorithm to make drone stabilize itself autonomously (only roll and pitch angle )
 * 
 * i'll make code commented more detailed for easy comprehension.
 * 
 */
#include <Adafruit_MPU6050.h> //library for mpu6050
#include <Adafruit_Sensor.h> //library for mpu6050
#include <Wire.h> //library for allow  mpu6050 i2c communication
#include <SimpleKalmanFilter.h> //kalman filter algorithm 
#include <IBusBM.h> // flysky command library
#include <Servo.h> // to allow controller to send command to motor
#include <LiquidCrystal_I2C.h> // optionaly lcd screen library

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // create lcd screen object 

Adafruit_MPU6050 mpu; //create mpu 6050 object

SimpleKalmanFilter xKalmanFilter(0.1, 0.01, 0.02); //pitch initialization for kalman filter
SimpleKalmanFilter yKalmanFilter(0.1, 0.01, 0.02); //roll initialization for kalman filter
SimpleKalmanFilter zKalmanFilter(0.1, 0.01, 0.02); //roll initialization for kalman filter


//int angle;  // Variable to store the servo angle
int gyro_pitch_input; // Variable to store the gyro pitch input
int gyro_roll_input; // Variable to store the roll pitch input
int acc_pitch_input; // Variable to store the accelerometer pitch input
int acc_roll_input; // Variable to store the accelerometer roll input
int pitch_output; // Variable to store the final pitch control output
int roll_output; // Variable to store the final roll control output
int Kp = 4;  // Proportional gain for the pid control
int Kd = 2;  // Derivative gain for the pid control
int prev_errorp = 0;
int prev_errorr = 0;

int desired_point_roll = -4; //don't forget to change this value to adapt it to your vehicule
int desired_point_pitch = 4; //don't forget to change this value to adapt it to your vehicule

// definition of motor pin

#define throtle_pin 11 
#define aileron_left_pin 10
#define aileron_right_pin 9
#define ruder_pin 6
#define elevator_pin 5

// definition of motor object 
Servo throtle;
Servo aileron_left;
Servo aileron_right;
Servo ruder;
Servo elevator;

// default value of motor control signal
int elevator_val = 1500;
int throttle_val = 1000;
int ruder_val = 1500;
int aileron_val = 1500;

//angle that will send to elevator(pid_pitch_angle) and aileron (pid_roll_angle) motor after pid tuning
int pid_pitch_angle = 0;
int pid_roll_angle = 0;

IBusBM IBus; // IBus object for command control

void setup() {                                              
  // Serial.begin(115200);   // remove comment from this line if you change the Serial port in the next line

  IBus.begin(Serial);    // iBUS connected to Serial0 (rx pin1 on arduino nano on our case)

  // attaches the motor to the motor pin
  throtle.attach(throtle_pin);  
  aileron_left.attach(aileron_left_pin);
  aileron_right.attach(aileron_right_pin);
  ruder.attach(ruder_pin);
  elevator.attach(elevator_pin);

//initilize lcd screen
  lcd.init();
  lcd.backlight();
  lcd.begin (16, 2);

  // Try to initialize mpu 6050 sensor!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.println("");
  delay(10);
  
}

void loop() {

  //get flysky command value
  ruder_val = IBus.readChannel(0);
  elevator_val = IBus.readChannel(1);
  throttle_val = IBus.readChannel(2); 
  aileron_val = IBus.readChannel(3); 

  //get mpu6050 gyroscop accelerometer value
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
 // pass accelerometer value on kalman filter           
    float estimated_x = xKalmanFilter.updateEstimate(a.acceleration.x);
    float estimated_y = yKalmanFilter.updateEstimate(a.acceleration.y);
    float estimated_z = zKalmanFilter.updateEstimate(a.acceleration.z);
//calcul of pitch and roll value after kalman filter to give it to a pid control
    acc_pitch_input = -(atan2(estimated_x, sqrt(estimated_y*estimated_y + estimated_z*estimated_z))*180.0)/M_PI;
    acc_roll_input = (atan2(estimated_y, estimated_z)*180.0)/M_PI;
//uncomment the following line if you want to print it
   /*Serial.print("acc_pitch : ");Serial.print(acc_pitch_input);
    Serial.print("  | acc_roll : ");Serial.println(acc_roll_input);*/
            
 // Calculate the pitch angle rate using the gyro
     gyro_pitch_input = g.gyro.y / 131.0;
     gyro_roll_input = g.gyro.x / 131.0;
            
 // Calculate the error between the desired angle and the current angle
     int errorp = acc_pitch_input - gyro_pitch_input;
     int errorr = acc_roll_input - gyro_roll_input;
// Calculate the derivative of the error
     int d_errorp = errorp - prev_errorp;
     int d_errorr = errorr - prev_errorr;
            
// Store the current error as the previous error for the next iteration
     prev_errorp = errorp;
     prev_errorr = errorr;
// final pitch and roll value after pid             
     pitch_output = Kp * errorp + Kd * d_errorp;
     roll_output = Kp * errorr + Kd * d_errorr;

            /* refert to the following line to make comprehention easy it can be different for you it depend of how you put mpu6050 on your uav
             * if roll_output > 0 plane fall in right side
             * if roll_output < -8 plane fall in left side
             * * pitch_output > 0 plane fall in front side
             * * pitch_output < -8 plane fall in back side
             */
//uncomment the following line if you want to print final pitch and roll value
        /*Serial.print("acc_pitch : ");Serial.print(pitch_output);
        Serial.print("  | acc_roll : ");Serial.println(roll_output);*/
            
// Limit the pitch and roll control output to the range of the servo
            pid_pitch_angle = map(pitch_output,-100, 100, 1000, 2000);
            pid_roll_angle = map(roll_output,-200, 200, 2000, 1000);

//control aileron left right and stabilizide it if it fall
            if(aileron_val < 1400 || aileron_val > 1600){
             aileron_val = aileron_val;
            }
            else{
              if (roll_output > 0 || roll_output<-8){
                aileron_val = pid_roll_angle;
              }
            }

// control elevator front back and stabilizide it if it fall
            if(elevator_val < 1400 || elevator_val > 1600){
             elevator_val = elevator_val;
            }
            else{
              if (pitch_output > 0 || pitch_output<-8){
                elevator_val = pid_pitch_angle;
              }
            }


            /*Serial.print("acc_pitch : ");Serial.print(acc_pitch_input);
            Serial.print("  | acc_roll : ");Serial.println(acc_roll_input);*/

//write value to motor

  throtle.writeMicroseconds(throttle_val); //roll
  aileron_left.writeMicroseconds(aileron_val); //roll
  aileron_right.writeMicroseconds(aileron_val);
  ruder.writeMicroseconds(ruder_val);
  elevator.writeMicroseconds(elevator_val); //pitch
// print controller command on lcd, this is just for debugin because when fly controler is pluged i cannot access to serial port on arduino nano
  lcd_print_val();
             
}

//function for lcd screen 

void lcd_print_val(){
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
}

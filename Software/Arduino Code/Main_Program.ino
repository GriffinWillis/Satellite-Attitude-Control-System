#include "Wire.h"
#include "I2Cdev.h"
#include <PID_v1.h> // Variables for gyro
double roll_deg;
//Variables for PID
double input;
double output;
double setpoint;
double Kp = 5, Ki = 0, Kd = 0;
PID PID_controller(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double pwmSignal;

// Variables for motor
const int inaPin = 13; 
const int inbPin = 9; 
const int pwmPin = 11; 
const int diagaPin = 10; 
const int diagbPin = 12; 
const int buttonPin = 2; 
const int trimPin = A0; 
int i = 0;

// ================================================================ // === INITIAL SETUP === // ================================================================
void setup () { 
  //Setup for PID 
setpoint = 20;
PID_controller.SetMode(AUTOMATIC); 
PID_controller.SetTunings (Kp, Ki, Kd); 
PID_controller.SetOutputLimits(-255, 255);

// join I2C bus (I2Cdev library doesn’t do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE 
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true); 
#endif

// I/O:s for motor pins
pinMode(buttonPin, INPUT); 
pinMode(inaPin, OUTPUT); 
pinMode(inbPin, OUTPUT); 
pinMode(pwmPin, OUTPUT);
pinMode(diagaPin, INPUT);
pinMode(diagbPin, INPUT);
pinMode (trimPin, INPUT);
}

// ================================================================ // === MAIN PROGRAM LOOP === // ================================================================
void loop () {
input = roll_deg = 45; //multiply by 180/M_PI to convert from radians if needed
//Serial.println(roll_deg);
PID_controller.Compute();

Serial.print(input); 
Serial.print("\t");
Serial.println(output);

if(output >= 0){
pwmSignal = output;
digitalWrite(inaPin, LOW); //CW direction of motor. 
digitalWrite(inbPin, HIGH);
}
else{
pwmSignal = -1*output;
digitalWrite(inaPin, HIGH); //CCW direction of motor. 
digitalWrite(inbPin, LOW);
}

analogWrite(pwmPin, pwmSignal);
}

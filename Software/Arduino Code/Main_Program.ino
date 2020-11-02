/* KEX13 
* PID Regulated Balancing Cube 
* PID Reglerad Balanserande Kub 
* Date 2020−05−29 
* Written by: Sebastian Brandmaier och Denis Ramsden 
* Examiner : Nihad Subasic 
* TRITA−ITM−Ex: 2020:33 
* Course code : MF133X 
* 
* Bachelor thesis at KTH in mechatronics 
* 
* This code tries to balance a cube with help of reaction wheels. 
* The components for this used is: 
* − 1 pcs Arduino UNO 
* − 1 pcs 24V brushed DC motor 
* − 1 pcs motor driver VNH3SP30 
* − 2 pcs hall effect sensor 
* − 1 pcs gyroscope MPU6050 
*/ 

// Includes
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h> // Include PID library 

// Defines
#define SAMPLE_RATE 10  // in Hz

// Constructors
Madgwick filter;  // Madgwick algorithm for roll, pitch, and yaw calculations

// Varibles for motor control
const int inaPin = 13; 
const int inbPin = 9; 
const int pwmPin = 11; 
const int diagaPin = 10; 
const int diagbPin = 12; 
const int buttonPin = 2; 
const int trimPin = A0; 
int on = 0; 
int i = 0; 
double pwmSignal; 

// Motor constants 
double k2 = 0.057; 
double R = 2; 

// Varible us for the gyro to be able to stabilze 
double Time; 

// PID−varibles 
double input; 
double output; 
double setpoint; 

//Different PID setups 
double Kp = 50, Ki = 3, Kd = 0.3; // PID−parameters 

float roll_deg; // stores the angle the cube is tilted 
float real_output; // Adjusting outputsignal 

// Define PID controller 
PID PID_controller(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

/* For Hall effect sensors
// Pin for reading hall effect sensor values 
int magnetSensor1 = A3; 
int magnetSensor2 = A2; 

// Store values from hall effect sensors 
int Sensorval1; 
int Sensorval2; 

// Help variables to calculate speed direction 
int state1; 
int state2; 
int state3; 

double oldTime; // Define a varible that current stores time. 
double currentTime; // Define a varible that old stores time. 
double dx; // Stores a time difference. 
double radVelocity; // Stores velocity in radians.
double velocity; // stores velocity in rpm. 
double U; // Voltage, number 0−255, 255 results in a 24V of output voltage. 
*/

// Code below is written by Jeff Rowberg and can be found at https://github.com/jrowberg/i2cdevlib 

#include "I2Cdev.h" 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE 
  #include "Wire.h" 
#endif 

#define LED_PIN 13 
bool blinkState = false;

// ================================================================ 
// === INITIAL SETUP === 
// ================================================================ 

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically) 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE 
  Wire.begin(); 
  // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Wire.setClock(400000); // 400kHz I2C clock. 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE 
  Fastwire::setup(400, true); 
#endif 

// initialize serial communication
Serial.begin(115200);  // initialize serial bus (Serial Monitor) 
while(!Serial); // wait for serial initialization
Serial.print("LSM6DS3 IMU initialization ");
if(IMU.begin()){  // initialize IMU
  Serial.println("completed successfully.");
} else {
  Serial.println("FAILED.");
  IMU.end();
  while(1);
  }
  Serial.println();
  filter.begin(SAMPLE_RATE);  // initialize Madgwick filter

// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio 
// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to 
// the baud timing being too misaligned with processor ticks. You must use 
// 38400 or slower in these cases, or use some kind of external separate 
// crystal solution for the UART timer. 
 
// Code written by Jeff Rowberg ends here. 

Time = micros(); // Return time since the arduino was powered on. 

// Setup pins for motor control as specified in datasheet for the VNH3SP30. 
pinMode(buttonPin, INPUT); 
pinMode(inaPin, OUTPUT); 
pinMode(inbPin, OUTPUT); 
pinMode(pwmPin, OUTPUT); 
pinMode(diagaPin, INPUT); 
pinMode(diagbPin, INPUT); 
pinMode(trimPin, INPUT); 

// Setup for PID controller 
PID_controller.SetMode(AUTOMATIC); 
PID_controller.SetTunings(Kp,Ki,Kd); 
PID_controller.SetOutputLimits(-255, 255);

// Vinkelhastighet 
//oldTime = micros(); // Stores the time when the program is start running. 
//state3 = 0; // Sets initial state to 0. 
} 


// ================================================================ 
// === MAIN PROGRAM LOOP === 
// ================================================================ 

void loop() {
    // blink LED to indicate activity 
    blinkState = !blinkState; 
    digitalWrite(LED_PIN, blinkState);

    static unsigned long previousTime = millis();
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 1000/SAMPLE_RATE){
      // printValues();
      printRotationAngles();
      previousTime = millis();
    }
  
  roll_deg = filter.getRoll() * 180/M_PI+86.5; // Stores data from the gyro in a varible, was 
    // adjusted so the angle is 0 when the cube is in horizontal position. 
  
  input = roll_deg; // Uses angle as input signal for the PID controller. 

/* 
  //Calculation of the angular speed of the reaction wheels using Hall effect sensors.
  
  //Read value from hall effect sensor 1 
  Sensorval1 = analogRead(magnetSensor1); // 560 55 
  
  //Read value from hall effect sensor 2 
  Sensorval2 = analogRead(magnetSensor2); // 650 630 

      if(state3 == 1){ 
      if(state1 == 1 && Sensorval1 > 500){ //If sensor 1 is sensing a magnet, used to calculate direction 
        state2 = 1; 
        state3 = 0; 
        //Serial.println(state2);
      } 
    
      if(state1 == 1 && Sensorval2 > 600){ //If sensor 2 is sensing a magnet, used to calculate direction 
        state2 = 2; 
        state3 = 0; 
        //Serial.println(state2); 
      }
    } 

    if(Sensorval1 > 500 && Sensorval2 > 600){ // If both sensor is sensing a magnet 
      if (state1 == 1) { //If both sensors has been low, to avoid measure many times at the same magnet. 
        currentTime = micros(); // get time 
        dx = currentTime - oldTime; // calculate time difference 
        radVelocity = 1000000*3.1415/dx; // calculate angular speed 
        oldTime = currentTime; // set current time to new time 
    
        state1 = 0; 
        state3 = 1;
      }
    } 

    if(Sensorval1 < 500 && Sensorval2 < 600){ // If both sensors are low, change state to 1. 
      state1 = 1; 
    } 
  
  if(state2 == 1){ // State2 stores the direction that the wheel is spinning. 
    velocity = -1*radVelocity; // Neagtive direction 
  }
  if(state2 == 2){
    velocity = radVelocity; // Positive. 
  } 
*/

  // Code for PID 
  setpoint = 45; // 0 degrees offset from the equillibrium 
  
    if(micros()-Time > 8000000){ // Needed a delay so that the IMU could stabilaze, using delay() caused overflow. 
      PID_controller.Compute(); // Compute new output value 
      // U=k2*velocity+(R/k2)*output; // Compute voltage the gives a certain torque, depending on velocity speed. 
  
    // if((input>46) || (input<44)){
      if(output >= 0){ // Since arduino only works with positive current, the direction was swapped instead. 
      pwmSignal = output; 
      digitalWrite(inaPin, LOW); //CW direction of motor. 
      digitalWrite(inbPin, HIGH); 
    } else {
      pwmSignal = -1*output; 
      digitalWrite(inaPin, HIGH); //CCW direction of motor. 
      digitalWrite(inbPin, LOW); 
    }
  } 

Serial.print(roll_deg); // Print the angle in serial monitor. 
Serial.print(" "); 
Serial.print(input); // Print input 
Serial.print(" "); 
Serial.println(output); // Print output 

// Output pwm signal. 
analogWrite(pwmPin, pwmSignal); 
}

// Prints IMU values.
void printValues(){
   char buffer[8];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   // Retrieve and print IMU values
   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      Serial.print("ax = ");  Serial.print(dtostrf(ax, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("ay = ");  Serial.print(dtostrf(ay, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("az = ");  Serial.print(dtostrf(az, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("gx = ");  Serial.print(dtostrf(gx, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gy = ");  Serial.print(dtostrf(gy, 7, 1, buffer));  Serial.print(" °/s, ");
      Serial.print("gz = ");  Serial.print(dtostrf(gz, 7, 1, buffer));  Serial.println(" °/s");
   }
}

// Prints rotation angles (roll, pitch, and yaw) calculated using the
// Madgwick algorithm.
// Note: Yaw is relative, not absolute, based on initial starting position.
// Calculating a true yaw (heading) angle requires an additional data source,
// such as a magnometer.
void printRotationAngles(){
   char buffer[5];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)){
      filter.updateIMU(gx, gy, gz, ax, ay, az);  // update roll, pitch, and yaw values

      // Print rotation angles
      Serial.print("Roll = ");  Serial.print(dtostrf(filter.getRoll(), 4, 0, buffer)); Serial.print(" °, ");
      Serial.print("Pitch = ");  Serial.print(dtostrf(filter.getPitch(), 4, 0, buffer)); Serial.print(" °, ");
      Serial.print("Yaw = ");  Serial.print(dtostrf(filter.getYaw(), 4, 0, buffer)); Serial.println(" °");
   }
}

/*
* Resource:
* PID Regulated Balancing Cube 
* PID Reglerad Balanserande Kub 
* Date 2020−05−29 
* Written by: Sebastian Brandmaier och Denis Ramsden 
* 
* 
* This code tries to control a satellite's attitude with a reaction wheel. 
* The components for this used is: 
* − 1 pcs Arduino UNO WiFi Rev 2
* − 1 pcs 12V brushless DC motor 
* − 1 pcs motor driver  
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
const int buttonPin = 2;  
int on = 0; 
int i = 0; 
double pwmSignal; 

// Motor constants 
//double k2 = 0.057; 
//double R = 2; 

// Varible us for the gyro to be able to stabilze 
double Time;
int buffersize=1000;     // Amount of readings used to average, make it higher to get more precision but sketch will be slower (default:1000)
int mean_pitch,state=0;
int pitch_offset;

// PID−varibles 
double input; 
double output; 
double setpoint; 

//Different PID setups 
double Kp = 50, Ki = 3, Kd = 0.3; // PID−parameters 

float pitch_deg; // stores the angle the cube is tilted 
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

bool blinkState = false;

// ================================================================ 
// === INITIAL SETUP === 
// ================================================================ 

void setup() {
  Wire.begin(); 
  Wire.setClock(400000); // 400kHz I2C clock. 
// initialize serial communication
Serial.begin(9600);  // initialize serial bus (Serial Monitor) 
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

Time = micros(); // Return time since the arduino was powered on. 

// Setup pins for motor control as specified in datasheet for the VNH3SP30. 
pinMode(buttonPin, INPUT); 
pinMode(inaPin, OUTPUT); 
pinMode(inbPin, OUTPUT); 
pinMode(pwmPin, OUTPUT); 

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
    digitalWrite(LED_BUILTIN, blinkState);

    static unsigned long previousTime = millis();
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 1000/SAMPLE_RATE){
      printValues();
      printRotationAngles();
      previousTime = millis();
    }

  if (state==0){
    Serial.println("\nCalculating offset...");
    calibration();
    Serial.print("pitch_offset= ");
    Serial.println(pitch_offset);
    state++;
    delay(1000);
  }
  
  pitch_deg = filter.getPitch() - pitch_offset; // Stores data from the gyro in a variable, was 
    // adjusted so the angle is 0 when the satellite body is in its initial position. 
  
  input = pitch_deg; // Uses angle as input signal to the PID controller. 

  // Code for PID 
  setpoint = 45; // Desired orientation set manually by the user
  
    if(micros()-Time > 8000000){ // Needed a delay so that the IMU could stabilaze, using delay() caused overflow. 
      PID_controller.Compute(); // Compute new output value 
  
    // if((input>46) || (input<44)){
      if(output >= 0){ // Since arduino only works with positive current, the direction was swapped instead. 
      pwmSignal = output; 
      digitalWrite(inaPin, LOW); // CW direction of motor. 
      digitalWrite(inbPin, HIGH); 
    } else {
      pwmSignal = -1*output; 
      digitalWrite(inaPin, HIGH); // CCW direction of motor. 
      digitalWrite(inbPin, LOW); 
    }
  } 

// Output pwm signal. 
analogWrite(pwmPin, pwmSignal); 

//Serial.print(roll_deg); // Print the angle in serial monitor. 
//Serial.print(" "); 
//Serial.print(input); // Print input 
//Serial.print(" "); 
//Serial.println(output); // Print output 
}

// ================================================================ 
// === FUNCTIONS === 
// ================================================================ 

// Prints IMU values.
void printValues(){
   char buffer[8];    // string buffer for use with dtostrf() function
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   // Retrieve and print IMU values
   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      //Serial.print("ax = ");  Serial.print(dtostrf(ax, 4, 1, buffer));  Serial.print(" g, ");
      //Serial.print("ay = ");  Serial.print(dtostrf(ay, 4, 1, buffer));  Serial.print(" g, ");
      //Serial.print("az = ");  Serial.print(dtostrf(az, 4, 1, buffer));  Serial.print(" g, ");
      Serial.print("gx = ");  Serial.print(dtostrf(gx, 7, 1, buffer));  Serial.print(" °/s, ");   // gx or gy will be the rotation speed of satellite
      Serial.print("gy = ");  Serial.print(dtostrf(gy, 7, 1, buffer));  Serial.print(" °/s, ");
      //Serial.print("gz = ");  Serial.print(dtostrf(gz, 7, 1, buffer));  Serial.println(" °/s");
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
      //Serial.print("Roll = ");  Serial.print(dtostrf(filter.getRoll(), 4, 0, buffer)); Serial.print(" °, ");
      Serial.print("Pitch = ");  Serial.print(dtostrf(filter.getPitch(), 4, 0, buffer)); Serial.println(" °, ");
      //Serial.print("Yaw = ");  Serial.print(dtostrf(filter.getYaw(), 4, 0, buffer)); Serial.print(" °");
   }
}

void calibration(){
  long i=0,buff_pitch=0;

  while (i<(buffersize+101)){
   float ax, ay, az;  // accelerometer values
   float gx, gy, gz;  // gyroscope values

   if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()
      && IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)){
      filter.updateIMU(gx, gy, gz, ax, ay, az);}  // update roll, pitch, and yaw values
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_pitch=buff_pitch+filter.getPitch();
    }
    if (i==(buffersize+100)){
      mean_pitch=buff_pitch/buffersize;
      pitch_offset=-1*mean_pitch;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

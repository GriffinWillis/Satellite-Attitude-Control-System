/*
  WiFi Web Server
  
  This example is written for a network using WPA encryption.  For WEP, change
  the Wifi.begin() call accordingly.
  Replace yourNetwork and secretPassword in the Network section with the proper
  credentials for your network.

  Circuit:
  * LEDs attached to pins 2, 3, and 5

  Created by Tom Igoe on 11/25/2012
  Modified by John Woolsey on 12/04/2018
*/


#include <SPI.h>
#include <WiFiNINA.h>  // for use with Arduino Uno WiFi Rev2

// Network
char ssid[] = "Sarah's Iphone";     // your network SSID (name)
char pass[] = "x10foswswvpee";  // your network password
//char ssid[] = "Jess's iPhone";     // your network SSID (name)
//char pass[] = "demodemo";  // your network password
int status = WL_IDLE_STATUS;
WiFiServer server(80);


        // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
        // for both classes must be in the include path of your project
        #include "I2Cdev.h"
        
        #include "MPU6050_6Axis_MotionApps_V6_12.h"
        //#include "MPU6050.h" // not necessary if using MotionApps include file
        
        // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
        // is used in I2Cdev.h
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        #include "Wire.h"
        #endif
        
        // class default I2C address is 0x68
        // specific I2C addresses may be passed as a parameter here
        // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
        // AD0 high = 0x69
        MPU6050 mpu;
        //MPU6050 mpu(0x69); // <-- use for AD0 high

        #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
        #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
        bool blinkState = false;
        
        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        
        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        
        // packet structure for InvenSense teapot demo
        uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

        // ================================================================
        // ===               INTERRUPT DETECTION ROUTINE                ===
        // ================================================================
        
        volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
        void dmpDataReady() {
          mpuInterrupt = true;
        }

        // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
        // pitch/roll angles (in degrees) calculated from the quaternions coming
        // from the FIFO. Note this also requires gravity vector calculations.
        // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
        // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
        #define OUTPUT_READABLE_YAWPITCHROLL

        // ================================================================
        // ===                     S.A.C. Defines                       ===
        // ================================================================
        
        // Includes
        #include <PID_v1.h>
        
        // Defines
        #define SAMPLE_RATE 10
        
        // Varibles for motor control
        const int inaPin = 4; 
        const int inbPin = 7; 
        const int pwmPin = 5;
        double pwmSignal;
        
        // PID−varibles 
        double input; 
        double output; 
        double setpoint; 
        
        //Different PID setups 
        //double Kp = 180, Ki = 5, Kd = 15; // PID−parameters 
        //double Kp = 100, Ki = 9, Kd = 36; // PID−parameters (0.2 o.s. 18dps for 90deg)
        double Kp = 100, Ki = 10, Kd = 60; // PID−parameters (0.2 o.s. 18dps for 90deg)
        
        double yaw_deg; // stores the angle the cube is rotated 
        double yaw = 0;
        
        double Time;
        int flag = 0;
        
        // Define PID controller 
        PID PID_controller(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
          // join I2C bus (I2Cdev library doesn't do this automatically)
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
        #endif

    Serial.begin(9600);  // initialize serial communication
        while (!Serial); // wait for Leonardo enumeration, others continue immediately
      
        // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
        // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
        // the baud timing being too misaligned with processor ticks. You must use
        // 38400 or slower in these cases, or use some kind of external separate
        // crystal solution for the UART timer.
      
        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);
      
        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      
        // wait for ready
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
      
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        mpu.setXGyroOffset(542);
        mpu.setYGyroOffset(-2);
        mpu.setZGyroOffset(5);
        mpu.setXAccelOffset(1036);
        mpu.setYAccelOffset(1167);
        mpu.setZAccelOffset(5392);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
          // Calibration Time: generate offsets and calibrate our MPU6050
          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          Serial.println();
          mpu.PrintActiveOffsets();
          // turn on the DMP, now that it's ready
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);
      
          // enable Arduino interrupt detection
          Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
          Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
          Serial.println(F(")..."));
          attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
      
          // set our DMP Ready flag so the main loop() function knows it's okay to use it
          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;
      
          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
        }
      
        // configure LED for output
        pinMode(LED_PIN, OUTPUT);

        // ================================================================
        // ===                      S.A.C. Setup                        ===
        // ================================================================
        
          Time = micros(); // Return time since the arduino was powered on.
        
          // Setup pins for motor control  
          pinMode(inaPin, OUTPUT); 
          pinMode(inbPin, OUTPUT); 
          pinMode(pwmPin, OUTPUT);
        
          // Setup for PID controller 
          PID_controller.SetMode(AUTOMATIC); 
          PID_controller.SetTunings(Kp,Ki,Kd); 
          PID_controller.SetOutputLimits(-255, 255);  // Max is 255 
        
        

  // Check for the presence of the WiFi device
  if (WiFi.status() == WL_NO_SHIELD || WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi device failed!");
    while (true);  // don't continue
  } else {
    Serial.println("WiFi device found.");
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware.");
  }

  // Attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network named: ");
    Serial.println(ssid);  // print the network name (SSID)

    // Connect to WPA/WPA2 network.  Change this line if using open or WEP network.
    status = WiFi.begin(ssid, pass);
    delay(10000);  // wait 10 seconds for connection
  }
  server.begin();     // start the web server on port 80
  printWifiStatus();  // you're connected now, so print the status
  Serial.println();
}


void loop() {
        // if programming failed, don't try to do anything
        if (!dmpReady) return;
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

          #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //    Serial.print("ypr\t");
        //    Serial.print(ypr[0] * 180 / M_PI);
        //    Serial.print("\t");
        //    Serial.print(ypr[1] * 180 / M_PI);
        //    Serial.print("\t");
        //    Serial.print(ypr[2] * 180 / M_PI);
              mpu.dmpGetAccel(&aa, fifoBuffer);
              mpu.dmpGetGyro(&gy, fifoBuffer);
        //      Serial.print("\tRaw Gyro XYZ\t");
        //      Serial.print(gy.z / 65.4);
        //    Serial.println();
        
        #endif
        
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
          }

  
  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                            // if you get a client,
    Serial.println("New Client");          // print a message to the serial port
    String currentLine = "";               // make a string to hold incoming data from the client
    while (client.connected()) {           // loop while the client is connected
      if (client.available()) {            // if there are bytes to read from the client,
        char c = client.read();            // read a byte, then
        Serial.write(c);                   // print it to the serial monitor
        if (c == '\n') {                   // if the byte is a newline character
          // If the current line is blank, you got two newline characters in a row
          // and that is the end of the client HTTP request, so send a response
          if (currentLine.length() == 0) {
            showWebPage(client);  // construct and display web page
            break;                // break out of the while loop
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        performRequest(currentLine);  // complete the client request
      }
    }

    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println();
  }

        // ================================================================
        // ===                      S.A.C. Loop                         ===
        // ================================================================
          setpoint = yaw; // User input to slew to desired orientation.
          //setpoint = 90;  // Override setpoint (for debugging).

          yaw_deg = ypr[0] * 180 / M_PI; // Variable to hold current yaw angle of the satellite.
          
          if (yaw_deg < 0) {      // Convert yaw range to 0-360deg.
            yaw_deg = yaw_deg + 360;
          }

          if (setpoint > 0) {
        
            // Code for PID 
          input = yaw_deg; // Uses angle as input signal for the PID controller.
          
          if(micros()-Time > 8000000){  // Needed a delay so that the IMU could stabilaze, using delay() caused overflow. 
              
              PID_controller.Compute();   // Compute new output value.
          
            // if((input>setpoint) || (input<setpoint)){
              if(input <= setpoint){ // Since arduino only works with positive current, the direction was swapped instead. 
              pwmSignal = output; 
              digitalWrite(inaPin, LOW); //CW direction of motor. 
              digitalWrite(inbPin, HIGH); 
            } else if (input > setpoint) {
              pwmSignal = -1*output; 
              digitalWrite(inaPin, HIGH); //CCW direction of motor. 
              digitalWrite(inbPin, LOW); 
              }
            }
          } else if (setpoint == 0) {
            pwmSignal = 0;
          }

          // Output PWM signal 
          analogWrite(pwmPin, pwmSignal); // Send PWM signal to DC motor.
          
          //  Print values
          Serial.print("Setpoint: ");
          Serial.print(setpoint, 0);
          Serial.print("°");
          Serial.print("\t");
          Serial.print("Yaw: ");
          Serial.print(yaw_deg);
          Serial.print("°");
          Serial.print("\t");
          Serial.print("Power: ");
          Serial.print(pwmSignal / 255 * 100, 0);
          Serial.print(" %");
          Serial.print("\t");
          Serial.print("Angular rate: ");
          Serial.print(gy.z / 65.4);
          Serial.print(" °/s");
          Serial.println();
          
}


void showWebPage(WiFiClient client) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a Content-Type so the client knows what's coming, then a blank line
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();

  // The content of the HTTP response follows the header
  client.println("<h1>Satellite Attitude Control</h1>");
  client.println("<table border=1 style='text-align:center'>");
  client.println("<tr><th>Component</th><th>Power</th><th colspan='4'>Set Angle</th></tr>");

  // Attitude Orientation
  client.println("<tr><td>Yaw</td>");
  client.println("<td><a href='/off'>OFF</a></td>");
  client.println("<td><a href='/45deg'>45</a></td>");
  client.println("<td><a href='/90deg'>90</a></td>");
  client.println("<td><a href='/180deg'>180</a></td>");
  client.println("<td><a href='/270deg'>270</a></td></tr>");
  client.println("</table>");
  
  // The HTTP response ends with another blank line
  client.println();

}


void performRequest(String line) {
  if (line.endsWith("GET /off")) {   // Turn off motor
    yaw = 0;
  } else if (line.endsWith("GET /45deg")) {  // Go to 45 deg
    yaw = 45;
  } else if (line.endsWith("GET /90deg")) {  // Go to 90 deg
    yaw = 90;
  } else if (line.endsWith("GET /180deg")) {  // Go to 180 deg
    yaw = 180;
  } else if (line.endsWith("GET /270deg")) {  // Go to 270 deg
    yaw = 270;
  }
}


void printWifiStatus() {
  // Print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi device's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  // Print where to go in a browser
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

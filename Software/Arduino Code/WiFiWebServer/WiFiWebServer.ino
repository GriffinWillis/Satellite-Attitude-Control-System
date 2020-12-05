/*
  WiFi Web Server

  Based on SimpleWebServerWiFi in WiFi Examples.

  A simple web server that lets you control LEDs via the web.
  This sketch will print the IP address of your WiFi device (once connected)
  to the Serial Monitor.  From there, you can open that address in a web browser
  to turn on and off the LEDs.

  If the IP address of your device is yourAddress:
  http://yourAddress                home page
  http://yourAddress/redLED/on      turns on the red LED
  http://yourAddress/redLED/off     turns off the red LED
  http://yourAddress/yellowLED/on   turns on the yellow LED
  http://yourAddress/yellowLED/off  turns off the yellow LED
  http://yourAddress/greenLED/on    turns on the green LED
  http://yourAddress/greenLED/off turns off the green LED

  This sketch works with the new Arduino Uno WiFi Rev2 board along with the
  original Arduino Uno with an Arduino WiFi Shield attached.
  Uncomment the appropriate #include line for your particular WiFi device
  configuration.
  
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
// Uncomment the appropriate include line below for your WiFi device
// #include <WiFi.h>      // for use with Arduino WiFi Shield
#include <WiFiNINA.h>  // for use with Arduino Uno WiFi Rev2


// Pin Mapping
const int    redLED = 2;
const int yellowLED = 3;
const int  greenLED = 5;


// Network
char ssid[] = "Airwave-5G-02-byz1w0xu1";     // your network SSID (name)
char pass[] = "";  // your network password
int status = WL_IDLE_STATUS;
WiFiServer server(80);


void setup() {
  Serial.begin(9600);  // initialize serial communication

  // Set pin modes
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

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
}


void showWebPage(WiFiClient client) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a Content-Type so the client knows what's coming, then a blank line
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();

  // The content of the HTTP response follows the header
  client.println("<h1>Arduino Remote Control</h1>");
  client.println("<table border=1 style='text-align:center'>");
  client.println("<tr><th>Component</th><th>Status</th><th>Control</th></tr>");

  // Red LED
  client.print("<tr><td>Red LED</td><td>");
  if (digitalRead(redLED)) {  // show colored status based on state of redLED
    client.print("<font style='color:green;'>ON</font>");
  } else {
    client.print("<font style='color:red;'>OFF</font>");
  }
  client.println("</td><td><a href='/redLED/on'>ON</a> / <a href='/redLED/off'>OFF</a></td></tr>");  // display redLED control links

  // Yellow LED
  client.print("<tr><td>Yellow LED</td><td>");
  if (digitalRead(yellowLED)) {  // show colored status based on state of yellowLED
    client.print("<font style='color:green;'>ON</font>");
  } else {
    client.print("<font style='color:red;'>OFF</font>");
  }
  client.println("</td><td><a href='/yellowLED/on'>ON</a> / <a href='/yellowLED/off'>OFF</a></td></tr>");  // display yellowLED control links

  // Green LED
  client.print("<tr><td>Green LED</td><td>");
  if (digitalRead(greenLED)) {  // show colored status based on state of greenLED
    client.print("<font style='color:green;'>ON</font>");
  } else {
    client.print("<font style='color:red;'>OFF</font>");
  }
  client.println("</td><td><a href='/greenLED/on'>ON</a> / <a href='/greenLED/off'>OFF</a></td></tr>");  // display greenLED control links

  client.println("</table>");

  // The HTTP response ends with another blank line
  client.println();
}


void performRequest(String line) {
  if (line.endsWith("GET /redLED/on")) {             // turn on red LED
    digitalWrite(redLED, HIGH);
  } else if (line.endsWith("GET /redLED/off")) {     // turn off red LED
    digitalWrite(redLED, LOW);
  } else if (line.endsWith("GET /yellowLED/on")) {   // turn on yellow LED
    digitalWrite(yellowLED, HIGH);
  } else if (line.endsWith("GET /yellowLED/off")) {  // turn off yellow LED
    digitalWrite(yellowLED, LOW);
  } else if (line.endsWith("GET /greenLED/on")) {    // turn on green LED
    digitalWrite(greenLED, HIGH);
  } else if (line.endsWith("GET /greenLED/off")) {   // turn off green LED
    digitalWrite(greenLED, LOW);
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

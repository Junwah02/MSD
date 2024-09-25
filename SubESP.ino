//last updated on 30/08/2024 8.41pm

//---------------------------------------------------------------------------------------------------------------------------
//                                      ESP32 AT BOTTOM OF SMART DUSTBIN
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
//                                           Declarations start here
//---------------------------------------------------------------------------------------------------------------------------
#define BLYNK_TEMPLATE_ID "TMPL6GHY4JWGN"
#define BLYNK_TEMPLATE_NAME "MSD Smart Dustbin"
#define BLYNK_AUTH_TOKEN "BCk0AIIhBqWlUvw2dqmyF5z1vz-6XOrk"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <ezButton.h>


//IMPORTANT: Please update Wifi credential that ESP32 is using below
const char* ssid = "JUNWAH";
const char* password = "Junwah2002";

WiFiServer server(10000);

int metal_qty = 0;  // Set all current trash quantity to 0 after esp32 is reset
int battery_qty = 0;
int electronic_qty = 0;
int generalwaste_qty = 0;

//FOR STEPPER MOTOR
int PUL = 25; //define Pulse pin for stepper motor
int DIR = 26; //define Direction pin
int ENA = 27; //define Enable Pin

//FOR METAL DETECTOR
const int metalDetector = 34;

//FOR SERVO MOTOR
const int servoPin = 14; //for servo signal output
Servo servo1;
ezButton limitSwitch1(32, INPUT_PULLUP); // touched after sliding platform opened fully
ezButton limitSwitch2(33, INPUT_PULLUP); // touched after sliding platform closed fully
bool clockwiseflag = false; //0:cw, 1:anticw

bool metal = false;
int count = 0;

// Define the MAC address of the receiver ESP32
uint8_t broadcastAddress[] = {0xcc, 0xdb, 0xa7, 0x31, 0x5a, 0x98}; // Update this with your receiver's MAC address

// Function to get the Wi-Fi channel
int32_t getWiFiChannel(const char* ssid) {
  int32_t channel = 0;
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        channel = WiFi.channel(i);
        break;
      }
    }
  }
  return channel;
}

// Declare the peer information
esp_now_peer_info_t peerInfo;

// Callback function to handle the status of transmitted packets
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//FOR TIMER
BlynkTimer timer;

//Function declarations
void myTimer();
void slidingplatform();
void sendIntegerESPNOW(int number_to_send);
void motor180deganticw();
void motor180degcw();
void motor90degcw();
void motor90deganticw();

//--------------------------------------------------------Declarations end here------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
//                                           Setup starts here
//---------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // FOR ESPNOW SETUP
  //Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Set the Wi-Fi channel to match the receiver
  const char* ssid = "JUNWAH";
  int32_t channel = getWiFiChannel(ssid);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  //Initialise ESPNOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function for data send status
  esp_now_register_send_cb(OnDataSent);

  // Register the peer (receiver)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }
  pinMode (metalDetector, INPUT);
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (2, OUTPUT);

  Serial.begin(115200);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  servo1.attach(servoPin);

  WiFi.begin(ssid, password);
  // Connect to Wi-F  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Still connecting to WiFi...");
  }

  // Start server for receiving detected trash type from laptop
  server.begin();

  Serial.println("Server started. Listening on port 10000");
  Serial.print("Please update current local IP: ");
  Serial.println(WiFi.localIP());
  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  
  // Set timer interval for calling myTimer() function
  timer.setInterval(1000L, myTimer);

  Serial.println("Setup complete. Ready to update Blynk widget.");
  Serial.print("Current metal quantity: ");
  Serial.println(metal_qty);
  Serial.print("Current battery quantity: ");
  Serial.println(battery_qty);
  Serial.print("Current electronics quantity: ");
  Serial.println(electronic_qty);
  Serial.print("Current general waste quantity: ");
  Serial.println(generalwaste_qty);



} //void setup()

//--------------------------------------------------------Setup ends here------------------------------------------------------------


//For resetting trash count if RESET Slidebar is slided

BLYNK_WRITE(V8) //Data stream V8 from Blynk app reset slidebar
{
  // any code you place here will execute when the virtual pin value changes
  int pinValue = param.asInt();
  if (pinValue == 1){
    metal_qty = 0;
    Blynk.virtualWrite(V0, 0); //reset current number of metal
  }
  else if (pinValue == 2){
    battery_qty = 0;
    Blynk.virtualWrite(V1, 0); //reset current number of battery
  }
  else if (pinValue == 3){
    electronic_qty = 0;
    Blynk.virtualWrite(V2, 0); //reset current number of electronic component
  }
  if (pinValue == 4){
    generalwaste_qty = 0;
    Blynk.virtualWrite(V3, 0); //reset current number of general waste
  }

  Serial.print("Blynk.Cloud is resetting value of trash");
}


//---------------------------------------------------------------------------------------------------------------------------
//                                             Main program starts here
//---------------------------------------------------------------------------------------------------------------------------

void loop() {
  Blynk.run();   // Allow Blynk to handle communication
  timer.run();   // Execute timer events

  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New client connected");
    
    // Wait for data from the client
    while (client.connected()) {
      if (client.available() > 0) {
        int receivedValue = 0; // for storing the instruction number (1/2/3/4)
        client.readBytes((char*)&receivedValue, sizeof(receivedValue));

        // Print the received integer value
        Serial.print("Received trash type is Type: ");
        Serial.println(receivedValue);

        //ADD IF ELSE FOR METAL DETECTOR HERE (received value might change after this)

        //sendIntegerESPNOW(receivedValue); //Update user on OLED about trash detected

        // Update metal_qty or battery_qty based on received instruction
        if (receivedValue == 1 || receivedValue == 4 || receivedValue == 5) { //metal or general waste // or no object detected

          if (analogRead(metalDetector) > 50 || receivedValue == 1){ //final detection result is metal
            sendIntegerESPNOW(1);
            metal_qty += 1;
            // Update Blynk virtual pin V0 (Metal Quantity)
            Blynk.virtualWrite(V0, metal_qty);
            Serial.print("Updated V0 (Metal Quantity) with value: ");
            Serial.println(metal_qty);
            motor180degcw();
            slidingplatform();
            motor180deganticw();
            receivedValue = 0; //reset received value for next cycle
          }

          else if (receivedValue == 4 && analogRead(metalDetector) < 50){ //final detection result remain as general waste
          sendIntegerESPNOW(4);
          generalwaste_qty += 1;
          // Update Blynk virtual pin V3 (General Waste Quantity)
          Blynk.virtualWrite(V3, generalwaste_qty);
          Serial.print("Updated V3 (General Waste Quantity) with value: ");
          Serial.println(generalwaste_qty);
          slidingplatform();
          receivedValue = 0; //reset received value for next cycle
          }

          else if (receivedValue == 5 && analogRead(metalDetector) < 50){ //final detection result remain as no trash
            sendIntegerESPNOW(5);
          }
        }

        else if (receivedValue == 2) {
          sendIntegerESPNOW(2);
          battery_qty += 1;
          // Update Blynk virtual pin V1 (Battery Quantity)
          Blynk.virtualWrite(V1, battery_qty);
          Serial.print("Updated V1 (Battery Quantity) with value: ");
          Serial.println(battery_qty);
          motor90degcw();
          slidingplatform();
          motor90deganticw();  
          receivedValue = 0; //reset received value for next cycle       
        }

         else if (receivedValue == 3) {
          sendIntegerESPNOW(3);
          electronic_qty += 1;
          // Update Blynk virtual pin V2 (Electronic Quantity)
          Blynk.virtualWrite(V2, electronic_qty);
          Serial.print("Updated V2 (Electronics Quantity) with value: ");
          Serial.println(electronic_qty);
          motor90deganticw();
          slidingplatform();
          motor90degcw();
          receivedValue = 0; //reset received value for next cycle
        }

        if (receivedValue == 0){ //If cam detected got any trash
        sendIntegerESPNOW(0); //Allow ESPCAM to start new cycle, and Inform user that segregation process is done
        }
        else if (receivedValue == 5){
          receivedValue = 0; //reset received value for next cycle
        }
} //while (client.connected())
    
    Serial.println("Client disconnected");
    Serial.println("Current metal quantity: " + String(metal_qty));
    Serial.println("Current battery quantity: " + String(battery_qty));
    Serial.println("Current electronics quantity: " + String(electronic_qty));
    Serial.println("Current general waste quantity: " + String(generalwaste_qty));

} //if client connected
}
} //loop()


void myTimer() { //Update trash (at sampling rate)
  // This function is called by the timer interval
  // Update Blynk virtual pins with current values of metal_qty and battery_qty
  Blynk.virtualWrite(V0, metal_qty);
  Blynk.virtualWrite(V1, battery_qty);
  Blynk.virtualWrite(V2, electronic_qty);
  Blynk.virtualWrite(V3, generalwaste_qty);
}

//Or can use switch to replace all these motor functions, parse an argument into the function

void motor180degcw() { //for Metal (instruction 1)
  digitalWrite(DIR, LOW);
  for (int i = 0; i < 7950; i++) // Forward 5000 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000); //delay affects rotation speed
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  Serial.println("Motor rotated 180 degree clockwise");
}

void motor180deganticw() { //for Metal (instruction 1)
  digitalWrite(DIR, HIGH);
  for (int i = 0; i < 7950; i++) // Forward 5000 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000); //delay affects rotation speed
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  Serial.println("Motor rotated 180 degree clockwise");
}

void motor90degcw() { //for Battery (instruction 2)
  digitalWrite(DIR, LOW);
  for (int i = 0; i < 3975; i++) // Forward 5000 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000); //delay affects rotation speed
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  Serial.println("Motor rotated 90 degree clockwise");
}

void motor90deganticw() { //for Electronic (instruction 3)
  digitalWrite(DIR, HIGH);
  for (int i = 0; i < 3975; i++) // Forward 5000 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000); //delay affects rotation speed
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }
  Serial.println("Motor rotated 90 degree anti-clockwise");
}



void slidingplatform() {
  // Sliding platform open and close
  bool platformOpening = true;
  
  // Move servo clockwise initially to open the platform
  servo1.write(0);
  Serial.println("SERVO IS MOVING CLOCKWISE TO OPEN PLATFORM");
  
  while (true) {
    limitSwitch1.loop(); // Check limit switch 1
    limitSwitch2.loop(); // Check limit switch 2
    
    int state1 = limitSwitch1.getState();
    int state2 = limitSwitch2.getState();  
    
    if (platformOpening && state1 == HIGH) {
      // Platform fully opened, reverse direction
      servo1.write(180);
      Serial.println("Limit switch 1 pressed. Reversing direction.");
      platformOpening = false;
      clockwiseflag = true;
    }

    if (!platformOpening && state2 == HIGH) {
      // Platform fully closed, stop the servo
      servo1.write(90); // Neutral position to stop the servo
      Serial.println("Limit switch 2 pressed. Stopping servo.");
      return;
    }

    delay(100); // Small delay to avoid rapid serial prints
  }
} //void slidingplatform()

// Function to send an integer via ESP-NOW
void sendIntegerESPNOW(int number_to_send) {
  // Send the integer via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&number_to_send, sizeof(number_to_send));

  if (result == ESP_OK) {
    Serial.println("Sent " + String(number_to_send) + " to ESPCAM with success");
  } else {
    Serial.println("Error sending the data");
  }
} //void sendIntegerESPNOW(int number_to_send)


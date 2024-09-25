//last updated on 13/09/2024 12pm

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <ESP32Servo.h>
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

//IMPORTANT: Update your WiFi Credentials here
const char* ssid = "JUNWAH";
const char* password = "Junwah2002";

// Define pin connections

//For Serial Communcation to ESPCAM
const int signalPin = 5;

//For Human Presence Detection
const int trigPin1 = 18;     // Ultrasonic sensor 1 trig pin
const int echoPin1 = 19;     // Ultrasonic sensor 1 echo pin

//For Opening Clearance Detection
const int trigPin2 = 27;     // Ultrasonic sensor 2 trig pin
const int echoPin2 = 14;     // Ultrasonic sensor 2 echo pin
const int irsensor1 = 25;     // Infrared sensor 1
const int irsensor2 = 35;     // Infrared sensor 2

//For sensors, servos, limit switches
//const int metalDetector = 16;
const int servoPin = 4;    // Servo motor pin

//ezButton limitsw_1(32);  //Upper limit swtich pin to close lid
const int switch1 = 21;    // Servo motor pin for lid closed fully
const int switch2 = 33;    // Servo motor pin for lid opened fully

// For OLED
#define I2C_SDA 22
#define I2C_SCL 23
TwoWire I2Cbus = TwoWire(0);

//For UART ESP to ESP communication
//HardwareSerial SerialPort(2);


Servo servo;               // Create a servo object

// Define constants for functions

//for OLED
// Display defines
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

enum ProcessState {DETECTION, SEGREGATION};  // Enum for process states

//for ESPNOW
int receivedInt = 0; //for storing received integer from bottom ESP32
volatile bool dataReceived = false; // Flag to indicate if data has been received

//for Human Presence Detection
const long humanDetectionDistance = 80; // Set human detection distance in cm
const long openDuration = 10000;   // Time to wait before closing the lid if human presence is gone (in ms)
const long continuousDetectionTime = 2000; //Continuous detection time for ultrasonic sensor = 2s

//for Opening Clearance Detection
long last_lid_detection_time = 0;
bool first_time = true;
bool item_put_before = false;
bool no_trash = false;
const long detectionDistance = 10; //Set Opening Clearance detection distance in cm (lesser than this will be detected)

// Function declarations
void receiveIntfromBottomESP32();
void closeLid();
void openLid();
bool checkHumanPresence();
void sendPhotoToLaptop();
int receiveObjectPresence();
void openingCleared();

// Callback function that will be triggered everytime when data is received via ESPNOW
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Copy the received data into the receivedInt variable
  memcpy(&receivedInt, incomingData, sizeof(receivedInt));

  // Set the flag to indicate that data has been received
  dataReceived = true;
  Serial.println("data is received, data received set to true");
  Serial.print("data receive in isr: ");
  Serial.println(dataReceived);
}

//Print out current receiving device MAC Address for ESPNOW communication
void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {

    pinMode(signalPin,OUTPUT);  
    digitalWrite(signalPin, LOW);
    
    Serial.begin(115200);

    //for OLED setup
    // Initialize I2C with our defined pins
    I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
    Serial.println("Initialize display");
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.printf("SSD1306 OLED display failed to initialize.\nCheck that display SDA is connected to pin %d and SCL connected to pin %d\n", I2C_SDA, I2C_SCL);
        while (true);
    }
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("starting up");
    display.display();
    

    //ESPNOW setup
    // Set the device as a Station and Soft Access Point simultaneously
    WiFi.mode(WIFI_AP_STA);

    // Set device as a Wi-Fi Station
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Setting as a Wi-Fi Station..");
    }
    Serial.print("Station IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Wi-Fi Channel: ");
    Serial.println(WiFi.channel());

    // Initialize ESP-NOW
    Serial.println("Initialising ESPNOW");
    if (esp_now_init() == ESP_OK) {
        Serial.println("ESP-NOW initialized successfully");
    } else {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register for a callback function when data is received (like registering an ISR)
    esp_now_register_recv_cb(OnDataRecv); 

    //print out current receiving device MAC Address with readMacAddress()
    Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
    readMacAddress();

    // Initialize pins
    //for Human Presence Detection
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);

    
    //for Opening Clearance Detection
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(irsensor1, INPUT);
    pinMode(irsensor2, INPUT);

    // Set limit switches with internal pull-up resistors
    pinMode(switch1, INPUT_PULLUP);  // Set pin 1 to input with internal pull-up
    pinMode(switch2, INPUT_PULLUP);  // Set pin 3 to input with internal pull-up

    closeLid();
    //closeLid(); //Close the lid initially

}

//------------------------------Main Program---------------------------------

void loop() {

    checkHumanPresence();             // Step 1: Polling loop, exits loop if Human present
    openLid();                        // Step 2: Open the dustbin lid
    openingCleared();                 // Step 3: Polling loop, check if opening cleared
    closeLid();                       // Step 4: Close the dustbin lid
    signalCam();                      // Step 5: Send signal to ESPCAM to turn on camera and send photo to laptop
    OLEDinProcess(DETECTION);         // Step 6: Update user on trash type detected
    OLEDinProcess(SEGREGATION);       // Step 7: Wait for segregation done signal from bottom esp32, and tell user segregation is in process

}//void loop()

//------------------------------End of Main Program---------------------------------

//Display dustbin current state, return to main program if state completed
void OLEDinProcess(ProcessState state) {
  dataReceived = false; //reset the flag
  int j = 0;
  
  if(no_trash == true){ //directly go back to check human presence if there was no trash at all
    return;
  }

  switch (state) {
    case DETECTION:
      while (dataReceived == 0) { //if data received, stop printing and exit loop
        Serial.println("waiting for detection completion signal");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Detecting");
        for (int i = 0; i < j; i++) {
          display.print(".");
        }
        display.display();
        delay(1000);

        j = (j + 1) % 4;  // Reset j to 0 after 3 dots
        Serial.print("data receive in Detection loop: ");
        Serial.println(dataReceived);
      } //while(!dataReceived)

      display.clearDisplay();
      display.setCursor(0, 0);      
      switch (receivedInt){
        case 1:
        display.print("Metal Detected :D");
        break;

        case 2:
        display.print("Battery Detected :D");       
        break;

        case 3:
        display.print("Electronics Detected :D");   
        break;

        case 4:
        display.print("General Waste Detected :D");     
        break;

        case 5:
        display.print("No trash Detected :(");  
        break;

        default:
        display.print("Unknown type detected.");  // Optional: handle unexpected values
        break; 
      }
      display.display();
      if (receivedInt != 4){
        delay(4000);
      }
      else if (receivedInt == 4){
      delay(1000);
      }

      digitalWrite(signalPin, LOW);
      break; //break for case(DETECTION)

    case SEGREGATION:
      if (receivedInt != 5){
      while (dataReceived == 0) { //if data received, stop printing and exit loop
        Serial.println("waiting for segregation completion signal");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Segregating");
        for (int i = 0; i < j; i++) {
          display.print(".");
        }
        display.display();
        delay(1000);
        j = (j + 1) % 4;  // Reset j to 0 after 3 dots
        Serial.print("data receive in Segregation loop: ");
        Serial.println(dataReceived);
      } //while(!dataReceived)

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Segregation done. Ready for next cycle.");
      display.display(); }
      break; //break for case(SEGREGATION)
  }
  return;

} //void OLEDinProcess(ProcessState state)


//Returns to main program if human present
bool checkHumanPresence() {
    digitalWrite(signalPin, LOW);
    unsigned long detectionStartTime = 0;
    bool continuousDetection = false;

    while (true) {
        digitalWrite(trigPin1, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin1, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin1, LOW);

        long durationA = pulseIn(echoPin1, HIGH);
        float distanceA = (durationA * 0.034) / 2; // distance is in cm

        Serial.print("Duration in micro sec: ");
        Serial.println(durationA);
        Serial.print("Distance in cm: ");
        Serial.println(distanceA);

        if (distanceA <= humanDetectionDistance) {
            if (!continuousDetection) {
                // Start timing when human is first detected within the distance
                detectionStartTime = millis();
                continuousDetection = true;
            } else {
                // Check if human has been detected continuously for 2 seconds
                if (millis() - detectionStartTime >= 1500) {
                    Serial.println("Human detected for 2 seconds. Confirmed.");
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.print("Human detected :D");
                    display.setCursor(30, 30);
                    display.print("Hello from");
                    display.setCursor(20, 40);
                    display.print("MSD Group 5 !!!");                        
                    display.display();
                    return true;
                }
            }
        } else {
            // Reset detection if human is not detected within the distance
            continuousDetection = false;
            Serial.println("continuous detection flag is resetted");
            Serial.println("Human not detected or moved out of range.");
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("Ready to take in trash, but...");            
            display.setCursor(0, 15);
            display.print("Human not detected :(");
            display.setCursor(0, 35);
            display.print("Please stay still for 2s");
            display.display();
        }

        delay(50); // Small delay to avoid hammering the sensor
    }
} //bool checkHumanPresence()

//Returns to main program if opening cleared
void openingCleared() {
    no_trash = false;
    bool item_blocking = false;
    Serial.println("CHECKING OPENING CLEARED OR NOTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT");
    last_lid_detection_time = millis(); // Initialize the time
    unsigned long item_blocking_time = 0; // Timer to check if item is blocking

    display.clearDisplay();

    while (true) {

        // Trigger ultrasonic sensor
        digitalWrite(trigPin2, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin2, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin2, LOW);
        
        long duration = pulseIn(echoPin2, HIGH);
        float distance = (duration * 0.034) / 2; // distance is in cm
        Serial.print("distance for lid: ");
        Serial.println(distance); 

        if (distance <= detectionDistance || digitalRead(irsensor1) == LOW || digitalRead(irsensor2) == LOW) {
            Serial.println("LESS THAN 30CM- Object detected by one or both sensors");
            last_lid_detection_time = millis(); // Reset time if object is detected

            if (!item_blocking) {
                item_blocking_time = millis(); // Start the blocking timer
                item_blocking = true;
            } else if (millis() - item_blocking_time >= 2000) {
                // Display "Item is blocking opening, please remove it" if detected for 2 seconds or more
                display.clearDisplay();
                display.setCursor(0, 20);
                display.print("Item is blocking");
                display.setCursor(0, 30);
                display.print("opening, please");
                display.setCursor(0, 40);
                display.print("remove it");
                display.display();
                Serial.println("Object is blocking, please remove");
            }

            item_put_before = true;
            first_time = true; // Reset the first_time flag to indicate an object was detected
            
        } else {
            Serial.println("MORE THAN 30CM - No object detected");
            if (first_time) {
                last_lid_detection_time = millis(); // Record the time when no object is detected
                first_time = false;
            }
            item_blocking = false; // Reset blocking flag if no object is detected
        }

        Serial.print("Time left: ");
        Serial.println(millis() - last_lid_detection_time);

        if (!item_put_before && (millis() - last_lid_detection_time >= 2000) && (millis() - last_lid_detection_time <= 6000)) {
            display.clearDisplay();
            display.setCursor(0, 20);
            display.print("Please dispose your trash now");
            display.display();          
        } else if (!item_put_before && (millis() - last_lid_detection_time > 6000) && (millis() - last_lid_detection_time <= 9000)) {
            display.clearDisplay();
            display.setCursor(0, 20);
            display.print("(Trash not disposed. Lid closing in 3s)");
            display.display();
        } else if (item_put_before && (millis() - last_lid_detection_time >= 2000)) {
            display.clearDisplay();
            display.setCursor(0, 20);
            display.print("(Trash is disposed. Lid closing in 3s)");
            display.display();
        }

        // Check if enough time has passed based on whether an item was detected
        if (!item_put_before && (millis() - last_lid_detection_time >= 12000)) {
            Serial.println("Item was never placed. Waited for 12 seconds.");
            Serial.println("-------------------------NEW LOOP----------------------------------");
            first_time = true;
            no_trash = true;
            return;
        } else if (item_put_before && (millis() - last_lid_detection_time >= 5000)) {
            Serial.println("Item was placed. Waited for 5 seconds.");
            Serial.println("-------------------------NEW LOOP----------------------------------");
            item_put_before = false;
            first_time = true;
            return;
        }

    } // End of while loop
} //void openingCleared2()

//signal ESP32-CAM to capture a photo
void signalCam(){
  if(no_trash == true){ //directly go back to check human presence if there was no trash at all
    return;
  }
  digitalWrite(signalPin, HIGH);
  Serial.println("PIn 5 set to high");
}

//close the dustbin lid
void closeLid(){
    servo.attach(servoPin);
    int state1 = digitalRead(switch1);
    Serial.print("state 1: ");
    Serial.println(state1);
    while (state1 == HIGH) {
        Serial.println("Lid is closing.");
        servo.write(0);    // Close the lid (servo rotates clockwise at full speed)
        state1 = digitalRead(switch1);  // Recheck the state of the limit switch
            Serial.print("state 1: ");
    Serial.println(state1);
    } 
    servo.detach();
    Serial.println("Lid closed fully");


}//void closeLid()

//open the dustbin lid
void openLid() {
    servo.attach(servoPin);
    int state2 = digitalRead(switch2);
    Serial.print("state 2: ");
    Serial.println(state2);
    while (state2 == HIGH) {
        Serial.println("Lid is opening.");
        servo.write(180);    // Close the lid (servo rotates clockwise at full speed)
        state2 = digitalRead(switch2);  // Recheck the state of the limit switch
    } 
    // Stop the servo once the lid is fully opened
    servo.detach();
    Serial.println("Lid opened fully."); 

} //void openLid()

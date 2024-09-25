#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

// Camera model AI Thinker GPIO Pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// LED Flash PIN (GPIO 4)
#define FLASH_LED_PIN 4  

#define buttonPin  15 // Example pin for reading input (GPIO 15)

// Insert your network credentials
const char* ssid = "JUNWAH";
const char* password = "Junwah2002";

// Server Address or Server IP
String serverName = "192.168.137.1";  // Change with your server computer's IP address or your Domain name
// The file path "upload_img.php" on the server folder
String serverPath = "/ESP32CAM/upload_img.php";
// Server Port
const int serverPort = 80;

// Variable to set capture photo with LED Flash
// Set to "false", then the Flash LED will not light up when capturing a photo
// Set to "true", then the Flash LED lights up when capturing a photo
bool LED_Flash_ON = true;


int buttonState = 0; // Variable to store the pin state

unsigned long buttonPressStartTime = 0; // Time when button was first pressed
const unsigned long requiredPressDuration = 3000; // 3 seconds in milliseconds


// Initialize WiFiClient
WiFiClient client;

// Function to send the captured photo to the server
void sendPhotoToServer() {
 
  String AllData;
  String DataBody;

  Serial.println();
  Serial.println("-----------");
 
  // Pre capture for accurate timing
  Serial.println("Taking a photo...");

  if (LED_Flash_ON == true) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(1000);
  }
  
  // Capture the image
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  } 

  if (LED_Flash_ON == true) digitalWrite(FLASH_LED_PIN, LOW);
  
  Serial.println("Photo capture successful.");
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");   
     
    String post_data = "--dataMarker\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"ESP32CAMCap.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String boundary = "\r\n--dataMarker--\r\n";
    
    uint32_t imageLen = fb->len;
    uint32_t totalLen = post_data.length() + boundary.length() + imageLen;
    
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=dataMarker");
    client.println();
    client.print(post_data);
  
    client.write(fb->buf, fb->len);
    client.print(boundary);
    
    esp_camera_fb_return(fb);
   
    int timeoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    Serial.println("Response : ");
    while ((startTimer + timeoutTimer) > millis()) {
      Serial.print(".");
      delay(200);
         
      // Skip HTTP headers   
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (AllData.length()==0) { state=true; }
          AllData = "";
        }
        else if (c != '\r') { AllData += String(c); }
        if (state==true) { DataBody += String(c); }
        startTimer = millis();
      }
      if (DataBody.length()>0) { break; }
    }
    client.stop();
    Serial.println(DataBody);
    Serial.println("-----------");
    Serial.println();
    
  } else {
    client.stop();
    DataBody = "Connection to " + serverName +  " failed.";
    Serial.println(DataBody);
    Serial.println("-----------");
  }
  

} //void sendPhotoToLaptop

void setup() {
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  //Serial.println();

  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(buttonPin, INPUT);

  // Setting the ESP32 WiFi to station mode
  WiFi.mode(WIFI_STA);
  //Serial.println();

  // The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router
  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int connecting_process_timed_out = 20; //--> 20 seconds timeout
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);

  // Set up the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  // 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  // 0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();

  // Set the camera resolution
  s->set_framesize(s, FRAMESIZE_SVGA); //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  Serial.println();
  Serial.println("Set camera ESP32 CAM successfully.");

}



void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    // Button is pressed, check if this is the start of the press
    if (buttonPressStartTime == 0) {
      // Record the time when the button was first pressed
      buttonPressStartTime = millis();
    } else {
      // Check if the button has been pressed for the required duration
      if (millis() - buttonPressStartTime >= requiredPressDuration) {
        // Capture and send the photo
        sendPhotoToServer();
        delay(30000); 
        }
      }
    
  } else {
    // Button is not pressed, reset the start time
    buttonPressStartTime = 0;
  }

  // Small delay to prevent excessive looping
  delay(100);
}



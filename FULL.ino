#include <WiFi.h>
#include <Wire.h>
#include <SimpleDHT.h>
#include <time.h>
#include <FirebaseESP32.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

#define WIFI_SSID "Feeder"            //connecting to "Feeder" named Router but direct to comfast is also possible
#define WIFI_PASSWORD "1234abcd"
#define API_KEY "omitted"

// Firebase credentials
#define FIREBASE_HOST "omitted.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "omitted"

// Firebase data object and configuration
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// Servo setup
Servo myServo;
#define SERVO_PIN 15
#define OPEN_POSITION 90 // Feed open position
#define CLOSE_POSITION 0 // Feed closed position

// DC motor setup
#define DC_MOTOR_PIN 14 // DC motor to water chickens

// Water level sensors setup
#define WATER_SENSOR_PIN_1 25 // Water level sensor 1 water exposed to chickens
#define WATER_SENSOR_PIN_2 26 // Water level sensor 2 water container
#define WATER_SENSOR_PIN_3 27 // Water level sensor 3 humidifier liquid

// Humidifier setup
#define HUMIDIFIER_PIN 12 // Humidifier

// DHT22 sensor setup
int pinDHT22 = 4;
SimpleDHT22 dht22(pinDHT22);

// I2C LCD setup
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
// Assuming you have an I2C display library to initialize the LCD

// Distance sensors setup (SRF05) srf05 is not working 
//#define DISTANCE_SENSOR_PIN_1 18 // Distance sensor 1 feeds exposed to chicken
//#define DISTANCE_SENSOR_ECHO_PIN_1 19 //ECHO PIN GPIO 19

//this ultrasonic sr04m-2 as well however we can get minimal data
#define RX_PIN 17  // Pin for RX (connect to TX of SR04M-2) 
#define TX_PIN 16  // Pin for TX (connect to RX of SR04M-2)
HardwareSerial mySerial(1);  // Use Serial1 for communication

// Thresholds
#define FEED_LOW_THRESHOLD 3 // In inches (for SRF05)
#define WATER_LOW_THRESHOLD 3 // Water level low threshold

// Initialize the LCD object with I2C address, columns, and rows
LiquidCrystal_I2C lcd(0x27, 16, 2); //detected I2C address (SDA GPIO21 SCL GPIO22)

bool Auto = false; //if user sets automatic feeding/sanitation/watering it sets to true
bool feed = true; // Global variable to not keep on feeding when its time

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Wire.begin(); // Initialize I2C bus
// Initialize the LCD with specified dimensions
  lcd.init();
// Turn on the backlight
  lcd.backlight();
  lcd.setCursor(0, 0); // Set cursor to column 0, row 0
  lcd.print("**LIVESTECH**");
  lcd.setCursor(0, 1); // Set cursor to column 0, row 1
  lcd.print("Looking_for_WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    //looking for wifi
    delay(1000);
  }
  lcd.setCursor(0, 1); // Set cursor to column 0, row 1
  lcd.print("                ");
  delay(1000);
  lcd.setCursor(0, 1); // Set cursor to column 0, row 1
  lcd.print("_WiFi CONNECTED_");
 
  //// Synchronize time
  const long gmtOffset_sec = 28800; // GMT+8
  const int daylightOffset_sec = 0;    // No DST
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  // Initialize Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Servo setup
  myServo.attach(SERVO_PIN);
  myServo.write(CLOSE_POSITION); // Initial position of the servo

  pinMode(DC_MOTOR_PIN, OUTPUT); // DC motor pin
  pinMode(WATER_SENSOR_PIN_1, INPUT); // Water level sensor pin exposed below
  pinMode(WATER_SENSOR_PIN_2, INPUT); // water level 
  pinMode(WATER_SENSOR_PIN_3, INPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT); // Humidifier pin

  // Distance sensors setup
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

//main loop
void loop() {
  // Checks
  checkChickenWater();//checks for chicken water (exposed)        WORKING 
  checkWaterContainer();//checks for water reserve/ container     WORKING

  checkHumidity_and_Temp();//checks humid and temp                WORKING TESTED
  checkHumidifierContainer();//checks for humidifier water level  WORKING

  checkContainerFeeds();//checks for feeds in container           WORKING

  // Functions
  manualServo();//manual user clicks button to feed               WORKING                  
  scheduleFeed();//schedule user schedules sensor to feed         WORKING

  manualWatering();//manual user clicks button to water           CODE DONE (NEED TESTING)
  autoWatering();//user auto sets water to hydrate                VERY OK BUT NOT PERFECT IDK SENSOR SOMETIMES THROW FALSE/TRUE

  manualHumid(); //sanitizes chickens                              CODE DONE (NEED TESTING)
  autoHumid(); //collaboration with DH22 automatic humidify when too dry

  //I2C DISPLAY("feeding now... etc... etc..");                     WORKING
  delay(1000);
}

//------------------------------------SENSORS
void checkChickenWater(){
  bool waterLevelLow = digitalRead(WATER_SENSOR_PIN_1) == LOW;
  Firebase.setBool(fbdo, "/sensor_exposed/chicken_water_islow", waterLevelLow);

  if (waterLevelLow) {
    Serial.println("chicken water exposed is low.");
  }
}

void checkWaterContainer(){
  bool waterContainerlow = digitalRead(WATER_SENSOR_PIN_2) == LOW;
  Firebase.setBool(fbdo, "/container/chicken_water_low", waterContainerlow);

  if (waterContainerlow) {
    //clear last
    lcd.clear(); // Clear the entire display
    lcd.setCursor(0, 0); // Set cursor to column 0, row 1
    lcd.print("Low Water");
    lcd.setCursor(0, 1); // Set cursor to column 0, row 1
    lcd.print("Please Refill");
    delay(750);
  }
}

float temperature = 0;
float humidity = 0;
void checkHumidity_and_Temp(){
  int err = SimpleDHTErrSuccess;
    if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); Serial.println(SimpleDHTErrDuration(err)); delay(2000);
    return;
  }

  // Send data to Firebase
  if (!Firebase.setInt(fbdo, "/DH22/DH22_temp", (int)temperature)) {
    Serial.println("Failed to send temperature to Firebase");
  }
  if (!Firebase.setInt(fbdo, "/DH22/DH22_humid", (int)humidity)) {
    Serial.println("Failed to send humidity to Firebase");
  }

  // Update LCD display
  lcd.clear(); // Clear the entire display
  lcd.setCursor(0, 0);
  lcd.print("Temperature: " + String(temperature) + " C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(humidity) + " %");
  delay(750);
}

void checkHumidifierContainer(){
  bool waterHumidLow = digitalRead(WATER_SENSOR_PIN_3) == LOW;
  Firebase.setBool(fbdo, "/container/chicken_humidifier_low", waterHumidLow);

  if (waterHumidLow) {
    lcd.clear(); // Clear the entire display
    lcd.setCursor(0, 0); // Set cursor to column 0, row 1
    lcd.print("Low");
    lcd.setCursor(0, 1); // Set cursor to column 0, row 1
    lcd.print("Sanitary Liquid");
    delay(750);
  }
}


///CHECK THIS THIS PHYSICALLY
void checkContainerFeeds(){
  byte data[32];  // Buffer to store incoming data
  int index = 0;   // Current position in buffer

  // Read bytes from the sensor
  while (mySerial.available()) {
    if (index < sizeof(data)) {
      data[index++] = mySerial.read();  // Store each byte in the buffer
    }
  }

  // If data was received, print and attempt to decode it
  if (index > 0) {
    // Process the data: Look for non-zero values
    int distance = 0;
    for (int i = 0; i < index; i++) {
      if (data[i] != 0x00) {  // Skip zero bytes (padding or invalid data)
        distance = data[i];  // Update distance with the valid byte value
      }
    }
    // the sr04m-2 has no support in github or anywhere else however we can still process its average data
    // the sr04m-2 uses ultrasonic or high pitch sound to process the data so it is not accurate

    //when the sensor detects the feeds are close (around 1-2 inches) it shows full.
    if (distance > 200) {
      //Serial.print("feeds full 80-99%: ");
      //Serial.println(distance);  
      Firebase.setBool(fbdo, "/container/chicken_feeds_low",false);
    }
    //around than 3-6 inches
    if (distance > 100 && distance < 200) {
      Firebase.setBool(fbdo, "/container/chicken_feeds_low",true);
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("Low Feeds");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Please Refill");
    }
    //farther than 10 inches
    if (distance < 100) {
      //Serial.print("empty 0-30%: ");
      //Serial.println(distance); 
      Firebase.setBool(fbdo, "/container/chicken_feeds_low",true);
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("Low Feeds");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Please Refill");
    }
  }
}

//--------------------------------FUNCTIONS
void manualServo() {
  if (Firebase.getBool(fbdo, "/manual/servo_on")) {
    bool isOn = fbdo.boolData();
    if (isOn) {
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("LivesTech");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Manual Feeding");
      myServo.write(OPEN_POSITION);
      delay(1000);
      myServo.write(CLOSE_POSITION);
      delay(1000);
      Firebase.setBool(fbdo, "/manual/servo_on",false);
    }
  }
}

int drop_count = 0; // Global variable to track drops
String feedTime;  // Global variable for schedule/feed_time1 
String feedTime2; // Global variable for schedule/feed_time2 
String feedTime3; // Global variable for schedule/feed_time3
void scheduleFeed() {
  // feed needs at least 1 minute interval to reset the feed boolean in order for the servo to not open everytime
  // time get from NTP server, can also do Firebase but it seems working so no need to change.
  time_t now = time(nullptr);
  struct tm* timeInfo = localtime(&now);

  char currentTime[6];
  snprintf(currentTime, sizeof(currentTime), "%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min);

  lcd.clear(); // Clear the entire display
  lcd.setCursor(0, 0); // Set cursor to column 0, row 1
  lcd.print("___LivesTech___");
  lcd.setCursor(0, 1); // Set cursor to column 0, row 1
  lcd.print(currentTime);
  delay(500);

  // Get the drop count from Firebase
  if (Firebase.getInt(fbdo, "/schedule/drop_count")) {
    drop_count = fbdo.intData();
  }
    if (Firebase.getString(fbdo, "/schedule/feed_time1")) {
      feedTime = fbdo.stringData();
      Serial.print("Feed time: ");
      Serial.println(feedTime);
    }
    if (Firebase.getString(fbdo, "/schedule/feed_time2")) {
      feedTime2 = fbdo.stringData();
      Serial.print("Feed time: ");
      Serial.println(feedTime2);
    }
    if (Firebase.getString(fbdo, "/schedule/feed_time3")) {
      feedTime3 = fbdo.stringData();
      Serial.print("Feed time: ");
      Serial.println(feedTime3);
    }

    if (feedTime != "00:00" && feedTime == currentTime && feed || feedTime2 != "00:00" && feedTime2 == currentTime && feed || feedTime3 != "00:00" && feedTime3 == currentTime && feed) {
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("___LivesTech___");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Scheduled Feeding");
      //loop opens servo based on how many drop_counts is set in firebase
      for (int i = 0; i < drop_count; i++) {
        myServo.write(OPEN_POSITION);
        delay(1000); // Wait for feed to dispense
        myServo.write(CLOSE_POSITION);
        delay(1000);
      }
      feed = false;//pause the feed
    }
    //feed needs at least 1 minute interval to reset the feed boolean in order for the servo to not open everytime
    if (feedTime != currentTime && feedTime2 != currentTime && feedTime3 != currentTime) {
      feed = true;//ready to feed if current time is feed time
    }
}//scheduleServo closing bracket

void manualWatering(){
  if (Firebase.getBool(fbdo, "/manual/water_on")){
    bool isOn = fbdo.boolData();
    if(isOn){
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("___LivesTech___");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Hydrating...");
      digitalWrite(DC_MOTOR_PIN, HIGH);
      delay(3000);
      digitalWrite(DC_MOTOR_PIN, LOW);
      delay(2000);
    Firebase.setBool(fbdo, "/manual/water_on", false ); 
    }
  }
}

bool Waterlow = false;
void autoWatering(){
  if (Firebase.getBool(fbdo,"Auto")){
    Auto = fbdo.boolData();
  }else {
    Serial.println("Failed to get Auto status from Firebase");
  }

  if (Firebase.getBool(fbdo, "/sensor_exposed/chicken_water_islow")){
    Waterlow = fbdo.boolData();
  }else {
    Serial.println("Failed to get water level status from Firebase");
  }

  if (Auto && Waterlow){
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("___LivesTech___");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Auto Water...");
      digitalWrite(DC_MOTOR_PIN, HIGH);
      delay(2000);// Run the motor for 3 seconds
      digitalWrite(DC_MOTOR_PIN, LOW);
      delay(2000);// Pause before re-checking conditions
  }
}

void manualHumid(){
  if (Firebase.getBool(fbdo, "/manual/humidify_on")){
    bool isOn = fbdo.boolData();
    if(isOn){
      lcd.clear(); // Clear the entire display
      lcd.setCursor(0, 0); // Set cursor to column 0, row 1
      lcd.print("___LivesTech___");
      lcd.setCursor(0, 1); // Set cursor to column 0, row 1
      lcd.print("Sanitizing...");
      digitalWrite(HUMIDIFIER_PIN, HIGH);
      delay(10000);
      digitalWrite(HUMIDIFIER_PIN, LOW);
      delay(2000);
      Firebase.setBool(fbdo, "/manual/humidify_on", false ); 
    }
  }
}

void autoHumid(){
  if (Firebase.getBool(fbdo,"Auto")){
    Auto = fbdo.boolData();
  }
  if (humidity < 75.0 && Auto) { // If humidity drops below 75*, turn on the humidifier
    digitalWrite(HUMIDIFIER_PIN, HIGH);
    lcd.clear(); // Clear the entire display
    lcd.setCursor(0, 0); // Set cursor to column 0, row 1
    lcd.print("___LivesTech___");
    lcd.setCursor(0, 1); // Set cursor to column 0, row 1
    lcd.print("Auto Sanitize...");
    delay(9000);
    digitalWrite(HUMIDIFIER_PIN, LOW);
    delay(2000);
  }
}












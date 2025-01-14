#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
// #include <ESP32Time.h>
#include <LiquidCrystal_I2C.h>
#include <HTTPClient.h>
#include <RTC3231.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS Channel 0 = SOLAR PANEL 1 VOLTAGE
// ADS Channel 1 = SOLAR PANEL 2 VOLTAGE
// ADS Channel 2 = ACS758 CURRENT SOLAR PANEL 1
// ADS Channel 3 = ACS758 CURRENT SOLAR PANEL 2

// Variable Definitions
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 100000

#define PGA 0.602
#define ACSOFFSET 1.67
#define ACSSENS 0.0134
#define LSB 0.0000763

// GPIO Definitions
#define sdChipSelect 5

// Wi-Fi Credentials
const char* ssid = "your_ssid";
const char* password = "your_password";

#define DATA_SAVE_SERVER "thingsboard.cloud"  // only for example
String DATA_SAVE_RESOURCE_API_KEY = "xxxxx";  // dummy api key
#define DATA_SAVE_PORT 80                     // only for example
#define DATA_SAVE_RESOURCE_PRE "/api/v1/"
#define DATA_SAVE_RESOURCE_POST "/telemetry"
#define GSM_APN_DEFAULT_1 "Internet"       //"Internet" for telkomsel
#define GSM_CONNECTION_TIMEOUT_MS 120000L  // 120 detik
#define MAX_DATA_SAVED 30

#define DATA_VALUES_KEY "values"
#define DATA_TIMESTAMP_KEY "ts"
#define DATA_READING_ID_KEY "rid"

#define DATA_VOLTAGE1_KEY "vgn1"
#define DATA_CURRENT1_KEY "crn1"
#define DATA_POWER1_KEY "pwr1"

#define DATA_VOLTAGE2_KEY "vgn2"
#define DATA_CURRENT2_KEY "crn2"
#define DATA_POWER2_KEY "pwr2"

// GLOBAL VAR
uint16_t rawAds0 = 0;
uint16_t rawAds1 = 0;
uint16_t rawAds2 = 0;
uint16_t rawAds3 = 0;

float voltageAds0 = 0;
float voltageAds1 = 0;
float voltageAds2 = 0;
float voltageAds3 = 0;

float currentAcs1 = 0;
float currentAcs2 = 0;

float powerSP1 = 0;
float powerSP2 = 0;

uint32_t lastReadingID;
uint16_t dataIndex = 0;

struct Data {  // Structure declaration
  uint32_t readingID[MAX_DATA_SAVED];
  uint64_t readingUnixTime[MAX_DATA_SAVED];
};
typedef struct {
  uint32_t readID;
  float voltageRead1;
  float currentRead1;
  float powerSP1;

  float voltageRead2;
  float currentRead2;
  float powerSP2;
  unsigned long long timeUnix;
} SensorData;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;         // Offset in seconds from GMT
const int daylightOffset_sec = 3600;  // Daylight offset in seconds (e.g., 3600 for 1 hour)

// ESP32Time rtc(0);
Adafruit_INA219 ina219;
LiquidCrystal_I2C lcd(0x27, 20, 4);
RTC_DS3231 rtc;
File dataFile;
HTTPClient http;
Adafruit_ADS1115 ads;

float current_mA = 0;
float busvoltage = 0;
float loadvoltage = 0;
char Request[1000];
DynamicJsonDocument saveData(1024);

TaskHandle_t wifiAndTimeTaskHandle;
TaskHandle_t readSensorTaskHandle;
TaskHandle_t logDataTaskHandle;
QueueHandle_t SendQueue;

SemaphoreHandle_t task1Semaphore;
SemaphoreHandle_t task2Semaphore;

SensorData savedData;
SensorData receivedData;

/**
 * Connects to the WiFi network using the provided SSID and password.
 * Continuously attempts to connect until successful, indicated by 
 * a change in the WiFi status to WL_CONNECTED. During the connection
 * attempt, a dot is printed to the serial monitor every 500 milliseconds.
 * Once connected, it prints a confirmation message to the serial monitor.
 */

void connectToWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}


/**
 * @brief Initializes the LCD display.
 * 
 * This function performs the initial setup for the LCD display by initializing it,
 * turning on the backlight, clearing any previous content, and displaying an
 * "Initializing..." message for 2 seconds.
 */

void initLcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Initializing...");
  delay(2000);
}

/**
 * @brief Reads the raw ADC value from the ADS1115.
 * @param channel The ADS1115 channel to read from.
 * @return The raw ADC value.
 */
float getRawAds(int channel) {
  return ads.readADC_SingleEnded(channel);
}

/**
 * @brief Reads the voltage from the ADS1115.
 * @param channel The ADS1115 channel to read from.
 * @return The voltage from the ADS1115 in volts.
 */
float getVoltageAds(int channel) {
  float voltage = getRawAds(channel) * LSB / PGA;
  return voltage * 10;
}

/**
 * @brief Calculates the current from a raw ADC value.
 * 
 * This function takes the raw ADC value from the ACS712 and calculates the
 * current in amps.
 * 
 * @param raw The raw ADC value from the ACS712.
 * @return The current in amps.
 */
float getCurrentFromRaw(int raw){
  float realVolt2 = (raw * LSB) / PGA;
  return (realVolt2 - ACSOFFSET) / ACSSENS;
}

/**
 * @brief Calculates the electrical power based on current and voltage.
 * 
 * This function computes the power by multiplying the provided current 
 * (in amps) and voltage (in volts).
 * 
 * @param current The current in amps.
 * @param voltage The voltage in volts.
 * @return The calculated power in watts.
 */
float getPower(float current, float voltage){
  return current * voltage;
}

/**
 * @brief Reads sensor data from the ADS1115 and ACS712 sensors.
 * 
 * This function retrieves voltage readings from four ADS1115 channels and 
 * computes the corresponding current values using the ACS712 sensors. It 
 * then calculates the power values for two channels by multiplying the 
 * current and voltage readings. The voltage, current, and power values 
 * are displayed on the serial monitor and updated on an LCD screen.
 */
void readSensor(){
  voltageAds0 = getVoltageAds(0);
  voltageAds1 = getVoltageAds(1);
  voltageAds2 = getVoltageAds(2);
  voltageAds3 = getVoltageAds(3);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  currentAcs1 = getCurrentFromRaw(getRawAds(2));
  currentAcs2 = getCurrentFromRaw(getRawAds(3));

  vTaskDelay(10 / portTICK_PERIOD_MS);

  powerSP1 = getPower(currentAcs1, voltageAds0);
  powerSP2 = getPower(currentAcs2, voltageAds1);

  Serial.println("V0: " + String(voltageAds0));
  Serial.println("V1: " + String(voltageAds1));
  Serial.println("V2: " + String(voltageAds2));
  Serial.println("V3: " + String(voltageAds3));
  Serial.println("I1: " + String(currentAcs1));
  Serial.println("I2: " + String(currentAcs2));
  Serial.println("P1: " + String(powerSP1));
  Serial.println("P2: " + String(powerSP2));

  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("V1:%.2fV", voltageAds0);
  lcd.setCursor(9, 0);
  lcd.printf("I1:%.2fA", currentAcs1);
  lcd.setCursor(0, 1);
  lcd.printf("P1:%.2fW", powerSP1);

  lcd.setCursor(0, 2);
  lcd.printf("V2:%.2fV", voltageAds1);
  lcd.setCursor(9, 2);
  lcd.printf("I2:%.2fA", currentAcs2);
  lcd.setCursor(0, 3);
  lcd.printf("P2:%.2fW", powerSP2);
}

/**
 * @brief Saves sensor data into a global structure and queues it for further processing.
 * 
 * This function retrieves the current Unix timestamp and scales it to milliseconds. It then populates 
 * a global SensorData structure with the latest readings for voltage, current, and power from two 
 * channels, along with the timestamp and a unique reading ID. The updated reading ID is incremented 
 * for future use. The function attempts to send this data to a queue for further processing, logging 
 * success or failure to the serial monitor.
 */
void saveDataSensor(){
  unsigned long long timestem = time(nullptr);
  unsigned long long timestem2 = timestem * 1000ULL;
  Serial.println("Epoch Time Before : " + String(timestem));
  Serial.println("Epoch Time After : " + String(timestem2));
  savedData.readID = lastReadingID;

  savedData.timeUnix = timestem2;
  savedData.voltageRead1 = voltageAds0;
  savedData.voltageRead2 = voltageAds1;
  savedData.currentRead1 = currentAcs1;
  savedData.currentRead2 = currentAcs2;
  savedData.powerSP1 = powerSP1;
  savedData.powerSP2 = powerSP2;

  lastReadingID++;

  if (xQueueSend(SendQueue, &savedData, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to input data to queue");
  } else {
    Serial.println("Data sent to queue successfully");
  }
}

/**
 * @brief Stores sensor data from queue to SD card log file.
 * 
 * This function opens an SD card log file in append mode and waits for data to be sent to a queue. 
 * Upon receiving data, it extracts the latest voltage, current, and power readings from two channels, 
 * along with the corresponding reading ID and Unix timestamp. It then constructs a JSON string 
 * from this data and writes it to the log file. The function also logs the data to the serial monitor.
 */
void storeData() {
  File logFile = SD.open("/log.txt", FILE_APPEND);
  if (!logFile) {
    Serial.println("Failed to open log file!");
    vTaskDelete(NULL);
  }

  if (xQueueReceive(SendQueue, &receivedData, portMAX_DELAY) == pdPASS) {
    saveData[DATA_VALUES_KEY][DATA_READING_ID_KEY] = receivedData.readID;
    saveData[DATA_VALUES_KEY][DATA_VOLTAGE1_KEY] = receivedData.voltageRead1;
    saveData[DATA_VALUES_KEY][DATA_CURRENT1_KEY] = receivedData.currentRead1;
    saveData[DATA_VALUES_KEY][DATA_POWER1_KEY] = receivedData.powerSP1;

    saveData[DATA_VALUES_KEY][DATA_VOLTAGE2_KEY] = receivedData.voltageRead2;
    saveData[DATA_VALUES_KEY][DATA_CURRENT2_KEY] = receivedData.currentRead2;
    saveData[DATA_VALUES_KEY][DATA_POWER2_KEY] = receivedData.powerSP2;

    saveData[DATA_TIMESTAMP_KEY] = receivedData.timeUnix;

    size_t jsonSize = measureJson(saveData);
    String jsonString;
    serializeJson(saveData, jsonString);

    // Write to SD Card
    logFile.println(jsonString);
    logFile.flush();
    Serial.println("Logged: " + jsonString);

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Task for initializing the Real Time Clock (RTC), connecting to WiFi, and
 *        setting the system time from an NTP server.
 *
 * This task initializes the RTC and attempts to connect to WiFi. If the RTC and
 * WiFi connections are successful, it sets the system time from an NTP server and
 * initializes the INA219 sensor. If any of these steps fail, the task halts the
 * system with a descriptive error message on the LCD screen and serial monitor.
 */
void wifiAndTimeTask(void* parameter) {
  Wire.begin((I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ));
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  if (!rtc.begin(&Wire)) {
    lcd.setCursor(0, 1);
    lcd.print("RTC FAIL!");
    Serial.println("Couldn't find RTC!");
    while (1);  // Halt system if RTC fails
  } else {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("RTC OK!");
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  connectToWifi();
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(1000);
  }

  time_t now = time(nullptr);
  DateTime dt(now);
  rtc.adjust(dt);
  Serial.print("Epoch time: ");
  Serial.println(now);

  lastReadingID = 0;
  dataIndex = 0;

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  if (!ina219.begin(&Wire)) {
    lcd.setCursor(0, 1);
    lcd.print("INA219 FAIL!");
    Serial.println("Failed to initialize INA219!");
    while (1);
  } else {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("INA219 OK!");
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Initialize SD Card
  if (!SD.begin(sdChipSelect)) {
    lcd.setCursor(0, 1);
    lcd.print("SD FAIL!");
    Serial.println("SD Card initialization failed!");
    while (1);  // Halt system if SD fails
  } else {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("SD OK!");
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  xSemaphoreGive(task1Semaphore);

  vTaskDelete(NULL);
}

/**
 * @brief      Task to read sensor data and save it to SD card.
 *
 *             This task waits for the task1Semaphore to be given, then
 *             enters an infinite loop where it reads the sensor data using
 *             readSensor() and saves it to the SD card using saveDataSensor().
 *             Then it waits for 3 seconds using vTaskDelay() before repeating
 *             the process.
 *
 * @param[in]  parameter  Unused parameter
 */
void readSensorTask(void* parameter) {
  xSemaphoreTake(task1Semaphore, portMAX_DELAY);
  for (;;) {
    readSensor();
    saveDataSensor();
    vTaskDelay(3000 / portTICK_PERIOD_MS);    // Read 3 sec delay
  }
}

/**
 * @brief Task to log sensor data to storage.
 *
 *        This task runs indefinitely, repeatedly calling the storeData() 
 *        function to log sensor data. The concept of sending data to the 
 *        cloud is indicated by the commented-out sendToCloudTask() call. 
 *        The task includes a 10 ms delay between each iteration to manage 
 *        task scheduling and CPU usage.
 *
 * @param[in] parameter Unused parameter
 */

void logDataTask(void* parameter) {
  for (;;) {
    storeData();
    // sendToCloudTask();     just concept
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  ESP_LOGI("SYSTEM LOG", "INFO: Bootup. Serial started.");
  initLcd();

  SendQueue = xQueueCreate(1000, sizeof(SensorData));
  task1Semaphore = xSemaphoreCreateBinary();
  task2Semaphore = xSemaphoreCreateBinary();

  xTaskCreate(wifiAndTimeTask, "Init all sensor, Read Time and Sync RTC", 10000, NULL, 1, &wifiAndTimeTaskHandle);
  xTaskCreate(readSensorTask, "Read Sensor", 10000, NULL, 1, &readSensorTaskHandle);
  xTaskCreate(logDataTask, "Logger", 10000, NULL, 1, &logDataTaskHandle);
}

void loop() {
  vTaskDelete(NULL);
}


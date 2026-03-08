/*
 * ESP32 Grip Strength Monitor - 6 FSR Sensors
 * 
 * Hardware:
 * - ESP32
 * - 2x ADS1115 (16-bit ADC)
 * - 6x FSR402 Force Sensors
 * - 6x 10kΩ Resistors
 * 
 * POWER CONFIGURATION:
 * - FSR Sensors: 5V (from ESP32 5V pin) - Better sensitivity
 * - ADS1115 Modules: 3.3V (from ESP32 3.3V pin) - I2C logic
 * - NO LEVEL SHIFTER NEEDED - ADS1115 can read 0-5V analog
 * 
 * Connections:
 * ADS1115 #1 (0x48):
 *   - VDD → ESP32 3.3V
 *   - GND → ESP32 GND
 *   - A0 → FSR1 (FSR powered by 5V, with 10kΩ to GND)
 *   - A1 → FSR2 (FSR powered by 5V, with 10kΩ to GND)
 *   - A2 → FSR3 (FSR powered by 5V, with 10kΩ to GND)
 *   - A3 → FSR4 (FSR powered by 5V, with 10kΩ to GND)
 *   - ADDR → GND
 *   
 * ADS1115 #2 (0x49):
 *   - VDD → ESP32 3.3V
 *   - GND → ESP32 GND
 *   - A0 → FSR5 (FSR powered by 5V, with 10kΩ to GND)
 *   - A1 → FSR6 (FSR powered by 5V, with 10kΩ to GND)
 *   - ADDR → VDD (3.3V) - CRITICAL for address 0x49
 *   
 * I2C Bus (shared):
 *   - SDA → GPIO21
 *   - SCL → GPIO22
 *   
 * FSR Wiring (each sensor):
 *   ESP32 5V → FSR Pin 1
 *   FSR Pin 2 → Junction:
 *              ├─ ADS1115 Input (A0-A3)
 *              └─ 10kΩ Resistor → GND
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ==================== CONFIGURATION ====================

// WiFi credentials
const char* WIFI_SSID = "R.X.A.X.SH";
const char* WIFI_PASSWORD = "#9vyr9cc8l";

// Backend server
const char* SERVER_URL = "http://13.127.165.226/api/data/ingest";

// Device identification
const String DEVICE_ID = "ESP32-00FF00005C8C";

// Measurement interval (milliseconds)
const unsigned long SEND_INTERVAL = 2000;  // 2 seconds

// Calibration factors (adjust based on your calibration)
const float VOLTAGE_TO_FORCE_MULTIPLIER = 3.5;
const float MIN_VOLTAGE_THRESHOLD = 0.1;  // Voltage below this = 0 kg

// ==================== HARDWARE SETUP ====================

// Two ADS1115 ADC modules
Adafruit_ADS1115 ads1;  // Address 0x48 (ADDR → GND)
Adafruit_ADS1115 ads2;  // Address 0x49 (ADDR → VDD)

// I2C addresses
const uint8_t ADS1_ADDRESS = 0x48;
const uint8_t ADS2_ADDRESS = 0x49;

// ==================== GLOBAL VARIABLES ====================

unsigned long lastSendTime = 0;
bool wifiConnected = false;

// Sensor readings
struct SensorReadings {
  float force[6];      // Individual sensor forces (kg)
  float totalGrip;     // Total grip strength (kg)
  int16_t adc[6];      // Raw ADC values
  float voltage[6];    // Voltage values (V)
};

SensorReadings currentReadings;

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  ESP32 Grip Strength Monitor");
  Serial.println("  6 FSR Sensors (2x ADS1115)");
  Serial.println("  FSR Power: 5V | ADS1115: 3.3V");
  Serial.println("========================================\n");
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  Serial.println("[I2C] Initializing...");
  delay(100);
  
  // Scan I2C bus
  scanI2C();
  
  // Initialize first ADS1115 (0x48)
  Serial.print("[ADS1115 #1] Initializing (0x48)... ");
  if (!ads1.begin(ADS1_ADDRESS)) {
    Serial.println("FAILED!");
    Serial.println("ERROR: Cannot find ADS1115 #1 at address 0x48");
    Serial.println("Check wiring and ADDR pin (should be connected to GND)");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK");
  
  // Initialize second ADS1115 (0x49)
  Serial.print("[ADS1115 #2] Initializing (0x49)... ");
  if (!ads2.begin(ADS2_ADDRESS)) {
    Serial.println("FAILED!");
    Serial.println("ERROR: Cannot find ADS1115 #2 at address 0x49");
    Serial.println("Check wiring and ADDR pin (should be connected to VDD)");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK");
  
  // Set gain for both ADCs
  // GAIN_TWOTHIRDS: +/- 6.144V, 1 bit = 0.1875mV
  ads1.setGain(GAIN_TWOTHIRDS);
  ads2.setGain(GAIN_TWOTHIRDS);
  Serial.println("[ADS1115] Gain set to TWOTHIRDS (+/-6.144V)");
  
  // Connect to WiFi
  connectToWiFi();
  
  Serial.println("\n========================================");
  Serial.println("  System Ready!");
  Serial.println("========================================\n");
  
  delay(1000);
}

// ==================== MAIN LOOP ====================

void loop() {
  // Read all sensors
  readAllSensors();
  
  // Print readings
  printReadings();
  
  // Send to backend if interval elapsed
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    sendToBackend();
    lastSendTime = millis();
  }
  
  delay(100);  // Small delay for stability
}

// ==================== SENSOR READING ====================

void readAllSensors() {
  // Read ADC values from both ADS1115 modules
  currentReadings.adc[0] = ads1.readADC_SingleEnded(0);  // FSR1
  currentReadings.adc[1] = ads1.readADC_SingleEnded(1);  // FSR2
  currentReadings.adc[2] = ads1.readADC_SingleEnded(2);  // FSR3
  currentReadings.adc[3] = ads1.readADC_SingleEnded(3);  // FSR4
  currentReadings.adc[4] = ads2.readADC_SingleEnded(0);  // FSR5
  currentReadings.adc[5] = ads2.readADC_SingleEnded(1);  // FSR6
  
  // Convert ADC to voltage and force
  currentReadings.totalGrip = 0;
  for (int i = 0; i < 6; i++) {
    // Convert ADC to voltage (0.1875mV per bit)
    currentReadings.voltage[i] = currentReadings.adc[i] * 0.0001875;
    
    // Convert voltage to force
    currentReadings.force[i] = voltageToForce(currentReadings.voltage[i]);
    
    // Add to total
    currentReadings.totalGrip += currentReadings.force[i];
  }
}

float voltageToForce(float voltage) {
  // Return 0 if voltage below threshold
  if (voltage < MIN_VOLTAGE_THRESHOLD) {
    return 0.0;
  }
  
  // Simple linear conversion (adjust based on calibration)
  float force = exp(voltage);
  
  // Advanced calibration curve (uncomment if you have calibration data)
  /*
  if (voltage < 0.5) {
    force = voltage * 2.0;
  } else if (voltage < 1.5) {
    force = 1.0 + (voltage - 0.5) * 4.0;
  } else if (voltage < 2.5) {
    force = 5.0 + (voltage - 1.5) * 5.0;
  } else {
    force = 10.0 + (voltage - 2.5) * 6.0;
  }
  */
  
  return force;
}

// ==================== DISPLAY ====================

void printReadings() {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║        6-SENSOR GRIP STRENGTH READING          ║");
  Serial.println("╠════════════════════════════════════════════════╣");
  
  // Individual sensors (2 rows of 3)
  Serial.printf("║ FSR1: %5.2f kg │ FSR2: %5.2f kg │ FSR3: %5.2f kg ║\n", 
                currentReadings.force[0], 
                currentReadings.force[1], 
                currentReadings.force[2]);
  
  Serial.printf("║ FSR4: %5.2f kg │ FSR5: %5.2f kg │ FSR6: %5.2f kg ║\n", 
                currentReadings.force[3], 
                currentReadings.force[4], 
                currentReadings.force[5]);
  
  Serial.println("╠════════════════════════════════════════════════╣");
  
  // Total grip
  Serial.printf("║          TOTAL GRIP: %6.2f kg               ║\n", 
                currentReadings.totalGrip);
  
  Serial.println("╚════════════════════════════════════════════════╝");
  
  // Optional: Show raw ADC and voltage values for debugging
  /*
  Serial.println("\nDEBUG - Raw Values:");
  for (int i = 0; i < 6; i++) {
    Serial.printf("  FSR%d: ADC=%5d, Voltage=%.3fV, Force=%.2fkg\n",
                  i+1, 
                  currentReadings.adc[i], 
                  currentReadings.voltage[i], 
                  currentReadings.force[i]);
  }
  */
}

// ==================== NETWORK ====================

void connectToWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println(" Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println(" FAILED!");
    Serial.println("[WiFi] Could not connect. Will retry later.");
  }
}

void sendToBackend() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Backend] WiFi not connected. Reconnecting...");
    connectToWiFi();
    return;
  }
  
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);  // 5 second timeout
  
  // Create JSON payload
  StaticJsonDocument<512> doc;
  doc["device_id"] = DEVICE_ID;
  doc["total_grip"] = round(currentReadings.totalGrip * 100) / 100.0;  // Round to 2 decimals
  doc["sensor1"] = round(currentReadings.force[0] * 100) / 100.0;
  doc["sensor2"] = round(currentReadings.force[1] * 100) / 100.0;
  doc["sensor3"] = round(currentReadings.force[2] * 100) / 100.0;
  doc["sensor4"] = round(currentReadings.force[3] * 100) / 100.0;
  doc["sensor5"] = round(currentReadings.force[4] * 100) / 100.0;
  doc["sensor6"] = round(currentReadings.force[5] * 100) / 100.0;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send POST request
  int httpCode = http.POST(jsonString);
  
  // Handle response
  if (httpCode > 0) {
    if (httpCode == 200) {
      Serial.println("\n[Backend] ✓ Data sent successfully (HTTP 200)");
      
      String response = http.getString();
      
      // Parse response JSON
      StaticJsonDocument<512> responseDoc;
      DeserializationError error = deserializeJson(responseDoc, response);
      
      if (!error && responseDoc.containsKey("success") && responseDoc["success"]) {
        Serial.println("[Backend] Backend confirmed data receipt");
        
        // Show recovery info if available
        if (responseDoc.containsKey("recovery_stage")) {
          int stage = responseDoc["recovery_stage"];
          float recovery = responseDoc["recovery_percent"];
          Serial.printf("[Backend] Patient Recovery: Stage %d, Progress %.1f%%\n", 
                        stage, recovery);
        }
      }
    } else if (httpCode == 400) {
      Serial.println("\n[Backend] ✗ Data rejected (HTTP 400)");
      Serial.println("[Backend] Error: No active session");
      Serial.println("[Backend] → Please START MEASUREMENT in dashboard");
    } else {
      Serial.printf("\n[Backend] ✗ HTTP Error: %d\n", httpCode);
      String response = http.getString();
      Serial.println("[Backend] Response: " + response);
    }
  } else {
    Serial.printf("\n[Backend] ✗ Connection failed: %s\n", 
                  http.errorToString(httpCode).c_str());
  }
  
  http.end();
}

// ==================== UTILITIES ====================

void scanI2C() {
  Serial.println("[I2C] Scanning bus...");
  byte error, address;
  int deviceCount = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("[I2C] Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify common devices
      if (address == 0x48) {
        Serial.println(" (ADS1115 #1)");
      } else if (address == 0x49) {
        Serial.println(" (ADS1115 #2)");
      } else {
        Serial.println(" (Unknown)");
      }
      
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("[I2C] No devices found!");
    Serial.println("[I2C] Check wiring and power supply");
  } else {
    Serial.printf("[I2C] Found %d device(s)\n", deviceCount);
  }
  Serial.println();
}

// ==================== END ====================

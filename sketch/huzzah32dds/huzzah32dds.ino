/*
  Arduino sketch for Adafruit Feather Huzzah32 DDS controller

  Author: Robin Svärd
  Company: MAX IV Laboratory
  Email: robin.svard@maxiv.lu.se

  Published under GNU General Public License, version 3
  https://www.gnu.org/licenses/gpl-3.0.en.html
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <SPI.h>

// Enable or disable serial communication
boolean serial = true;

// WiFi and MQTT configuration
#include "credentials.h"
const char *ssid         = wifi_ssid;
const char *ssidPw       = wifi_password;
const char *broker       = broker_ip;
const char *brokerPw     = broker_password;
const char *brokerUser   = broker_user;
const char *brokerUserPw = broker_user_password;

boolean staticIP = false;
IPAddress ip(192, 168, 10, 240);      // Move these into the credentials file?
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT topics
const char *tpcReset     = "Huzzah32dds/Reset";
const char *tpcSetF      = "Huzzah32dds/SetFreq";
const char *tpcGetF      = "Huzzah32dds/GetFreq";
const char *tpcSetPh     = "Huzzah32dds/SetPhase";
const char *tpcGetPh     = "Huzzah32dds/GetPhase";
const char *tpcSetRefClk = "Huzzah32dds/SetRefClk";
const char *tpcGetRefClk = "Huzzah32dds/GetRefClk";
const char *tpcSetMulti  = "Huzzah32dds/SetMulti";
const char *tpcGetMulti  = "Huzzah32dds/GetMulti";

// This line is here to cure a compiler error
extern void mqtt_callback(char* topic, byte* payload, unsigned int length);

// Initialize WiFi and MQTT connection
String clientName;
WiFiClient wifiClient;
PubSubClient mqttClient(broker, 1883, mqtt_callback, wifiClient);

// Frequency and clock variables
uint32_t _frequency;                     // Current frequency
const uint32_t freqMin = 1000000;        // 1 MHz
const uint32_t freqMax = 200000000;      // 200 MHz
uint16_t _phase;                         // Current phase offset
const uint16_t phaseMin = 0;
const uint16_t phaseMax = 360;
uint32_t _refClk;                        // Current reference clock value
uint8_t _refClkMulti;                    // Current reference clock multiplier
const uint8_t clkMultiMin = 4;
const uint8_t clkMultiMax = 20;
float _sysClk;                           // Current system clock value

// EEPROM addresses
const uint8_t EE_frequency = 0;
const uint8_t EE_phase = 4;
const uint8_t EE_refclk = 6;
const uint8_t EE_multi = 10;
#define EE_size 14

// SPI configuration
const uint16_t spiClk = 1000000;         // 1 MHz
SPIClass * vspi = NULL;                  // Uninitalised pointer to SPI object

// Pin definitions
#define pulseHigh(pin) {digitalWrite(pin, HIGH); delayMicroseconds(30); digitalWrite(pin, LOW);}
const uint8_t pinLED = 13;
const uint8_t pinIOupdate = 4;       // Pin 4   ->  GPIO 10
const uint8_t pinCS = 2;             // Pin 2   ->  GPIO NA
const uint8_t pinSCLK = 5;           // Pin 5   ->  GPIO 11
const uint8_t pinMOSI = 18;          // Pin 18  ->  GPIO 12
const uint8_t pinMISO = 19;          // Pin 19  ->  GPIO 13
const uint8_t pinReset = 21;         // Pin 21  ->  GPIO 16

// DDS standard registry addresses
#define CFR1 0x00         // Control Function Register 1
#define CFR2 0x01         // Control Function Register 2
#define ASF 0x02          // Amplitude Scale Factor
#define ARR 0x03          // Amplitude Ramp Rate
#define FTW0 0x04         // Frequency Tuning Word 0
#define POW 0x05          // Phase Offset Word


// ----------- Initialization

void setup() {
  if (serial) {
    Serial.begin(115200);
  }
  pinMode(pinLED, OUTPUT);
  pinMode(pinIOupdate, OUTPUT);
  digitalWrite(pinIOupdate, LOW);
  pinMode(pinReset, OUTPUT);
  digitalWrite(pinReset, LOW);
  
  // Generate client name based on MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName = "Huzzah32dds_" + mac_to_str(mac);
  if (serial) {
    Serial.print("Client name generated as: ");
    Serial.println(clientName);
  }
  
  // Only refresh WiFi config if SSID has changed
  if (WiFi.SSID() != ssid) {
    if (staticIP) {
      WiFi.config(ip, gateway, subnet);
    }
    WiFi.begin(ssid, ssidPw);
  }
  
  // Initialize EEPROM
  EEPROM.begin(EE_size);
  
  // Connect to WiFi and MQTT broker
  wifi_connect();
  mqtt_connect();
  
  // Initialize SPI instance
  vspi = new SPIClass(VSPI);
  vspi -> begin(pinSCLK, pinMISO, pinMOSI, pinCS);
  
  // Ensure a clean power-up
  delay(250);
  
  // Reset DDS to initial state
  //dds_reset();
  
  // Load initial configuration based on saved values in EEPROM
  dds_configure();
}


// ----------- WiFi routines

String mac_to_str(const uint8_t *mac) {
  // Generate a client name based on MAC address
  String result;
  
  for (int i = 0; i < 6; i++) {
    if ((mac[i] & 0xF0) == 0) {
      // Stop suppression of leading zero
      result += String(0, HEX);
    } else {
      result += String(mac[i], HEX);
    }
    
    if (i < 5) {
      result += ':';
    }
  }
  
  return result;
}

void wifi_connect() {
  if (WiFi.status() != WL_CONNECTED) {
    if (serial) {
      Serial.print("Connecting to: ");
      Serial.println(ssid);
    }
    
    if (WiFi.status() == WL_CONNECT_FAILED) {
      if (serial) {
        Serial.println("Failed to connect. Verify login credentials.");
      }
      return;
    }
    
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      delay(500);
      if (serial) {
        Serial.println("......");
      }
    }
    
    if (serial) {
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
}


// ----------- MQTT routines

void mqtt_connect() {
  // Make sure WiFi is connected before attempting to reconnect to MQTT
  if (WiFi.status() == WL_CONNECTED) {
    
    // Loop until we are connected to the MQTT broker
    while (!mqttClient.connected()) {
      if (serial) {
        Serial.print("Attempting MQTT connection... ");
      }
      
      if (mqttClient.connect((char*) clientName.c_str(), brokerUser, brokerUserPw)) {
        if (serial) {
          Serial.println("Success!");
        }
        
        // Subscribe to topics
        mqtt_subscribe();
        
      } else {
        if (serial) {
          Serial.println("Failed!");
        }
        
        // Try again in a few seconds
        delay(5000);
      }
    }
  }
}

void mqtt_subscribe() {
  // Loop over these instead? Define as array in the beginning
  mqttClient.subscribe(tpcReset);
  mqttClient.subscribe(tpcSetF);
  mqttClient.subscribe(tpcSetPh);
  mqttClient.subscribe(tpcSetRefClk);
  mqttClient.subscribe(tpcSetMulti);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (serial) {
    Serial.print("Callback received with topic: ");
    Serial.println(topic);
    Serial.print("Payload: ");
    Serial.write(payload, length);
    Serial.println("");
  }
  
  // Convert char array to a string by NULL terminating it
  payload[length] = '\0';
  
  if (strcmp(topic, tpcReset) == 0) {
    dds_reset();
  
  } else if (strcmp(topic, tpcSetF) == 0) {
    uint32_t temp = atoi((char*)payload);
    dds_set_frequency(temp);
  
  } else if (strcmp(topic, tpcSetPh) == 0) {
    uint16_t temp = atoi((char*)payload);
    dds_set_phase(temp);
  
  } else if (strcmp(topic, tpcSetRefClk) == 0) {
    _refClk = atoi((char*)payload);
    eeprom_write(EE_refclk, _refClk);
    dds_set_sysclk();
    dds_set_frequency(_frequency);
  
  } else if (strcmp(topic, tpcSetMulti) == 0) {
    uint8_t temp = atoi((char*)payload);
    dds_set_refclkmulti(temp);
    dds_set_sysclk();
    dds_set_frequency(_frequency);
  }
}

void mqtt_publish() {
  //
}


// ----------- DDS routines

void dds_reset() {
  // Reset the DDS to default settings
  pulseHigh(pinReset);
  dds_configure();
}

void dds_configure() {
  // Set initial settings for the DDS
  
  // Set DDS SDIO pin to input only
  spi_send(CFR1, 0x200, 4);
  
  // Disable some peripherals not in use
  // Comparator
  spi_send(CFR1, 0x40, 4);
  // SYNC_CLK output unless required. This can cause some noise in the GND plane.
  //spi_send(CFR1, 0x2, 4);
  
  // Read last reference clock value from EEPROM
  // Will either be 20.0 or 99.98... MHz
  _refClk = eeprom_read(EE_refclk, _refClk);
  
  // Read last refrence clock multiplier from EEPROM and send to DDS
  _refClkMulti = eeprom_read(EE_multi, _refClkMulti);
  dds_set_refclkmulti(_refClkMulti);
  
  // Update system clock
  dds_set_sysclk();
  
  // Read last frequency from EEPROM and send to DDS
  _frequency = eeprom_read(EE_frequency, _frequency);
  dds_set_frequency(_frequency);
  
  // Read last phase offset from EEPROM and send to DDS
  _phase = eeprom_read(EE_phase, _phase);
  dds_set_phase(_phase);
}

void dds_update_registers() {
  // Push data from IO buffers into the internal registers
  // IO update must be high for at least 1 rising edge of the sync_clk cycle
  // For 1.8 and 3.3 V, this is 6 and 4 ns, respectively
  pulseHigh(pinIOupdate);
}

void dds_set_refclkmulti(uint8_t multi) {
  // Set the reference clock multiplier
  // Bits [7:3] 0x04 to 0x14 for 4x to 20x multiplication
  // 0000 0000 0000 0000 *0000 0*000
  
  // Examples:
  // 4x:  0000 0000 0000 0000 0010 0000
  // 5x:  0000 0000 0000 0000 0010 1000
  // 6x:  0000 0000 0000 0000 0011 0000
  // 20x: 0000 0000 0000 0000 1010 0000   ( 0, 0, 160 )
  
  multi = constrain(multi, clkMultiMin, clkMultiMax);
  
  // Save reference clock multiplier to EEPROM if it has changed
  if (_refClkMulti != multi) {
    eeprom_write(EE_multi, multi);
    _refClkMulti = multi;
  }
  
  if (serial) {
    Serial.print("Setting REF CLK multiplier to: ");
    Serial.println(multi);
  }
  
  // Shift multiplier bits to their correct position
  multi = (uint32_t)multi << 3;
  
  // Set the VCO control bit depending on frequency range
  // Bit 2 should be 0 for f_sysclk = 0 to 250 MHz and 1 for f_sysclk = 250 to 400 MHz.
  multi = multi ^ (uint32_t)0x4;
  spi_send(CFR2, multi, 3);
}

void dds_set_sysclk() {
  // Update the value for the system clock
  _sysClk = _refClk * _refClkMulti;
}

uint32_t dds_get_ftw(uint32_t frequency) {
  // Calculate a 32 bit frequency tuning word
  // FTW = (f_output * 2e32) / f_sysclk  for 0 <= FTW <= 2e31
  // FTW = (1 - (f_output / f_sysclk)) * 2e32  for 2e31 < FTW < 2e32-1
  
  return frequency * (0xFFFFFFFF / _sysClk);
}

void dds_set_frequency(uint32_t frequency) {
  // Set the output frequency of the DDS
  frequency = constrain(frequency, freqMin, freqMax);
  
  // Save frequency to EEPROM if it has changed
  if (_frequency != frequency) {
    eeprom_write(EE_frequency, frequency);
    _frequency = frequency;
  }
  
  uint32_t ftw0 = dds_get_ftw(frequency);
  
  if (serial) {
    Serial.print("Setting frequency [Hz]: ");
    Serial.println(frequency);
    Serial.print("Frequency tuning word: ");
    Serial.println(ftw0);
    Serial.print("System clock [Hz]: ");
    Serial.println(_sysClk); 
  }
  
  spi_send(FTW0, ftw0, 4);
}

uint16_t dds_get_pow(uint16_t phase) {
  // Calculate a 14 bit phase offset word
  // POW = (ph_offset / 360) * 2e14
  
  return (phase / 360.) * 0x4000;
}

void dds_set_phase(uint16_t phase) {
  // Set the phase offset of the DDS output
  phase = constrain(phase, phaseMin, phaseMax);
  
  // Save phase offset to EEPROM if it has changed
  if (_phase != phase) {
    eeprom_write(EE_phase, phase);
    _phase = phase;
  }
  
  uint16_t pow0 = dds_get_pow(phase);
  
  if (serial) {
    Serial.print("Setting phase offset [degrees]: ");
    Serial.println(phase);
    Serial.print("Phase offset word: ");
    Serial.println(pow0);
  }
  
  spi_send(POW, pow0, 2);
}


// ----------- SPI routines

void spi_send(uint8_t instr, uint32_t data, uint8_t bytes) {
  // Send new settings to DDS
  vspi -> beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  
  // Send instruction byte
  if (serial) {
    Serial.print("Instruction address: ");
    Serial.println(instr);
    Serial.println("Data bytes:");
  }
  vspi -> transfer(instr);
  
  // Send data, shifted in 1 byte as a time as MSB
  for (int8_t b = bytes - 1; b >= 0; b--) {
    uint8_t payload = (data >> b*8) & 0xFF;
    if (serial) {
      Serial.println(payload);
    }
    vspi -> transfer(payload);
  }
  
  vspi -> endTransaction();
  
  // Update DDS internal registers;
  dds_update_registers();
}

void spi_receive() {
  //
}


// ----------- EEPROM routines

uint8_t eeprom_read(uint8_t address, uint8_t type) {
  // Read a 8 bit value from the EEPROM
  uint8_t value = EEPROM.get(address, type);
  
  if (serial) {
    Serial.print("Reading from 8 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  return value;
}

uint16_t eeprom_read(uint8_t address, uint16_t type) {
  // Read a 16 bit value from the EEPROM
  uint16_t value = EEPROM.get(address, type);
  
  if (serial) {
    Serial.print("Reading from 16 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  return value;
}

uint32_t eeprom_read(uint8_t address, uint32_t type) {
  // Read a 32 bit value from the EEPROM
  uint32_t value = EEPROM.get(address, type);
  
  if (serial) {
    Serial.print("Reading from 32 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  return value;
}

void eeprom_write(uint8_t address, uint8_t value) {
  // Save a 8 bit value to the EEPROM
  if (serial) {
    Serial.print("Saving to 8 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  EEPROM.put(address, value);
  EEPROM.commit();
}

void eeprom_write(uint8_t address, uint16_t value) {
  // Save a 16 bit value to the EEPROM
  if (serial) {
    Serial.print("Saving to 16 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  EEPROM.put(address, value);
  EEPROM.commit();
}

void eeprom_write(uint8_t address, uint32_t value) {
  // Save a 32 bit value to the EEPROM
  if (serial) {
    Serial.print("Saving to 32 bit address: ");
    Serial.println(address);
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  EEPROM.put(address, value);
  EEPROM.commit();
}


void loop() {
  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    if (serial) {
      Serial.println("Lost connection to WiFi!");
    }
    wifi_connect();
  }

  if (!mqttClient.connected()) {
    mqtt_connect();
  }

  // Maintain MQTT connection
  mqttClient.loop();
  
  // Heartbeat LED
  delay(1500);
  digitalWrite(pinLED, HIGH);
  delay(1500);
  digitalWrite(pinLED, LOW);
}

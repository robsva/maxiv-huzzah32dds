/*
  Arduino sketch for Adafruit Feather Huzzah32 DDS controller

  Author: Robin Svärd
  Company: MAX IV Laboratory
  Email: robin.svard@maxiv.lu.se
  Date: 2020-12-22

  Published under GNU General Public License, version 3
  https://www.gnu.org/licenses/gpl-3.0.en.html
  
  Revision 1.1 - 2020-12-22
  Changed something.
*/

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// Enable or disable serial communication
boolean serial = true;

// WiFi and MQTT configuration
#include "credentials.h"
const char *ssid     = wifi_ssid;
const char *ssidPw   = wifi_password;
const char *broker   = broker_ip;
const char *brokerPw = broker_password;

// Frequency and clock variables
uint32_t _frequency;                     // Current frequency
const uint32_t freqMin = 1000000;        // 1 MHz
const uint32_t freqMax = 200000000;      // 200 MHz
uint16_t _phase;                         // Current phase offset
uint8_t _refClkMulti = 1;                // Current reference clock multiplier

// SPI configuration
const uint16_t spiClk = 1000000;     // 1 MHz
SPIClass * vspi = NULL;              // Uninitalised pointer to SPI object

// Pin definitions
#define pulseHigh(pin) {digitalWrite(pin, HIGH); delayMicroseconds(30); ;digitalWrite(pin, LOW); }
const uint8_t pinLED = 13;
const uint8_t pinIOupdate = 4;    // Pin 4   ->  GPIO 10
const uint8_t pinCS = 2;          // Pin 2   ->  GPIO NA
const uint8_t pinSCLK = 5;        // Pin 5   ->  GPIO 11
const uint8_t pinMOSI = 18;       // Pin 18  ->  GPIO 12
const uint8_t pinMISO = 19;       // Pin 19  ->  GPIO 13
const uint8_t pinReset = 21;      // Pin 21  ->  GPIO 16

// Standard registry addresses
#define CFR1 0x00     // Control Function Register 1
#define CFR2 0x01     // Control Function Register 2
#define ASF 0x02      // Amplitude Scale Factor
#define ARR 0x03      // Amplitude Ramp Rate
#define FTW0 0x04     // Frequency Tuning Word 0
#define POW 0x05      // Phase Offset Word


// ----------- Initialization

void setup() {
  if (serial) {
    Serial.begin(115200);
  }
  pinMode(pinLED, OUTPUT);
  pinMode(pinIOupdate, OUTPUT);
  pinMode(pinReset, OUTPUT);
  
  // Initialize WiFi and MQTT connection
  //wifi_connect();
  
  //PubSubClient mqttclient(broker, 1883, callback, client);
  // mqtt_connect();
  
  // Initialize SPI instance
  vspi = new SPIClass(VSPI);
  vspi -> begin(pinSCLK, pinMISO, pinMOSI, pinCS);
  
  // Reset DDS to initial state
  dds_reset();
  
  // Ensure a clean power-up
  delay(250);
  
  // Set DDS SDIO pin to input only
  // 0000 0000   or   0x00
  // 0000 0000 0000 0000 0000 0010 0000 0000    ( 0, 0, 2, 0 )
  spi_send(CFR1, 0x200, 4);
  
  // Disable peripherals not in use
  // Comparator
  // 0000 0000   or   0x00
  // 0000 0000 0000 0000 0000 0000 0100 0000    ( 0, 0, 0, 64 )
  //spi_send(CFR1, 0x40, 4);
  // SYNC_CLK output unless required. This can cause some noise in the GND plane.
  //spi_send(CFR1, 0x2, 4);
  
  // Read last ref clk multiplier from EEPROM and send to DDS
  //
  dds_set_refclk(4);
  
  // Can we get the sysclk from somewhere?
  // Will either be 20*20 or 4*99.98... MHz
  
  // Read last frequency from EEPROM and send to DDS
  //
  dds_set_frequency(38000000);
  
  // Read last phase offset from EEPROM and send to DDS
  //
  //dds_set_phase(45);
}


// ----------- WiFi routines

void wifi_connect() {
  if (serial) {
    Serial.println("Connecting to: ");
    Serial.print(ssid);
  }

  WiFi.begin(ssid, ssidPw);

  while (WiFi.status() != WL_CONNECTED) {
    if (serial) {
      Serial.println("......");
    }
    delay(500);
  }

  if (serial) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.print(WiFi.localIP());
  }
}


// ----------- MQTT routines

void mqtt_connect() {
  //
}

void mqtt_subscribe() {
  //
}

void mqtt_callback (char* topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  Serial.write(payload, length);
  Serial.println("");
}

void mqtt_publish() {
  //
}


// ----------- DDS routines

void dds_reset() {
  pulseHigh(pinReset);
}

void dds_update_registers() {
  // Push data from IO buffers into the internal registers
  pulseHigh(pinIOupdate);
  // IO update must be high for at least 1 rising edge of the sync_clk cycle
  // For 3.3 V, this is 4 ns
}

void dds_set_refclk(uint32_t multi) {
  // Set the reference clock multiplier
  // 0000 0001   or   0x01
  // 0000 0000 0000 0000 *0000 0*000
  // [7:3] 0x04 to 0x14 for 4x to 20x multiplication
  
  // 4x:  0000 0000 0000 0000 0010 0000
  // 5x:  0000 0000 0000 0000 0010 1000
  // 6x:  0000 0000 0000 0000 0011 0000
  // 20x: 0000 0000 0000 0000 1010 0000   ( 0, 0, 160 )

  // Shift multiplyer bits to their correct position
  multi = multi << 3;
  
  if (serial) {
    Serial.print("Setting REF CLK multiplier to: ");
    Serial.println(multi);
  }
  
  // Set the VCO control bit depending on frequency range
  // bit 2 should be 0 for f = 0 to 250 MHz and 1 for f = 250 to 400 MHz.
  multi = multi ^ (uint32_t)0x4;
  spi_send(CFR2, multi, 3);
}

uint32_t dds_get_ftw(uint32_t frequency) {
  // Calculate a 32 bit frequency tuning word
  // FTW = (f_output * 2e32) / f_sysclk  for 0 <= FTW <= 2e31
  // FTW = (1 - (f_output / f_sysclk)) * 2e32  for 2e31 < FTW < 2e32-1
  
  frequency = constrain(frequency, freqMin, freqMax);
  
  // We need to know if the f_sysclk is changed...
  //return frequency * (4294967296. / 0x17D78400);   // Why floating point precision needed here?
  // Maybe smoother to keep sysclk as the floating point
  return frequency * (0xFFFFFFFF / 400000000.);
}

void dds_set_frequency(uint32_t frequency) {
  // Set the output frequency of the DDS
  uint32_t ftw0 = dds_get_ftw(frequency);
  
  if (serial) {
    Serial.print("Setting frequency [Hz]: ");
    Serial.println(frequency);
    Serial.print("Frequency tuning word: ");
    Serial.println(ftw0);
  }
  
  spi_send(FTW0, ftw0, 4);
  
  // Save frequency to EEPROM if it has changed
  
}

uint16_t dds_get_pow(uint8_t phase) {
  // Calculate a 14 bit phase offset word
  // POW = (ph_offset / 360) * 2e14
  
  return (phase / 360.) * 0x4000;
  // either 360 or phase as a float
}

void dds_set_phase(uint8_t phase) {
  // Set the phase offset of the DDS output
  uint16_t pow0 = dds_get_pow(phase);
  
  if (serial) {
    Serial.print("Setting phase offset [degrees]: ");
    Serial.println(phase);
    Serial.print("Phase offset word: ");
    Serial.println(pow0);
  }
  
  spi_send(POW, pow0, 2);

  // Save phase offset to EEPROM if it has changed
  
}


// ----------- SPI routines

void spi_send(uint8_t instr, uint32_t data, uint8_t depth) {
  // Send new settings to DDS
  vspi -> beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));  // or SPI_MODE3
  
  // Send instruction byte
  if (serial) {
    Serial.print("Instruction address: ");
    Serial.println(instr);
    Serial.println("Data bytes:");
  }
  vspi -> transfer(instr);
  
  // Send data, shifted in 1 byte as a time as MSB
  for (int8_t b=depth-1; b>=0; b--) {
    uint8_t payload = (data >> b*8) & 0xFF;
    if (serial) {
      Serial.println(payload);
    } 
    vspi -> transfer(payload);
  }
  
  vspi -> endTransaction();

  // Update DDS internal registers
  dds_update_registers();
}

void spi_receive() {
  //
}


void loop() {
  // Maintain WiFi connection
  //if (WiFi.status() != WL_CONNECTED) {
  //  wifi_connect()
  //}

  // Maintain MQTT connection
  //mqttClient.loop();
  
  // Heartbeat LED
  delay(1500);
  digitalWrite(pinLED, HIGH);
  delay(1500);
  digitalWrite(pinLED, LOW);
  
}

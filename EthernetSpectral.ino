/*
 ___ ___   _____ __    _  _ __  _ _____ _ _     __ ___  __  _  _____ __  _  
| __| \ \_/ | __| _\  | || |  \| |_   _| | |   |  | _ \/__\| |/ | __|  \| | 
| _|| |> , <| _|| v | | \/ | | ' | | | | | |_  | -| v | \/ |   <| _|| | ' | 
|_| |_/_/ \_|___|__/   \__/|_|\__| |_| |_|___| |__|_|_\\__/|_|\_|___|_|\__|
Garner M 
2025

Ethernet based spectral sensor with a PICO/wiznet 5500. 

Commands:
*IDN? -identify. Returns the SN which is also the last byte of the MAC 
Color? X -responds with the color data (X=0,1,2). Each color is delimited by a comma ",". Colors are in this order: Violet (450nm), Blue (500nm), Green (550nm), Yellow (570nm), Orange (600nm),Red (650nm)
Temp? -Reads the internal temp of the RP2040 returns in C
SYS XX -sets the last byte of the mac address/SN based on the bit value of the byte(in ascii) you send it. Don't change this often. Should be a setup only task.

\n is the end of command
*/
#include <Ethernet.h>
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <Wire.h>
#include <Adafruit_AS726x.h>

// Set the target offest to the last sector of flash (config space)
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define TCA_ADDR 0x70  // TCA9548A default I2C address
#define BUFFER_SIZE 64
#define WIZNET_RSTn 20
#define WIZNET_CS   17
#define WIZNET_INT  21

int buf[FLASH_PAGE_SIZE/sizeof(int)];  // One page buffer of ints
int *p, addr;
unsigned int page; // prevent comparison of unsigned and signed int
int first_empty_page = -1;

byte mac[] = { 0xDE, 0xAD, 0x10, 0x10, 0x00, 0x00};
char buffer[BUFFER_SIZE];
EthernetServer server(5024); //Socket port

// Spectral sensors
Adafruit_AS726x as7262_ch0;
Adafruit_AS726x as7262_ch1;
Adafruit_AS726x as7262_ch2;
float calibratedValues[AS726x_NUM_CHANNELS];

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  // get the chip out of reset
  pinMode(WIZNET_RSTn, OUTPUT);
  digitalWrite(WIZNET_RSTn, LOW);
  delayMicroseconds(500);
  digitalWrite(WIZNET_RSTn, HIGH);
  delay(1);
   //Get Byte(read it from flash)
  addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
  p = (int *)addr;
  mac[5] = {*p};
  Serial.begin(9600);
    delay(2000);
  // Initialize Ethernet
  Ethernet.begin(mac);
  server.begin();
  Serial.println(Ethernet.localIP());
  // Initialize spectral sensors
    // Initialize I2C
  Wire.begin();
  initializeSpectralSensors();
  adc_init();
  adc_set_temp_sensor_enabled (true);
  adc_select_input(4);
}



void loop() {
  EthernetClient client = server.available();

  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          processCommand(buffer, client);
          memset(buffer, 0, BUFFER_SIZE);
        } else if (c != '\r'){
          //ignores any \r
          strncat(buffer, &c, 1);
        }
      }
    }
    client.stop();
  }
}

void processCommand(char* command, EthernetClient& client) {
  char* token = strtok(command, " ");
  
  // Identification command
  if (strcmp(token, "*IDN?") == 0) {
    client.print("Peek-A-Hue, FW V0.1, SN: ");
    client.println(mac[5]);
  }   
  // Temperature command
  else if (strcmp(token, "TEMP?") == 0) {
    client.println(readInternalTempC());
  }
  // System configuration command
  else if (strcmp(token, "SYS") == 0) {
    token = strtok(NULL, "\n");
    if (token) {
      if (strlen(token) >= 2) {
        byte hexValue = asciiToHex(token[0]) << 4 | asciiToHex(token[1]);
        *buf = hexValue;
        uint32_t ints = save_and_disable_interrupts();
       flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
      flash_range_program(FLASH_TARGET_OFFSET + (0*FLASH_PAGE_SIZE), (uint8_t *)buf, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
      client.println("Last byte MAC set to " + String(*buf, HEX));
      } else {
        client.println("Invalid input length");
      }
    } else {
      client.println("Missing argument");
    }
  }
  // Color sensor command
  else if (strcmp(token, "Color?") == 0) {
    token = strtok(NULL, "\n");
    if (token) {
      uint8_t channel = atoi(token);
      if (channel >= 0 && channel <= 2) {
        readAndSendColorData(channel, client);
      } else {
        client.println("Invalid channel (0-2 only)");
      }
    } else {
      client.println("Missing channel number");
    }
  }
  else {
    client.println("Invalid command");
  }
}

void readAndSendColorData(uint8_t channel, EthernetClient& client) {
  Adafruit_AS726x* sensor;
  
  switch(channel) {
    case 0:
      tcaSelect(0);
      sensor = &as7262_ch0;
      break;
    case 1:
      tcaSelect(1);
      sensor = &as7262_ch1;
      break;
    case 2:
      tcaSelect(2);
      sensor = &as7262_ch2;
      break;
    default:
      client.println("Invalid channel");
      return;
  }
  
  // Wait for data to be ready with a timeout
  unsigned long startTime = millis();
  sensor->startMeasurement();
  while (!sensor->dataReady()) {
    if (millis() - startTime > 1000) { // 1 second timeout
      client.println("Timeout waiting for data ready");
      return;
    }
    delay(10);
  }
  
  // Read calibrated values
  sensor->readCalibratedValues(calibratedValues);
  
  // Send comma-delimited values
  for (int i = 0; i < AS726x_NUM_CHANNELS; i++) {
    client.print(calibratedValues[i]);
    if (i < AS726x_NUM_CHANNELS - 1) {
      client.print(",");
    }
  }
  client.println();
}

// ASCII to HEX conversion
byte asciiToHex(char ascii) {
  if (ascii >= '0' && ascii <= '9') {
    return ascii - '0';
  } else if (ascii >= 'A' && ascii <= 'F') {
    return ascii - 'A' + 10;
  } else if (ascii >= 'a' && ascii <= 'f') {
    return ascii - 'a' + 10;
  } else {
    return 0; // Invalid ASCII character, return 0
  }
}

// Read internal temperature
float readInternalTempC() {
  uint16_t raw = adc_read();  // Internal sensor is on ADC4
  float voltage = (raw / 4095.0) * 3.3;
  float temperature = 27.0 - (voltage - 0.706) / 0.001721;
  return temperature;
}
void initializeSpectralSensors() {
  // Initialize sensor on channel 0
  tcaSelect(0);
  delay(100);
  if (!as7262_ch0.begin()) {
    Serial.println("AS7262 not detected on mux channel 0!");
    while (1);
  }
  as7262_ch0.setIntegrationTime(50);
  // Initialize sensor on channel 1
  tcaSelect(1);
  delay(100);
  if (!as7262_ch1.begin()) {
    Serial.println("AS7262 not detected on mux channel 1!");
    while (1);
  }
  as7262_ch1.setIntegrationTime(50);
  // Initialize sensor on channel 2
  tcaSelect(2);
  delay(100);
  if (!as7262_ch2.begin()) {
    Serial.println("AS7262 not detected on mux channel 2!");
    while (1);
  }
  as7262_ch2.setIntegrationTime(50);
}

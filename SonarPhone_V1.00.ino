/**
 * Connects to SP200A SonarPhone as a first or second master after
 * first master has been established.
 * The SonarPhone phone app is not required unless SP200A is 
 * factory reset.
 * Numerical depth, battery voltage, and temperature is transmitted via esphome-uart-p2p via seral1 with pins 16/17.
 * 
 * Original code for SP200A-esp32 connection by Jim McKeown, https://github.com/jim-mckeown/SP200A-Client
 * Original code for esphome-uart-p2p by KG3RK3N, https://github.com/KG3RK3N/esphome-uart-p2p
 */

#include "WiFi.h"
#include "AsyncUDP.h"
#include <WiFiManager.h>

const char * ssid = "T-BOX-720";
const char * password = "";
uint8_t FC[] = {70,67,21,0,244,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // mac address, units, beam width, depth max/min, checksum not set
uint8_t FX[] = {70,88,21,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,179,0,0,0,0,0,0,0,0,0}; // 

float metersToFeet = 3.28084;

bool haveMac = false;
int depthMin = 0;
int depthMax = 0; 
int beamWidth = 8; // 8 = 20 deg, 2 = 40 deg
int dataCount = 0;
int depthUnits = 1; //0 = meters, 1 = feet
float depth = 0.0;
float depthFrac = 0.0;
float vBatt = 0.0;
float vBattFrac = 0.0;
float temp = 0.0;
int loopCount = 0;
bool newData = false;
bool FCsent = false;

//Pins for Serial1 connection to be used with esphome-uart-p2p
#define RX1 16
#define TX1 17

AsyncUDP udp;

/*  Send updates via esphome-uart-p2p.
 */  
 
bool sendSonarData()
{
  float depthToSend = 0.0;

  // set depth units to feet - convert meters to feet
  if(depthUnits == 1)
  {
    depthToSend = depth; 
  }
  else
  {
    depthToSend = depth * metersToFeet;
  }
  
  uint8_t address1 = 0x01;  // <- thats the address from the esphome sensor
  uint8_t sensorType = 0; // 0 = number, 1 = binary, 2 = text
  sendUartData(address1, &depthToSend, sizeof(depthToSend), sensorType);
  uint8_t address2 = 0x02;  // <- thats the address from the esphome sensor
  sendUartData(address2, &vBatt, sizeof(vBatt), sensorType);
  uint8_t address3 = 0x03;  // <- thats the address from the esphome sensor
  sendUartData(address3, &temp, sizeof(temp), sensorType);
  
  return true;
}

//esphome-uart-p2p start
// code to generate & send uart message
static const int headerSize = 3;

void sendUartData(uint8_t address, const void *data, size_t dataLength, uint8_t sensorType) {
  uint8_t uartData[headerSize + dataLength];
  uartData[0] = address;
  uartData[1] = sensorType;
  uartData[2] = dataLength;

  memcpy(uartData + headerSize, data, dataLength);

  uint8_t checksum = calculateChecksum(uartData, headerSize + dataLength);
  uartData[headerSize + dataLength] = checksum;

  Serial1.write(uartData, headerSize + dataLength + sizeof(uint8_t));
}

uint8_t calculateChecksum(const uint8_t *data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}
//esphome-uart-p2p end

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200,SERIAL_8N1,RX1,TX1); //for UART to esphome receiver

    //WiFiManager intialization
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    //wm.resetSettings();
    bool res;
    res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    if(!res) 
    {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else 
    {
        Serial.println("Connected to WiFi");
    }    
    
    if(udp.connect(IPAddress(192,168,1,1), 5000)) 
    {
        Serial.println("UDP connected");
        //udp.write(FC, 29);
        udp.onPacket([](AsyncUDPPacket packet) {
            // check for REDYFX. Update FC request
            if(packet.data()[6] == 82 && packet.data()[7] == 69 && packet.data()[8] == 68 && packet.data()[9] == 89 && packet.data()[10] == 70 && packet.data()[11] == 88) // is this REDYFX?
            {
              // set FC mac address
              for(int i = 0;i < 6;i++)
              {
                FC[21 + i] = packet.data()[26 + i];
              }
              
              // set units
              FC[11] = depthUnits;              
              
              // set max, min depth (4 bytes)
              FC[6] = depthMin % 256; // lower byte
              FC[7] = depthMin / 256; // upper byte
              FC[8] = depthMax % 256; // lower byte
              FC[9] = depthMax / 256; // upper byte
              
              // set beam width
              FC[13] = beamWidth;
              
              // set checksum
              int checksum = 0;
              for(int i = 0;i < 19;i++)
              {
                checksum += FC[i];
              }
              FC[19] = checksum % 256; // lower byte
              FC[20] = checksum / 256; // upper byte
              haveMac = true;
            }
            // check for REDYFC. Set Units, Depth, Range, Temp, BatteryVolts, newData flag
            if(packet.data()[6] == 82 && packet.data()[7] == 69 && packet.data()[8] == 68 && packet.data()[9] == 89 && packet.data()[10] == 70 && packet.data()[11] == 67) // is this REDYFC?
            {
              depthUnits = packet.data()[21];
              depthFrac = packet.data()[25];
              depth = packet.data()[23] + (packet.data()[24] * 256) + (depthFrac / 100);
              depthMin = packet.data()[16] + (packet.data()[17] * 256);
              depthMax = packet.data()[18] + (packet.data()[19] * 256);
              beamWidth = packet.data()[32];
              vBattFrac = packet.data()[31];
              vBatt = packet.data()[30] + (vBattFrac / 100);
              temp = packet.data()[26] * 9.0/5.0 + 32.0; //convert reported °C to °F
              newData = true;
            }
            dataCount++;
            packet.flush();          
        });
        delay(1000);
    }

    
}

void loop()
{
    if(haveMac)
    {
      if(newData)
      {
        Serial.printf("Depth: %.1f ", depth);
        //Serial.println();
        //Serial.printf("Temp: %.1f", temp);
        //Serial.println();
        //Serial.printf("vBatt: %.1f ", vBatt);
        if(!sendSonarData())
        {
          Serial.println("Error in sendSonarData()");
        }
        newData = false;
      }
      else
      {
        Serial.print("Depth: --.-- ");
      }
      
      
      if(FCsent)
      {
        Serial.println("-");
        FCsent = false;
      }
      else
      {
        Serial.println("");
      }
      
      if(loopCount > 19)
      {
        // update units
        FC[11] = depthUnits;
        
        // update max, min depth (4 bytes)
        FC[6] = depthMin % 256; // lower byte
        FC[7] = depthMin / 256; // upper byte
        FC[8] = depthMax % 256; // lower byte
        FC[9] = depthMax / 256; // upper byte
  
        // set beam width
        FC[13] = beamWidth;
        
        // update checksum
        int checksum = 0;
        for(int i = 0;i < 19;i++)
        {
          checksum += FC[i];
        }
        FC[19] = checksum % 256; // lower byte
        FC[20] = checksum / 256; // upper byte      
        
        dataCount = 0;
        udp.write(FC, sizeof FC);
        loopCount = 0;
        FCsent = true;
     }
      
      loopCount++;
      delay(500);
    }
    else
    {
      udp.write(FX, sizeof FX);
      delay(1000);
    }
}

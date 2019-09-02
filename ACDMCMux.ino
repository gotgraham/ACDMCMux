#include <SPI.h>
#include "mcp_can.h"
#include <Arduino.h>
#include <EEPROM.h>

// Configure CAN and Display
MCP_CAN CAN(10);                                   

//
// Definitions
//

#define uint16 unsigned int
#define uint8 unsigned char
#define uint32 unsigned long int

#pragma pack(push, 1)  // push current alignment to stack

// DC-DC Status
struct DCDCStatus {
 uint32 temp1;
 uint32 temp2;
 uint32 voltage;
 uint32 current;       
} dc_dc_status;

#pragma pack(pop)   // restore original alignment from stack

#pragma pack(push, 1)  // push current alignment to stack

// Controller Status
struct ControllerStatus {
  uint32 temps[6];
  uint32 current[4];
} controller_status;

#pragma pack(pop)   // restore original alignment from stack


// Fan State
bool fanEnabled = false;
uint8 fanPin = 23;

//
// Helper Functions
//

float cToF(float val) {
  return (val * 9/5) + 32;
}


#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

// 
// DC DC 
//

void dcDcCheckStatus(void) {
  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    
    unsigned long canId = CAN.getCanId();

    // Read status info from the DC-DC
    if (canId == 0x1d6) {
      dc_dc_status.temp1 = (cToF(buf[3] - 40) * 100);
      dc_dc_status.temp2 = (cToF(buf[4] - 40) * 100);
      dc_dc_status.voltage = (buf[2] / 12.7 * 100);
      dc_dc_status.current = (buf[5] * 100);
    }
  }
  // Enable the DC DC
  unsigned char msg[2] = {0xA0, 0xAF};
  CAN.sendMsgBuf(0x01D4, 0, 2, msg);
}

//
// Controller
//

uint16 readControllerValue(char request)
{
  uint16 result = 0;   
  uint16 masked_result = 0; 
    
  while(Serial2.read() >= 0); // Depleat the receive buffer

  do {
    Serial2.write(request); 
    
    // Request value    
    if (Serial2.available() > 0) {      
      char inBuff[3]; // Read a buffer
      Serial2.readBytes(inBuff, sizeof(inBuff));
      result = ntohs(*((uint16*)&inBuff[0]));        
    } 
    delay(10);

    masked_result = result & 0xF000;
    
  } while(masked_result != 0x8000 && masked_result != 0x7000 && masked_result != 0x6000);
 
  Serial2.write('*'); // Stop value
 
  return result;
}

void storeTemp(uint8 which, uint16 raw_value) {
  float temp = ((float)(raw_value & 0x7FFF) / 2);
  if (temp > 0 && temp < 150) {
    controller_status.temps[which] = (temp * 100);
  }
}

void storeCurrent(uint8 which, uint16 raw_value) {
  float current = raw_value >= 0x8000 ? (float)(raw_value & 0x7FFF) : 0;//(float)(raw_value - 0x8000);
  current /= 10;
  
  controller_status.current[which] = (current * 100);
}

void controllerCheckStatus() {
  static uint8 current_temp_sensor = 0;
  storeTemp(current_temp_sensor, readControllerValue(current_temp_sensor + 108)); // 108 is ASCII for 'l'
  if (current_temp_sensor == 5) {
    current_temp_sensor = 0;
  }
  else {
    ++current_temp_sensor;
  }
}

void fanHandler() {
  if (dc_dc_status.temp1 > 100.0 ||  dc_dc_status.temp2 > 100.0) {
    fanEnabled = true;
  }
  else if ( dc_dc_status.temp1 < 90 &&  dc_dc_status.temp2 < 90.0) {
    fanEnabled = false;
  }
  
  digitalWrite(fanPin, fanEnabled);
}

void sendData() 
{
  // Write the header
  byte header[] = {0xAA, 0xBB, 0xCC, 0xDD };
  Serial.write(header, sizeof(header)); 
  Serial.write((byte *)&dc_dc_status, sizeof(dc_dc_status));  
  Serial.write((byte *)&controller_status, sizeof(controller_status));   
}

void setup()
{
    //Serial
    Serial2.begin(115200);
    Serial.begin(115200);
  
    // Clear the data
    memset(&dc_dc_status, 0, sizeof(DCDCStatus));
    memset(&controller_status, 0, sizeof(ControllerStatus));

    // Set the fan state
    pinMode(fanPin, OUTPUT);
    digitalWrite(fanPin, fanEnabled);
    
    // Initialize the CAN
    while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
    {
      delay(100);
    }
}

void loop()
{
  fanHandler();
  dcDcCheckStatus(); 
  controllerCheckStatus();
  sendData();
}


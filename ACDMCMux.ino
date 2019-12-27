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
  uint32 phi_int;
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
      dc_dc_status.voltage = (buf[2] / 12.7) * 100;
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

void serialFlush(){
  byte w = 0;

   for (int i = 0; i < 10; i++)
   {
     while (Serial.available() > 0)
     {
       char k = Serial.read();
       w++;
       delay(1);
     }
     delay(1);
   }
}   

uint16 readControllerValue(char request)
{
  uint16 result = 0;   
  uint16 masked_result = 0; 
  
  Serial2.write('*');         // Send stop character
  delay(10);                  // Wait for it to be processed
    
  serialFlush();

  static char inBuff[2]; // Read a buffer
    
  Serial2.write(request);       
  Serial2.readBytes(inBuff, sizeof(inBuff));

  result = ntohs(*((uint16*)&inBuff[0]));  
      
  Serial2.write('*'); // Send Stop Char
 
  return result;
}

void storeTemp(uint8 which, uint16 raw_value) {
  float temp = ((float)(raw_value & 0x7FFF) / 2);
  controller_status.temps[which] = (cToF(temp) * 100);
}

void storeCurrent(uint8 which, uint16 raw_value) {
  // Raw value - 0 point (0x8000)
  // float current = raw_value - 0x800;

  // Divide by Clarke Factor (1.5)
  // Divide by Scaling Factor (16)
  // Divide by 10-bit ADC (1024)
  // Multiply by 5v (since 0-5v range)
  // Divide by mV/A for current sensor  
  controller_status.current[which] = raw_value; //current / 16 / 1.5 / 1024 * 5 / 0.0012;
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
  storeCurrent(0, readControllerValue('c')); // Phase Current, Filtered
  storeCurrent(1, readControllerValue('d')); // Phase Current, Requested
  storeCurrent(2, readControllerValue('t')); // Max Phase Current
  storeCurrent(3, readControllerValue('u')); // Field Weakening
  controller_status.phi_int = readControllerValue('b'); // Phi Int
}

void fanHandler() {
  if (dc_dc_status.temp1 > 10000 ||  dc_dc_status.temp2 > 10000) {
    fanEnabled = true;
  }
  else if ( dc_dc_status.temp1 < 9000 &&  dc_dc_status.temp2 < 9000) {
    fanEnabled = false;
  }
  
  digitalWrite(fanPin, fanEnabled);
}

void sendData() 
{  
  // Write the header
  byte header[] = { 0xAA, 0xBB, 0xCC, 0xDD };
  Serial.write(header, sizeof(header)); 

  // Write the dc-dc status out
  Serial.write((byte *)&dc_dc_status, sizeof(dc_dc_status));  
  Serial.write((byte *)&controller_status, sizeof(controller_status));   
}

void setup()
{
    // Serial
    Serial2.begin(115200);
    Serial.begin(19200);
  
    // Clear the data
    memset(&dc_dc_status, 0x00, sizeof(DCDCStatus));
    memset(&controller_status, 0x00, sizeof(ControllerStatus));

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
  sendData();
  fanHandler();
  dcDcCheckStatus(); 
  controllerCheckStatus();
}


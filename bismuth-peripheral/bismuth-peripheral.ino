#include <HCSR04.h>
#include <mcp_can.h>
#include <SPI.h>

UltraSonicDistanceSensor distanceSensor(29, 28);  // Initialize sensor that uses digital pins 13 and 12.


long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

byte sensorData[8] = {0};

union {
  float floatValue;
  byte data[4];
} distanceRead;

#define CAN0_INT 20                              // Set INT to pin 20
MCP_CAN CAN0(17);                               // Set CS to pin 17

void setup () {
    Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.

    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");
    
    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

    pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
    
    Serial.println("MCP2515 Library Receive Example...");
}

void loop () {
    // Read interuption pin in the MCP2515
    if(!digitalRead(CAN0_INT)) {
      //Read CAN frame
      CAN0.readMsgBuf(&rxId, &len, rxBuf);
      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

      Serial.print(msgString);

      if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME\n");
        Serial.print(msgString);
        distanceRead.floatValue = distanceSensor.measureDistanceCm();
        sensorData[0] = distanceRead.data[0];
        sensorData[1] = distanceRead.data[1];
        sensorData[2] = distanceRead.data[2];
        sensorData[3] = distanceRead.data[3];
        // send data:  ID = 0x224, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
        byte sndStat = CAN0.sendMsgBuf(0x224, 0, 4, sensorData);
        if(sndStat == CAN_OK){
          Serial.println("Message Sent Successfully!");
        } else {
          Serial.println("Error Sending Message...");
        }

      } else {
        for(byte i = 0; i<len; i++){
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
    }
    Serial.println();
    // delay(500);
}

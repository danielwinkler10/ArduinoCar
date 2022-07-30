/*************************************************************************************************
    OBD-II_PIDs TEST CODE
    LOOVEE @ JUN24, 2017

    Query
    send id: 0x7df
      dta: 0x02, 0x01, PID_CODE, 0, 0, 0, 0, 0

    Response
    From id: 0x7E9 or 0x7EA or 0x7EB
      dta: len, 0x41, PID_CODE, byte0, byte1(option), byte2(option), byte3(option), byte4(option)

    https://en.wikipedia.org/wiki/OBD-II_PIDs

    Input a PID, then you will get reponse from vehicle, the input should be end with '\n'
***************************************************************************************************/
#include <SPI.h>

#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

#define PID_ENGIN_PRM       0x0C
#define PID_VEHICLE_SPEED   0x0D
#define PID_COOLANT_TEMP    0x05

#define CAN_ID_PID          0x7DF

unsigned char PID_INPUT;
unsigned char getPid    = 0;
int counter             = 0;
unsigned char tmp       = 0;

void set_mask_filt() {
    /*
        set mask, set both the mask to 0x3ff
    */
    CAN.init_Mask(0, 0, 0x7FC);
    CAN.init_Mask(1, 0, 0x7FC);

    /*
        set filter, we can receive id from 0x04 ~ 0x09
    */
    CAN.init_Filt(0, 0, 0x7E8);
    CAN.init_Filt(1, 0, 0x7E8);

    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8);
    CAN.init_Filt(5, 0, 0x7E8);
}

void sendPid(unsigned char __pid) {
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
//    SERIAL_PORT_MONITOR.print("SEND PID: 0x");
//    SERIAL_PORT_MONITOR.println(__pid, HEX);
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    set_mask_filt();
}

char send_receive(unsigned char* buf, unsigned char pid) {
  sendPid(pid);
  delay(20);
  CAN.readMsgBuf(&tmp, buf);
  CAN.readMsgBuf(&tmp, buf);
  return getBuffer(buf, pid);
}

void loop() {
  unsigned char buf[8];
  /*
  SERIAL_PORT_MONITOR.print("Loop number: ");
  SERIAL_PORT_MONITOR.println(counter);
  */
  
  if (send_receive(buf, 0xd)) {
    parsekph(buf);
  }

  if (send_receive(buf, 0xc)) {
    parseEngineSpeed(buf);
  }

  if (send_receive(buf, 0x11)){
    throttlePosition(buf);
  }



  // Cleanup
  CAN.readMsgBuf(&tmp, buf);
  CAN.readMsgBuf(&tmp, buf);
  counter++;
  /*
  sendPid(0xd);
  if (getBuffer(buf, 0xd)){
    parsekph(buf);
  }

  delay(1000);

  
  sendPid(0xc);
  if (getBuffer(buf, 0xc)){
    parseEngineSpeed(buf);
  }

  delay(1000);
  */
//
//  sendPid(0x11);
//  if (getBuffer(buf, 0x11)){
//    throttlePosition(buf);
//  }
//
//  delay(1000);

  SERIAL_PORT_MONITOR.println("\r\n------------------------------------------------------------------");
}

void parseEngineSpeed(unsigned char* buf){
    unsigned int A = buf[3];
    unsigned int B = buf[4];

    float Result = (A * 256 + B) / 4;
    
    SERIAL_PORT_MONITOR.print("Engine Speed: ");
    SERIAL_PORT_MONITOR.print(Result);
    SERIAL_PORT_MONITOR.println(" rpm");
}

void parsekph (unsigned char* buf){
    unsigned int kph = buf[3];
    SERIAL_PORT_MONITOR.print("Speed: ");
    SERIAL_PORT_MONITOR.print(kph, DEC);
    SERIAL_PORT_MONITOR.println(" km/h");
}

void throttlePosition (unsigned char* buf){
    unsigned int A = buf[3];

    float Result = (A * 100) / 255;
    SERIAL_PORT_MONITOR.print("Throttle position: ");
    SERIAL_PORT_MONITOR.print(Result);
    SERIAL_PORT_MONITOR.println("%");
}

void transmissionActualGear(unsigned char* buf){

  //A4
    unsigned int A = buf[3];
    unsigned int B = buf[4];
    unsigned int C = buf[5];
    unsigned int D = buf[6];

    float Result = ((C * 256) + D) / 1000;
    SERIAL_PORT_MONITOR.print("Transmission actual gear: ");
    SERIAL_PORT_MONITOR.println(Result);   
}

void fuelTankLevelInput (unsigned char* buf){
  // 2F
    unsigned int A = buf[3];

    float Result = (A * 100) / 255;
    SERIAL_PORT_MONITOR.print("Fuel tank level input : ");
    SERIAL_PORT_MONITOR.print(Result);  
  SERIAL_PORT_MONITOR.println("%");
 
}

// 5E

   void engineFuelRate   (unsigned char* buf){
// 5E
    unsigned int A = buf[3];
    unsigned int B = buf[4];

    float Result = ((A * 256) + B) / 20;
    SERIAL_PORT_MONITOR.print("Engine fuel rate: ");
    SERIAL_PORT_MONITOR.print(Result);  
    SERIAL_PORT_MONITOR.println("L/h");
 
}

char getBuffer(unsigned char* buf, unsigned char pid){
  unsigned char len = 0;
  if (CAN_MSGAVAIL == CAN.checkReceive()) {                // check if get data
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    if (buf[2] != pid) {
      SERIAL_PORT_MONITOR.print("Expected: ");
      SERIAL_PORT_MONITOR.print(pid);
      SERIAL_PORT_MONITOR.print(" bur received: ");
      SERIAL_PORT_MONITOR.println(buf[2]);
    }
    return 1;
  }
  SERIAL_PORT_MONITOR.println("No message available");
  return 0;
}


void taskCanRecv() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive()) {                // check if get data
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        SERIAL_PORT_MONITOR.println("\r\n------------------------------------------------------------------");
        SERIAL_PORT_MONITOR.print("Get Data From id: 0x");
        SERIAL_PORT_MONITOR.println(CAN.getCanId(), HEX);
        for (int i = 0; i < len; i++) { // print the data
            SERIAL_PORT_MONITOR.print("0x");
            SERIAL_PORT_MONITOR.print(buf[i], HEX);
            SERIAL_PORT_MONITOR.print("\t");
        }
        SERIAL_PORT_MONITOR.println();
    }
}

void taskDbg() {
    while (SERIAL_PORT_MONITOR.available()) {
        char c = SERIAL_PORT_MONITOR.read();

        if (c >= '0' && c <= '9') {
            PID_INPUT *= 0x10;
            PID_INPUT += c - '0';

        } else if (c >= 'A' && c <= 'F') {
            PID_INPUT *= 0x10;
            PID_INPUT += 10 + c - 'A';
        } else if (c >= 'a' && c <= 'f') {
            PID_INPUT *= 0x10;
            PID_INPUT += 10 + c - 'a';
        } else if (c == '\n') { // END
            getPid = 1;
        }
    }
}
// END FILE

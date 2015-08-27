#include <SoftwareSerial.h>
#include <JPEGCamera.h>
#include <XBee.h>

/**
 * Arduino UNO R3
 *   0: UART RX - XBee
 *   1: UART TX - XBee
 *   2: XBee CTS Interrupt
 * ~ 3: Motor A
 *   4: XBee RSSI ( pulseIn(digitalPin, LOW, 200); ) no need
 * ~ 5: Servo 5
 * ~ 6: Servo 6
 *   7: Camera XBee RX
 *   8: Motor B Brake
 * ~ 9: Motor A Brake
 * ~10: Camera XBee TX
 * ~11: Motor B
 *  12: Motor A Direction
 *  13: Motor B Direction
 *
 * A0(~14): Motor A Sensing
 * A1(~15): Motor B Sensing
 * A2( 16):
 * A3( 17):
 * A4( 18):
 * A5( 19):
 */



#define USE_XBEE 1

// #define BAUD_RATE 115200 // noise on SoftwareXBee
#define BAUD_RATE 38400


// PINS
#define INUPT_PIN_XBEE_RSSI 4
#define INTERRUPT_PIN 2

// SIZE
#define CMD_HEAD_SIZE 5
#define CMD_READ_SIZE 32
#define CMD_FOOT_SIZE 6

const word read_size = CMD_READ_SIZE;
const word interval = 0x000A * 10; // XX XX * 0.01ms
const word BUF_SIZE = CMD_HEAD_SIZE + CMD_READ_SIZE + CMD_FOOT_SIZE + 6;
byte recv_buf[BUF_SIZE];

word picture_size = 0;
word picture_offset = 0;
byte is_big = 0;
volatile byte cts = LOW;

SoftwareSerial cameraSerila(7, 10);
JPEGCamera camera(cameraSerila);
XBee xbee;

AtCommandRequest atRequest;

void xbee_cts() {
  cts = (cts == LOW ? HIGH : LOW);
}

void setup() {
  // setup pins
  pinMode(INUPT_PIN_XBEE_RSSI, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  cts = digitalRead(INTERRUPT_PIN);
  attachInterrupt(0, xbee_cts, CHANGE); // 0: PIN_2, 1: PIN_3

  // XBee Sheild
  Serial.begin(BAUD_RATE);
  // wait XBee connet
  while (!Serial) delay(100);
  xbee.begin(Serial);

  // LS-Y201 setup
  cameraSerila.begin(BAUD_RATE);
  // set image size only one time
  camera.imageSize(JPEGCamera::IMG_SZ_160x120);
  
  // reset camera
  camera.reset();

  delay(4000);

  atRequest.setCommand((byte*)"Init end.");
  xbee.send(atRequest);
}

void loop() {
  // recv cmd
  word index = 0;
  byte cmd_buf[16];
  
  // byte xbee_rssi = pulseIn(INUPT_PIN_XBEE_RSSI, LOW, 200);

  xbee.readPacket();
  

  if (index > 0) {
    // TODO: execute cmd

    // take new picture if no data or send over.
    if (picture_size <= 0) {
      if (is_big) {
        camera.imageSize(JPEGCamera::IMG_SZ_640x480);
        is_big = 0;
      }

      // take picture
      camera.takePicture();
      
      camera.getSize(&picture_size);
      picture_offset = 0;
    }

    byte recv_size = camera.readData(recv_buf, read_size, picture_offset);
    if (recv_size != read_size)
      // read data failed.
      return;

    // calculate next read addres.
    picture_size -= recv_size;
    picture_offset += recv_size;
    // send to PC

    // stop picture
    if (picture_size <= 0) {
      camera.stopPictures();
      delay(100);
    }
  }
}


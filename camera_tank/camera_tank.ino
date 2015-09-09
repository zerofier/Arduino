#include <SoftwareSerial.h>
#include <JPEGCamera.h>
#include <XBee.h>

/**
 * Arduino UNO R3
 *   0: UART RX - XBee
 *   1: UART TX - XBee
 *   2: XBee CTS Interrupt
 * ~ 3: Motor A
 *   4:
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

// #define BAUD_RATE 115200 // noise on SoftwareSerila
#define BAUD_RATE 38400


// PINS
#define INTERRUPT_PIN 2

// SIZE
#define CMD_HEAD_SIZE 5
#define CMD_READ_SIZE 32
#define CMD_FOOT_SIZE 6

volatile byte CTS = LOW;

const word read_size = CMD_READ_SIZE;
const word BUF_SIZE = CMD_HEAD_SIZE + CMD_READ_SIZE + CMD_FOOT_SIZE + 6;
byte recv_buf[BUF_SIZE];
word picture_size = 0;
word picture_offset = 0;
byte take_big = 0;
bool active = false;

SoftwareSerial cameraSerila(7, 10);
JPEGCamera camera(cameraSerila);
XBee xbee;

AtCommandRequest atRequest;
AtCommandResponse atResponse;
XBeeAddress64 addr64;
ZBTxRequest zbTxRequest;
ZBTxStatusResponse zbTxStatus;
ZBRxResponse zbRxResponse;

void xbee_cts() {
  CTS = (CTS == LOW ? HIGH : LOW);
}

void setup() {
  // setup pins
  pinMode(INTERRUPT_PIN, INPUT);
  CTS = digitalRead(INTERRUPT_PIN);
  attachInterrupt(0, xbee_cts, CHANGE); // 0: PIN_2, 1: PIN_3

  // wait XBee connet
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(100);
  xbee.begin(Serial);
  findCoord();

  // LS-Y201 setup
  cameraSerila.begin(BAUD_RATE);
  // set image size only one time
  camera.imageSize(JPEGCamera::IMG_SZ_160x120);
  // reset camera
  camera.reset();
  delay(4000);
}

void loop() {
  // recv command from controller
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(zbRxResponse);

      active = true;
    }
  }

  if (active) {
    // take new picture if no data or send over.
    if (picture_size <= 0) {
      if (take_big) {
        camera.imageSizeOnce(JPEGCamera::IMG_SZ_640x480);
        take_big = 0;
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

    // add info to send data
    memcpy(recv_buf + 2, recv_buf, recv_size);
    recv_buf[0] = picture_offset >> 8;
    recv_buf[1] = picture_offset & 0xFF;
    
    // send to remote
    zbTxRequest.setPayload(recv_buf);
    zbTxRequest.setPayloadLength(recv_size + 2);
    // send data to controller
    xbee.send(zbTxRequest);

    // calculate next read addres.
    picture_size -= recv_size;
    picture_offset += recv_size;

    // stop picture
    if (picture_size <= 0) {
      camera.stopPictures();
      delay(100);
    }

    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
      if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(zbTxStatus);
      }
    } else {
      // TODO: timeout -> retry send ?
    }
  }
}

bool findCoord() {
  atRequest.setCommand((byte*)"ND");

  // send the command
  xbee.send(atRequest);

  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
    if (xbee.getResponse().getApiId() == AT_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);

      addr64 = XBeeAddress64(0x00000000, 0x0000FFFF);
      zbTxRequest = ZBTxRequest(addr64, NULL, NULL);
      return true;
    }
  }

  return false;
}


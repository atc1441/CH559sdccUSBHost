uint8_t  uartRxBuff[1024];
int  rxPos = 0;
int  cmdLength = 0;
uint8_t  cmdType = 0;
long lastRxReceive = 0;

String deviceType[] = {"UNKNOWN", "POINTER", "MOUSE", "RESERVED", "JOYSTICK", "GAMEPAD", "KEYBOARD", "KEYPAD", "MULTI_AXIS", "SYSTEM"};
String keyboardstring;
void setup(void) {
  Serial.begin(230400);
  Serial1.begin(1000000, SERIAL_8N1, 16, 17);
  Serial.println("OK There");
}

void loop() {
  while (Serial1.available())
  {
    lastRxReceive = millis();
    //Serial.print("h0x");//Only for Debug
    //Serial.print(Serial1.peek(),HEX);//Only for Debug
    //Serial.print(" ");//Only for Debug
    uartRxBuff[rxPos] = Serial1.read();
    if (rxPos == 0 && uartRxBuff[rxPos] == 0xFE) {
      cmdType = 1;
    } else if (rxPos == 1 && cmdType == 1) {
      cmdLength = uartRxBuff[rxPos];
    } else if (rxPos == 2 && cmdType == 1) {
      cmdLength += (uartRxBuff[rxPos] << 8);
      //printf( "Length: %i\n", cmdLength);//Only for Debug
    } else if (cmdType == 0 && uartRxBuff[rxPos] == '\n') {
      printf("No COMMAND Received\n");
      for (uint8_t i = 0; i < rxPos; i ++ )
      {
        printf( "0x%02X ", uartRxBuff[i]);
      }
      printf("\n");
      rxPos = 0;
      cmdType = 0;
    }
    if (rxPos > 0 && rxPos == cmdLength + 11 && cmdType || rxPos > 1024) {
      filterCommand(cmdLength, uartRxBuff);
      for (int i = 0; i < rxPos; i ++ )
      {
        //printf( "0x%02X ", uartRxBuff[i]);//Only for Debug
      }
      //printf("\n");//Only for Debug
      rxPos = 0;
      cmdType = 0;
    } else {
      rxPos++;
    }

  }
  rxPos = 0;

  if (Serial.available())
  {
    Serial1.write(Serial.read());
  }
}

#define MSG_TYPE_CONNECTED      0x01
#define MSG_TYPE_DISCONNECTED   0x02
#define MSG_TYPE_ERROR          0x03
#define MSG_TYPE_DEVICE_POLL    0x04
#define MSG_TYPE_DEVICE_STRING  0x05
#define MSG_TYPE_DEVICE_INFO    0x06
#define MSG_TYPE_HID_INFO       0x07
#define MSG_TYPE_STARTUP        0x08



void filterCommand(int buffLength, unsigned char *msgbuffer) {
  int cmdLength = buffLength;
  unsigned char msgType = msgbuffer[3];
  unsigned char devType = msgbuffer[4];
  unsigned char device = msgbuffer[5];
  unsigned char endpoint = msgbuffer[6];
  unsigned char idVendorL = msgbuffer[7];
  unsigned char idVendorH = msgbuffer[8];
  unsigned char idProductL = msgbuffer[9];
  unsigned char idProductH = msgbuffer[10];
  switch (msgType) {
    case MSG_TYPE_CONNECTED:
      Serial.print("Device Connected on port");
      Serial.println(device);
      break;
    case MSG_TYPE_DISCONNECTED:
      Serial.print("Device Disconnected on port");
      Serial.println(device);
      break;
    case MSG_TYPE_ERROR:
      Serial.print("Device Error ");
      Serial.print(device);
      Serial.print(" on port ");
      Serial.println(devType);
      break;
    case MSG_TYPE_DEVICE_POLL:
      Serial.print("Device HID Data from port: ");
        Serial.print(device);
        Serial.print(" , Length: ");
        Serial.print(cmdLength);
        Serial.print(" , Type: ");
        Serial.print (deviceType[devType]);
        Serial.print(" , ID: ");
        for (int j = 0; j < 4; j++) {
        Serial.print("0x");
        Serial.print(msgbuffer[j + 7], HEX);
        Serial.print(" ");
        }
        Serial.print(" ,  ");
        for (int j = 0; j < cmdLength; j++) {
        Serial.print("0x");
        Serial.print(msgbuffer[j + 11], HEX);
        Serial.print(" ");
        }
        Serial.println();
      break;
    case MSG_TYPE_DEVICE_STRING:
      Serial.print("Device String port ");
      Serial.print(devType);
      Serial.print(" Name: ");
      for (int j = 0; j < cmdLength; j++)
        Serial.write(msgbuffer[j + 11]);
      Serial.println();
      break;
    case MSG_TYPE_DEVICE_INFO:
      Serial.print("Device info from port");
      Serial.print(device);
      Serial.print(", Descriptor: ");
      for (int j = 0; j < cmdLength; j++) {
        Serial.print("0x");
        Serial.print(msgbuffer[j + 11], HEX);
        Serial.print(" ");
      }
      Serial.println();
      break;
    case MSG_TYPE_HID_INFO:
      Serial.print("HID info from port");
      Serial.print(device);
      Serial.print(", Descriptor: ");
      for (int j = 0; j < cmdLength; j++) {
        Serial.print("0x");
        Serial.print(msgbuffer[j + 11], HEX);
        Serial.print(" ");
      }
      Serial.println();
      break;
    case MSG_TYPE_STARTUP:
      Serial.println("USB host ready");
      break;

  }
}

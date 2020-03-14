// REPORTING INTEGRATION

/***DEFINES***************************************************************************************************/

#define BLUETOOTH
#define ULTRASOUND
#define HEARTBEAT
#define FLEX
#define PIEZO
//#define STRUCT_DEBUG

/***STRUCT****************************************************************************************************/

// Aligned/Packed bit field struct for bluetooth transmission
typedef struct __attribute__((aligned(2), packed)) data_st {
  unsigned long int MtoS; // Message Sequence Number
  unsigned short f1; // Flex Data
  unsigned short f2; // Flex Data
  unsigned short f3; // Flex Data
  unsigned short f4; // Flex Data
  unsigned short us; // Ultrasound Data
  unsigned short hb; // Heartbeat Data
  unsigned short delayNum; // Sampling Rate
  unsigned char p1 : 1, p2 : 1, p3 : 1, p4 : 1, rType : 1, S_ACK : 1, : 2;
  // Piezo Data, Report type (0 = Periodic), Slave ACK bit
} DATA;
DATA data = {0, 1023, 1022, 1021, 1020, 1019, 1018, 1, 1, 0, 1, 0, 0, 0}; // Initialize data struct variable

//#ifndef STRUCT_DEBUG
//DATA data;
//#endif
//
//#ifdef STRUCT_DEBUG
//DATA data = {1023, 1022, 1021, 1020, 1019, 1018, 1, 0, 1, 0}; // Initialize data struct variable for debugging
//#endif

/***BLUETOOTH HEADER******************************************************************************************/

#ifdef BLUETOOTH
// ARDUINO UNO -> BLUETOOTH_TX
// UART -> 10 (RX), 11 (TX)
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX,TX
#endif

/***ULTRASOUND HEADER*****************************************************************************************/

#ifdef ULTRASOUND
const int trigPin = 8;    // Trigger
const int echoPin = 9;    // Echo
long duration;
#endif

/***HEARTBEAT HEADER******************************************************************************************/

#ifdef HEARTBEAT
unsigned short hbSensor = 0;
int SensorPin = 0;  // Pulse Sensor YELLOW WIRE connected to ANALOG PIN 0
#endif

/***FLEX HEADER***********************************************************************************************/

#ifdef FLEX
#include <SparkFun_ADS1015_Arduino_Library.h>
ADS1015 flexSensor;
#endif

/***PIEZO HEADER**********************************************************************************************/

#ifdef PIEZO
unsigned int piezoPin1 = 4;
unsigned int piezoPin2 = 5;
unsigned int piezoPin3 = 6;
unsigned int piezoPin4 = 7;
#endif

/***SETUP*****************************************************************************************************/

void setup() {
#ifdef BLUETOOTH
  bluetooth_master_INIT();
#endif

#ifdef ULTRASOUND
  ultrasound_INIT();
#endif

#ifdef PIEZO
  piezo_INIT();
#endif

#ifdef HEARTBEAT
  heartbeat_INIT();
#endif

#ifdef FLEX
  flex_INIT();
#endif
}

/***LOOP******************************************************************************************************/

void loop() {
#ifdef BLUETOOTH
  bluetooth_master_LOOP();
#endif

#ifdef ULTRASOUND
  ultrasound_LOOP();
#endif

#ifdef PIEZO
  piezo_LOOP();
#endif

#ifdef HEARTBEAT
  heartbeat_LOOP();
#endif

#ifdef FLEX
  flex_LOOP();
#endif
}

/***BLUETOOTH*************************************************************************************************/

#ifdef BLUETOOTH
void bluetooth_master_INIT() {
  Serial.begin(57600);
  BTSerial.begin(38400);  // HC-05 default speed in AT command more

  handshake2();
  handshake1();
}

void bluetooth_master_LOOP() {
  if (data.rType == 0) {
    data_Tx();
    delay(data.delayNum);
  } else {
    if (BTSerial.available()) {
      char BTbuff[100] = "";
      size_t readSize = BTSerial.readBytesUntil('\n', BTbuff, 100);
      if (strcmp(BTbuff, "DATA") == 0) {
        data_Tx();
      } else {
        Serial.println(BTbuff);
        command(BTbuff, readSize);
        data_Tx();
      }
      data.MtoS++;
    }
  }
}
#endif

/***ULTRASOUND************************************************************************************************/

#ifdef ULTRASOUND
void ultrasound_INIT() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void ultrasound_LOOP() {
  // Pulse Tx
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); // Pulse Rx
  unsigned short cm = (duration / 2) / 29.1; // Conversion to cm
  data.us = cm;
}
#endif

/***HEARTBEAT*************************************************************************************************/

#ifdef HEARTBEAT
void heartbeat_INIT() {
}

void heartbeat_LOOP() {
  data.hb = analogRead(SensorPin);  // Read the PulseSensor's value.
}
#endif

/***FLEX******************************************************************************************************/

#ifdef FLEX
void flex_INIT() {
  Wire.begin();
  if (flexSensor.begin(ADS1015_ADDRESS_GND, Wire) == false) {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }
  flexSensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS);
}

void flex_LOOP() {
  data.f1 = flexSensor.getAnalogData(0);
  data.f2 = flexSensor.getAnalogData(1);
  data.f3 = flexSensor.getAnalogData(2);
  data.f4 = flexSensor.getAnalogData(3);
}
#endif

/***PIEZO*****************************************************************************************************/

#ifdef PIEZO
void piezo_INIT() {

  pinMode(piezoPin1, INPUT);
  pinMode(piezoPin2, INPUT);
  pinMode(piezoPin3, INPUT);
  pinMode(piezoPin4, INPUT);
}

void piezo_LOOP() {
  data.p1 = digitalRead(piezoPin1);
  data.p2 = digitalRead(piezoPin2);
  data.p3 = digitalRead(piezoPin3);
  data.p4 = digitalRead(piezoPin4);
}
#endif

/***BLUETOOTH HELPER FUNCTIONS********************************************************************************/

void data_Tx() {
  bool DTAK = 0;
  BTSerial.write("DATA\n");
  size_t writeSize = BTSerial.write((char*)&data, sizeof(DATA));
  data.S_ACK = 0;
  if (writeSize != sizeof(DATA))
    Serial.print("writeSize ERROR");

  while (!DTAK) {
    char BTbuff[100] = "";
    if (BTSerial.available()) {
      size_t readSize = BTSerial.readBytesUntil('\n', BTbuff, 100);
      if (strcmp(BTbuff, "DTAK") == 0) {
        Serial.println("DATA ACK");
        DTAK = 1;
      } else {
        Serial.println(BTbuff);
        command(BTbuff, readSize);
      }
      data.MtoS++;
    }
  }
}

void command(char *command, size_t csize) {
  static bool delay_MOD = 0;
  data.S_ACK = 1;
  if (!strcmp(command, "RTYPE")) {
    data.rType = (data.rType == 0) ? 1 : 0;
  } else if (!strcmp(command, "HANDSHAKE")) {
    handshake2();
    handshake1();
  } else if (!strcmp(command, "DELAY")) {
    delay_MOD = 1;
  } else if (delay_MOD) {
    long num = strtol(command, csize, 10);
    if (num >= 1 && num <= 1000) {
      data.delayNum = num;
    }
    delay_MOD = 0;
  }
}

void handshake1() {
  Serial.write("Initiating Handshake\n");
  bool linked = 0;
  while (!linked) {
    char buff[100] = "";
    BTSerial.write("HANDSHAKE\n");
    BTSerial.readBytesUntil('\n', buff, 100);
    if (!strcmp(buff, "CONFIRMED")) {
      Serial.write("Handshake Complete\n");
      linked = 1;
    }
  }
}

void handshake2() {
  Serial.write("Initiating Handshake\n");
  bool linked = 0;
  while (!linked) {
    char buff[100] = "";
    BTSerial.readBytesUntil('\n', buff, 100);
    if (!strcmp(buff, "HANDSHAKE")) {
      Serial.write("Handshake Complete\n");
      BTSerial.write("CONFIRMED\n");
      linked = 1;
    }
  }
}

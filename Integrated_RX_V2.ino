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
  unsigned char p1 : 1, p2 : 1, p3 : 1, p4 : 1, rType : 1, S_ACK : 1 , : 2;
  // Piezo Data, Report type (0 = Periodic), Slave ACK bit
} DATA;
DATA data;

bool b_display = 0;
unsigned long int StoM = 0; // Records number of messages sent from Slave to Master
unsigned long int TtoM = 0; // Records number of messages sent from Terminal to Master


/***MIDI******************************************************************************************************/

#include <MIDI.h>
MIDI_CREATE_DEFAULT_INSTANCE(); // Create an instance of MIDI

/***BLUETOOTH HEADER******************************************************************************************/

#ifdef BLUETOOTH
// ARDUINO MEGA -> BLUETOOTH_TX
// UART -> 10 (RX), 11 (TX)
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX,TX
#endif

/***ULTRASOUND HEADER*****************************************************************************************/

#ifdef ULTRASOUND
const int track_us = 1; // MIDI TRACK NO.
#endif

/***HEARTBEAT HEADER******************************************************************************************/

#ifdef HEARTBEAT
const int track_hb = 2; // MIDI TRACK NO.
#endif

/***FLEX HEADER***********************************************************************************************/

#ifdef FLEX
const int track_f1 = 3; // MIDI TRACK NO.
const int track_f2 = 4; // MIDI TRACK NO.
const int track_f3 = 5; // MIDI TRACK NO.
const int track_f4 = 6; // MIDI TRACK NO.
#endif

/***PIEZO HEADER**********************************************************************************************/

#ifdef PIEZO
const int track_p1 = 7; // MIDI TRACK NO.
const int track_p2 = 8; // MIDI TRACK NO.
const int track_p3 = 9; // MIDI TRACK NO.
const int track_p4 = 10; // MIDI TRACK NO.
#endif

/***SETUP*****************************************************************************************************/

void setup() {
#ifdef BLUETOOTH
  bluetooth_master_INIT();
#endif

#ifdef ULTRASOUND
  ultrasound_INIT();
#endif

#ifdef HEARTBEAT
  heartbeat_INIT();
#endif

#ifdef FLEX
  flex_INIT();
#endif

#ifdef PIEZO
  piezo_INIT();
#endif
}

/***LOOP******************************************************************************************************/

void loop() {
  if (!b_display) {
#ifdef BLUETOOTH
    bluetooth_master_Tx_LOOP();
#endif

#ifdef ULTRASOUND
    ultrasound_LOOP();
#endif

#ifdef HEARTBEAT
    heartbeat_LOOP();
#endif

#ifdef FLEX
    flex_LOOP();
#endif

#ifdef PIEZO
    piezo_LOOP();
#endif
  } else {
    bluetooth_master_LOOP();

  }
}

/***BLUETOOTH*************************************************************************************************/

#ifdef BLUETOOTH
void bluetooth_master_INIT() {
  Serial.begin(57600);
  BTSerial.begin(38400);  // HC-05 default speed in AT command more

  handshake1();
  handshake2();
}

void bluetooth_master_Tx_LOOP() {
  Serial.setTimeout(10);
  if (BTSerial.available()) {
    char BTbuff[100] = "";
    BTSerial.readBytesUntil('\n', BTbuff, 100);
    if (strcmp(BTbuff, "DATA") == 0) {
      BTSerial.readBytes((char*)&data, sizeof(DATA));
      BTSerial.write("DTAK\n");
    }
  }

  if (Serial.available()) {
    char buff[100] = "";
    Serial.readBytesUntil('\n', buff, 8);
    if (strcmp(buff, "DISPLAY") == 0) {
      b_display = !b_display;
    }
  }
}

void bluetooth_master_LOOP() {
  if (BTSerial.available()) {
    char BTbuff[100] = "";
    BTSerial.readBytesUntil('\n', BTbuff, 100);
    if (strcmp(BTbuff, "DATA") == 0) {
      size_t readSize = BTSerial.readBytes((char*)&data, sizeof(DATA));
      if (readSize != sizeof(DATA))
        Serial.print("readSize ERROR");
      else {
        BTSerial.write("DTAK\n");
        if (b_display) {
          printData();
        }
        if (data.S_ACK) {
          Serial.println("------SLAVE ACK---------");
        }
      }
    }
    StoM++;
  }
  if (Serial.available()) {
    char buff[100] = "";
    Serial.readBytesUntil('\n', buff, 100);
    Serial.println("------MASTER ACK--------");
    TtoM++;
    if (strcmp(buff, "CONFIG") == 0) {
      printConfig();
    } else if (strcmp(buff, "STATS") == 0) {
      printStats();
    } else if (strcmp(buff, "DISPLAY") == 0) {
      b_display = !b_display;
    } else if (b_display) {
      BTSerial.write(buff);
      BTSerial.write("\n");
    }
  }
}
#endif

/***ULTRASOUND************************************************************************************************/

#ifdef ULTRASOUND
void ultrasound_INIT() {
  MIDI.sendNoteOn(60, 127, track_us); // Send modulated Note
}

void ultrasound_LOOP() {
  if (data.us <= 15) {
    // Modulates note depending on nearness
    // Written so that the further you are the less the note is modulated,
    // with a cutoff at 15 cm.
    int pitch = map(data.us, 0, 15, 8192, 16383);
    MIDI.sendPitchBend(pitch, track_us); // Send modulated Note
  }
}
#endif

/***HEARTBEAT*************************************************************************************************/

#ifdef HEARTBEAT
void heartbeat_INIT() {

}

void heartbeat_LOOP() {
  const int sensitivity = 500;
  static bool hb_trigger_prev = 0;
  bool hb_trigger = 0;
  hb_trigger = (data.hb >= sensitivity) ? 1 : 0;

  if (hb_trigger && !hb_trigger_prev) {
    MIDI.sendNoteOn(60, 127, track_hb);
  }
  hb_trigger_prev = hb_trigger;
}
#endif

/***FLEX******************************************************************************************************/

#ifdef FLEX
void flex_INIT() {
}

void flex_LOOP() {
  static unsigned short prev1 = 0;
  static unsigned short prev2 = 0;
  static unsigned short prev3 = 0;
  static unsigned short prev4 = 0;
  int p1Mapped = map(data.f1, 1200, 720, 8192, 16383);
  int p2Mapped = map(data.f2, 1200, 720, 8192, 16383);
  int p3Mapped = map(data.f3, 1200, 720, 8192, 16383);
  int p4Mapped = map(data.f4, 1200, 720, 8192, 16383);

  if (p1Mapped != prev1) {
    // pinkie
    MIDI.sendPitchBend(p1Mapped, track_p3);
  }
  if (p2Mapped != prev2) {
    //index
    MIDI.sendPitchBend(p2Mapped, track_p4);
  }
  if (p3Mapped != prev3) {
    // Ring 
    MIDI.sendPitchBend(p3Mapped, track_p2);
  }
  if (p4Mapped != prev4) {
    // Middle
    MIDI.sendPitchBend(p4Mapped, track_hb);
  }
  prev1 = p1Mapped;
  prev2 = p2Mapped;
  prev3 = p3Mapped;
  prev4 = p4Mapped;
}
#endif

/***PIEZO*****************************************************************************************************/


#ifdef PIEZO
void piezo_INIT() {
}

void piezo_LOOP() {
  static bool prev1 = 0;
  static bool prev2 = 0;
  static bool prev3 = 0;
  static bool prev4 = 0;
  if (data.p1 && !prev1) {
    // pinkie
    MIDI.sendNoteOn(60, 127, track_us); // Start the note
  }
  if (data.p2 && !prev2) {
    // ring
    MIDI.sendNoteOn(60, 127, track_p2); // Start the note
  }
  if (data.p3 && !prev3) {
    // middle
    MIDI.sendNoteOn(60, 127, track_p3); // Start the note
  }
  if (data.p4 && !prev4) {
    // index
    MIDI.sendNoteOn(60, 127, track_p4); // Start the note
  }
  prev1 = data.p1;
  prev2 = data.p2;
  prev3 = data.p3;
  prev4 = data.p4;
}
#endif

/***BLUETOOTH HELPER FUNCTIONS********************************************************************************/

void printConfig() {
  char cbuf[100];
  sprintf(cbuf, "---------CONFIG---------");
  Serial.println(cbuf);
  sprintf(cbuf, "Sampling Rate: 1000/%u SPS", data.delayNum);
  Serial.println(cbuf);
  if (data.rType) {
    sprintf(cbuf, "Reporting Type: On-Demand");
  } else {
    sprintf(cbuf, "Reporting Type: Periodical");
  }
  Serial.println(cbuf);
  sprintf(cbuf, "--------/CONFIG---------");
  Serial.println(cbuf);
}

void printStats() {
  char cbuf[100];
  sprintf(cbuf, "---------STATS----------");
  Serial.println(cbuf);
  sprintf(cbuf, "Terminal to Master: %u", TtoM);
  Serial.println(cbuf);
  sprintf(cbuf, "Master to Slave: %u", data.MtoS);
  Serial.println(cbuf);
  sprintf(cbuf, "Slave to Master: %u", StoM);
  Serial.println(cbuf);
  sprintf(cbuf, "--------/STATS----------");
  Serial.println(cbuf);
}

void printData() {
  char cbuf[100];
  sprintf(cbuf, "Flex 1: %u, ", data.f1);
  Serial.print(cbuf);
  sprintf(cbuf, "Flex 2: %u, ", data.f2);
  Serial.print(cbuf);
  sprintf(cbuf, "Flex 3: %u, ", data.f3);
  Serial.print(cbuf);
  sprintf(cbuf, "Flex 4: %u, ", data.f4);
  Serial.print(cbuf);
  sprintf(cbuf, "Ultrasound: %u, ", data.us);
  Serial.print(cbuf);
  sprintf(cbuf, "Heartbeat: %u, ", data.hb);
  Serial.print(cbuf);
  sprintf(cbuf, "Piezo 1: %u, ", data.p1);
  Serial.print(cbuf);
  sprintf(cbuf, "Piezo 2: %u, ", data.p2);
  Serial.print(cbuf);
  sprintf(cbuf, "Piezo 3: %u, ", data.p3);
  Serial.print(cbuf);
  sprintf(cbuf, "Piezo 4: %u\n", data.p4);
  Serial.print(cbuf);
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

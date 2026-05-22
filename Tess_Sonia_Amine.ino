/******************************************************************************
HC-SR04 + LoRaWAN + Buzzer
Version allégée pour Arduino UNO / NANO

SUPPRIMÉ :
- FastLED
- IMU
- LTR303
- Animations LED

GARDÉ :
- HC-SR04
- Buzzer
- Température/Humidité SHTC3
- LoRaWAN LMIC
******************************************************************************/

#include "pitches.h"

#define Buzzer 7

// =====================================================
// LMIC
// =====================================================

#define CFG_EU 1
#define DISABLE_JOIN

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// =====================================================

#include <Wire.h>
#include "SHTC3.h"

SHTC3 s(Wire);

// =====================================================
// LORAWAN KEYS
// =====================================================

static const u4_t DEVADDR = 0x260BEEF3;

static const PROGMEM u1_t NWKSKEY[16] = {
  0x66,0x8B,0x27,0x13,
  0x82,0x9B,0xA3,0xB0,
  0x41,0x71,0xEA,0x77,
  0x9F,0x50,0xA0,0xD6
};

static const u1_t PROGMEM APPSKEY[16] = {
  0xF2,0x20,0x77,0x9D,
  0x42,0xA4,0xC6,0xA9,
  0xA6,0x95,0x55,0xCA,
  0x41,0x84,0x61,0x7B
};

// =====================================================

static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

// =====================================================
// LMIC PINS
// =====================================================

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};

// =====================================================
// HC-SR04
// =====================================================

const int trigPin = A3;
const int echoPin = A2;

float duration;
float distance;

// =====================================================
// BUZZER
// =====================================================

int melody[] = {
  NOTE_C4,
  NOTE_G3,
  NOTE_A3,
  NOTE_C4
};

int noteDurations[] = {
  4,4,4,4
};

// =====================================================
// BATTERY
// =====================================================

long readVcc() {

  long result;

  ADMUX = _BV(REFS0)
           | _BV(MUX3)
           | _BV(MUX2)
           | _BV(MUX1);

  delay(2);

  ADCSRA |= _BV(ADSC);

  while (bit_is_set(ADCSRA, ADSC));

  result = ADCL;
  result |= ADCH << 8;

  result = 1126400L / result;

  return result;
}

// =====================================================
// LORA SEND
// =====================================================

void do_send(osjob_t* j) {

  if (LMIC.opmode & OP_TXRXPEND) {

    return;
  }

  s.begin(true);

  int t = s.readTempC() * 10;
  int h = s.readHumidity() * 2;
  int bat = readVcc() / 10;
  int d = (int)distance;

  // Cayenne LPP Payload

  unsigned char mydata[15];

  // Temperature
  mydata[0] = 0x01;
  mydata[1] = 0x67;
  mydata[2] = t >> 8;
  mydata[3] = t & 0xFF;

  // Humidity
  mydata[4] = 0x02;
  mydata[5] = 0x68;
  mydata[6] = h & 0xFF;

  // Battery
  mydata[7] = 0x03;
  mydata[8] = 0x02;
  mydata[9] = bat >> 8;
  mydata[10] = bat & 0xFF;

  // Distance
  mydata[11] = 0x04;
  mydata[12] = 0x02;
  mydata[13] = d >> 8;
  mydata[14] = d & 0xFF;

  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
}

// =====================================================

void onEvent(ev_t ev) {

  switch(ev) {

    case EV_TXCOMPLETE:

      os_setTimedCallback(
        &sendjob,
        os_getTime() + sec2osticks(TX_INTERVAL),
        do_send
      );

      break;

    default:
      break;
  }
}

// =====================================================

void setup() {

  Serial.begin(9600);

  Wire.begin();

  s.begin(true);

  // HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Buzzer
  pinMode(Buzzer, OUTPUT);

  // ================= LMIC =================

  os_init();

  LMIC_reset();

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];

  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

  LMIC_setSession(
    0x1,
    DEVADDR,
    nwkskey,
    appskey
  );

#if defined(CFG_EU)

  LMIC_setupChannel(
    0,
    868100000,
    DR_RANGE_MAP(DR_SF12, DR_SF7),
    BAND_CENTI
  );

  LMIC_setupChannel(
    1,
    868300000,
    DR_RANGE_MAP(DR_SF12, DR_SF7B),
    BAND_CENTI
  );

  LMIC_setupChannel(
    2,
    868500000,
    DR_RANGE_MAP(DR_SF12, DR_SF7),
    BAND_CENTI
  );

#endif

  LMIC_setLinkCheckMode(0);

  LMIC.dn2Dr = DR_SF9;

  LMIC_setDrTxpow(DR_SF7, 14);

  do_send(&sendjob);
}

// =====================================================

void loop() {

  os_runloop_once();

  // =================================================
  // HC-SR04
  // =================================================

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = (duration * 0.0343) / 2;

  // =================================================
  // BUZZER
  // =================================================

  if(distance < 50.0) {

    for(int thisNote = 0; thisNote < 4; thisNote++) {

      int noteDuration = 1000 / noteDurations[thisNote];

      tone(
        Buzzer,
        melody[thisNote],
        noteDuration
      );

      delay(noteDuration * 1.3);

      noTone(Buzzer);
    }
  }

  delay(200);
}
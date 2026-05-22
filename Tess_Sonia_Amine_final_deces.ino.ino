/************************************************************
 UCA21 - HC-SR04 + SHTC3 + LoRaWAN (ABP) + Buzzer + DEBUG
 Arduino UNO / NANO
************************************************************/
 
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include "SHTC3.h"
 
// =====================================================
// DEBUG
// =====================================================
#define DEBUG 0 // Set to 1 for Serial logs; 0 leaves more SRAM for LMIC.
 
#if DEBUG
  #define DBG(x) do { Serial.println(F(x)); } while (0)
  #define DBG2(a,b) do { Serial.print(F(a)); Serial.println(b); } while (0)
#else
  #define DBG(x) do {} while (0)
  #define DBG2(a,b) do {} while (0)
#endif
 
// =====================================================
// CAPTEUR SHTC3
// =====================================================
SHTC3 s(Wire);
 
// =====================================================
// BUZZER
// =====================================================
const uint8_t BUZZER = 5;
const float BUZZER_DISTANCE_THRESHOLD_CM = 50.0f;
const unsigned int BUZZER_FREQUENCY_HZ = 2000;
const unsigned long BUZZER_ON_MS = 120;
const unsigned long BUZZER_OFF_MS = 180;
bool buzzerOn = false;
unsigned long nextBuzzerToggleMs = 0;

// =====================================================
// HC-SR04
// =====================================================
const int trigPin = A3;
const int echoPin = A2;
const unsigned long ECHO_TIMEOUT_US = 30000UL; // ~5 m max, avoids blocking the LMIC loop.
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 500;
 
unsigned long duration = 0;
float distance = 0;
unsigned long lastDistanceSampleMs = 0;
 
// =====================================================
// LORA (ABP TTN)
// =====================================================
static const u4_t DEVADDR = 0x260BEEF3;
 
static const PROGMEM u1_t NWKSKEY[16] = {
  0x66,0x8B,0x27,0x13,0x82,0x9B,0xA3,0xB0,
  0x41,0x71,0xEA,0x77,0x9F,0x50,0xA0,0xD6
};
 
static const u1_t PROGMEM APPSKEY[16] = {
  0xF2,0x20,0x77,0x9D,0x42,0xA4,0xC6,0xA9,
  0xA6,0x95,0x55,0xCA,0x41,0x84,0x61,0x7B
};
 
// =====================================================
void os_getArtEui (u1_t* buf) {}
void os_getDevEui (u1_t* buf) {}
void os_getDevKey (u1_t* buf) {}
 
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;
const unsigned TX_RETRY_INTERVAL = 5;
 
// =====================================================
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {6, 6, 6},
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
 
  return 1126400L / result;
}

int16_t encodeTemperature(float value) {
  if (isnan(value)) {
    return 0;
  }

  return (int16_t)(value * 10.0f);
}

uint16_t encodeHumidity(float value) {
  if (isnan(value) || value < 0) {
    return 0;
  }

  if (value > 100) {
    value = 100;
  }

  return (uint16_t)(value * 2.0f);
}

uint16_t encodeDistance(float value) {
  if (isnan(value) || value < 0) {
    return 0;
  }

  if (value > 65535) {
    value = 65535;
  }

  return (uint16_t)value;
}

void buzzerStart() {
  tone(BUZZER, BUZZER_FREQUENCY_HZ);
  buzzerOn = true;
}

void buzzerStop() {
  noTone(BUZZER);
  digitalWrite(BUZZER, LOW);
  buzzerOn = false;
}

void updateBuzzer() {
  bool shouldBuzz = distance > 0 && distance < BUZZER_DISTANCE_THRESHOLD_CM;

  if (!shouldBuzz) {
    if (buzzerOn) {
      buzzerStop();
    }
    return;
  }

  unsigned long now = millis();
  if ((long)(now - nextBuzzerToggleMs) < 0) {
    return;
  }

  if (buzzerOn) {
    buzzerStop();
    nextBuzzerToggleMs = now + BUZZER_OFF_MS;
  } else {
    buzzerStart();
    nextBuzzerToggleMs = now + BUZZER_ON_MS;
  }
}

void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  distance = duration > 0 ? (duration * 0.0343f) / 2.0f : 0;

  DBG2("Duration:", duration);
  DBG2("Distance:", distance);
}
 
// =====================================================
// SEND DATA
// =====================================================
void do_send(osjob_t* j) {
 
  DBG("====================================");
  DBG("DO_SEND appelé");
 
  if (LMIC.opmode & OP_TXRXPEND) {
    DBG("TX en cours -> skip");
    os_setTimedCallback(
      &sendjob,
      os_getTime() + sec2osticks(TX_RETRY_INTERVAL),
      do_send
    );
    return;
  }
 
  // ===== SHTC3 =====
  bool shtcOk = s.sample();
  float temp = shtcOk ? s.readTempC() : NAN;
  float hum  = shtcOk ? s.readHumidity() : NAN;
 
  int16_t t = encodeTemperature(temp);
  uint16_t h = encodeHumidity(hum);
 
  DBG2("Temp:", temp);
  DBG2("Hum :", hum);
 
  // ===== HC-SR04 =====
  measureDistance();
 
  uint16_t d = encodeDistance(distance);
#if DEBUG
  int bat = readVcc() / 10;
 
  DBG2("Battery:", bat);
#endif
 
  // ===== PAYLOAD =====
  byte payload[6];
 
  payload[0] = d >> 8;
  payload[1] = d & 0xFF;
 
  payload[2] = ((uint16_t)t) >> 8;
  payload[3] = ((uint16_t)t) & 0xFF;
 
  payload[4] = h >> 8;
  payload[5] = h & 0xFF;
 
  DBG("Payload bytes:");
#if DEBUG
  for (int i = 0; i < 6; i++) {
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif
 
  DBG("Envoi LMIC...");
 
  LMIC_setTxData2(1, payload, sizeof(payload), 0);
 
  DBG("LMIC_setTxData2 OK");
}
 
// =====================================================
// EVENTS LMIC
// =====================================================
void onEvent(ev_t ev) {
 
  DBG("LMIC EVENT");
 
  switch(ev) {
 
    case EV_TXCOMPLETE:
      DBG("EV_TXCOMPLETE -> envoi OK");
 
      os_setTimedCallback(
        &sendjob,
        os_getTime() + sec2osticks(TX_INTERVAL),
        do_send
      );
      break;
 
    case EV_TXSTART:
      DBG("EV_TXSTART");
      break;
 
    case EV_RXCOMPLETE:
      DBG("EV_RXCOMPLETE");
      break;
 
    default:
      DBG("Autre event");
      break;
  }
}
 
// =====================================================
// SETUP
// =====================================================
void setup() {
 
#if DEBUG
  Serial.begin(115200);
  delay(2000);
#endif
 
  DBG("BOOT START");
 
  Wire.begin();
 
  s.begin(true);
  DBG("SHTC3 OK");
 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(BUZZER, OUTPUT);
  buzzerStop();
 
  DBG("GPIO OK");
 
  // ===== LMIC =====
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
 
  uint8_t appskey[16];
  uint8_t nwkskey[16];
 
  memcpy_P(appskey, APPSKEY, 16);
  memcpy_P(nwkskey, NWKSKEY, 16);
 
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
 
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
 
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
 
  DBG("LMIC INIT OK");
 
  do_send(&sendjob);
}
 
// =====================================================
// LOOP
// =====================================================
void loop() {
 
  os_runloop_once();

  unsigned long now = millis();
  if (now - lastDistanceSampleMs >= DISTANCE_SAMPLE_INTERVAL_MS) {
    lastDistanceSampleMs = now;
    measureDistance();
  }

  updateBuzzer();
}

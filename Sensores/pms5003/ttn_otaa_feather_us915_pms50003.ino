/*******************************************************************************

 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#define RXD2 16 // To sensor TXD
#define TXD2 17 // To sensor RXD
// DHT digital pin and sensor type

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x32, 0x54, 0x76, 0x98, 0x78, 0x56, 0x34, 0x12 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xA8, 0x4A, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x77, 0x0E, 0xB6, 0x66, 0x81, 0xD9, 0xC2, 0xCE, 0x08, 0xD4, 0xD8, 0xF6, 0xE6, 0x14, 0xE5, 0x21 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// payload to send to TTN gateway
static uint8_t payload[13];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping for Adafruit Feather M0 LoRa
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26,33,LMIC_UNUSED_PIN},
};
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      if (readPMSdata(&Serial1)) {
        //particulas de 2.5
        float pm25 = data.particles_25um;
        Serial.print("Particles > 2.5 um / 0.1L air: "); Serial.print(pm25);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm25 = pm25 / 100000;

        //particulas de 0.3
        float pm03 = data.particles_03um;
        Serial.print("Particles > 0.3 um / 0.1L air: "); Serial.print(pm03);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm03 = pm03 / 100000;

        //particulas de 0.5
        float pm05 = data.particles_05um;
        Serial.print("Particles > 0.5 um / 0.1L air: "); Serial.print(pm05);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm05 = pm05 / 100000;

        //particulas de 1
        float pm01 = data.particles_10um;
        Serial.print("Particles > 1.0 um / 0.1L air: "); Serial.print(pm01);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm01 = pm01 / 100000;

        //particulas de 5
        float pm5 = data.particles_03um;
        Serial.print("Particles > 5.0 um / 0.1L air: "); Serial.print(pm5);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm5 = pm5 / 100000;

        //particulas de 10
        float pm10 = data.particles_100um;
        Serial.print("Particles > 10.0 um / 0.1L air: "); Serial.print(pm10);
        Serial.println("");
        // adjust for the f2sflt16 range (-1 to 1)
        pm10 = pm10 / 100000;

        //particulas2.5
        uint16_t payloadpm25 = LMIC_f2sflt16(pm25);
        // int -> bytes
        byte pm25Low = lowByte(payloadpm25);
        byte pm25High = highByte(payloadpm25);
        // place the bytes into the payload
        payload[0] = pm25Low;
        payload[1] = pm25High;

        // particulas03
        uint16_t payloadpm3 = LMIC_f2sflt16(pm03);
        // int -> bytes
        byte pm3Low = lowByte(payloadpm3);
        byte pm3High = highByte(payloadpm3);
        // place the bytes into the payload
        payload[2] = pm3Low;
        payload[3] = pm3High;
        
        // particulas0.5
        uint16_t payloadpm05 = LMIC_f2sflt16(pm05);
        // int -> bytes
        byte pm05Low = lowByte(payloadpm05);
        byte pm05High = highByte(payloadpm05);
        // place the bytes into the payload
        payload[4] = pm05Low;
        payload[5] = pm05High;

        // particulas1
        uint16_t payloadpm1 = LMIC_f2sflt16(pm01);
        // int -> bytes
        byte pm1Low = lowByte(payloadpm1);
        byte pm1High = highByte(payloadpm1);
        // place the bytes into the payload
        payload[6] = pm1Low;
        payload[7] = pm1High;

        // particulas5
        uint16_t payloadpm5 = LMIC_f2sflt16(pm5);
        // int -> bytes
        byte pm5Low = lowByte(payloadpm5);
        byte pm5High = highByte(payloadpm5);
        // place the bytes into the payload
        payload[8] = pm5Low;
        payload[9] = pm5High;

        // particulas10
        uint16_t payloadpm10 = LMIC_f2sflt16(pm10);
        // int -> bytes
        byte pm10Low = lowByte(payloadpm10);
        byte pm10High = highByte(payloadpm10);
        // place the bytes into the payload
        payload[10] = pm10Low;
        payload[11] = pm10High;
      
        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
      }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
 
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println(F("Starting"));
    delay(5000);
    while (! Serial);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0);
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF7,14);
    // in the US, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
    // get overridden after the join by parameters from the network. If working with other
    // networks or in other regions, this will need to be changed.
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
  // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
  // of the things that will happen is callbacks for transmission complete or received messages. We also
  // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
  // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
  // will want to call `os_runloop_once()` every so often, to keep the radio running.
  os_runloop_once();
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
 
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
 
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */
 
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
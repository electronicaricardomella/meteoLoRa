/*******************************************************************************
 * The Things Network - Sensor Data Example
 *
 * Example of sending a valid LoRaWAN packet with DHT22 temperature and
 * humidity data to The Things Networ using a Feather M0 LoRa.
 *
 * Learn Guide: https://learn.adafruit.com/the-things-network-for-feather
 *
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2018 Brent Rubell, Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//#include <Adafruit_BME280.h>
#include "DFRobot_LTR390UV.h"
DFRobot_LTR390UV ltr390(/*addr = */LTR390UV_DEVICE_ADDR, /*pWire = */&Wire);

//Adafruit_BME280 bme; // use I2C interface
//Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
//Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
//Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

// include the DHT22 Sensor Library
//#include "DHT.h"

// DHT digital pin and sensor type
//#define DHTPIN 10
//#define DHTTYPE DHT22

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
static const u1_t PROGMEM APPEUI[8] = { 0x41, 0x65, 0x20, 0x16, 0x51, 0x20, 0x16, 0x95 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xB8, 0x4D, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x54, 0xA9, 0xC0, 0x17, 0xD8, 0xCC, 0x5A, 0x03, 0xFA, 0x89, 0x17, 0x16, 0xBE, 0xDC, 0xC2, 0x09 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// payload to send to TTN gateway
static uint8_t payload[5];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for Adafruit Feather M0 LoRa
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, LMIC_UNUSED_PIN},
    //.rxtx_rx_active = 0,
    //.rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    //.spi_freq = 8000000,
};

// init. DHT
//DHT dht(DHTPIN, DHTTYPE);

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
        // read the temperature from the DHT22
        //float temperature = bme.readTemperature();
        ltr390.setMode(ltr390.eALSMode);//Set ambient light mode
        float Luminosidad = 0;
        Luminosidad = ltr390.readALSTransformData();//Get data converted from ambient light intensity, which can only be used in ambient light mode
        Serial.print("Luminosidad: "); 
        Serial.print(Luminosidad);
        Serial.println("Lux");
        delay(1000);
        // adjust for the f2sflt16 range (-1 to 1)
        Luminosidad = Luminosidad / 10000;

        // read the humidity from the DHT22
        //float rHumidity = bme.readHumidity();
        ltr390.setMode(ltr390.eUVSMode);//Set UV mode
        uint32_t Indice_UV = 0;
        float Indice_UV_float = 0;
        Indice_UV = ltr390.readOriginalData();//Get UV raw data
        Serial.print("Indice UV: ");
        Serial.println(Indice_UV);
        delay(1000);
        // adjust for the f2sflt16 range (-1 to 1)

        Indice_UV_float = float(Indice_UV);
        Serial.print("Indice UV_float: ");
        Serial.println(Indice_UV_float);

        Indice_UV_float = Indice_UV_float / 10;

        // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadLuminosidad = LMIC_f2sflt16(Luminosidad);
        // int -> bytes
        byte LuminosidadLow = lowByte(payloadLuminosidad);
        byte LuminosidadHigh = highByte(payloadLuminosidad);
        // place the bytes into the payload
        payload[0] = LuminosidadLow;
        payload[1] = LuminosidadHigh;

       
        // float -> int
        uint16_t payloadIndice_UV = LMIC_f2sflt16(Indice_UV_float);
        // int -> bytes
        byte Indice_UVLow = lowByte(payloadIndice_UV);
        byte Indice_UVHigh = highByte(payloadIndice_UV);
        payload[2] = Indice_UVLow;
        payload[3] = Indice_UVHigh;

        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    //delay(5000);
    //while (! Serial);
    Serial.begin(115200);
    Serial.println(F("Starting"));

    //bme.begin(0x76);

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
    //LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    //Serial.begin(115200);
    while(ltr390.begin() != 0){
      Serial.println(" Sensor initialize failed!!");
      delay(1000);
    }
    Serial.println(" Sensor  initialize success!!");
    ltr390.setALSOrUVSMeasRate(ltr390.e18bit,ltr390.e100ms);//18-bit data, sampling time of 100ms 
    ltr390.setALSOrUVSGain(ltr390.eGain3);//Gain of 3
    ltr390.setMode(ltr390.eALSMode);//Set ambient light mode
}

void loop() {
  // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
  // of the things that will happen is callbacks for transmission complete or received messages. We also
  // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
  // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
  // will want to call `os_runloop_once()` every so often, to keep the radio running.
  os_runloop_once();
  float Luminosidad = 0;
  Luminosidad = ltr390.readALSTransformData();//Get data converted from ambient light intensity, which can only be used in ambient light mode
  //Serial.print("ALS:");
  //Serial.print(als);
  //Serial.println("Lux");
  //delay(1000);
   uint32_t Indice_UV = 0;
  Indice_UV = ltr390.readOriginalData();//Get UV raw data
  //Serial.print("data:");
  //Serial.println(data);
}

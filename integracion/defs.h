#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_sleep.h>
#include <esp_adc_cal.h>
#include <WiFi.h>
#include <DFRobot_MultiGasSensor.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_INA219.h>
#include <SensorModbusMaster.h>
#include "PMS.h"

#define INTERVALO 60  // 1 minuto de siesta
#define SCK 10 // GPIO41 -- SX1278's SCK3
#define MISO 11 // GPIO43 -- SX1278's MISO3
#define MOSI 12 // GPIO42 -- SX1278's MOSI3
#define SS 13 // GPIO40 -- SX1278's SS3
#define RST 45 // GPIO45 -- SX1278's RESET
#define DI0 21 // GPIO44 -- SX1278's IRQ(Interrupt Request)
#define DI1 14 // No se usa pero hay que asignarlo. Poner uno libre.

#define SDA_PIN 9 
#define SCL_PIN 8
//#define I2C_CO 0x60  
#define I2C_NO2 0x61
#define I2C_SO2 0x74
#define I2C_O3 0x75
#define I2C_ENS160 0x53
#define I2C_batCarga 0x40
#define I2C_batDescarga 0x41
#define I2C_CO2 0x69

#define dirAnem 0x01
#define dirPira 0x02
#define dirPluv 0x03
#define dirVele 0x04

//DFRobot_GAS_I2C CO(&Wire ,I2C_CO); Retirado
DFRobot_GAS_I2C NO2(&Wire ,I2C_NO2);
DFRobot_GAS_I2C SO2(&Wire ,I2C_SO2);
DFRobot_GAS_I2C O3(&Wire ,I2C_O3);
Adafruit_INA219 batCarga(I2C_batCarga);
Adafruit_INA219 batDescarga(I2C_batDescarga);
DFRobot_ENS160_I2C ENS160(&Wire,I2C_ENS160);
Ezo_board CO2 = Ezo_board(I2C_CO2,&Wire);

Adafruit_BME280 bme;
modbusMaster anem;
modbusMaster pira;
modbusMaster pluv;
modbusMaster vele;

PMS pms(Serial2);

uint8_t datasend[40];

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RST,
    .dio = {DI0, DI1, LMIC_UNUSED_PIN},
};
#include "defs.h"
#include "TTNdata.h"

void setup() {
  Serial.begin(9600);
  Serial.println(F("Iniciando periféricos ..."));
  WiFi.disconnect(true);  // Desconecta de la red
  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  btStop(); 
  SPI.begin(SCK,MISO,MOSI,SS);
  Wire.begin(SDA_PIN, SCL_PIN);         
  delay(100); 
  // *** Sensor de ambiente ***
  while(!bme.begin(0x76, &Wire) && !bme.begin(0x77, &Wire))
  {
    Serial.println("Sensor BME no encontrado en las direcciones 0x76 ni 0x77");  delay(1000);
  }
  Serial.println(F("--- *** ---"));   Serial.print("Detectado sensor BM* de tipo: 0x"); Serial.println(bme.sensorID(),16);   Serial.println(F("--- *** ---"));
  
  // *** Sensores de gases ***
  /*while(!CO.begin())
  {
    Serial.println("Sensor de CO no encontrado !");   delay(1000);
  }
  Serial.println("Sensor de CO conectado correctamente !");
  CO.changeAcquireMode(CO.PASSIVITY);
  delay(500);
  CO.setTempCompensation(CO.OFF);
  */  
  while(!NO2.begin())
  {
    Serial.println("Sensor de NO2 no encontrado !");  delay(1000);
  }
  Serial.println("Sensor de NO2 conectado correctamente !");
  NO2.changeAcquireMode(NO2.PASSIVITY);
  NO2.setTempCompensation(NO2.OFF);
  while(!O3.begin())
  {
    Serial.println("Sensor de O3 no encontrado !");  delay(1000);
  }
  Serial.println("Sensor de O3 conectado correctamente !");
  O3.changeAcquireMode(O3.PASSIVITY);
  O3.setTempCompensation(O3.OFF);

  while(!SO2.begin())
  {
    Serial.println("Sensor de SO2 no encontrado !");  delay(1000);
  }
  Serial.println("Sensor de SO2 conectado correctamente !");
  SO2.changeAcquireMode(SO2.PASSIVITY);
  SO2.setTempCompensation(SO2.OFF);
   
  while(NO_ERR != ENS160.begin())
  {
    Serial.println("ENS160 no encontrado !");  delay(1000);  
  }
  Serial.println("ENS160 conectado correctamente !");
  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(/*temperature=*/21.0, /*humidity=*/60.0);
  // *** Sensores de corriente ***
  if (!iCarga.begin(&Wire))
  {
    Serial.println("Sensor de corriente de carga no encontrado");
    while (1) { delay(10); }
  }
  Serial.println("Sensor de corriente de carga conectado correctamente !");
  if (!iDescarga.begin(&Wire))
  {
    Serial.println("Sensor de corriente de descarga no encontrado");
    while (1) { delay(10); }
  }
  Serial.println("Sensor de corriente de descarga conectado correctamente !");

  
  delay(500); 
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    //Serial.println(F("Después de os_init"));
    LMIC_reset();
    //Serial.println(F("Después de LMIC_reset"));
    leeSensores();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}
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

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
			      siesta();			   
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
			      siesta();
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
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, datasend, sizeof(datasend)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void loop() {
    os_runloop_once();
}
void siesta()
{
  Serial.flush();
  esp_sleep_enable_timer_wakeup(INTERVALO * 1000000); 
  esp_deep_sleep_start();
}

void leeSensores() {

  Serial.println("---------- Values ----------");
  int temp = bme.readTemperature() * 100.0F; Serial.print("Temperature = ");  Serial.print(temp / 100.0F, 2);  Serial.println(" *C");
  int pres = bme.readPressure() / 10.0F;  Serial.print("Pressure = ");  Serial.print(pres / 10.0F, 1);  Serial.println(" mbar (abs)");
  int humi = bme.readHumidity() * 100.0F; Serial.print("Humidity = ");  Serial.print(humi / 100.0F, 2);  Serial.println(" % (rel)");
  int airQ = ENS160.getAQI(); Serial.print("Air Quality = ");  Serial.println(airQ);
  int eCO2 = ENS160.getECO2(); Serial.print("eCO2 = ");  Serial.println(eCO2);
  int TVOC = ENS160.getTVOC(); Serial.print("TVOC = ");  Serial.println(TVOC);
  //int COppm = CO.readGasConcentrationPPM(); Serial.print("CO = ");  Serial.print(COppm);  Serial.println(" ppm");
  int NO2ppm = NO2.readGasConcentrationPPM() * 10; Serial.print("NO2 = ");  Serial.print(NO2ppm / 10.0F, 1);  Serial.println(" ppm");
  int O3ppm = O3.readGasConcentrationPPM() * 10; Serial.print("O3 = ");  Serial.print(O3ppm / 10.0F, 1);  Serial.println(" ppm");
  int SO2ppm = SO2.readGasConcentrationPPM() * 10; Serial.print("SO2 = ");  Serial.print(SO2ppm / 10.0F, 1);  Serial.println(" ppm");
  int vCarga = iCarga.getBusVoltage_V() * 100; Serial.print("V Carga = ");  Serial.print(vCarga / 100.0F, 1);  Serial.println(" V");
  int intCarga = iCarga.getCurrent_mA() * 10; Serial.print("I Carga = ");  Serial.print(intCarga / 10.0F, 1);  Serial.println(" mA");
  int vDescarga = iDescarga.getBusVoltage_V() * 100; Serial.print("V Descarga = ");  Serial.print(vDescarga / 100.0F, 1);  Serial.println(" V");
  int intDescarga = iDescarga.getCurrent_mA() * 10; Serial.print("I Descarga = ");  Serial.print(intDescarga / 10.0F, 1);  Serial.println(" mA");
  
  
  Serial.println();
  // datasend Bytes LSB|MSB: temp|temp|pres|pres|humi|humi|AirQ|eCO2|eCO2|TVOC|TVOC|NO2ppm|O3
  datasend[0] = temp;
  datasend[1] = temp >> 8;
  datasend[2] = pres;
  datasend[3] = pres >> 8;
  datasend[4] = humi;
  datasend[5] = humi >> 8;
  datasend[6] = airQ;
  datasend[7] = eCO2;
  datasend[8] = eCO2 >> 8;
  datasend[9] = TVOC;
  datasend[10] = TVOC >> 8;
  //datasend[11] = COppm;
  //datasend[12] = COppm >> 8;
  datasend[11] = NO2ppm;
  datasend[12] = O3ppm;
  datasend[13] = SO2ppm;
  datasend[14] = vCarga;
  datasend[15] = vCarga >> 8;
  datasend[16] = intCarga;
  datasend[17] = intCarga >> 8;
  datasend[14] = vDescarga;
  datasend[15] = vDescarga >> 8;
  datasend[16] = intDescarga;
  datasend[17] = intDescarga >> 8;
  
   
  Serial.print("Paquete : ");
  uint8_t i;
  for (i=0;i<sizeof(datasend)-1;i++){
    if (i%2 ==0) Serial.print(" ");
    Serial.printf("%02X",datasend[i]);    
  }
  Serial.println("");
  Serial.println("----------------------------");
}

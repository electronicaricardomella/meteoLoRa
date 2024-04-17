/*****************************************************************************
GetValues.ino

This example writes a setting value to a holding register, reads it to confirm
the value has changed, and then reads several data values from holding registers.

The register numbers in this example happen to be for an S::CAN oxy::lyser.
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <SensorModbusMaster.h>
#include <HardwareSerial.h>

// ---------------------------------------------------------------------------
// Set up the sensor specific information
//   ie, pin locations, addresses, calibrations and related settings
// ---------------------------------------------------------------------------

// Define the sensor's modbus address
byte modbusAddress = 0x02;   // The sensor's modbus address, or SlaveID
long modbusBaudRate = 9600; // The baud rate the sensor uses

uint16_t radiacion_solar = 0;

const int DEREPin = -1;       // The pin controlling Recieve Enable and Driver Enable
                              // on the RS485 adapter, if applicable (else, -1)
                              // Setting HIGH enables the driver (arduino) to send text
                              // Setting LOW enables the receiver (sensor) to send text


HardwareSerial MySerial(1); // definir un Serial para UART1
const int MySerialRX = 17;
const int MySerialTX = 18;

// Construct the modbus instance
modbusMaster modbus;

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void setup()
{
    // Set various pins as needed
    if (DEREPin >= 0)
    {
        pinMode(DEREPin, OUTPUT);
    }
    

    // Turn on the "main" serial port for debugging via USB Serial Monitor
    Serial.begin(57600);

    
    MySerial.begin(modbusBaudRate, SERIAL_8N1, MySerialRX, MySerialTX);
   


    // Turn on debugging, if desired
    //modbus.setDebugStream(&Serial);

    // Start the modbus instance
    modbus.begin(modbusAddress, MySerial, DEREPin);

}

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void loop()
{
   
  bool success = modbus.getRegisters(0x03, 0x01, 1); //leer el holding register 0x01
    // ^ This gets the values and stores them in an internal "frame" with the hex values of the response
  Serial.print("success: ");
  Serial.println(success);
  radiacion_solar = modbus.uint16FromFrame(bigEndian, 3);
  Serial.print("Radiación Solar (W/m2)= ");
  Serial.println(radiacion_solar);
  delay(1000);
    
}
#include <Wire.h>

int i;

int data[10];

void setup()
{
// join izc bus (address optional for master) 
Wire.begin();
Serial.begin(9600); // start serial for output
}


void loop(){

Wire.beginTransmission(0x70);
Wire.write(0x0C);
Wire.write(0x00);
Wire.write(0x00);
Wire.write(0x00);
Wire.write(0x00); 
Wire.write(0xF3);
Wire.endTransmission();

// stop transmitting

delay(1000);

Wire.requestFrom(0x70, 7);
i = 0;
// request 7 bytes from slave device

while(Wire.available())

// slave may send less than requested
{
data[i++]=Wire.read(); // receive 7 bytes "VOC, COZ, RS(MSB), RS, RS(LSB), STATUS, CRC
}
int VOC = ((data[0])-13)*(1000/229); 
int CO2 = ((data[1])-13)*(1600/229)+400;

Serial.print("VOC: "); // print the character

Serial.print(VOC);

Serial.print(" ");

Serial.print("C02: ");

Serial.print(CO2);

Serial.print(" ");

Serial.println();

delay(1000);
}
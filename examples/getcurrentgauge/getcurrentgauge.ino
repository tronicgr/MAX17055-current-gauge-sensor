// This example was writen for Arduino Zero and uses two serial ports, the USB and the SERIAL1.
// Most of the messages will appear on the USB port fine.

#define Serial Serial1
#include "Wire.h"
#include <Thanos_MAX17055.h>

Thanos_MAX17055 MAX17055;

// MAX17055 valiables
float cellVoltage = 0;
float current_mA = 0;
float Avg_current_mA = 0;
float stateOfCharge = 0;
float FullRepCap = 0;
float RepCap = 0;
float DieTemp = 0;
float cellTTE = 0;
float cellTTF = 0;

void setup() {
  byte error, address;
  int nDevices;

  Serial.begin(115200);  // Initialize hardware serial port, pins 0/1
  SerialUSB.begin(115200); // Initialize the USB serial port
  Wire.begin();

#ifndef ESP8266
  while (!SerialUSB);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  SerialUSB.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      if (address < 112) {
        SerialUSB.print("I2C device found at address 0x");
        SerialUSB.print(address, HEX);
        SerialUSB.println("  !");
        //Serial.print(" 0x");
        //Serial.print(address, HEX);
        //Serial.print(",  ");
        nDevices++;
      }
    }
    else if (error == 4)
    {
      if (address < 112) {
        SerialUSB.print("Unknow error at address 0x");
        SerialUSB.println(address, HEX);
      }
    }
  }
  if (nDevices == 0) {
    SerialUSB.println("No I2C devices found\n");
    Serial.println("No I2C devices found\n");
  }
  else
  { SerialUSB.println("done\n");
    Serial.print(" done\r");
  }

  Wire.begin();

  // Initialize the Max17055.
  MAX17055.begin();
  SerialUSB.println(" MAX17055 started\r");
  SerialUSB.println("Measuring voltage and current with MAX17055 ...");
}

void loop() {
  cellVoltage = MAX17055.getVoltageCell();
  current_mA = MAX17055.getCurrent_mA();
  Avg_current_mA = MAX17055.getAvgCurrent_mA();
  stateOfCharge = MAX17055.getRepSOC();
  FullRepCap = MAX17055.getFullRepCap();
  RepCap = MAX17055.getRepCap();
  DieTemp = MAX17055.getTemp();
  cellTTE = MAX17055.getTTE();
  cellTTF = MAX17055.getTTF();
  uint16_t cellVoltagefix = cellVoltage;
  
  SerialUSB.print("(");
  SerialUSB.print(Avg_current_mA); // print the reading ( offset-reading multiply by sense resistor range minus offset
  SerialUSB.print(" / ");
  SerialUSB.print(current_mA); // print the reading ( offset-reading multiply by sense resistor range minus offset
  SerialUSB.print(") - ");
  SerialUSB.print("SOC ");
  SerialUSB.print(stateOfCharge);   // print the reading High Byte only percent value
  SerialUSB.print("% - ");
  SerialUSB.print(cellVoltagefix);   // print the reading
  SerialUSB.print("mV - ");
  SerialUSB.print(" CAP ");
  SerialUSB.print(RepCap);   // print the reading
  SerialUSB.print("mAh of ");
  SerialUSB.print(FullRepCap);   // print the reading
  SerialUSB.print("mAh - ");
  SerialUSB.print(DieTemp);  // print the reading
  SerialUSB.print("C - ");
  SerialUSB.print("Time to [Empty ");
  SerialUSB.print(cellTTE); // print the reading
  SerialUSB.print("hrs] - [Full ");
  SerialUSB.print(cellTTF); // print the reading
  SerialUSB.println("hrs]  ");
}

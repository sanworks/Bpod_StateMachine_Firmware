/*
----------------------------------------------------------------------------

This file is part of the Sanworks Bpod Firmware repository
Copyright (C) 2021 Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef AD5592R_h
#define AD5592R_h
#endif
#include <Arduino.h>

class AD5592R{
  public:
    AD5592R(byte ChipSelect, byte BusyPin); // Constructor
    void init();
    void writeDAC(byte channel, uint16_t value); // Convert to setDAC() for LDAC operation
    void setChannelType(byte channel, byte type);
    void updateChannelTypes();
    byte nDAC = 0; // Number of DAC channels configured
    byte nADC = 0; // Number of ADC channels configured
    byte nDO = 0; // Number of digital outputs configured
    byte nDI = 0; // Number of digital inputs configured
    byte nHighZ = 0; // Number of Tri-State (High-Z) channels configured
    byte DOstate = 0; // Bits indicate logic levels written to channels configured as DO
    byte DIstate = 0; // Bits indicate logic levels read from channels configured as DI
    byte nReadsPerMeasurement = 1; // Configure oversampling by setting nReadsPerMeasurement above 1
    void setDO(byte channel, byte value); // Sets logic of an output channel in DOstate. Must be written to the chip with a call to writeDO()
    void writeDO(); // Sets digital output channel logic to equal bits of DOstate
    boolean getDI(byte channel); // Returns logic state of a channel returned to DIstate from last call to readDI()
    void readDI(); // Reads digital input channels and stores results in bits of DIstate
    uint16_t adcReadout[8] = {0}; // Results of ADC conversions returned from last call to readADC()
    uint16_t getADC(byte channel); // Returns analog value of a channel returned to adcReadout from last call to readADC()
    void readADC(); // Reads analog input channels and stores results in adcReadout

  private:
    byte CSPin = 0;
    byte BusyPin = 0;
    byte isDAC = 0; // Bits indicate whether each I/O channel is configured as a DAC
    byte isADC = 0; // Bits indicate whether each I/O channel is configured as an ADC
    byte isDO = 0; // Bits indicate whether each I/O channel is configured as a TTL output
    byte isDI = 0; // Bits indicate whether each I/O channel is configured as a TTL input
    byte isHighZ = 0; // Bits indicate whether each I/O channel is configured as Tri-State (High Z)
    union {
      uint8_t uint8[2];
      uint16_t uint16[1];
    } registerBuffer;
    void writeRegister();
    void readRegister();
};

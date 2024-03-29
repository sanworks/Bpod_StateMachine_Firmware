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
#include <Arduino.h>
#include <SPI.h>
#include "AD5592R.h"

int SPI_speed = 20000000; // 20MHz can be used for ADC reads, clocking data from IC. 50MHz can be used for DAC and DO writes

// Constructor
AD5592R::AD5592R(byte ChipSelect, byte TheBusyPin) {
  CSPin = ChipSelect;
  BusyPin = TheBusyPin;
}

void AD5592R::init() {
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH);
  pinMode(BusyPin, INPUT);
  SPI.begin(); // Initialize SPI interface
  
  // Reset the AD5592R
  registerBuffer.uint8[1] = B01111101;
  registerBuffer.uint8[0] = B10101100;
  writeRegister();
  
      // Set reference/power register to power up outputs and activate internal reference
  registerBuffer.uint8[1] = B01011000;
  registerBuffer.uint8[0] = 0;
  writeRegister();

  // Set general purpose register to enable ADC buffer and set outputs to 2x ref
  registerBuffer.uint8[1] = B00011011;
  registerBuffer.uint8[0] = B00110000;
  writeRegister();

  // Set all pins (except for busy pin) as High Z (Tri-State) by configuring them as powered down DACs
  isHighZ = B01111111;
  registerBuffer.uint8[1] = B01011010; // Power down all channels configured HighZ if configured as DAC
  registerBuffer.uint8[0] = isHighZ;
  writeRegister();
  registerBuffer.uint8[1] = B00101000; // Configure channels as DAC
  registerBuffer.uint8[0] = isHighZ;
  writeRegister();

  // Set pin 7 to act as ADC busy pin
  isDO = B10000000;
  registerBuffer.uint8[1] = B01000001;
  registerBuffer.uint8[0] = isDO;
  writeRegister();
}

void AD5592R::setDO(byte channel, byte value) {
  if (bitRead(isDO,channel)) {
    bitWrite(DOstate, channel, value);
  }
}

void AD5592R::writeDO() {
  registerBuffer.uint8[1] = B01001000;
  registerBuffer.uint8[0] = DOstate;
  writeRegister();
}

void AD5592R::writeDAC(byte channel, uint16_t value) {
  if (bitRead(isDAC,channel)) {
    registerBuffer.uint16[0] = value;
    bitSet(registerBuffer.uint8[1], 7);
    bitWrite(registerBuffer.uint8[1], 6, bitRead(channel, 2));
    bitWrite(registerBuffer.uint8[1], 5, bitRead(channel, 1));
    bitWrite(registerBuffer.uint8[1], 4, bitRead(channel, 0));
    writeRegister();
  }
}

void AD5592R::readDI() {
  registerBuffer.uint8[1] = B01010100;
  registerBuffer.uint8[0] = isDI;
  writeRegister(); // Request DI
  readRegister(); // Read DI into registerBuffer
  DIstate = registerBuffer.uint8[0];
  registerBuffer.uint16[0] = 0;
}

boolean AD5592R::getDI(byte channel) {
  byte result = 0;
  if (bitRead(isDI,channel)) {
    result = bitRead(DIstate, channel);
  }
  return result;
}

uint16_t AD5592R::getADC(byte channel) {
  uint16_t adcValue = adcReadout[channel];
  return adcValue;
}

void AD5592R::readADC() {
  uint32_t adcReadTotal[8] = {0};
  registerBuffer.uint8[1] = B00010010;
  registerBuffer.uint8[0] = isADC;
  writeRegister(); // Request ADC sequence
  delayMicroseconds(2);
  registerBuffer.uint16[0] = 0; // NOP
  writeRegister(); // Dummy read (necessary to initialize conversion of first channel in sequence)
  for (int j = 0; j < nReadsPerMeasurement; j++) {
    for (int i = 0; i < 7; i++) {
      if (bitRead(isADC,i)) {
        while(1-digitalRead(BusyPin)){}
        delayMicroseconds(1);
        registerBuffer.uint16[0] = 0; // NOP
        readRegister();
        bitClear(registerBuffer.uint8[1], 7);
        bitClear(registerBuffer.uint8[1], 6);
        bitClear(registerBuffer.uint8[1], 5);
        bitClear(registerBuffer.uint8[1], 4);
        adcReadTotal[i] += registerBuffer.uint16[0];
      }
    }
  }
  for (int i = 0; i < 7; i++) {
    if (bitRead(isADC,i)) {
      adcReadout[i] = adcReadTotal[i]/nReadsPerMeasurement;
    }
  }
  registerBuffer.uint8[1] = B00010010;
  registerBuffer.uint8[0] = 0;
  writeRegister(); // End ADC sequence
  //delayMicroseconds(1);
}

void AD5592R::setChannelType(byte channel, byte type) { 
  // Sets channel type: 0 = DI, 1 = DO, 2 = AI, 3 = AO, 4 = TriState (High-Z). Must be followed with a call to updateChannelTypes() to write the channel config to the AD5592R
  switch(type) {
    case 0: // Digital input
      bitClear(isADC, channel);
      bitClear(isDAC, channel);
      bitClear(isDO, channel);
      bitClear(isHighZ, channel);
      bitSet(isDI,channel);
    break;
    case 1: // Digital output
      bitClear(isADC, channel);
      bitClear(isDAC, channel);
      bitClear(isDI, channel);
      bitClear(isHighZ, channel);
      bitSet(isDO,channel);
    break;
    case 2: // Analog input
      bitClear(isDAC, channel);
      bitClear(isDO, channel);
      bitClear(isDI, channel);
      bitClear(isHighZ, channel);
      bitSet(isADC,channel);
    break;
    case 3: // Analog output
      bitClear(isADC, channel);
      bitClear(isDO, channel);
      bitClear(isDI, channel);
      bitClear(isHighZ, channel);
      bitSet(isDAC,channel);
    break;
    case 4: // High-Z (Tri-State)
      bitClear(isADC, channel);
      bitClear(isDO, channel);
      bitClear(isDI, channel);
      bitClear(isDAC,channel);
      bitSet(isHighZ,channel);
    break;
  }
}

void AD5592R::updateChannelTypes() {
  // Set Digital inputs
  registerBuffer.uint8[1] = B01010000;
  registerBuffer.uint8[0] = isDI;
  writeRegister();
  writeRegister();
  // Set Digital outputs
  registerBuffer.uint8[1] = B01000001;
  registerBuffer.uint8[0] = isDO;
  writeRegister();
  // Set ADC inputs
  registerBuffer.uint8[1] = B00100000;
  registerBuffer.uint8[0] = isADC;
  writeRegister();
  // Set DAC outputs
  registerBuffer.uint8[1] = B01011010; // Power up all channels not configured HighZ
  registerBuffer.uint8[0] = isHighZ;
  writeRegister();
  registerBuffer.uint8[1] = B00101000;
  registerBuffer.uint8[0] = isDAC;
  writeRegister();
  // Set High-Z channels
  registerBuffer.uint8[1] = B01011010; // Power down all HighZ channels to achieve tri-state when configured as DAC
  registerBuffer.uint8[0] = isHighZ;
  writeRegister();
  registerBuffer.uint8[1] = B00101000; // Configure tri-state channels (and DAC channels) as DAC
  registerBuffer.uint8[0] = isHighZ | isDAC;
  writeRegister();
  nDAC = 0; nADC = 0; nDO = 0; nDI = 0; nHighZ = 0;
  
  for (int i = 0; i < 7; i++) { // Update channel counts
    if (bitRead(isADC,i)) {
      nADC++;
    }
    if (bitRead(isDAC,i)) {
      nDAC++;
    }
    if (bitRead(isDO,i)) {
      nDO++;
    }
    if (bitRead(isDI,i)) {
      nDI++;
    }
    if (bitRead(isHighZ,i)) {
      nHighZ++;
    }
  }
}

void AD5592R::writeRegister() {
  #if defined(TEENSYDUINO) 
    // SPI settings can be used to speed up DAC and DO writes
    SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE2));
    digitalWriteFast(CSPin, LOW);
    SPI.transfer16(registerBuffer.uint16[0]);
    digitalWriteFast(CSPin, HIGH);
    SPI.endTransaction();
    SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1)); // To force clock line low so LED is off
    SPI.endTransaction();
  #endif
}

void AD5592R::readRegister() {
  #if defined(TEENSYDUINO) 
    SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE2));
    digitalWriteFast(CSPin, LOW);
    registerBuffer.uint16[0] = SPI.transfer16(registerBuffer.uint16[0]);
    digitalWriteFast(CSPin, HIGH);
    SPI.endTransaction();
    SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE1)); // To force clock line low so LED is off
    SPI.endTransaction();
  #endif
}

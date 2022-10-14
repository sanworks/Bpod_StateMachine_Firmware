/*
----------------------------------------------------------------------------

This file is part of the Sanworks Bpod Firmware repository
Copyright (C) 2016 Sanworks LLC, Sound Beach, New York, USA

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
#include "ArCOMvE.h"

ArCOMvE::ArCOMvE(Stream &s) {
  ArCOMstream = &s;  // Sets the interface (Serial, Serial1, SerialUSB, etc.)
  prefixBuffer[0] = 82; // Op-code to relay message
}
unsigned int ArCOMvE::available() {
  return ArCOMstream->available();
}
void ArCOMvE::flush() {
  ArCOMstream->flush();
}
void ArCOMvE::writeByte(byte byte2Write) {
  writePrefix(1);
  ArCOMstream->write(byte2Write);
}
void ArCOMvE::writeUint8(byte byte2Write) {
  writePrefix(1);
  ArCOMstream->write(byte2Write);
}
void ArCOMvE::writeChar(char char2Write) {
  writePrefix(1);
  ArCOMstream->write(char2Write);
}
void ArCOMvE::writeUint16(uint16_t int2Write) {
    writePrefix(2);
    typeBuffer.uint16 = int2Write;
    ArCOMstream->write(typeBuffer.byteArray, 2);
}

void ArCOMvE::writeUint32(uint32_t int2Write) {
    writePrefix(4);
    typeBuffer.uint32 = int2Write;
    ArCOMstream->write(typeBuffer.byteArray, 4);
}
byte ArCOMvE::readByte(){
  while (ArCOMstream->available() == 0) {}
  return ArCOMstream->read();
}
byte ArCOMvE::readUint8(){
  while (ArCOMstream->available() == 0) {}
  return ArCOMstream->read();
}
char ArCOMvE::readChar(){
  while (ArCOMstream->available() == 0) {}
  return ArCOMstream->read();
}
unsigned short ArCOMvE::readUint16() {
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[0] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[1] = ArCOMstream->read();
  return typeBuffer.uint16;
}

unsigned long ArCOMvE::readUint32() {
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[0] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[1] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[2] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[3] = ArCOMstream->read();
  return typeBuffer.uint32;
}

void ArCOMvE::writeInt8(int8_t int2Write) {
  writePrefix(1);
  typeBuffer.int8 = int2Write;
  ArCOMstream->write(typeBuffer.byteArray[0]);
}

void ArCOMvE::writeInt16(int16_t int2Write) {
  writePrefix(2);
  typeBuffer.int16 = int2Write;
  ArCOMstream->write(typeBuffer.byteArray, 2);
}

void ArCOMvE::writeInt32(int32_t int2Write) {
  writePrefix(4);
  typeBuffer.int32 = int2Write;
  ArCOMstream->write(typeBuffer.byteArray, 4);
}

int8_t ArCOMvE::readInt8() {
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[0] = ArCOMstream->read();
  return typeBuffer.int8;
}
int16_t ArCOMvE::readInt16() {
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[0] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[1] = ArCOMstream->read();
  return typeBuffer.int16;
}
int32_t ArCOMvE::readInt32() {
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[0] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[1] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[2] = ArCOMstream->read();
  while (ArCOMstream->available() == 0) {}
  typeBuffer.byteArray[3] = ArCOMstream->read();
  return typeBuffer.int32;
}
void ArCOMvE::writeByteArray(byte numArray[], unsigned int nValues) {
  writePrefix(nValues);
  ArCOMstream->write(numArray, nValues);
}
void ArCOMvE::writeUint8Array(byte numArray[], unsigned int nValues) {
  writePrefix(nValues);
  ArCOMstream->write(numArray, nValues);
}
void ArCOMvE::writeCharArray(char charArray[], unsigned int nValues) {
  writePrefix(nValues);
  ArCOMstream->write(charArray, nValues);
}
void ArCOMvE::writeInt8Array(int8_t numArray[], unsigned int nValues) {
  writePrefix(nValues);
  for (int i = 0; i < nValues; i++) {
    typeBuffer.int8 = numArray[i];
    ArCOMstream->write(typeBuffer.byteArray[0]);
  }
}
void ArCOMvE::writeUint16Array(unsigned short numArray[], unsigned int nValues) {
  writePrefix(nValues*2);
  for (unsigned int i = 0; i < nValues; i++) {
    typeBuffer.uint16 = numArray[i];
    ArCOMstream->write(typeBuffer.byteArray, 2);
  }
}
void ArCOMvE::writeInt16Array(int16_t numArray[], unsigned int nValues) {
  writePrefix(nValues*2);
  for (int i = 0; i < nValues; i++) {
    typeBuffer.int16 = numArray[i];
    ArCOMstream->write(typeBuffer.byteArray, 2);
  }
}
void ArCOMvE::writeUint32Array(unsigned long numArray[], unsigned int nValues) {
  writePrefix(nValues*4);
  for (unsigned int i = 0; i < nValues; i++) {
    typeBuffer.uint32 = numArray[i];
      ArCOMstream->write(typeBuffer.byteArray, 4);
  }
}
void ArCOMvE::writeInt32Array(long numArray[], unsigned int nValues) {
  writePrefix(nValues*4);
  for (unsigned int i = 0; i < nValues; i++) {
    typeBuffer.int32 = numArray[i];
      ArCOMstream->write(typeBuffer.byteArray, 4);
  }
}
void ArCOMvE::readByteArray(byte numArray[], unsigned int nValues) {
  ArCOMstream->readBytes(numArray, nValues);
}
void ArCOMvE::readUint8Array(byte numArray[], unsigned int nValues) {
  ArCOMstream->readBytes(numArray, nValues);
}
void ArCOMvE::readCharArray(char charArray[], unsigned int nValues) {
    ArCOMstream->readBytes(charArray, nValues);
}
void ArCOMvE::readInt8Array(int8_t numArray[], unsigned int nValues) {
  for (unsigned int i = 0; i < nValues; i++) {
    while (ArCOMstream->available() == 0) {}
    typeBuffer.byteArray[0] = ArCOMstream->read();
    numArray[i] = typeBuffer.int8;
  }
}
void ArCOMvE::readUint16Array(unsigned short numArray[], unsigned int nValues) {
  for (unsigned int i = 0; i < nValues; i++) {
    ArCOMstream->readBytes(typeBuffer.byteArray, 2);
    numArray[i] = typeBuffer.uint16;
  }
}
void ArCOMvE::readInt16Array(short numArray[], unsigned int nValues) {
  for (unsigned int i = 0; i < nValues; i++) {
    ArCOMstream->readBytes(typeBuffer.byteArray, 2);
    numArray[i] = typeBuffer.int16;
  }
}
void ArCOMvE::readUint32Array(unsigned long numArray[], unsigned int nValues) {
  for (unsigned int i = 0; i < nValues; i++) {
    ArCOMstream->readBytes(typeBuffer.byteArray, 4);
    numArray[i] = typeBuffer.uint32;
  }
}
void ArCOMvE::readInt32Array(long numArray[], unsigned int nValues) {
  for (unsigned int i = 0; i < nValues; i++) {
    ArCOMstream->readBytes(typeBuffer.byteArray, 4);
    numArray[i] = typeBuffer.int32;
  }
}
void ArCOMvE::writePrefix(uint32_t nValues) {
  typeBuffer.int32 = nValues;
  for (int i = 0; i < 4; i++) {
    prefixBuffer[i+1] = typeBuffer.byteArray[i];
  }
  ArCOMstream->write(prefixBuffer, 5);
}

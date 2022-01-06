/*
  Gen2 Chevy Volt Ambient light sensor LIN bus parser

  This code decodes the LIN bus data on LIN bus 5 of the Body Control Module in a 2019 Volt for ambient light sensor readings.
  It's possible that it will work on other Gen2 Volts.

  Frame Example:
  0x1A 0x1C 0x0D 0x88 0xFF 0x34
  
  0x1A - Frame identifier and protect id
  0x1C - Unknown, always 0x1C
  0x0D - Light sensor reading 0x00-0xAF
  0x88 - Temperature, assumed for sun load calculation
  0xFF - Unknown, always 0xFF
  0x34 - LIN v2 CRC

  Required Libraries:
  https://github.com/Locoduino/RingBuffer
*/

#include <RingBuf.h>

const int sBufferSize = 64; // Ring buffer size, smaller sizes tend to result in missed messages
const uint8_t version = 2;  // LIN bus version for CRC calculations
const byte intFrame = 0x00;   // Frame Sync Break
const byte startFrame = 0x55; // Frame initilizer
const byte nodeFrameId = 0x1A;  // Frame identifier of node

bool frameComplete = false;  // whether the frame is complete

//#define enDebug // enable debug mode

RingBuf<uint8_t, sBufferSize> sBuffer;

void setup() {
  // initialize serial
  Serial.begin(10417, SERIAL_8N1);  // Setup serial for GM LIN bus baud
  Serial.setTimeout(5);
  #ifdef enDebug
    Serial.println("LIN BUS module started...");
  #endif
}

void loop() {

  int frameStart = 0;
  // Look for LIN bus start in ring buffer and check the buffer size,  if frame data is complete set the rx led, frame start and frame complete
  for (int i=0; i < sBuffer.size(); i++) {
    if (sBuffer[i]==intFrame && sBuffer[i+1]==startFrame && sBuffer[i+2]==nodeFrameId) {    // Find Frame starting with sync break, initilizer and identifier
       frameStart = i+2;   // set Frame start to identifier byte
       digitalWrite(13, HIGH);
       if (sBuffer.size()-frameStart > 5) {   // if ring buffer has six bytes in it consider it a complete frame
        frameComplete = true;
       }
       break;   // break out of for loop since we have an interesting frame identified in the buffer
    }
  }

  // verify frame is complete and crc is valid then transmit the results
  if (frameComplete) {
    uint8_t data[] = {sBuffer[frameStart+1], sBuffer[frameStart+2], sBuffer[frameStart+3], sBuffer[frameStart+4]};    // assemble LIN frame data for CRC calculation
    int chk = checksum(sBuffer[frameStart], 4, data);   // calculate CRC

    if (chk==sBuffer[frameStart+5]) {
        int lightLevel = map(sBuffer[frameStart + 2], 0, 0xAF, 0, 100);   // map max sensor reading to 100 for percent
        Serial.print(lightLevel);   // transmit the results
        #ifdef enDebug
          Serial.print(lightLevel,DEC);
          Serial.println("%");
        #endif
      } else {
        #ifdef enDebug
          Serial.println("Checksum Error!");
        #endif
      }
      
    // clear the frame complete, turn off receive led and clear the buffer
    frameComplete = false;
    digitalWrite(13, LOW);
    sBuffer.clear();
  }
}

void serialEvent() {
  while (Serial.available()) {    // Wait for data in the hardware rx buffer
    int msgLength = Serial.available();
    // Check if hardware buffer will fill ring buffer and then clear the ring buffer
    if (sBuffer.size()+msgLength > sBufferSize) {
      sBuffer.clear();
      #ifdef enDebug
        Serial.println("Serial Buffer Cleared!");
      #endif
    }
      
    // copy received bytes to serial buffer     
    for (int i=0; i<msgLength; i++) {
      if (! sBuffer.push(Serial.read())) {
        #ifdef enDebug
          Serial.println("Serial Buffer Error!");
        #endif
      }
    }
  }
}

uint8_t protectID(uint8_t id)
{
  uint8_t  pid;       // result = protected ID
  uint8_t  tmp;       // temporary variable for calculating parity bits

  // copy (unprotected) ID
  pid = id;

  // protect ID  with parity bits
  pid  = (uint8_t) (pid & 0x3F);                                          // clear upper bit 6&7
  tmp  = (uint8_t) ((pid ^ (pid>>1) ^ (pid>>2) ^ (pid>>4)) & 0x01);       // -> pid[6] = PI0 = ID0^ID1^ID2^ID4
  pid |= (uint8_t) (tmp << 6);
  tmp  = (uint8_t) (~((pid>>1) ^ (pid>>3) ^ (pid>>4) ^ (pid>>5)) & 0x01); // -> pid[6] = PI1 = ~(ID1^ID3^ID4^ID5)
  pid |= (uint8_t) (tmp << 7);

  // return protected identifier
  return pid;

} // LIN_Master::protectID()

uint8_t checksum(uint8_t id, uint8_t numData, uint8_t *data)
{
  uint16_t chk=0x00;

  // protect the ID
  id = protectID(id);

  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID 0x3C and 0x3D/0x7D always use classical checksum (see LIN spec "2.3.1.5 Checkum")
  if (!((version == 1) || (id == 0x3C) || (id == 0x7D)))    // if version 2  & no diagnostic frames (0x3C=60 (PID=0x3C) or 0x3D=61 (PID=0x7D))
    chk = (uint16_t) id;

  // loop over data bytes
  for (uint8_t i = 0; i < numData; i++)
  {
    chk += (uint16_t) (data[i]);
    if (chk>255)
      chk -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk));   // bitwise invert

  // return frame checksum
  return chk;

} // LIN_Master::checksum()

/*
Copyright 2022 Lyle Gardner

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

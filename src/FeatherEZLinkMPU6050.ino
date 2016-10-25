/**
 * Copyright(C) 2016 Singapore ETH Centre, Future Cities Laboratory
 * All rights reserved.
 * 
 * This software may be modified and distributed under the terms
 * of the MIT license.See the LICENSE file for details.
 * 
 * Author:  Filip Schramka (schramka@arch.ethz.ch)
 * Summary: This software runs on a adafruit feather. A MPU 6050 
 *          Gyroscope & Accelerometer is attached. The meassured 
 *          quaternion will get delivered over bluetooth with a 
 *          bluefruit EZ-Link device (Serial1) or over USB
 *          (Serial) to the computer. 
 *          MPU6050 needs SDA/SCL & EZLink needs RX/TX. 
 *          In that case all regular hardware interrupt ports 
 *          are occupied.Interrupts from the MPU 6050 can 
 *          be attached on PCINT ports with the 
 *          PCattachInterrupt(...) method. 
 *          
 * Credit:
 *          This piece of software was modified from its original 
 *          form which was written by:
 *          
 *          I2Cdev device library code is placed under the MIT license
 *          Copyright (c) 2012 Jeff Rowberg
 *          https://github.com/jrowberg/i2cdevlib
 *          
 *          Permission is hereby granted, free of charge, to any person obtaining a copy
 *          of this software and associated documentation files (the "Software"), to deal
 *          in the Software without restriction, including without limitation the rights
 *          to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *          copies of the Software, and to permit persons to whom the Software is
 *          furnished to do so, subject to the following conditions:
 *          
 *          The above copyright notice and this permission notice shall be included in
 *          all copies or substantial portions of the Software.
 *          
 *          THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *          IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,$
 *          FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *          AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *          LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *          OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *          THE SOFTWARE.
 *          
 *          -----------------------------------------------------------------------------
 *          
 *          PCINT Tutorial
 *          Author unknown
 *          http://playground.arduino.cc/Main/PcInt
 * 
 */


/* Pin to interrupt map:
   PIN 9-11 = PortB (PB) 5-7 = PCINT 5-7 = pcmsk0 bits 5-7 = PCICR Bit 1
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// choose output, Serial is USB, Serial1 is Bluetooth
#define sysout           Serial1
// choose PCINT pin
#define PCINT_PIN        11

// MPU control / status variables
MPU6050 mpu;
uint8_t devStatus;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
bool dmpReady = false;
uint8_t fifoBuffer[64];
String out;

// MPU output variables
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PCINT variables
static int PCintMode[8];
typedef void (*voidFuncPtr)(void);
volatile static voidFuncPtr PCintFunc[8] = {
  NULL
};
volatile static uint8_t PCintLast;

volatile bool mpuInterrupt = false;

// get physical Pin on the CPU
uint8_t getRealPin(uint8_t pin) {

  uint8_t bit;

  switch (pin) {
    case 9:
      bit = 5;
      break;
    case 10:
      bit = 6;
      break;
    case 11:
      bit = 7;
      break;
    default:
      bit = 0;
      break;
  }

  return bit;
}


// attach an interrupt to a specific pin using pin change interrupts.
void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {

  uint8_t bit = getRealPin(pin);

  if (bit == 0) {
    sysout.println("Wrong Pin chosen, can not attach interrupt");
    return;
  }

  PCintMode[bit] = mode;
  PCintFunc[bit] = userFunc;
  // set the mask
  PCMSK0 |= 0x01 << bit;
  // enable the interrupt
  PCICR |= 0x01;
}

// detach an interrupt from a pin
void PCdetachInterrupt(uint8_t pin) {

  uint8_t bit = getRealPin(pin);

  if (bit == 0) {
    sysout.println("Wrong Pin chosen, can not detach interrupt");
    return;
  }

  uint8_t mask = 0x01;

  for (int i = 0; i < bit - 1; ++i) {
    mask = mask << 1;
    mask++;
  }

  // reset the mask
  PCMSK0 &= mask;
  // disable the interrupt --> just one interrupt can work at a time
  PCICR &= 0x00;
}

// interrupt handling
SIGNAL(PCINT0_vect) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states
  curr = *portInputRegister(2);
  mask = curr ^ PCintLast;
  PCintLast = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK0) == 0) {
    return;
  }

  // mask is pcint pins that have changed.
  for (uint8_t i = 0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // Trigger interrupt if mode is CHANGE, or if mode is RISING and
      // the bit is currently high, or if mode is FALLING and bit is low.
      if ((PCintMode[pin] == CHANGE
           || ((PCintMode[pin] == RISING) && (curr & bit))
           || ((PCintMode[pin] == FALLING) && !(curr & bit)))
          && (PCintFunc[pin] != NULL)) {
        PCintFunc[pin]();
      }
    }
  }
}

// interrupt routine
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()
{

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  sysout.begin(19200);
  
  // for PCINT
  pinMode(PCINT_PIN, INPUT);

  sysout.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  sysout.println(F("Testing device connections..."));
  sysout.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  sysout.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    sysout.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    sysout.println(F("Enabling interrupt detection (PCINT7 on feather Pin11)..."));
    PCattachInterrupt(PCINT_PIN, dmpDataReady, CHANGE);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    sysout.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    sysout.print(F("DMP Initialization failed (code "));
    sysout.print(devStatus);
    sysout.println(F(")"));
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize)
    ;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    sysout.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    out = "q ";
    out += q.x;
    out += " ";
    out += q.y;
    out += " ";
    out += q.z;
    out += " ";
    out += q.w;
    
    sysout.println(out);
  }
}

/* 
 * This file is part of the distribution (https://github.com/GormR).
 * Copyright (c) 2020 Gorm Rose.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// ADS1015 simulator for AgOpenGPS
///////////////////////////////////////////////////////////////////////////////////
// replaces the need of a WAS by a rotary incremental encoder at the steering wheel
//
// setup process (after every cold start):
// - turn wheels to the full left (min. angle in AOG)
// - turn wheels to the full right => 0° is exactly in the middle of the range
// - if using AgOpenGPS the first time: adjust parameters in steer menue
//
// setup process with pushbutton or proximity sensor:
// - if no values in EEPROM, e. g. starting the 1st time: like above + 
//   in case of using a push button: press it when in center position,
//   save values to EEPROM
// - after every cold start: go to center position and in case of using a push button: press it
//
// store max. wheel angle and center postion to EEPROM:
// - do in AOG: "A2D Coverter" = "Differential", "Send To Module", "A2D Coverter" = "Single", 
//   "Send To Module", "A2D Coverter" = "Differential", "Send To Module"
//
// erase EEPROM values:
// - do in AOG: "A2D Coverter" = "Single", "Send To Module"
// - reset or repower Arduino mini pro board (or all)
// - do in AOG: "A2D Coverter" = "Differential", "Send To Module"
//
// setup process with GPS support:
// - wait for the future ;)

#include <Wire.h>
#include <string.h>
#include <ctype.h>
#include <EEPROM.h> 
 
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1015_ADDRESS                 (0x48)    // 1001 000 (ADDR = GND)
/*=========================================================================*/

// Steering wheel incremental encoder
#define Ch_A  2             // D2: interrupt input
#define Ch_B  A1            // A1

#define centerkey A3        // pushbutton for manual use or proximity switch (NPN NO type) for automatic use
                            // the center between the encoder values at the high-low and low-high edge = 0°
#define WAS0deg 13280       // 0° by definition (BrianTee)

// if not in eeprom, overwrite 
#define EEP_Ident 4711 
uint16_t EEread = 0;

//Variables for settings  
struct Storage 
{
  uint16_t SWPmax = 100;    // max. position, will be 100 + number_of_encoder_turns x resolution_of_encoder 
  uint16_t SWPoffset = WAS0deg - 100; // this is added to the SWP when calculating the I²C data
};  Storage steerSettings;  // 2 bytes


uint16_t SWP = 100;         // position of the steering wheel (100 is min)
uint8_t  centerState = 0;   // state machine for centering (0 = off, 1 = got 1-0 edge, 2 = got 0-1 edge)
uint16_t SWP10;             // helper for auto-centering
uint8_t  I2Cresponse[2];    // data sent to AOG via I²C to emulate ACS1015
uint8_t  I2Cstate = 0;      // used for EEPROM storage function: 
                            // 0 = start, 1 = normal operating, 2 = store, 3 = clear, 4 = wait for idle
bool     storedValues = false; // set, if values came from EEPROM

String   inputString = "";  // a string to hold incoming NMEA data

uint16_t report = 0;        // send debug output every second

////////////////////////////////////////////////////////////////////////////
// Steering wheel incremental encoder interrupt
void SteeringWheelEncoderInterrupt() 
{
  if (digitalRead(Ch_B)) SWP++; else SWP--;
}

////////////////////////////////////////////////////////////////////////////
void setup()
{

  // Steering wheel incremental encoder
  pinMode(Ch_A, INPUT_PULLUP);  // may need further external pullup depending on the length of the cables!
  pinMode(Ch_B, INPUT_PULLUP);  // may need further external pullup depending on the length of the cables!
  attachInterrupt(digitalPinToInterrupt(Ch_A), SteeringWheelEncoderInterrupt, RISING);

  pinMode(centerkey, INPUT_PULLUP);  // may need further external pullup depending on the length of the cables!

  //set up communication (I²C, NMEA input and debug output)
  Wire.begin();
  Serial.begin(4800);

  // set I2C address
  Wire.begin(ADS1015_ADDRESS);

  // Define I2C handler
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // read config in EEPROM if some there
  Serial.println();
  EEPROM.get(0, EEread);                                    // read identifier
  if (EEread == EEP_Ident) 
  {
    EEPROM.get(2, steerSettings);    // get last values
    storedValues = true;
    Serial.println("Values from EEPROM: ");
    Serial.print("SWPmax: ");
    Serial.println(steerSettings.SWPmax);
    Serial.print("SWPoffset: ");
    Serial.println(steerSettings.SWPoffset);
  }
  else Serial.println("Starting with default values.");
  delay(100);
}

////////////////////////////////////////////////////////////////////////////

void evalNMEA() // incomplete - work for the future!
{
  // you can also look for a substring in the middle of a string:
  if (inputString.substring(1, 6) == "GPRMC")
  {
    Serial.println(inputString);
    // algorithm will fail when a "degree line" is passed     
    int posSN = inputString.indexOf(",N,") + inputString.indexOf(",S,") + 1;  // look out for North or South
    int posWE = inputString.indexOf(",W,") + inputString.indexOf(",E,") + 1;  // look out for East or West
    if ((posSN >= 0) && (posWE >= 0)) 
    {
      String strSN = inputString.substring(0, posSN);
      String strWE = inputString.substring(0, posWE);
      posSN = strSN.indexOf(",");
      while (posSN >= 0)
      {
        strSN = strSN.substring(posSN + 1);
        posSN = strSN.indexOf(",");
      }
      posWE = strWE.indexOf(",");
      while (posWE >= 0)
      {
        strWE = strWE.substring(posWE + 1);
        posWE = strWE.indexOf(",");
      }
      String str2SN = strSN.substring(0, 2);
      String str3SN = strSN.substring(2, 4);
      strSN = str3SN + strSN.substring(5);
      long SN = strSN.toInt();
      String str2WE = strWE.substring(0, 3);
      String str3WE = strWE.substring(3, 5);
      strWE = str3WE + strWE.substring(6);
      long WE = strWE.toInt();
      Serial.println(SN);
      Serial.println(WE);
    }
  }
  // clear the string:
  inputString = "";
}

void loop()
{
  if (not report++) 
  {
    Serial.print("Actual SWP: ");
    Serial.print(SWP);
    Serial.print(", Offset: ");
    Serial.print(steerSettings.SWPoffset);
    Serial.print(", Max: ");
    Serial.println(steerSettings.SWPmax);
  }
  
  // limits of encoder range reached?
  if (SWP < 100)
  {
    if (not storedValues)
    {
      steerSettings.SWPmax += 100 - SWP;
      if (centerState == 0) steerSettings.SWPoffset = WAS0deg - ((steerSettings.SWPmax + 100) >> 1); // ">> 1" is faster then "/ 2"
    }
    SWP = 100;
  }
  if (SWP > steerSettings.SWPmax)
  {
    if (not storedValues)
    {
      steerSettings.SWPmax = SWP;
      if (centerState == 0) steerSettings.SWPoffset = WAS0deg - ((steerSettings.SWPmax + 100) >> 1); // ">> 1" is faster then "/ 2"
    }
    else SWP = steerSettings.SWPmax;
  }

  // something to do for EEPROM?
  if (I2Cstate == 2)
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(2, steerSettings);   
    Serial.println("New values stored to EEPROM: ");
    Serial.print("SWPmax: ");
    Serial.println(steerSettings.SWPmax);
    Serial.print("SWPoffset: ");
    Serial.println(steerSettings.SWPoffset);
    I2Cstate = 4;
  }
  if (I2Cstate == 3)
  {
    EEPROM.put(0, 0);                // clear EERPROM
    Serial.println("EEPROM reset.");
    I2Cstate = 4;
  }

  // center detected?
  if (digitalRead(centerkey) == LOW)  // suto centering with the help of a proximity sensor or pushbutton
  {
    if (centerState != 1) // 10 (= active) edge of sensor or bushbutton detected
    {
      centerState = 1;
      SWP10 = SWP;
      Serial.println("Center input active.");
    }
  }
  else
  {
    if (centerState == 1) // 01 (= inactive) edge of sensor or bushbutton detected => set center
    {
      centerState++;
      steerSettings.SWPoffset = WAS0deg - ((SWP10 + SWP) >> 1); // = AOGzero - center of active input period
      Serial.print("Center set to ");
      Serial.println((SWP10 + SWP) >> 1);
    }
  }
  
  // provide data for I²C
  uint16_t SWPtemp = SWP + steerSettings.SWPoffset;
  I2Cresponse[1] = byte(SWPtemp & 0xff);  // when inside requestEvent: timing problems!
  I2Cresponse[0] = byte(SWPtemp >> 8);
    
   while (Serial.available()) 
   {
     char inChar = (char)Serial.read();
     if (inChar == '\r' || inChar == '\n') evalNMEA(); else inputString += inChar;
   }
}   

////////////////////////////////////////////////////////////////////////////

void receiveEvent(int howMany)  // not used
{
  while(Wire.available())
  {
    char c = Wire.read();
    if (c == -127) I2Cstate = 1;                      // AOG differential mode = standard idle
    if ((I2Cstate == 0) && (c == -63)) I2Cstate = 3;  // AOG single mode after reset = erase EEPROM
    if ((I2Cstate == 1) && (c == -63)) I2Cstate = 2;  // AOG single mode during operation = write values to EEPROM
  }
}

void requestEvent()
{
  //SWP++;
  Wire.write(I2Cresponse, 2);// 2 bytes of data to be sent
}

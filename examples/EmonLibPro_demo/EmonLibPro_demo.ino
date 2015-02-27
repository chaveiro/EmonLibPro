/*
    Energy monitor Library Pro
    Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    This is a generic library for any voltage and current sensors.
    Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
    Reports Voltage, Frequency, Current, Active Power, Aparent Power and Power Factor.
    Completely line frequency independent.
    This library idea cames from ATMEL AVR465: Single-Phase Power/Energy Meter.
    
    Get latest version at https://github.com/chaveiro/EmonLibPro
    
    Copyright (C) 2013-2015  Nuno Chaveiro  nchaveiro[at]gmail.com  Lisbon, Portugal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/     
 
#include <Arduino.h>
#include "EmonLibPro.h"

/*
    This file is a demonstration of usage.
    
    Edit configuration in EmonLibPro.h.
*/

//--------------------------------------------------------------------------------------------------

int userCommand;
EmonLibPro Emon ;


void setup()
{
  Serial.begin(9600);
  Serial.print("EMON: v");
  Serial.println(EMONLIBPROVERSION);
  
  Emon = EmonLibPro();    // Required
  Emon.begin();           // Required
  
  printMenu();
}

void loop()
{
  if(userCommand == 1 && Emon.FlagCALC_READY) { 
      Emon.calculateResult();
      for (byte i=0;i<CURRENTCOUNT;i++){
        printResults(i);
      }
  }
  
  if(userCommand == 2 && Emon.FlagCYCLE_FULL) {
      for (byte i=0;i<VOLTSCOUNT + CURRENTCOUNT;i++){
        Serial.print("ADC");
        Serial.print(0+i);
        Serial.print(",");
        serialDataTable(i);
      }
      Emon.FlagCYCLE_FULL = false;  //Must reset after read to know next batch.
  }
  
  if(userCommand == 3 && Emon.FlagCALC_READY) {
        Emon.printStatus();
        userCommand = 0;
  }  

  if (userCommand == 4 && EmonLibPro::FlagOutOfTime) { // This should never be true.  Reduce sampling rate if it is!
	Serial.println("$");         // Alarm that previous ADC processing has run out of time before timer event. Must decrease SAMPLESPSEC
  }
  
  if(Serial.available())
  {
    userCommand=Serial.parseInt();
    Serial.print("Got: ");
    Serial.println(userCommand);
  }
  
}


void printMenu(){
    Serial.println("EmonLipPro Demo");
    Serial.println(" 1 - Print cycle data. (internal var data for each cycle)");
    Serial.println(" 2 - Print Calculated data. Change interval in define CALCULATECYCLES.");
    Serial.println(" 3 - Print Lib Status.");
    Serial.println(" 4 - Check if sample rate is acceptable. Should not see the $ char.");

    Serial.println("Press a key...");
}


// Print calculated results
void printResults(byte i)
{
    Serial.print("Result");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(Emon.ResultV[0].U);
    Serial.print("VAC\t");
    Serial.print(Emon.ResultV[0].HZ);
    Serial.print("Hz\t");
    Serial.print(Emon.ResultP[i].I);
    Serial.print("A\t");
    Serial.print(Emon.ResultP[i].P);
    Serial.print("W\t");
    Serial.print(Emon.ResultP[i].S);
    Serial.print("VA\t");
    Serial.print(Emon.ResultP[i].F);
    Serial.print("Pfact");
#ifdef USEPLL
    if(!Emon.pllUnlocked) Serial.print(" L");
#endif
    Serial.println("\t");
}


// Exposes the internal sampled values for all samples on a cycle
// Usefull to display the signal on a graph.
void serialDataTable(byte b){
  int sizearr = (Emon.SamplesPerCycleTotal/Emon.CyclesPerTotal);
  if (sizearr > CYCLEARRSIZE) sizearr = CYCLEARRSIZE;
  for (byte i=0;i<=sizearr;i++){
    Serial.print(Emon.Sample[b].CycleArr[i]);
    Serial.print(",");
  }
  Serial.println("\t");
}

/*
    Energy monitor Library Pro
    Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    This is a generic library for any voltage and current sensors.
    Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
    Completely line frequency independent.

    Get latest version at https://github.com/chaveiro/EmonLibPro
    
    Copyright (C) 2013  Nuno Chaveiro  nchaveiro[at]gmail.com  Lisbon, Portugal

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
/*	
	---------------------------------------------------------------------------
	Calibration
	1 - Phase
	The algorithm uses two subsequent samples to interpolate an intermediate point. 
	This means that the higher the sampling frequency, the lower the phase adjustment
	margin. At approximately 2000Hz sampling rate (666.66Hz per channel) and 50Hz mains
	frequency, the highest phase delay that can be interpolated is 360 x (50Hz/666.66Hz) =
	27 degrees.
	The effect of the phase calibration coefficients is shown in the following equation:
		Z = (PCC/65536) * (360º * Fm * 128 * 13 * 3 / Fclk)
		
		Fm = 50Hz Main frequency
		Fclk = 16000000 System frequency
		PCC = Phase calibration coefficient
	
	There is one phase calibration coefficient for each input channel, i.e. three in total.
	Note, that the 16-bit phase calibration coefficients are treated unsigned.
	
	Apply nominal voltage and a large current to the meter. Voltage and current should be in 
	phase, i.e. the power factor should be unity. Allow readings to stabilize and then 
	record active power, voltage and current readings. Use voltage and current readings 
	to calculate apparent power; S = U*I. Compare with active power readings and evaluate 
	phase error, as follows: 
		PE= acos(|P| / S)
	Derive phase calibration coefficient based on phase error, and add the result to the
	default phase calibration coefficient. 
	
 */

extern "C" {
    #include <inttypes.h>
    #include <math.h>
    #include <avr/interrupt.h>
    #include <avr/io.h>
}

#include <Arduino.h>
#include "EmonLibPro.h"


//////////////////////////////////////////////////////////////////////////
// Global Variables
//////////////////////////////////////////////////////////////////////////

boolean                     EmonLibPro::FlagCYCLE_FULL;     // Flags a new cycle.
boolean                     EmonLibPro::FlagCALC_READY;     // Flags new data ready for calculate
boolean                     EmonLibPro::FlagINVALID_DATA;   // Flags Invalid data
uint8_t                     EmonLibPro::pllUnlocked;        // If = 0 pll is locked
uint8_t                     EmonLibPro::SamplesPerCycle;       // --- Gives number of samples that got summed in summed Cycle Data Structure (adjusted/detected by soft pll for each AC cycle)
unsigned long               EmonLibPro::SamplesPerCycleTotal;  // --- Number of cycles added for all sums of Total Var.
unsigned int                EmonLibPro::CyclesPerTotal;       // --- Number of sums on Total var.
TotVoltageDataStructure     EmonLibPro::TotalV[VOLTSCOUNT];    //  |- Total Vars (Sum of some cycles)
TotPowerDataStructure       EmonLibPro::TotalP[CURRENTCOUNT];  // -/
ResultVoltageDataStructure  EmonLibPro::ResultV[VOLTSCOUNT];   // -\_ Result is here!
ResultPowerDataStructure    EmonLibPro::ResultP[CURRENTCOUNT]; // -/


//////////////////////////////////////////////////////////////////////////
// ISR routine vars
//////////////////////////////////////////////////////////////////////////
uint8_t                     EmonLibPro::AdcId;              // ADC PIN number of sensor measured
boolean                     EmonLibPro::FlagOutOfTime;      // Warn ISR routing did not complete before next timer

SampleStructure             EmonLibPro::Sample[VOLTSCOUNT + CURRENTCOUNT]; //Data for last sample

AccVoltageDataStructure     EmonLibPro::AccumulatorV[VOLTSCOUNT];   // --- Sum of all samples (copyed to cycle var at end of cycle)
AccPowerDataStructure       EmonLibPro::AccumulatorP[CURRENTCOUNT]; // -/

signed int                 EmonLibPro::Temp[VOLTSCOUNT + CURRENTCOUNT];     // Internal Aux vars

//////////////////////////////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////////////////////////////

// Constructor
EmonLibPro::EmonLibPro(void) { }

// calculate voltage, current, power and frequency
void EmonLibPro::calculateResult()
{
  if(SamplesPerCycleTotal!=0) {
        // Process accumulated data as follows:
        //
        // Active power:    P = (accumulated data) / (number of samples)
        // Voltage:     U = SQRT [ (accumulated data) / (number of samples) ]
        // Current:     I = SQRT [ (accumulated data) / (number of samples) ]
        // Apparent power:  S = U * I
        //
  
        byte i;
        for (i=0;i<VOLTSCOUNT;i++){
                ResultV[i].U = CalCoeff.VRATIO[i] * (sqrt((float)TotalV[i].U2/SamplesPerCycleTotal)); //Vrms
                ResultV[i].HZ = ((float)((F_CPU)*100) / ((TIMERTOP * (uint8_t)(SamplesPerCycleTotal/CyclesPerTotal)) + ((float)TotalV[i].PeriodDiff/CyclesPerTotal) ))/100;
                TotalV[i].U2=0;
				TotalV[i].PeriodDiff=0;
        }

        for (i=0;i<CURRENTCOUNT;i++){
                //I
                ResultP[i].I = CalCoeff.IRATIO[i] * (sqrt((float)TotalP[i].I2/SamplesPerCycleTotal)); // Irms
                //P
                #if CURRENTCOUNT == VOLTSCOUNT
                ResultP[i].P = (CalCoeff.IRATIO[i] * CalCoeff.VRATIO[i]) * ((float)TotalP[i].P/SamplesPerCycleTotal); // Active power (RealPower)
                ResultP[i].S = ResultV[i].U * ResultP[i].I; // Apparent power
                #else
                ResultP[i].P = (CalCoeff.IRATIO[i] * CalCoeff.VRATIO[0]) * ((float)TotalP[i].P/SamplesPerCycleTotal); // Active power (RealPower)
                ResultP[i].S = ResultV[0].U * ResultP[i].I; // Apparent power
                #endif
                ResultP[i].F = absolute((float)ResultP[i].P / ResultP[i].S);
                if (isnan(ResultP[i].F)) ResultP[i].F = 1;  // division by 0 
                TotalP[i].I2 = 0;
                TotalP[i].P = 0;
        }
        SamplesPerCycleTotal=0;
        CyclesPerTotal=0;
    
  } else {
        EmonLibPro::FlagINVALID_DATA = true;
        // Clean results, we have invalid data.
        EmonLibPro::SamplesPerCycle=0;
        byte i;
        for (i=0;i<VOLTSCOUNT;i++){
            EmonLibPro::ResultV[i].U = 0;
            EmonLibPro::ResultV[i].HZ = 0;
        }
        for (i=0;i<CURRENTCOUNT;i++){
            EmonLibPro::ResultP[i].I = 0;
            EmonLibPro::ResultP[i].P = 0;
            EmonLibPro::ResultP[i].S = 0;
            EmonLibPro::ResultP[i].F = 0;
        }
  }
  FlagCALC_READY=false;
}


// Call this one time
void EmonLibPro::begin()
{
     FlagOutOfTime = false;
     FlagCALC_READY = false;
     FlagINVALID_DATA = true;
     pllUnlocked = 0xFF;       //initial number of correct cycles to unlock PLL
     
     DIDR0   = 0b00111111;     // ADC5D...ADC0D: Digital Input Disable, saves power
     
     cli();
     //set timer 1 interrupt for required period
     TCCR1A = 0; // clear control registers
     TCCR1B = 0;
     TCNT1  = 0; // clear counter
     OCR1A  = TIMERTOP - 1;      // Output Compare Register 1A for timer period

     TCCR1B |= (1UL<<WGM12);   // CTC mode
     TCCR1B |= (1UL<<CS10);    // clkIO/1 (No prescaling)
     TIMSK1 |= (1UL<<OCIE1A);  // enable timer 1A compare interrupt

     ADCSRA = adc_sra;         // Sets ADC interrupt and prescaller
     
#ifdef USE_ANALOG_COMP
	 TCCR1B |= (1UL<<ICNC1);    // Input Capture Noise Canceler
	 //TCCR1B |= (1UL<<ICES1);    // Input Capture Edge Select: a rising(positive) edge will trigger the capture.
	 TIMSK1 |= (1UL<<ICIE1);    // Input Capture Interrupt Enable

     // Analog Comparator
     DIDR1 = 0b00000011;       // AIN1D..AIN0D: AIN1, AIN0 Digital Input Disable, saves power
     ACSR = (0<<ACD) |     // Analog Comparator: Enabled
            (0<<ACBG) |    // Analog Comparator Bandgap Select: AIN0 is the positive input of comparator
            (0<<ACO) |     // Analog Comparator Output: Off
            (1UL<<ACI)|    // Analog Comparator Interrupt Flag : Clear Pending Interrupt
            //(1UL<<ACIE)|   // Analog Comparator Interrupt Enable
            (1UL<<ACIC)|   // Analog Comparator Input Capture Enable on Timer/Counter1
            (0UL<<ACIS1)|  // --- Comparator Interrupt on Output Toggle.
            (0UL<<ACIS0);  // -/ 
#endif   

#ifdef ARDUINO_HI_RES
         //TIMSK0 &= ~(1UL<<TOIE0);  // Disable Timer0 !!! Arduino delay() is now not available
         OCR1B  = TIMERTOP - 25;   // Output Compare Register 1B for timer period (SLEEP)
         TIMSK1 |= (1UL<<OCIE1B);  // enable timer 1B compare interrupt
         SMCR = 0b00000001;              // SLEEP: idle sleep mode
         //SMCR = 0b00000011;              // SLEEP: ADC noise reduction, returns to adc int
         //SMCR = 0b00000111;              // SLEEP: power save
#endif
     
     sei(); //Enable interrupts
}


//////////////////////////////////////////////////////////////////////////
// INTERRUPT Routines
//////////////////////////////////////////////////////////////////////////

// Timer 1 interrupt handler
ISR(TIMER1_COMPA_vect) {
    ADMUX   = _BV(REFS0) | adc_pin_order[0]; // [AVCC with external capacitor at AREF pin] | [PIN=ADC0]
    ADCSRA = adc_sra | _BV(ADSC);            // Start ADC conversion

    EmonLibPro::AdcId=0;
    EmonLibPro::SamplesPerCycle++;
    if (EmonLibPro::SamplesPerCycle >= 200 && !EmonLibPro::FlagINVALID_DATA) { // Clean data when no zero cross detected.
        EmonLibPro::SamplesPerCycle=0;
        EmonLibPro::SamplesPerCycleTotal=0;
        EmonLibPro::FlagCALC_READY = true;
    }
    if (EmonLibPro::FlagOutOfTime) { // This should never be true.  Reduce sampling rate if it is!
        Serial.println("$");         // Alarm that previous ADC processing has run out of time before timer event.
    }

    // Bellow is a hack to force Arduino Timer0 overflow to ocours only when wont disrupt our calculations
    if (TIFR0 & _BV(TOV0)) {
        TCNT0=0;
        TIMSK0 |= 1 << TOIE0;    // Sets Overflow Interrupt Enable
    } else {
        TIMSK0 &= ~(1 << TOIE0); // Clear Overflow Interrupt Enable
    }
}


#ifdef USE_ANALOG_COMP
ISR(TIMER1_CAPT_vect)
{
	if (EmonLibPro::Sample[0].WaitNextCross) {
		EmonLibPro::Sample[0].FlagZeroDetec = true;
		EmonLibPro::Sample[0].WaitNextCross=false;
	}
   
}
#endif

#ifdef ARDUINO_HI_RES
// Put the MCU to sleep JUST before the CompA ISR goes off
ISR(TIMER1_COMPB_vect, ISR_NAKED)
{
    __asm volatile ("sei"); 
    __asm volatile ("sleep"); // go to sleep 
    __asm volatile ("reti"); 
}
#endif


// ADC interrupt handler
ISR(ADC_vect) {
    unsigned int timerVal = TCNT1;    // Saves timer counter for freq measurement
    signed int TempI;
    signed long TempL;
    uint8_t i;
    
    // Maintain multiplexing of input channels
    if (EmonLibPro::AdcId < VOLTSCOUNT + CURRENTCOUNT - 1) {
        ADMUX = _BV(REFS0) | adc_pin_order[(EmonLibPro::AdcId + 1)];   // Select next ADC
        ADCSRA = adc_sra | _BV(ADSC);                                  // Start ADC conversion
    } 

	
    // ## 1 - Saves read on local var
    unsigned int _fresh = ADC;

	
    // ## 2 - Apply filter for DC offset removal. Must use long data types for precision!
    // y[n] <- y[n-1]
    EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered=EmonLibPro::Sample[EmonLibPro::AdcId].Filtered;
   
    //  y[n] = 0.996*y[n-1] + 0.996*x[n] - 0.996*x[n-1]
	// Algorith float point - >169cpu cycles
    /*
	EmonLibPro::Sample[EmonLibPro::AdcId].Filtered = 0.996 * (EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered + 
                                              (signed int)(_fresh - EmonLibPro::Sample[EmonLibPro::AdcId].PreviousADC)  );
	*/
	//EmonLibPro::Sample[EmonLibPro::AdcId].Filtered=(long)_fresh-512<<8;
	
    // Algorithm Alt A - 36cpu cycles
    TempI=(signed int)_fresh - (signed int)EmonLibPro::Sample[EmonLibPro::AdcId].PreviousADC;  // find the input change
    TempL= (long)TempI<<8;                                                 // rescale the input change (x256)
    TempL=TempL + EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered;  // add the previous o/p
    EmonLibPro::Sample[EmonLibPro::AdcId].Filtered=TempL-((long)TempL>>8); // subtract 1/256, same as x255/256
	
	// Algorithm Alt B - 45cpu cycles
    /*
    TempL=(EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered<<8)-EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered;
    TempL=TempL>>8;
    TempI=(signed int)_fresh-(signed int)EmonLibPro::Sample[EmonLibPro::AdcId].PreviousADC;
    TempL=TempL + (((long)TempI<<8)-(long)TempI);
    EmonLibPro::Sample[EmonLibPro::AdcId].Filtered=TempL;
    */
 
    // Algorithm App note - 124cpu cycles
	/*
    TempL=255*(long)EmonLibPro::Sample[EmonLibPro::AdcId].Filtered;
    TempL=TempL>>8;
    TempI=(signed int)_fresh-(signed int)EmonLibPro::Sample[EmonLibPro::AdcId].PreviousADC;
    TempL=TempL + 255*(long)TempI;
    EmonLibPro::Sample[EmonLibPro::AdcId].Filtered=TempL;
    */

	
    // ## 3 - Saves Previous ADC read for next int
    // x[n+1] <- x[n]
    EmonLibPro::Sample[EmonLibPro::AdcId].PreviousADC=_fresh;


    // ## 4 - Apply phase calibration. Due to multiplexing, there will be a delay
    // between subsequent data channels, as follows:
    //
    //  Delay = [ 360 degrees * f(mains) ] / [ 3 * f(sampling) ]
    //
    // At 2400Hz sampling rate, this means that fresh samples from ADC0
    // and ADC1, for example, will actually have a time difference of
    // 1/2400 = 0.42ms, which corresponds to (50*360)/2400 = 7.5 degrees
    // in a 50Hz mains environment. In addition, current transformers in
    // the analogue front end may typically contribute up to six degrees
    // of additional phase lag. Phase distortions are calibrated using
    // linear interpolation, as follows:
    //
    //  Calibrated = x[n-1] + CalibCoefficient * ( x[n] - x[n-1] )
    //
    // Note that linear interpolation is the same as a fixed time delay.
    // This means that higher frequency components will not be adjusted
    // correctly. Typically, though, not enough energy is present in the
    // higher frequency components to cause this to degrade accuracy below
    // acceptable levels.
    //
    // Also note that the higher the sampling rate, the lower is the
    // calibration range in degrees. If the sampling rate is too high,
    // then the linear interpolation as such can not compensate for the
    // phase lag of some DC imuned current transformers. Such
    // transformers can have a phase lag in excess of six degrees.
    TempL=EmonLibPro::Sample[EmonLibPro::AdcId].Filtered-EmonLibPro::Sample[EmonLibPro::AdcId].PreviousFiltered;
    TempL=TempL*CalCoeff.PCC[EmonLibPro::AdcId];
    TempL=TempL>>16;
    EmonLibPro::Sample[EmonLibPro::AdcId].PreviousCalibrated = EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated;
    //EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated=EmonLibPro::Sample[EmonLibPro::AdcId].Filtered-TempL;
	EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated=(signed int)((EmonLibPro::Sample[EmonLibPro::AdcId].Filtered-TempL)>>8);

	if ((EmonLibPro::SamplesPerCycle <= 51)) {
		EmonLibPro::Sample[EmonLibPro::AdcId].CycleArr[EmonLibPro::SamplesPerCycle]=EmonLibPro::Sample[EmonLibPro::AdcId].Filtered>>8 ;	
	}

    // ## 5 - Perform Zero cross check
  

    if (!EmonLibPro::Sample[EmonLibPro::AdcId].WaitNextCross) {
        EmonLibPro::Sample[EmonLibPro::AdcId].WaitNextCross=EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated < -32; // Only allow next zero cross if already passed this value on the cycle
    } 
#ifndef USE_ANALOG_COMP  
	else {
        //Is Positive ?
        if (EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated >= 0) {
            //Is Rising?
            if (EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated >= EmonLibPro::Sample[EmonLibPro::AdcId].PreviousCalibrated) {
                EmonLibPro::Sample[EmonLibPro::AdcId].FlagZeroDetec=true;
                EmonLibPro::Sample[EmonLibPro::AdcId].WaitNextCross=false;
            }
        }
    }
#endif

    //## 6 - Saves timer val to calc freq later
    EmonLibPro::Sample[EmonLibPro::AdcId].PreviousTimerVal = EmonLibPro::Sample[EmonLibPro::AdcId].TimerVal;
    EmonLibPro::Sample[EmonLibPro::AdcId].TimerVal = timerVal;    


    // Save measurement data for voltage, current and active power on all
    // channels. For active power measurements; accumulate instantaneous
    // power product. For current and voltage measurements: accumulate
    // square of samples
    //
    // Range check:
    //
    //  Filtered data:      Filtered  Range=[FFFE0280...0001FD80]
    //  Prescaled data (>>5):   TempX     Range=[FFFFF014...00000FEC]
    //  Multiplication result:  (n/a)     Range=[FF027E70...00FD8190]
    //
    //  Prescaled data (>>6):   TempX     Range=[FFFFF80A...000007F6]
    //  Multiplication result:  (n/a)     Range=[FFC09F9C...003F6064]
    //
    // Assuming that sampled data is stuck at full-scale (which it can't
    // be, since the high-pass filter would bias such a signal to zero),
    // then for accumulation purposes the maximum number of samples that
    // can be integrated is:
    //
    //  MaxSamples(presc=32) = 7FFFFFFFh / 00FD8190h = 0081h = 129d
    //  MaxSamples(presc=64) = 7FFFFFFFh / 003F6064h = 0205h = 517d
    //
    // Note: Index=0 -> ADC0, Index=1 -> ADC1, Index=2 -> ADC2
    EmonLibPro::Temp[EmonLibPro::AdcId]=EmonLibPro::Sample[EmonLibPro::AdcId].Calibrated;//>>6;

    if (EmonLibPro::AdcId < VOLTSCOUNT)
    {
        EmonLibPro::AccumulatorV[EmonLibPro::AdcId].U2 += ((signed long)EmonLibPro::Temp[EmonLibPro::AdcId]*(signed long)EmonLibPro::Temp[EmonLibPro::AdcId]);
        EmonLibPro::AccumulatorV[EmonLibPro::AdcId].PeriodDiff += ((unsigned int) EmonLibPro::Sample[EmonLibPro::AdcId].PreviousTimerVal - (unsigned int)timerVal);
    }
    else
    {
        EmonLibPro::AccumulatorP[EmonLibPro::AdcId-VOLTSCOUNT].I2 += ((signed long)EmonLibPro::Temp[EmonLibPro::AdcId]*(signed long)EmonLibPro::Temp[EmonLibPro::AdcId]);
        #if CURRENTCOUNT == VOLTSCOUNT
        EmonLibPro::AccumulatorP[EmonLibPro::AdcId-VOLTSCOUNT].P += ((signed long)EmonLibPro::Temp[EmonLibPro::AdcId - CURRENTCOUNT]*(signed long)EmonLibPro::Temp[EmonLibPro::AdcId]);
        #else
        EmonLibPro::AccumulatorP[EmonLibPro::AdcId-VOLTSCOUNT].P += ((signed long)EmonLibPro::Temp[VOLTSCOUNT-1]*(signed long)EmonLibPro::Temp[EmonLibPro::AdcId]); //V0 * I[AdcId]
        #endif
    
    }


    // Last sensor reading
    if (EmonLibPro::AdcId == VOLTSCOUNT + CURRENTCOUNT - 1)
    {
        //Serial.println(EmonLibPro::Sample[0].Calibrated);

        // Quatrant 0, Cycle start. Only for Voltage sensor 0 this time.
        if (EmonLibPro::Sample[0].FlagZeroDetec) {
            // One New CYCLE Starts
            EmonLibPro::Sample[0].FlagZeroDetec=false;
			//ACSR |= (1UL<<ACIE);    // Analog Comparator Interrupt enable
            EmonLibPro::FlagCYCLE_FULL=true;
            
            // ### PLL processing ###
            // The idea is to adjust timer to get a close to 0VAC read on the first measure of the next cycle.
            // Without PLL max resolution is 1/SAMPLESPSEC (0.83 ms for 1200samples/s)
//#ifndef USE_ANALOG_COMP   
            TCNT1 = TCNT1 + ((unsigned int)EmonLibPro::Sample[0].Calibrated * (float)(PLLTIMERDELAYCOEF));
//#endif
            if((unsigned int)(EmonLibPro::Sample[0].Calibrated) > (CalCoeff.VRATIO[0] * 512/32)) { //min temporal resolution 
                EmonLibPro::pllUnlocked=EmonLibPro::SamplesPerCycle; // we're unlocked
            } else {
                if(EmonLibPro::pllUnlocked) {
                    EmonLibPro::pllUnlocked--;
                }
            }
            
            #ifdef LEDISLOCK
            //digitalWrite(LEDPIN,pllUnlocked?LOW:HIGH);
            #endif

            // When a full cycle of data has been accumulated; make a copy of accumulated
            // data before it is overwritten by next sampling instance.

            for (i=0;i<VOLTSCOUNT;i++){
                EmonLibPro::TotalV[i].U2 +=EmonLibPro::AccumulatorV[i].U2;
                EmonLibPro::TotalV[i].PeriodDiff += EmonLibPro::AccumulatorV[i].PeriodDiff;
                EmonLibPro::AccumulatorV[i].U2 = 0;
                EmonLibPro::AccumulatorV[i].PeriodDiff  = 0;
            }
            for (i=0;i<CURRENTCOUNT;i++){
                EmonLibPro::TotalP[i].I2 +=EmonLibPro::AccumulatorP[i].I2;
                EmonLibPro::TotalP[i].P +=EmonLibPro::AccumulatorP[i].P;
                EmonLibPro::AccumulatorP[i].I2 = 0;
                EmonLibPro::AccumulatorP[i].P = 0;
            }
            
            EmonLibPro::SamplesPerCycleTotal+=(EmonLibPro::SamplesPerCycle);
            EmonLibPro::SamplesPerCycle=0;
            EmonLibPro::CyclesPerTotal++;          // for average frequency calculation
            //EmonLibPro::FlagCYCLE_FULL=false;
            if (EmonLibPro::CyclesPerTotal >= CALCULATESAMPLES) {
                EmonLibPro::FlagCALC_READY = true;
                EmonLibPro::FlagINVALID_DATA = false;
            }

        }
        EmonLibPro::FlagOutOfTime = (TIFR1 & (1 << OCF1A));  // If timer counter is set we already missed a timer event. Signal this.
    }

    EmonLibPro::AdcId++;                // ADC index increment
}

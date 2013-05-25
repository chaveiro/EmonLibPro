/*
    Energy monitor Library Pro
    Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    This is a generic library for any voltage and current sensors.
    Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
    Completely line frequency independent.
    
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

#ifndef boolean
#define boolean uint8_t
#endif
#ifndef byte
#define byte uint8_t
#endif
#ifndef absolute
#define absolute(x) ((x)>0?(x):-(x))
#endif

#ifndef EmonLibPro_H
#define EmonLibPro_H


// User configurable Section 

#define VOLTSCOUNT 1      // Number of V sensors installed, can be 1 or CURRENTCOUNT number of voltage sensors
#define CURRENTCOUNT 2      // Number of CT sensors installed.

#define V1CAL 245.23        // calculated value is 243:10.9 for transformer x 11:1 for resistor divider = 122.61
#define I1CAL 10            // calculated value is 20A:0.02A for transformer / 100 Ohms for resistor = 10
#define I2CAL 10            // this is for CT2
//#define I3CAL 10          // this is for CT3

//#define SAMPLESPSEC	900	  // Samples per second 
//#define SAMPLESPSEC	1000  // Samples per second 
//#define SAMPLESPSEC	1250  // Samples per second (50Hz ok) (one sample unit includes all sensors)
#define SAMPLESPSEC	1600      // Samples per second (50Hz ok) (one sample unit includes all sensors)
//#define SAMPLESPSEC   2000  // Samples per second (50Hz ok) (one sample unit includes all sensors)
//#define SAMPLESPSEC   2500  // Samples per second (50Hz ok) (one sample unit includes all sensors)

#define CALCULATESAMPLES 50 * 2 // Number of cycles to activate FlagCALC_READY

#define SUPPLY_VOLTS 4.95     // used here because it's more accurate than the internal band-gap reference


// Don't modify below this line
//--------------------------------------------------------------------------------------------------
// constants calculated at compile time
#define TIMERTOP F_CPU/SAMPLESPSEC          // Sampling timer
#define PLLTIMERDELAYCOEF TIMERTOP/V1CAL    // PLL delay coefficient

//--------------------------------------------------------------------------------------------------
// Checks at compile time
#if !((SAMPLESPSEC % 50 == 0) && (F_CPU % SAMPLESPSEC == 0))
#warning For 50Hz line, SAMPLESPSEC defined value must be divisible by 50 and multiple of F_CPU for better performance of PLL.
#endif
#if !((SAMPLESPSEC % 60 == 0) && (F_CPU % SAMPLESPSEC == 0))
#warning For 60Hz line, SAMPLESPSEC defined value must be divisible by 60 and multiple of F_CPU for better performance of PLL.
#endif
#ifndef F_CPU
#error "F_CPU not defined"
//# define F_CPU 16000000UL // 16 Mhz
#endif


//##################################################################################################
// Variables
// Calibration Data Structure, change IRATIO/VRATIO if sensors count is changed.
 const struct	 CalDataStructure { unsigned int PCC[VOLTSCOUNT + CURRENTCOUNT];
                                    float        IRATIO[CURRENTCOUNT];
                                    float        VRATIO[VOLTSCOUNT];
                                    unsigned int MC, DPC;
                                  } static CalCoeff = {
                                    {   0,      // {0,24576,49152,73728}angle offset - must set for each sensor 
                                        24576,
                                        49152,
                                        //73728
                                    },		
                                    {   (I1CAL * SUPPLY_VOLTS)/1024,      // current gain - must set a line for each I sensor
                                        (I2CAL * SUPPLY_VOLTS)/1024,
                                        //(I3CAL * SUPPLY_VOLTS)/1024
                                    },
                                    { 
                                        (V1CAL * SUPPLY_VOLTS)/1024       // voltage gain - must set a line for each V sensor
                                    },
                                        10000, 100                        // TBD
                                 };
                                 
                                 
struct  AccVoltageDataStructure  	{ signed long U2; };
struct  AccPowerDataStructure    	{ signed long P, I2;};
struct  TotVoltageDataStructure     { unsigned long U2, cHZ; };
struct  TotPowerDataStructure       { signed long P, I2;};
struct	ResultVoltageDataStructure  { float U,HZ; };
struct	ResultPowerDataStructure    { float I,P,S,F; };

/*
extern TotVoltageDataStructure      CycleV[VOLTSCOUNT];	    //  |- Cycle Vars
extern TotPowerDataStructure        CycleP[CURRENTCOUNT];	// -
extern TotVoltageDataStructure      TotalV[VOLTSCOUNT];
extern TotPowerDataStructure        TotalP[CURRENTCOUNT];
extern ResultVoltageDataStructure   ResultV[VOLTSCOUNT];
extern ResultPowerDataStructure     ResultP[CURRENTCOUNT];
extern AccVoltageDataStructure     AccumulatorV[VOLTSCOUNT];
extern AccPowerDataStructure       AccumulatorP[CURRENTCOUNT];
*/

struct  SampleStructure     {      boolean FlagZeroDetec, WaitNextCross;
                                signed int Fresh        , Previous;
                                signed int Filtered     , PreviousFiltered;
                                signed int Calibrated   , PreviousCalibrated;
                                unsigned int TimerVal, PreviousTimerVal;
                            };
/*                                    
extern	SampleStructure   			Sample[VOLTSCOUNT + CURRENTCOUNT];
*/

class EmonLibPro
{
	public:
	    EmonLibPro(void);       //Constructor
        void begin();
        void addCycle();
		void calculateResult();
		
		static	boolean         FlagCYCLE_FULL;		// Flags a new cycle.
        static  boolean         FlagCALC_READY;     // Flags new data to calculate
        static  boolean         FlagINVALID_DATA;   // Flags Invalid data
		static	boolean         FlagOutOfTime;		// Warn ISR routing did not complete before next timer
		static	uint8_t         pllUnlocked;		// If = 0 pll is locked
		static	uint8_t                     SamplesPerCycle;       // --- Gives number of samples that got summed in summed Cycle Data Structure
        static	TotVoltageDataStructure     CycleV[VOLTSCOUNT];	   //  |- Cycle Vars
		static	TotPowerDataStructure       CycleP[CURRENTCOUNT];  // /
        static  unsigned long               SamplesPerCycleTotal;  // --- Number of cycles added for all sums of Total Var.
        static  unsigned int                CyclesPerTotal;        // --- Number of sums on Total var.
		static	TotVoltageDataStructure     TotalV[VOLTSCOUNT];    //  |- Total Vars (Sum of cycles until calculation)
		static	TotPowerDataStructure       TotalP[CURRENTCOUNT];  // -/
        static	ResultVoltageDataStructure  ResultV[VOLTSCOUNT];   // -\_ Result is here!
		static	ResultPowerDataStructure    ResultP[CURRENTCOUNT]; // -/


		// ISR vars
		static	uint8_t   		         AdcId;              // ADC PIN number of sensor measured
		static	boolean                  FlagPllUpdated;		// Flags that PLL was updated
		static	SampleStructure   		 Sample[VOLTSCOUNT + CURRENTCOUNT]; //Data for last sample
		static	uint8_t                  SamplePerAcc;               // ---Samples counter, Number of samples ajusted/detected by soft pll for each AC cycle
        static	AccVoltageDataStructure  AccumulatorV[VOLTSCOUNT];   //  |- Sum of all samples (copyed to cycle var at end of cycle)
		static	AccPowerDataStructure    AccumulatorP[CURRENTCOUNT]; // -/
		static	signed long              Temp[VOLTSCOUNT + CURRENTCOUNT];    // Internal Aux var
		
	private:
};

//extern EmonLibPro EmonLib;

#endif /* emonLibPro_H */

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
     
    History:
    1.0 - 25-05-2013 - First public release.
    1.1 - 02-06-2013 - Added suport for ADC pin MAP and revised freq calculations an timer1 fix.
    1.2 - 07-06-2013 - Speed up ISR routine a little. (Now 120 adc samples/sec for 3 sensors.)
    1.3 - 10-06-2013 - Speed up ISR by offloading freq calculation from ISR to user code.
    1.4 - 14-06-2013 - Speed up ISR by doing HPF calculation in integer math.
    1.5 - 20-06-2013 - Added CycleWaveVisualizer.html, a graph viewer in realtime on browser via ethernet.
    2.0 - 21-02-2015 - Changed ADC Triger from Timer1 A comparator to Timer1 B comparator with ADC auto trigger activated. Less gitter.
                       Improved line frequency calculation.
                       Added auto sample rate detection. Works for any line frequency at max attainable sample rate.
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

#define EMONLIBPROVERSION "2.0Pro"

//--------------------------------------------------------------------------------------------------
// User configurable Section
//--------------------------------------------------------------------------------------------------
#define VOLTSCOUNT 1        // Number of V sensors installed, can be 0 or 1, multiple voltage sensors are not supported this time
#define CURRENTCOUNT 1      // Number of CT sensors installed can be 1 to 3.

#define CONSTVOLTAGE 220    // Only used if VOLTSCOUNT = 0 - Voltage RMS fixed value 
#define CONSTFREQ     50    // Only used if VOLTSCOUNT = 0 - Line frequency fixed value

//#define V1CAL 245.23      // Gain for Voltage1 calculated value is 243:10.9 (for transformer) x 11:1 (for resistor divider) = 122.61
#define V1CAL 267.53        // Gain for Voltage1 calculated value is 243:10.9 (for transformer) x 12:1 (for resistor divider) = 122.61

#define I1CAL 10            // Gain for CT1 calculated value is 20A:0.02A (for CT spirals) / 100 Ohms (for burden resistor) = 10
#define I2CAL 10            // Gain for CT2
#define I3CAL 10            // Gain for CT3

//#define AUTOSAMPLERATE      // ADC sample rate is auto tracked for max sample rate. Requires a voltage sensor.
//#define USEPLL            // PLL is active to track zero crosses as close to 0vac as possible. (not recommended as induces gitter on Hz calculation when pll is unlocked)
#define DIAG                // Will populate CycleArray with one cycle data for diagnostics

// Only used if not using AUTOSAMPLERATE
// Samples per second (one sample unit includes all sensors)
//#define SAMPLESPSEC   1250  // Samples per second (50Hz ok)
//#define SAMPLESPSEC   1600  // Samples per second (50Hz ok)
//#define SAMPLESPSEC   2000  // Samples per second (50Hz ok) (for 3 CT, 1 volt sensors)
//#define SAMPLESPSEC   2500  // Samples per second (50Hz ok)
#define SAMPLESPSEC   3200  // Samples per second (50Hz ok) (for 2 CT, 0 Voltage sensors)
//#define SAMPLESPSEC   4000  // Samples per second (50Hz ok) (for 1 CT, 1 Voltage sensors)
//#define SAMPLESPSEC   5000  // 250khz adc Samples per second (for 2 CT, 0 Voltage sensors)
//#define SAMPLESPSEC   6400  // Samples per second (50Hz ok) (for 1 CT, 0 Voltage sensors)

#define CALCULATECYCLES 50 * 5 // Number of line cycles to activate FlagCALC_READY (50cycles x 5 = 5 secs)

#define SUPPLY_VOLTS 4.95     // used here because it's more accurate than the internal band-gap reference


//--------------------------------------------------------------------------------------------------
// Hardware Configuration Variables
//--------------------------------------------------------------------------------------------------
//ADC sampling pin order map. First values must be Voltage, last Current. If no voltage sensors are used, don't put any pin number.
//const byte adc_pin_order[] = {4,1,2,3,0,5};
const byte adc_pin_order[] = {0,1,2,3,4,5};


//--------------------------------------------------------------------------------------------------
// Macro checks - Don't change
//--------------------------------------------------------------------------------------------------
#if (VOLTSCOUNT == 0)
    #warning VOLTSCOUNT is 0, using only CT sensors
    #define VOLTSCOUNTVARS 1
    #ifdef AUTOSAMPLERATE
        #error "AUTOSAMPLERATE cant be used with VOLTSCOUNT set to 0, must use a Voltage sensor for this."
    #endif
#else
    #define VOLTSCOUNTVARS VOLTSCOUNT
#endif


//--------------------------------------------------------------------------------------------------
// Sensor Calibration Data Structure, change IRATIO/VRATIO if sensors count is changed.
//--------------------------------------------------------------------------------------------------
const struct     CalDataStructure { unsigned int PCC[VOLTSCOUNTVARS + CURRENTCOUNT];
                                    float        IRATIO[CURRENTCOUNT];
                                    float        VRATIO[VOLTSCOUNTVARS];
                                    unsigned int MC, DPC;
                                  } static CalCoeff = {
                                    {   0      // 1st sensor Phase calibration angle offset - Range is 0 to 65535
#if VOLTSCOUNTVARS + CURRENTCOUNT >= 2          // {0,23576,47152,73728}
                                        ,20076  // 2st sensor - must set for each sensor (voltage and CTs) adc pin is the same as index order of adc_pin_order
#endif
#if VOLTSCOUNTVARS + CURRENTCOUNT >= 3
                                        ,40152  // 3nd sensor
#endif
#if VOLTSCOUNTVARS + CURRENTCOUNT >= 4
                                        ,60228  // 4rd sensor
#endif                                      
                                    },      
                                    {   (I1CAL * SUPPLY_VOLTS)/1024      // 1st CT - current gain - must set a line for each I sensor same order as adc_pin_order
#if CURRENTCOUNT >= 2
                                       ,(I2CAL * SUPPLY_VOLTS)/1024      // 2nd CT
#endif
#if CURRENTCOUNT >= 3
                                       ,(I3CAL * SUPPLY_VOLTS)/1024      // 3rd CT
#endif                                      
                                    },
                                    {
#if VOLTSCOUNT == 1
                                        (V1CAL * SUPPLY_VOLTS)/1024      // voltage gain - must set a line for each V sensor
#endif                                      
                                    },
                                        10000, 100                       // TBD
                                 };


//--------------------------------------------------------------------------------------------------
// Macro checks at compile time - Don't change
//--------------------------------------------------------------------------------------------------
#ifndef F_CPU
    #error "F_CPU not defined"
    //# define F_CPU 16000000UL // 16 Mhz
#endif

#ifndef AUTOSAMPLERATE
    #if (!((SAMPLESPSEC % 50 == 0) && (F_CPU % SAMPLESPSEC == 0)) && (CONSTFREQ != 60))
        #warning For 50Hz line, SAMPLESPSEC defined value must be divisible by 50 and multiple of F_CPU for better performance of PLL.
    #endif
    #if (!((SAMPLESPSEC % 60 == 0) && (F_CPU % SAMPLESPSEC == 0)) && (CONSTFREQ != 50))
        #warning For 60Hz line, SAMPLESPSEC defined value must be divisible by 60 and multiple of F_CPU for better performance of PLL.
    #endif
#endif


//--------------------------------------------------------------------------------------------------
// Internal constants - No need to change
//--------------------------------------------------------------------------------------------------
#define TIMERTOP F_CPU/SAMPLESPSEC          // Sampling timer

#define PLLTIMERDELAYCOEF TIMERTOP/V1CAL    // PLL delay coefficient

#ifdef AUTOSAMPLERATE
    #define CYCLEARRSIZE 192/(VOLTSCOUNT + CURRENTCOUNT)  // Size of CycleArray for saving sample data if one cycle for debug
    #define TIMERSAFE 10000                               //  A value for timer increment that is know to be in close range 
#else
    #define CYCLEARRSIZE (SAMPLESPSEC/CONSTFREQ) // Size of CycleArray for saving sample data if one cycle for debug
#endif


//##################################################################################################
// Don't modify below this line
//##################################################################################################
// Does not work, first adc read after sleep is garbage.
//#define USE_ANALOG_COMP       // Specifies the usage of analog comparator for zero cross check. Requires ADC0 voltage connected to analog comp.


// If a lower resolution than 10 bits is needed, the input clock frequency to the ADC can be higher than 200kHz to get a higher sample rate.
const byte adc_sra = (1UL<<ADEN) |          // ADC Enable
                     (1UL<<ADATE) |         // ADC Auto Trigger Enable
                     (1UL<<ADIF) |          // ADC Interrupt Flag
                     (1UL<<ADIE) |          // ADC Interrupt Enable
                     (1UL<<ADPS2) |         // --- ADC Prescaler Select Bits /128 = 125kHz clock
                     (1UL<<ADPS1) |         //  |
                     (1UL<<ADPS0);          // -- comment this line to change ADC prescaler to /64 = 250kHz clock (less precision)


const byte adc_srb = //(1UL<<ACME) |        // Analog Comparator Multiplexer Enable
                     (1UL<<ADTS2) |         // ADC Auto Trigger Source | Timer/Counter1 Compare Match B
                     (1UL<<ADTS0);          //                         |

                     
struct  AccVoltageDataStructure     { unsigned long U2; unsigned long PeriodDiff; };
struct  AccPowerDataStructure       { signed long P;    unsigned long I2; };
struct  TotVoltageDataStructure     { unsigned long U2; unsigned long PeriodDiff; };
struct  TotPowerDataStructure       { signed long P;    unsigned long I2; };
struct  ResultVoltageDataStructure  { float U,HZ; };
struct  ResultPowerDataStructure    { float I,P,S,F; };


struct  SampleStructure     {   boolean FlagZeroDetec, WaitNextCross;
                                unsigned int PreviousADC;
                                signed long Filtered   , PreviousFiltered;
                                signed int Calibrated  , PreviousCalibrated;
                                unsigned int TimerVal  , PreviousTimerVal;
#ifdef DIAG
                                signed int CycleArr[CYCLEARRSIZE];
#endif
                            };

class EmonLibPro
{
    public:
        EmonLibPro(void);       //Constructor
        void begin();
        void printStatus();
        void calculateResult();
#ifdef DIAG
        static  boolean         FlagCYCLE_DIAG;     // Flags cycle diagnostics data it to be gathered on next cycle
#endif      
        static  boolean         FlagCYCLE_FULL;     // Flags a new cycle.
        static  boolean         FlagCALC_READY;     // Flags new data to calculate
        static  boolean         FlagINVALID_DATA;   // Flags Invalid data
        static  boolean         FlagOutOfTime;      // Warn ISR routing did not complete before next timer
#ifdef USEPLL
        static  uint8_t         pllUnlocked;        // If = 0 pll is locked
#endif
#ifdef AUTOSAMPLERATE        
        static  unsigned int                timeNextSample;        // Used for AUTOSAMPLERATE mode to auto adjust sample interval
#endif
        static  uint8_t                     SamplesPerCycle;       // --- Gives number of samples that got summed in summed Cycle Data Structure (ajusted/detected by soft pll for each AC cycle)
        static  unsigned long               SamplesPerCycleTotal;  // --- Number of cycles added for all sums of Total Var.
        static  unsigned int                CyclesPerTotal;        // --- Number of sums on Total var.
        static  TotVoltageDataStructure     TotalV[VOLTSCOUNTVARS];    //  |- Total Vars (Sum of cycles until calculation)
        static  TotPowerDataStructure       TotalP[CURRENTCOUNT];      // -/
        static  ResultVoltageDataStructure  ResultV[VOLTSCOUNTVARS];   // -\_ Result is here!
        static  ResultPowerDataStructure    ResultP[CURRENTCOUNT];     // -/


        // ISR vars
        static  uint8_t                  AdcId;                      // ADC PIN number of sensor measured
        static  SampleStructure          Sample[VOLTSCOUNT + CURRENTCOUNT]; //Data for last sample
        static  AccVoltageDataStructure  AccumulatorV[VOLTSCOUNTVARS];      //  |- Sum of all samples (copyed to cycle var at end of cycle)
        static  AccPowerDataStructure    AccumulatorP[CURRENTCOUNT];        // -/
        static  signed int               Temp[VOLTSCOUNT + CURRENTCOUNT];   // Internal Aux var
    private:
};


#endif /* emonLibPro_H */
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/815168b5f0b1497798c6ccf8b3b7c25e)](https://www.codacy.com/app/nchaveiro/EmonLibPro?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=chaveiro/EmonLibPro&amp;utm_campaign=Badge_Grade)
  ______                       _      _ _     _____           
 |  ____|                     | |    (_) |   |  __ \          
 | |__   _ __ ___   ___  _ __ | |     _| |__ | |__) | __ ___  
 |  __| | '_ ` _ \ / _ \| '_ \| |    | | '_ \|  ___/ '__/ _ \ 
 | |____| | | | | | (_) | | | | |____| | |_) | |   | | | (_) |
 |______|_| |_| |_|\___/|_| |_|______|_|_.__/|_|   |_|  \___/ 
                                                             
* Example folder contains an Arduino Library Demo.
* Help and support: http://openenergymonitor.org/emon/node/2406


Energy monitor Library Pro 
Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/

Generic library for any voltage and current sensors.
Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
Completely line frequency independent.

Reports Voltage, Frequency, Current, Active Power, Aparent Power and Power Factor.

This library idea cames from ATMEL AVR465: Single-Phase Power/Energy Meter.

Edit configuration in EmonLibPro.h.

Author: Nuno Chaveiro  nchaveiro[at]gmail.com  Lisbon, Portugal

History:
    1.0 - 25-05-2013 - First public release.
    1.1 - 02-06-2013 - Added suport for ADC pin MAP and revised freq calculations and timer1 fix.
    1.2 - 07-06-2013 - Speed up ISR routine a little. (Now 120 adc samples/cycle for 3 sensors.)
    1.3 - 10-06-2013 - Speed up ISR by offloading freq calculation from ISR to user code.
    1.4 - 14-06-2013 - Speed up ISR by doing HPF calculation in integer math.
    1.5 - 20-06-2013 - Added CycleWaveVisualizer.html, a graph viewer in realtime on browser via ethernet.
    2.0 - 21-02-2015 - Changed ADC Triger from Timer1 A comparator to Timer1 B comparator with ADC hardware auto trigger activated. Less gitter.
                       Improved line frequency calculation by now using full Timer1 range.
                       Added auto sample rate detection mode. Adjusts automaticaly max attainable sample rate for any line frequency.

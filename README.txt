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
1.1 - 02-06-2013 - Added suport for ADC pin MAP and revised freq calculations an timer1 fix.


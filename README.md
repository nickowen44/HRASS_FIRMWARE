High Resolution Analogue Sensor Simulator (HRASS) 
by Nick Owen 2025

//postional command :

Rain,AT,ET
Rain is in frequency

HRASS Message format
Rain,AT,ET,WD,RH,BP,WS \n

Rain = Fixed Pulse width 150ms, Pulse interval is configurable in mS
AT = variable between -40 to + 60, no decemials
ET = variable between -40 to + 60, no decemials
WD = variable 0 - 360 degrees not super accurate but functional 
RH = variable 0-100%
BP = variable 500 - 1100 hPa
WS = 0 - 100 m/s

example 
  850,23,-10,180,50,1000,50

  Rain = 150+850 = 1000mS, one tip every second
  AT = +23 Deg C
  ET = -10 Deg C
  WD = 180 Degrees
  RH = 50% RH
  BP = 1000 hPa
  WS = 50 m/s 


HRASS Settings Format
-1, RS232 Baud, WS Units, Barometer Type,0,0,0\n

-1 = settings

RS232 Baud 
  1 = RS232 Baud:9600, Databits:8, Parity:N, Stop Bits:1
  2 = RS232 Baud:1200, Databits:7, Parity:E, Stop Bits:1
WS Units
  1 = windspeed set to metres per a second
  2 = windspeed set to knots
Barometer Type
  1 = PTB330A DMU801 Feb2025 Message Format
  2 = PTB220 Almos Message Format

Example -1,1,1,1,0,0,0 \n
9600 baud, windspeed in m/s, PTB330A barometer 

todo:
  - implement key value pair parser
  - add pulse width to rain

  Change Number - Date -         - Engineer -    Description   - Note
*****************************************************************************************************************
    001          6/02/2025       N.O            Added, WD and PTB330B 
    002         10/02/2025       N.O            Added, Simulation mode using the HRASS on board barometer 
    003         17/02/2025       N.O            Added, temperature lookup table so that temperature can be commanded with signed value
    004         21/02/2025       N.O            Started work on WD look up table
    005         22/02/2025       N.O            Added hardware WS functionalit"y 
    006         27/02/2025       N.O            Added Settings 
    007         28/02/2025       N.0            Added PTB220 Almos Functionality
    
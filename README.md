GPS_time_841
======
**GPS_time_841** is a module of the Greenlogger project.  
 This module runs in a separate microcontroller.  
 It manages power, startup, and shutdown of the GPS hardware
as well as keeping itself in deep sleep most of the time.  
 It turns on the GPS and waits for a stable time signal, ignoring all other NMEA data. When the time is valid, this module translates that into a set-time command to the main microcontroller.  
 In this way, it keeps the instrument's real time clock current
avoiding drift over months/years of unattended operation, or
recovery from solar power depletion during polar night.  

#### Works on
* AVR microcontroller
ATTiny841


## Contact
#### Developer/Company
* Project of: http://www.rickshory.com/
* e-mail: rickshory@gmail.com

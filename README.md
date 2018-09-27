# justSi4432

Poor man's Si4432 driver.

The implementation is based on the various specs that can be found in the "docs" folder as well as
on the message format used by the existing RadioHead RH_RF22 library. 

This implementation is meant to deal with the Si4432 modules that can be bought (on AliExpress, etc)
for about $1.50 apiece (they also seem to be used in HopeRF RFM22). Even though there are other very similar
modules (e.g. Si4430, Si4431) this implementation has NOT been tested with anything else but Si4432 and it
only handles a subset of that transceiver's capabilities.

![alt text](https://github.com/wothke/justSi4432/raw/master/docs/Si4432.jpg "Si4432 transceivers")


If you are looking for a more portable/comprehensive implementation you might want to use RH_RF22
from the "RadioHead" library instead - which wasn't an option for me due to licensing concerns.

Personally I prefer to NOT use anything that comes with a shitty GPL license as a base
for my work. (I don't like GPL for the same reason that I don't hand out blank checks.)
Since I didn't find anything acceptable, I decided to write my own Si4432 library 
instead. (I feel that it would be stupid to let some trivial functionality library - 
like this one - dictate the licenses that I can or cannot use for my own work.. in particular
regarding potential commercial exploitation of my work by 3rd parties.)


## Known limitations

* only one module can be used in the same device, depends on hardware SPI, analog-digital converter (ADC) 
not used (see temperature sensor, etc), no frequency hopping, no EZMAC, no wake-up timer,
no support for power saving sleep modes, etc. 
* Has only been tested with ATmega128, ATmega328P.
* The Sloeber 3.0 IDE has been used duriung development. The code might not work with old Arduino IDEs and
no effort whatsoever has been made in that regard.


## License
Copyright (C) 2018 Juergen Wothke

Terms of Use: This software is licensed under a CC BY-NC-SA (http://creativecommons.org/licenses/by-nc-sa/4.0/). Commercial
licenses available on request. 
	
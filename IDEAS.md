# Future ideas

Smaller?
Probably need to drop a module.

Software toggle shunt? Or charging IC that handles all of this.

Battery bypass LDO? (Just esp?) USB must not.

Add power switch? 

Add current monitor? (INA219/226?)

Add an LP-GPIO wake button so a deep sleep can be interrupted. Deep sleep
is timer-only today, so the 5 min clamp on 0x13 is the only thing keeping
the board reachable.
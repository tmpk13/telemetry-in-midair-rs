Wi-Fi Server. Hotspot style.

Need fw-upload to trigger turning on the WIO.

Add an LP-GPIO wake button so a deep sleep can be interrupted. Deep sleep
is timer-only today, so the 5 min clamp on 0x13 is the only thing keeping
the board reachable.

Make the 15 s advertising window configurable - it dominates the current
draw at every interval.
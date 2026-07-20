Wi-Fi Server. Hotspot style.

Need fw-upload to trigger turning on the WIO.

Esp needs to sleep

Decrease packet size on wio to maximize range?

Swap to WIO-S3?

Reduce packet size by adding gps precision from a point. (Given a point in toml or BLE. Find the difference and send that to a given precision)
Now worth doing: nano-mesh padded every payload to 32 bytes, so shrinking
one saved nothing on air. Payloads now go out at their true length.

Try a slow preset now that nothing caps the listen window. SF12 is about
12 dB over SF7 and was unreachable while the mesh needed a listen period
longer than one packet's air time.


GPS free WIO-E5 listene


Need antenna for BLE.
Remove W.FL antenna add wire antenna (31 mm).
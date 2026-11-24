# V3 diagnostic build
GPS Serial2 (460800), TM171 Serial7 (115200), HWT Serial5 (9600, angle 0x53 at 10 Hz).
Danfoss ON: diagnostic virtual WAS, motor deliberately locked. Danfoss OFF: analog ADC with startup connection check.
First zero requires manual driving, fresh RTK-fixed GGA and valid VTG, >=4 km/h, >=8 m and >=3 seconds stable. A sensor timeout or implausible jump invalidates zero. No automatic motor re-arm; no drift correction in this diagnostic version.
Web page shows status. A separate UDP broadcast on port 17999 reports WAS3 status; no change to the AOG PGNs. The Windows overlay is an independent read-only display, not a modified AgOpenGPS executable.
Bench test only with physical motor disconnected. Not field-ready; sensor direction, timing, mounting, drift, GNSS validity and actual motor interlock must be verified. A future release must implement and test explicit rearm before enabling the motor.

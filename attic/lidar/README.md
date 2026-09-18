# Archived lidar USB passthrough

These sources used USB CDC #1 and uart1 (TX GP4, RX GP5, 460800 baud).
The single-port firmware no longer builds or initializes this bridge.
Restoring it requires the CDC descriptors, TinyUSB CDC count, I/O polling,
line-coding callback and wiring configuration as well as these sources.

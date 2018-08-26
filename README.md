# Hexapod Firmware

### General
    - This Hexapod frame is a PhantomX AX Mark3 with the Dynamixel AX12A servo motors from Trossen Robotics.
    - I am using the FRDM K82F ARM based micro controller from NXP/Freescale
    - The control signal is received using a ESP8266-01, which is connected to the K82F vis UART
    - The control signal is a UDP Datagram of a specific length
  ----
 
### UDP Datagram
    - Total length of 10 bytes plus delimiter
    - Bytes 0-1: Mode bitmask
    - Bytes 2-5: Left X and Y values
    - Bytes 6-9: Right X and Y values
    - Delimiter is #

###### Bitmasks
Mode|16 Bit binary  
--|--
  WALKING|0000000000000001
  RISE_BODY|0000000000000010
  WAVE_HAND|0000000000000100

Gait|16 Bit binary  
--|--
  WAVE|1000000000000000
  RIPPLE|0100000000000000
  TRIPOD|0010000000000000
----

### Something else

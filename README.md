# iot-watertank-level
Measure the level of water in the tank and send it using sigfox chip.

used sensor is https://wiki.dfrobot.com/URM07-UART_Ultrasonic_Sensor_SKU__SEN0153
it is connected to MKRFOX board like this:

| URM board | MKR Board |
|-----------|-----------|
| RX        | TX        |
| TX        | RX        |
| GND       | GND       |
| VCC       | VCC       |

There are a bunch of commented out lines.
They are mainly to check that the board is properly awakening. I used a simple dual color LED connected like this 

| RG LED | MKR Board |
|--------|-----------|
| R      | ~3        |
| G      | ~2        |
| GND    | ~4        |

Digital Pin 4 serves as ground.

To flash the board, easiest way is to use platformIO. Plug the board in USB port and press ALT+CMD+U.
If it fails to find the board, it is probable that the board is in deep sleep. You can reset the board by pressing the RST button twice. If it worked, the onboard LED shall blink gently.

before running, files contained in the patched_files folders shall be copied to :
```~/.platformio/⁨lib/⁨Arduino SigFox for MKRFox1200_ID1432⁩/src⁩/SigFox.cpp```
```~/.platformio/⁨packages/framework-arduinosam/cores⁩/samd⁩/wiring.c```

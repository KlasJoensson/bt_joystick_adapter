# bt_joystick_adapter

## Overview

This is a small program with the aim to be able to connect a joystick to a computer via bluetooth.
The adapter was created using the Zephyr code-sample <code>bluetooth_bap_broadcast_sink</code> 
from Nordic semiconductor as a starting template.

## Joystick connections

The joystick was connected to pin P1.05 to P1.08 and GND on the board via a DB9 male connector with cords from the pins. It was connected as described in the table below:

|Pin (connector) | Pin on the board | Joystick input |
|----------------|------------------|----------------|
|       1        |      P1.05       |       Up       |
|       2        |      P1.06       |      Down      |
|       3        |      P1.07       |      Left      |
|       4        |      P1.08       |      Right     |
|       5        |       ---        |       ---      |
|       6        |      P1.04       |     Buttons    |
|       7        |       ---        |       ---      |
|       8        |       GND        |     GND/VCC    |
|       9        |       ---        |       ---      |


## Output from Bluetooth

The output is a hexadecimal encoded 32 bit integer. Once the Bluetooth has connected it starts by sending 42 and then it sends other integers depending on the input from the pins, see the table below.

|Integer value|Sent hex value|Pin on the board|Meaning|
|----|---------|------|-----|
|1|010000|P1.05|Stick up|
|2|020000|P1.06|Stick down|
|4|040000|P1.07|Stick left|
|8|080000|P1.08|Stick Right|
|16|100000|P1.04|Fire button pressed|
|42|2A0000|---|A first message to tell it the game the device is connected and ready|
|2042|FA0700|sw0/P0.11|Just debug leftovers, meaning that button 1 on the hardware was pressed|



# CSS Controller main board

This board started out as a perf board, and has gone through multiple iterations. An ESP32 reads the cross-slide scale and the spindle Hall sensor, runs the display, keypad and buttons, and commands the VFD over isolated RS485 (Modbus RTU).

v0.4 Board
<img width="2000" height="1500" alt="constant-surface-speed-0 4" src="https://github.com/user-attachments/assets/d8e0d61e-7cd5-4da0-8d7d-c815cd2dbcfd" />



## At a glance

| | |
|---|---|
| Size | 126 x 54 mm, 1.6 mm, 4 layers (inner layers are ground and 3.3 V planes) |
| Controller | ESP32-WROOM-32E-N4, antenna at the right-hand edge |
| Supply | 12-24 V DC on J1, LMR33630 buck to the logic rail (set to 3.2 V) |
| RS485 | Isolated: ISO7741, B0305S-1WR3 and SP3485 |
| Mounting | Five M3 holes |

## Connectors

All are JST XH except J11, which is a 2.54 mm header. Pins are listed from pin 1.

| Ref | Use | Pins | ESP32 |
|---|---|---|---|
| J1 | Supply in | +V, GND | |
| J2 | Scale tap board | 3V3, A, B, GND, shield | A GPIO26, B GPIO27 |
| J4 | Hall sensor | +VIN, SIG, GND, shield | GPIO14 |
| J6 | RS485 to the VFD | A, B, isolated GND, shield | TX GPIO17, RX GPIO16, DE GPIO4 |
| J7 | Display (ILI9341) | VCC, GND, CS, RST, DC, SDI, SCK, LED, SDO | CS 5, RST 21, DC 19, SDI 23, SCK 18, LED 22 (SDO not connected) |
| J8 | Buttons | ON, OFF, STOP, SCREEN, GND | GPIO36, 39, 34, 35 |
| J9 | Keypad (I2C) | 3V3, GND, SDA, SCL, INT | GPIO32, 33, 25 |
| J10 | Spare | IO13, IO15, 3V3, GND | GPIO13, 15 |
| J11 | Programming | GND, TXD0, RXD0 | UART0 |

The status LED (D7) is on GPIO2.

## Circuit blocks

- **Power input.** A 0.5 A resettable fuse, TVS, common-mode choke and reverse-polarity diode, then an LC filter and the buck. LK1 and LK2 are 0R links at the buck's input and output. Leave them off to run the board from a 3.3 V bench supply on J10.
- **Isolated RS485.** Everything left of the "ISOLATED" line has its own ground and supply. Only the isolator (U7), the DC-DC module (PS1) and a 1 nF 2 kV capacitor (C25) cross the 3 mm gap. LK3 feeds PS1. JP1 fits the 120R termination. Solder jumpers JP2 and JP3 add 390R bias.
- **Hall input.** The sensor is powered from the supply through its own 0.1 A fuse. The signal passes through 22k, a clamp to 3.3 V and a Schmitt buffer. R12 (pull-up) and R13 (pull-down) stay unfitted when an RPM gauge on the same signal provides the pull-up.
- **Other inputs.** ESD arrays (TPD4E1U06) protect the scale, keypad and button lines. Buttons switch to GND; the 10k pull-ups are on the board.

## Programming

There is no USB port. Connect a 3.3 V USB-serial adapter to J11 (adapter RX to TXD0, TX to RXD0). Hold BOOT (SW2), tap EN (SW1), release BOOT, then upload. 115200 baud worked on the previous board; 921600 did not.

## Assembly notes

- **PS1:** pin 1 goes at the corner mark, the end nearest J1.
- **U5, U10 and U11** all sit the same way round.
- **L1** is square and fits four ways. Match its pin 1 dot. L3 has one too.
- Dots mark the LED cathodes.
- Use nylon screws in **H2 and H5**, which are next to the antenna. H1 is the plated chassis hole.
- Don't run "Update Footprints from Library" in KiCad. It would undo footprint edits made on the board, including PS1's outline.

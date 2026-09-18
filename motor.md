DDSM210
DDSM210
DDSM210 V2.jpg

UART Communication Port
Overview
Based on an integrated development concept, DDSM210 (Direct Drive Servo Motor) integrates an outer rotor brushless motor, encoder, and servo drive into a high-reliability permanent magnet synchronous motor. Its compact structure, convenient installation, stable operation, small size, and large torque make it particularly suitable for applications in the following direct drive fields: robot joints, small AGV drive wheels, self-balancing vehicle drive wheels, and development vehicle platforms for advanced robot projects.

Through optimization of the number of pole slots, slot type, air gap, permanent magnet material, and other related parameters, the motor has greater torque output, the smaller torque fluctuations, and achieves direct drive at low speeds with large torque, providing users with high-performance direct drive application solutions.

The driver compatible with this motor utilizes the Field Oriented Control (FOC) algorithm, coupled with the motor's built-in high-precision sensor, to achieve precise control of the motor and better noise reduction. The driver features a complete and reliable motor On-board Diagnostics (OBD) monitoring mechanism and protection functions, ensuring the safe and reliable operation of the motor.

Simultaneously, we provide an open source four-wheel drive off-road vehicle structure model for this model of direct drive servo motor, which can be downloaded from the "Resources"-"Open Source Structure" section at the bottom.

Usage Precaution
Please confirm whether the operating voltage is the voltage range specified in this article before use.
Please make sure the motor is used under the specified environment range (-25℃~45℃), the motor over-temperature protection threshold is 80°C, and the protection is released when the temperature is lower than 5°C.
Please avoid immersing the motor in water, otherwise, it may cause abnormal operation or damage to the motor.
Please ensure that the wiring is correct and stable before use to avoid poor contact.
Please refer to the installation instructions before using the motor to ensure that the motor is installed correctly and firmly.
Please refer to the installation instructions before using the motor to ensure that the external output part of the motor is installed correctly and firmly.
Please avoid damage to the wire during use, otherwise, it may cause abnormal operation or damage to the motor.
Do not touch the rotating part of the motor during use to avoid injury.
When the motor outputs high torque, it will generate heat. Do not touch the motor to avoid burns.
Do not disassemble the motor without permission, otherwise, it may cause abnormal operation or damage to the motor and may bring safety hazards.
Features
Ultra-low noise.
High precision and zero-backlash.
Fast response, direct drive without delay.
Integrated motor and driver, compact structure, and high integration.
Supports UART communication.
Through communication, obtaining motor information such as position, speed, current, error code, etc..
With Hall position detection, over-current protection, etc..
Supports electric brake.
No drive mechanical friction, drive efficiency close to 99.99%.
Parameters
No-load speed: 210±11rpm
No-load current: 0.15A
Rated speed: 98rpm
Rated torque: 0.25Nm
Rated current: 0.5A
Locked-rotor torque: 0.85Nm
Locked-rotor current: ≦3A
Input voltage: 11 ~ 22V DC
Speed constant: 14rpm/V
Operating ambient temperature: -25~45℃
Total weight: 260g
Encoder resolution: 4096
Relative precision: 1024
Noise level: ≦48dB(A)
Single-wheel load: 3kg
Note: The above parameters are measured at the rated voltage of 14.4 VDC.

Motor Interface and Cable Sequence Specification
DDSM115-A.jpg

Signal Cable (ZH1.5*4P)	Name	Type	Description
1	UART	Tx	UART_Tx
2	UART	Rx	UART_Rx
3	GND	-	/
4	VCC	+	/


How To Use
The simple control method is only used to verify the working principle of the motor and to do some simple tests. For the specific method, please refer to the chapter on the communication protocol.
It is recommended to use USB to TTL for the device with a USB interface to control the direct drive servo motor.
Signal cable wiring. Refer to the table provided above for the pin definition, which references the motor interface and the cable sequence specification. Connect the motor's TX to the converter's RX, connect the motor's RX to the converter's TX, and connect the GND of the motor signal cable to the GND of the converter.
Power cable. You can refer to the above motor interface and cable sequence specification for the pin definition. Connect the VCC of the signal cable to the positive poles of the 12V DC power (9~28V DC), and connect the GND of the signal cable to the negative poles of the 12V DC power (9~28V DC).
Download OSDA open-source serial port assistant.
x86 Github download link
x64 Github download link
Run the OSDA open source serial port debugging assistant, select the serial port of the device, select 115200 for the baud rate, check Hex to receive, check Hex to send, and then open the serial port to send commands to the motor.
DDSM115 spec10.jpg

Note that the motor will start to rotate after some commands are sent. Do not touch the rotating parts of the motor. If you do not have suitable structural parts to install the motor, be sure to prepare for power failure at any time, and do not give high-speed commands.
The following is the UART communication command set:
1. Switch to velocity loop (02), this command has no feedback.
01 A0 02 00 00 00 00 00 00 E4
2. For additional feedback, query the motor mode.
01 75 00 00 00 00 00 00 00 47
3. Brake command, valid in velocity loop mode.
01 64 00 00 00 00 00 FF 00 D1
4. ID setting（01), send the command five times continuously.
AA 55 53 01 00 00 00 00 00 CB
5. ID query.
C8 64 00 00 00 00 00 00 00 DE
6. Velocity loop command (－210～210 rpm)
01 64 FF CE 00 00 00 00 00 DA (-5rpm)
01 64 FF 9C 00 00 00 00 00 9A (-10rpm)
01 64 00 00 00 00 00 00 00 50 (0rpm)
01 64 00 32 00 00 00 00 00 D3 (5rpm)
01 64 00 64 00 00 00 00 00 4F (10rpm)
7. Position loop command (0～32767 corresponds to 0～360°)
01 64 00 00 00 00 00 00 00 50 (0)
01 64 27 10 00 00 00 00 00 57 (10000)
01 64 4E 20 00 00 00 00 00 5E (20000)
01 64 75 30 00 00 00 00 00 A7 (30000)

Communication Protocol
Baudrate: 115200
Data bits: 8bit
Stop bit: 1bit
Parity bit: none
Data length: 10 bytes
Reply form: one question and one answer
Rate: up to 500Hz
In velocity loop mode: -2100~2100 corresponds to the range -210rpm~210rpm, unit: 0.1rpm, the data type: signed 16-bit
In position loop mode: 0~32767 corresponds to the range 0°~360°, data type: unsigned 16-bit
Steps:
①Set the motor ID (save when power is off)
②Set the motor mode (open loop, velocity loop, position loop, the default is velocity loop)
③Send the given value
CRC8 value:
The value after the CRC8 check is performed on the values DATA[0]~DATA[8].
CRC algorithm: CRC-8/MAXIM
Polynomial: x8 + x5 + x4 +1
The verification product stage can use this website to calculate the check digit: https://crccalc.com/
Protocol 1: Drive Motor to Rotate
Send to the motor:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x64	Speed/position set high 8 bits	Speed/position set low 8 bits	Feedback 1 content	Feedback 2 content	Acceleration time	Brake	0	CRC8
Motor feedback:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x64	Feedback 1 content high 8 bits	Feedback 1 content low 8 bits	Feedback 2 content high 8 bits	Feedback 2 content low 8 bits	Acceleration time	Temperature	Error code	CRC8
Acceleration time: valid in the speed loop mode, the acceleration time per 1rpm, the unit is 0.1ms. When it is set to 1, the acceleration time per 1rpm is 0.1ms. When it is set to 10, the acceleration time per 1rpm is 10*0.1 ms=1ms. When set to 0, the default is 1, and the acceleration time per 1rpm is 0.1ms.
Brake: 0XFF other values do not brake, valid in speed loop mode.
Feedback content: 0X01 - speed value 0X02 - bus current 0X03 - position value
Protocol 2: Get Other Feedback
Send to the motor:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x74	0	0	0	0	0	0	0	CRC8
Motor feedback:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x74	Mileage laps high 8 bits	Mileage laps second high 8 bits	Mileage laps second low 8 bits	Mileage laps low 8 bits	Position high 8 bits	Position low 8 bits	Error code	CRC8
Mileage laps: Loop range -2147483467 to 2147483467, reset to 0 when power on it again.
Position value: 0~65535 corresponds to 0~360°
Error code:
Error Code	BIT7	BIT6	BIT5	BIT4	BIT3	BIT2	BIT1	BIT0
Content	Reserved	Reserved	Reserved	Overtemperature error	Reserved	Reserved	Overcurrent error	Reserved
For example, the error code 0x02, that is 0b00000010, means an overcurrent error.

Protocol 3: Motor Mode Switch Sending Protocol
Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0xA0	Mode Value	0	0	0	0	0	0	CRC8
Motor feedback:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0xA0	Mode Value	0	0	0	0	0	0	CRC8
Mode value:
0x00: set to open loop
0x02: set to velocity loop
0x03: set to position loop

Protocol 4: Moto ID Sets Sending Protocol
Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	0xAA	0x55	0x53	ID	0	0	0	0	0	CRC8
Motor feedback:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x64	0	0	0	0	0	0	0	CRC8
Note: When setting the ID, ensure that only one motor is on the bus. Each time power is applied, only one setting is allowed. The motor will execute the setting after receiving the ID setting command five times.

Protocol 5: Obtain Mode Feedback
Send to the motor:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x75	0	0	0	0	0	0	0	CRC8
Motor feedback:

Data Field	DATA[0]	DATA[1]	DATA[2]	DATA[3]	DATA[4]	DATA[5]	DATA[6]	DATA[7]	DATA[8]	DATA[9]
Content	ID	0x75	Mode Value	0	0	0	0	0	0	CRC8
Mode value:

0x00: open loop
0x02L velocity loop
0x03: position loop
Protection Rules
1. Bus overcurrent protection threshold: 2.8A. If the overcurrent duration exceeds 5 seconds, the shutdown protection will be triggered, and it will be released after 5 seconds;

2. Motor overtemperature protection threshold:

 1) 80℃, the shutdown protection will be triggered when the temperature is higher than 80°C, and it will be automatically 
released when the temperature drops below this threshold by 5℃;
 2) -25℃, the shutdown protection will be triggered when the temperature is below -25 ℃, and it will be automatically 
released when the temperature is 5°C above the threshold;
3. Stall prevention: the protection will be triggered after the stall duration exceeds 5S, and it will be automatically released after 5S (the speed loop is effective)

4. Overvoltage protection threshold:

 1) 28V, the shutdown protection will be triggered when the voltage is higher than 28V, and the protection will be 
automatically released when the voltage is 0.5V lower than the threshold;
 2) 9V, the shutdown protection will be triggered when the voltage is lower than 9V, and the protection will be 
automatically released when the voltage is 0.5V higher than the threshold;
5. Short-circuit protection: When the bus current exceeds 20A, the shutdown protection is triggered and can only be released by restarting

Installation Guide
Refer to the motor mounting hole dimensions and positions to install the motor on the corresponding device. Unit: mm
The mounting holes on the motor installation end are Ø2.3, 6 mm deep, with the hole centers evenly distributed on a diameter of 18 mm.

The mounting holes are straight holes without threads; self‑tapping screws must be used.

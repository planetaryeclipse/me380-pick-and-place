#!/bin/python3

import serial
import struct
import time

comms = serial.Serial('/dev/ttyUSB0', baudrate=115200)

toggle_control_mode = True

activate_sweep = True
activate_pushoff_1 = True
activate_pushoff_2 = True
activate_pushoff_3 = True
stepper_dir = True

arm_1_servo = 1000
arm_2_servo = 2000
arm_3_servo = 3000

steps = 4000

cmd_buf = struct.pack('<cccBHHHHcc',
    b'S',
    b'A',
    b'M',
    (toggle_control_mode << 7) | (activate_sweep << 6) | (activate_pushoff_1 << 5) | (activate_pushoff_2 << 4) | (activate_pushoff_3 << 3) | (stepper_dir << 2),
    arm_1_servo,
    arm_2_servo,
    arm_3_servo,
    steps,
    b'\r',
    b'\n'
)

print('Sending test command:')

for ch in cmd_buf:
    print(hex(ch), end='')
    print(' ', end='')
print()

comms.write(cmd_buf)

# while True:
#     recv = comms.read_all()

#     if len(recv) > 0:
#         print(recv)

#     time.sleep(0.05)

comms.close()
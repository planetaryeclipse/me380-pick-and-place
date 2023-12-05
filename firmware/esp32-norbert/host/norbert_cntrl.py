#!/bin/python3

import serial
import struct
import time

import curses

from contextlib import nullcontext

from dataclasses import dataclass

from evdev import InputDevice, ecodes, list_devices

from threading import Lock, Thread
from typing import Optional

#region Controller

# Please note that this controller read code was produced as part of a project
# that I worked on during the summer. This only acts as the interface and the
# use of this controller has been developed independently.

@dataclass
class ControllerRead:
    is_connected: bool = False

    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    button_right_trigger: bool = False
    button_left_trigger: bool = False
    button_right_thumb: bool = False
    button_left_thumb: bool = False
    button_start: bool = False
    button_select: bool = False
    button_mode: bool = False

    left_joy_horiz: float = 0
    left_joy_vert: float = 0
    right_joy_horiz: float = 0
    right_joy_vert: float = 0

    dpad_horiz: float = 0
    dpad_vert: float = 0

    right_bumper: float = 0
    left_bumper: float = 0

RAW_AXIS_DEADZONE = 30

def read_controller(
        curr_controller_read: ControllerRead,
        curr_controller_read_lock: Optional[Lock] = None):
    
    controller_mapping = {
        # Buttons
        ecodes.BTN_A: 'button_a',
        ecodes.BTN_B: 'button_b',
        ecodes.BTN_X: 'button_x',
        ecodes.BTN_Y: 'button_y',
        ecodes.BTN_TR: 'button_right_trigger',
        ecodes.BTN_TL: 'button_left_trigger',
        ecodes.BTN_THUMBR: 'button_right_thumb',
        ecodes.BTN_THUMBL: 'button_left_thumb',
        ecodes.BTN_START: 'button_start',
        ecodes.BTN_SELECT: 'button_select',
        ecodes.BTN_MODE: 'button_mode',

        # Axes
        ecodes.ABS_HAT0X: 'dpad_horiz',
        ecodes.ABS_HAT0Y: 'dpad_vert',
        ecodes.ABS_X: 'left_joy_horiz',
        ecodes.ABS_Y: 'left_joy_vert',
        ecodes.ABS_RX: 'right_joy_horiz',
        ecodes.ABS_RY: 'right_joy_vert',
        ecodes.ABS_Z: 'left_bumper',
        ecodes.ABS_RZ: 'right_bumper'
    }

    while True:
        # Sets is_connected to False if the connection fails
        with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
            curr_controller_read.is_connected = False
        try:
            # Finds the path of the X-Box controller
            cntrllr_path = None
            devices = [InputDevice(path) for path in list_devices()]
            for device in devices:
                # print(f'path={device.path}, name={device.name}, phys={device.phys}\n')
                if device.name == 'Generic X-Box pad':
                    cntrllr_path = device.path

            if cntrllr_path is None:
                continue

            # For some reason a simple time delay is necessary to properly read the
            # initial state of pressed buttons. However, time delays are insufficient
            # to be able to properly read initial joystick states and seems to require
            # a full open and close again.
            
            device = InputDevice(cntrllr_path)
            device.close()

            device = InputDevice(cntrllr_path)
        except:
            continue  # Does not attempt to continue with controller read if it fails
        
        # Sets connection as successful as the device has successfully setup
        with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
            curr_controller_read.is_connected = True

        print('Controller is connected!')

        try:
            # Reads the initial buttons and axes of the controller
            axes_min_max = dict()
            with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
                button_reads = device.active_keys()
                for button_id in button_reads:
                    setattr(curr_controller_read, controller_mapping[button_id], True)

                axes_capabilities = device.capabilities()[ecodes.EV_ABS]
                for axis_id, info in axes_capabilities:
                    axes_min_max[axis_id] = (info.min, info.max)
                    
                    if abs(info.value) < RAW_AXIS_DEADZONE:
                        continue
                    scaled_value = info.value / info.max if info.value >= 0 else info.value / abs(info.min)
                    setattr(curr_controller_read, controller_mapping[axis_id], scaled_value)
            
            # Reads updates made to the buttons and axes of the controller thorugh a series of events
            for event in device.read_loop():
                with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
                    # print(event)
                    if event.type == ecodes.EV_KEY:
                        setattr(curr_controller_read, controller_mapping[event.code], event.value == 1)  # Forces boolean
                    elif event.type == ecodes.EV_ABS:
                        if abs(event.value) < RAW_AXIS_DEADZONE and (event.code != 16 and event.code != 17):
                            scaled_value = 0
                        else:
                            scaled_value = event.value / axes_min_max[event.code][1] if event.value >= 0 else event.value / abs(axes_min_max[event.code][0])
                        setattr(curr_controller_read, controller_mapping[event.code], scaled_value)
        except:
            # If this fails at any time it means the controller has become disconnected
            # therefore want to simply continue to attempt reconnection again
            print('Controller is disconnected!')
            pass

#endregion

#region Norbert control

NORBERT_SERIAL_FILE = '/dev/ttyUSB0'
NORBERT_BAUD_RATE = 115200

STARTUP_MANUAL_CNTRL = True

MIN_ARM_SERVO_VAL = 0
MAX_ARM_SERVO_VAL = 65535

ARM_SERVO_RATE_TOT_TIME = 2
ARM_SERVO_RATE = MAX_ARM_SERVO_VAL / ARM_SERVO_RATE_TOT_TIME

ROTATION_TOT_TIME = 1
BASE_STEPS_PER_ROTATION = 200
MICROSTEP = 1
STEP_RATE = 200 / ROTATION_TOT_TIME

@dataclass
class NorbertControl:
    manual_cntrl_enabled: bool = STARTUP_MANUAL_CNTRL

    toggle_cntrl_enabled: bool = False

    # States of non-lifting servos
    activate_sweep: bool = False
    activate_pushoff_1: bool = False
    activate_pushoff_2: bool = False
    activate_pushoff_3: bool = False
    stepper_dir: bool = False
    
    # Position of lifting servos
    arm_1_servo: int = (MIN_ARM_SERVO_VAL + MAX_ARM_SERVO_VAL) / 2
    arm_2_servo: int = (MIN_ARM_SERVO_VAL + MAX_ARM_SERVO_VAL) / 2
    arm_3_servo: int = (MIN_ARM_SERVO_VAL + MAX_ARM_SERVO_VAL) / 2

    # Position of rotating base
    total_steps: int = 0

    # Steps to take (positive is dir. +ve, negative is dir. -ve)
    curr_steps_to_take: int = 0

    # Needed for implementing a toggle button
    a_btn_curr_held: bool = False
    b_btn_curr_held: bool = False
    x_btn_curr_held: bool = False
    y_btn_curr_held: bool = False
    right_trig_btn_curr_held: bool = False
    left_trig_btn_curr_held: bool = False
    start_btn_curr_held: bool = False

def _clamp(val, min, max):
    if val > max:
        return max
    elif val < min:
        return min
    return val

def update_norbert_control_from_controller(cntrl_read: ControllerRead, norbert_cntrl: NorbertControl):
    # Only triggers change in servo positions when the button is first pressed
    if not norbert_cntrl.x_btn_curr_held and cntrl_read.button_x:
        norbert_cntrl.activate_pushoff_1 = not norbert_cntrl.activate_pushoff_1
    if not norbert_cntrl.a_btn_curr_held and cntrl_read.button_a:
        norbert_cntrl.activate_pushoff_2 = not norbert_cntrl.activate_pushoff_2
    if not norbert_cntrl.b_btn_curr_held and cntrl_read.button_b:
        norbert_cntrl.activate_pushoff_3 = not norbert_cntrl.activate_pushoff_3
    if not norbert_cntrl.y_btn_curr_held and cntrl_read.button_y:
        norbert_cntrl.activate_sweep = not norbert_cntrl.activate_sweep

    # Handles stepped rotation by a set angle
    if not norbert_cntrl.right_trig_btn_curr_held and cntrl_read.button_right_trigger:
        norbert_cntrl.curr_steps_to_take += BASE_STEPS_PER_ROTATION * MICROSTEP /3
        norbert_cntrl.stepper_dir = True
    if not norbert_cntrl.left_trig_btn_curr_held and cntrl_read.button_left_trigger:
        norbert_cntrl.curr_steps_to_take += BASE_STEPS_PER_ROTATION * MICROSTEP / 3
        norbert_cntrl.stepper_dir = False

    if not norbert_cntrl.start_btn_curr_held and cntrl_read.button_start:
        norbert_cntrl.toggle_cntrl_enabled = True

    # Handles up/down stepping with D-pad
    if cntrl_read.dpad_vert > 0.5:
        # Sets to the down position (axis is inverted)
        norbert_cntrl.arm_1_servo = MIN_ARM_SERVO_VAL
        norbert_cntrl.arm_2_servo = MIN_ARM_SERVO_VAL
        norbert_cntrl.arm_3_servo = MIN_ARM_SERVO_VAL
    elif cntrl_read.dpad_vert < -0.5:
        # Sets to the up position (axis is inverted)
        norbert_cntrl.arm_1_servo = MAX_ARM_SERVO_VAL
        norbert_cntrl.arm_2_servo = MAX_ARM_SERVO_VAL
        norbert_cntrl.arm_3_servo = MAX_ARM_SERVO_VAL

    # Handles up/down control with right vertical
    if cntrl_read.right_joy_vert:
        time_for_lift = 2
        change_in_pos = MAX_ARM_SERVO_VAL / time_for_lift * 0.05 * cntrl_read.right_joy_vert * -1  # Gets a time rate

        norbert_cntrl.arm_1_servo += change_in_pos
        norbert_cntrl.arm_2_servo += change_in_pos
        norbert_cntrl.arm_3_servo += change_in_pos

        norbert_cntrl.arm_1_servo = _clamp(norbert_cntrl.arm_1_servo, MIN_ARM_SERVO_VAL, MAX_ARM_SERVO_VAL)
        norbert_cntrl.arm_2_servo = _clamp(norbert_cntrl.arm_2_servo, MIN_ARM_SERVO_VAL, MAX_ARM_SERVO_VAL)
        norbert_cntrl.arm_3_servo = _clamp(norbert_cntrl.arm_3_servo, MIN_ARM_SERVO_VAL, MAX_ARM_SERVO_VAL)

    if cntrl_read.left_joy_horiz:
        steps_per_sec = 60
        steps_to_take = steps_per_sec * 0.05 * abs(cntrl_read.left_joy_horiz)

        norbert_cntrl.stepper_dir = cntrl_read.left_joy_horiz > 0
        norbert_cntrl.curr_steps_to_take = steps_to_take


    # Updates states for toggle implementation
    norbert_cntrl.a_btn_curr_held = cntrl_read.button_a
    norbert_cntrl.b_btn_curr_held = cntrl_read.button_b
    norbert_cntrl.x_btn_curr_held = cntrl_read.button_x
    norbert_cntrl.y_btn_curr_held = cntrl_read.button_y

    norbert_cntrl.right_trig_btn_curr_held = cntrl_read.button_right_trigger
    norbert_cntrl.left_trig_btn_curr_held = cntrl_read.button_left_trigger

    norbert_cntrl.start_btn_curr_held = cntrl_read.button_start


def update_norbert_cntrl(norbert: serial.Serial, norbert_ctrl: NorbertControl):
    # print('update')
    # print(norbert_ctrl.arm_1_servo)
    # print(norbert_ctrl.arm_2_servo)
    # print(norbert_ctrl.arm_3_servo)
    # print(norbert_ctrl.curr_steps_to_take)

    cmd_buf = struct.pack('<cccBHHHHcc',
        b'S',
        b'A',
        b'M',
        (norbert_ctrl.toggle_cntrl_enabled << 7) |
            (norbert_ctrl.activate_sweep << 6) | 
            (norbert_ctrl.activate_pushoff_1 << 5) | 
            (norbert_ctrl.activate_pushoff_2 << 4) | 
            (norbert_ctrl.activate_pushoff_3 << 3) | 
            (norbert_ctrl.stepper_dir << 2),
        int(norbert_ctrl.arm_2_servo),
        int(norbert_ctrl.arm_1_servo),
        int(norbert_ctrl.arm_3_servo),
        int(norbert_ctrl.curr_steps_to_take),
        b'\r',
        b'\n')
    norbert.write(cmd_buf)

    # Updates internal trackers
    if norbert_ctrl.toggle_cntrl_enabled:
        norbert_ctrl.manual_cntrl_enabled = not norbert_ctrl.manual_cntrl_enabled
        norbert_ctrl.toggle_cntrl_enabled = False

    norbert_ctrl.total_steps += norbert_ctrl.curr_steps_to_take * (1 if norbert_ctrl.stepper_dir else -1)
    norbert_ctrl.curr_steps_to_take = 0

#endregion

def main(stdscr):
    # Sets up communication with the Norbert prorotype
    norbert = None
    try:
        norbert = serial.Serial(NORBERT_SERIAL_FILE, NORBERT_BAUD_RATE)
    except:
        pass

    norbert_ctrl = NorbertControl()

    # Sets up the thread to read from the controller to ensure that the
    # most recent updates have been polled
    
    curr_controller_read: ControllerRead = ControllerRead()
    curr_controller_read_lock: Lock = Lock()

    controller_thread = Thread(
        target=read_controller,
        daemon=True,
        args=[
            curr_controller_read,
            curr_controller_read_lock])
    controller_thread.start()

    while True:
        stdscr.clear()

        stdscr.addstr(f'Norbert v0.1 - Control Software\n')
        stdscr.addstr(f'ME 380 - Group 20 - Classic XX\n')
        stdscr.addstr(f'Author: Samuel Street\n\n')

        with curr_controller_read_lock:
            if curr_controller_read.is_connected:
                stdscr.addstr(
                    f'Controller input: \tbtn_a={curr_controller_read.button_a!s:5}, '
                    f'btn_b={curr_controller_read.button_b!s:5}, '
                    f'btn_x={curr_controller_read.button_x!s:5}, '
                    f'btn_y={curr_controller_read.button_y!s:5}'
                    f'\n\t\t\tbtn_left_trigger={curr_controller_read.button_left_trigger!s:5}, '
                    f'btn_right_trigger={curr_controller_read.button_right_trigger!s:5}, '
                    f'lbumper={curr_controller_read.left_bumper:+.03f}, '
                    f'rbumper={curr_controller_read.right_bumper:+.03f}'
                    f'\n\t\t\tleft_x={curr_controller_read.left_joy_horiz:+.03f}, '
                    f'left_y={curr_controller_read.left_joy_vert:+.03f}, '
                    f'right_x={curr_controller_read.right_joy_horiz:+.03f}, '
                    f'right_y={curr_controller_read.right_joy_vert:+.03f}, '
                    f'\n\t\t\tdpad_x={curr_controller_read.dpad_horiz:+.03f}, '
                    f'dpad_y={curr_controller_read.dpad_vert:+.03f}\n')
                update_norbert_control_from_controller(curr_controller_read, norbert_ctrl)
            else:
                stdscr.addstr(f'Unable to find X-Box controller, verify the connection\n')

        stdscr.addstr('\n')

        stdscr.addstr(f'Norbert connected: \t{norbert is not None}')

        stdscr.addch('\n')

        stdscr.addstr(f'Current control mode: \t{"manual" if norbert_ctrl.manual_cntrl_enabled else "automatic"}')

        stdscr.addstr('\n\n')

        stdscr.addstr(f'Non-lifting servos: \tsweep={norbert_ctrl.activate_sweep!s:5}, pushoff_1={norbert_ctrl.activate_pushoff_1!s:5}, pushoff_2={norbert_ctrl.activate_pushoff_2!s:5}, pushoff_3={norbert_ctrl.activate_pushoff_3!s:5}')
        stdscr.addch('\n')
        stdscr.addstr(f'Lifting servos: \tarm_1={norbert_ctrl.arm_1_servo:05}, arm_2={norbert_ctrl.arm_2_servo:05}, arm_3={norbert_ctrl.arm_3_servo:05}')
        stdscr.addch('\n')
        stdscr.addstr(f'Base rot. (+ dir.): \ttot_steps={norbert_ctrl.total_steps}, curr_dir={"+" if norbert_ctrl.stepper_dir else "-"}')

        stdscr.addstr('\n\n')

        # Detects whether the ESP32 is disconnected and attempts reconnection
        if norbert is None:
            try:
                norbert = serial.Serial(NORBERT_SERIAL_FILE, NORBERT_BAUD_RATE)
                norbert_ctrl = NorbertControl()  # Resets control
            except:
                stdscr.addstr('Norbert is not connected and cannot be controlled!\nRetrying connection...')
        else:
            try:
                # Updates Norbert using command generated after update of the controller
                update_norbert_cntrl(norbert, norbert_ctrl)
            except Exception as err:
                # print(err)
                # while(True):
                #    pass
                norbert = None
        
        stdscr.refresh()

        time.sleep(0.05)  # 20 Hz update to prototype and UI

if __name__ == "__main__":
    curses.wrapper(main)

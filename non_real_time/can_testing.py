#!/usr/bin/env python3

import argparse
import os
import re
import struct
import time
import select
import sys
import termios
import tty
import can

# ============================================
# CONFIG
# ============================================

CAN_INTERFACE = "can0"
CAN_BITRATE = 500000

# ============================================
# COMMAND IDS
# ============================================

CMD_LINEAR_MOVE      = 0x10
CMD_AUTO_TUNE        = 0x11
CMD_QUADRATIC_MOVE   = 0x12
CMD_SINGLE_AXIS_MOVE = 0x13
CMD_RAPID_MOVE       = 0x14

CMD_LEG_STATE        = 0x20

# ============================================
# ISO-TP TYPES
# ============================================

ISO_TP_SINGLE_FRAME      = 0x0
ISO_TP_FIRST_FRAME       = 0x1
ISO_TP_CONSECUTIVE_FRAME = 0x2
ISO_TP_FLOW_CONTROL      = 0x3

# ============================================
# CAN SETUP
# ============================================

bus = can.interface.Bus(
    interface="socketcan",
    channel=CAN_INTERFACE
)

# ============================================
# ISO-TP RX STATE
# ============================================

rx_buffer = bytearray()
expected_size = 0
rx_active = False
expected_seq = 1

# ============================================
# NONBLOCKING KEY PRESS
# ============================================

def key_pressed():

    dr, _, _ = select.select([sys.stdin], [], [], 0)

    return dr != []

# ============================================
# ISO-TP SEND
# ============================================

def send_isotp(node_id: int, payload: bytes):

    payload_len = len(payload)

    if payload_len > 256:
        raise ValueError("Payload too large")

    # ----------------------------------------
    # SINGLE FRAME
    # ----------------------------------------

    if payload_len <= 7:

        data = bytearray(8)

        data[0] = (
            (ISO_TP_SINGLE_FRAME << 4) |
            (payload_len & 0x0F)
        )

        data[1:1 + payload_len] = payload

        msg = can.Message(
            arbitration_id=node_id,
            data=data[:payload_len + 1],
            is_extended_id=False
        )

        bus.send(msg)

        print("\nTX SINGLE FRAME")
        print(f"CAN ID : 0x{node_id:X}")
        print(f"DATA   : {[hex(x) for x in msg.data]}")

        return

    # ----------------------------------------
    # FIRST FRAME
    # ----------------------------------------

    first = bytearray(8)

    first[0] = (
        (ISO_TP_FIRST_FRAME << 4) |
        ((payload_len >> 8) & 0x0F)
    )

    first[1] = payload_len & 0xFF

    first[2:8] = payload[:6]

    msg = can.Message(
        arbitration_id=node_id,
        data=first,
        is_extended_id=False
    )

    bus.send(msg)

    print("\nTX FIRST FRAME")
    print(f"CAN ID : 0x{node_id:X}")
    print(f"DATA   : {[hex(x) for x in msg.data]}")

    # ----------------------------------------
    # CONSECUTIVE FRAMES
    # ----------------------------------------

    offset = 6
    seq = 1

    while offset < payload_len:

        frame = bytearray(8)

        frame[0] = (
            (ISO_TP_CONSECUTIVE_FRAME << 4) |
            (seq & 0x0F)
        )

        remaining = payload_len - offset

        copy_len = min(7, remaining)

        frame[1:1 + copy_len] = payload[offset:offset + copy_len]

        msg = can.Message(
            arbitration_id=node_id,
            data=frame[:copy_len + 1],
            is_extended_id=False
        )

        bus.send(msg)

        print("\nTX CONSECUTIVE FRAME")
        print(f"SEQ    : {seq}")
        print(f"CAN ID : 0x{node_id:X}")
        print(f"DATA   : {[hex(x) for x in msg.data]}")

        offset += copy_len

        seq += 1

        #
        # Small pacing delay
        #
        time.sleep(0.001)

# ============================================
# ISO-TP RX
# ============================================

def process_rx_message(msg):

    global rx_buffer
    global expected_size
    global rx_active
    global expected_seq

    d = msg.data

    if len(d) == 0:
        return

    pci = (d[0] >> 4) & 0x0F

    # ----------------------------------------
    # SINGLE FRAME
    # ----------------------------------------

    if pci == ISO_TP_SINGLE_FRAME:

        payload_len = d[0] & 0x0F

        payload = d[1:1 + payload_len]

        handle_payload(msg.arbitration_id, payload)

    # ----------------------------------------
    # FIRST FRAME
    # ----------------------------------------

    elif pci == ISO_TP_FIRST_FRAME:

        expected_size = (
            ((d[0] & 0x0F) << 8) |
            d[1]
        )

        rx_buffer = bytearray(d[2:8])

        rx_active = True

        expected_seq = 1

    # ----------------------------------------
    # CONSECUTIVE FRAME
    # ----------------------------------------

    elif pci == ISO_TP_CONSECUTIVE_FRAME:

        if not rx_active:
            return

        seq = d[0] & 0x0F

        if seq != (expected_seq & 0x0F):

            print("ISO-TP SEQUENCE ERROR")

            rx_active = False

            return

        rx_buffer.extend(d[1:])

        expected_seq += 1

        if len(rx_buffer) >= expected_size:

            payload = rx_buffer[:expected_size]

            handle_payload(
                msg.arbitration_id,
                payload
            )

            rx_active = False

# ============================================
# PAYLOAD HANDLER
# ============================================

latest_leg_telemetry = {}


def clear_screen():

    if os.name == "nt":
        os.system("cls")
    else:
        print("\033[2J\033[H", end="")


def display_telemetry_table():

    clear_screen()

    header = (
        "Leg |   Pos X   Pos Y   Pos Z   Torque X  Torque Y  Torque Z"
    )

    print("LEG TELEMETRY MONITOR")
    print("==========================================================")
    print(header)
    print("----+-----------------------------------------------------")

    for leg in range(6):

        if leg in latest_leg_telemetry:
            positions, torques = latest_leg_telemetry[leg]

            print(
                f" {leg}  | "
                f"{positions[0]:8.3f} "
                f"{positions[1]:8.3f} "
                f"{positions[2]:8.3f}   "
                f"{torques[0]:8.3f} "
                f"{torques[1]:8.3f} "
                f"{torques[2]:8.3f}"
            )
        else:
            print(
                f" {leg}  | {'-':>8} {'-':>8} {'-':>8}   {'-':>8} {'-':>8} {'-':>8}"
            )

    print("\nPress any key to exit monitor mode.")


def handle_payload(arbitration_id, payload):

    if len(payload) == 0:
        return

    cmd = payload[0]

    # ----------------------------------------
    # LEG TELEMETRY
    # ----------------------------------------

    if cmd == CMD_LEG_STATE:

        if len(payload) < 25:

            print("Invalid telemetry payload")

            return

        positions = struct.unpack(
            "<fff",
            payload[1:13]
        )

        torques = struct.unpack(
            "<fff",
            payload[13:25]
        )

        leg_number = arbitration_id - 0x100

        if leg_number < 0 or leg_number > 5:
            return

        latest_leg_telemetry[leg_number] = (
            positions,
            torques
        )

        display_telemetry_table()

# ============================================
# MONITOR MODE
# ============================================

def monitor_mode():

    print("\n===================================")
    print(" MONITOR MODE")
    print(" Press any key to exit")
    print("===================================\n")

    display_telemetry_table()

    #
    # Put terminal in raw mode
    #
    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    tty.setcbreak(fd)

    try:

        while True:

            #
            # Exit on key press
            #
            if key_pressed():

                sys.stdin.read(1)

                print("\nExiting monitor mode\n")

                break

            #
            # Receive CAN frames
            #
            msg = bus.recv(timeout=0.01)

            if msg is not None:

                process_rx_message(msg)

    finally:

        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

# ============================================
# MACRO MODE
# ============================================

def parse_command_sequence(command_text):

    if not command_text:
        return []

    return [token for token in re.split(r"[\s,]+", command_text.strip()) if token]


def parse_command_file(file_path):

    if not os.path.isfile(file_path):
        raise FileNotFoundError(file_path)

    target = None
    commands = []

    with open(file_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].strip()

            if not line:
                continue

            tokens = [token for token in re.split(r"[\s,]+", line) if token]

            if not tokens:
                continue

            first = tokens[0].lower()

            if first in ("target", "leg"):
                if len(tokens) < 2:
                    raise ValueError(f"Missing target value on line: {raw_line.strip()}")

                if tokens[1].lower() == "all":
                    target = "all"
                else:
                    leg_number = int(tokens[1])

                    if leg_number < 0 or leg_number > 5:
                        raise ValueError(f"Invalid leg target: {leg_number}")

                    target = leg_number

                continue

            if first == "all":
                target = "all"
                continue

            if first.isdigit() and len(tokens[0]) == 1:
                choice = tokens[0]
                params = tokens[1:]
                commands.append((choice, params))
                continue

            raise ValueError(f"Invalid command line in file: {raw_line.strip()}")

    return {"target": target, "commands": commands}


def build_payload(choice, params=None):

    if choice == "1":
        if params and len(params) >= 2:
            axis = int(params[0])
            position = float(params[1])
            return build_single_axis_move(axis, position)

        return build_single_axis_move()

    elif choice == "2":
        print("TODO: Linear move not implemented")
        return None

    elif choice == "3":
        print("TODO: Auto tune not implemented")
        return None

    elif choice == "4":
        print("TODO: Quadratic move not implemented")
        return None

    elif choice == "5":
        print("TODO: Rapid move not implemented")
        return None

    print(f"Invalid command choice: {choice}")

    return None


def run_macro(node_id):

    print("\n===================================")
    print(" MACRO MODE")
    print(" Enter a sequence of command ids separated by spaces or commas")
    print(" Available commands: 1=Single Axis Move, 2=Linear Move, 3=Auto Tune, 4=Quadratic Move, 5=Rapid Move")
    print("===================================\n")

    sequence_text = input("Command sequence: ").strip()

    command_choices = parse_command_sequence(sequence_text)

    if not command_choices:
        print("No commands entered for macro.")

        return

    for index, choice in enumerate(command_choices, start=1):

        print(f"\nMacro step {index}/{len(command_choices)}: command {choice}")

        payload = build_payload(choice)

        if payload is None:
            print("Macro aborted due to unsupported command.")

            return

        print("Sending CAN command...")

        send_isotp(node_id, payload)

        time.sleep(0.05)

    print("\nMacro complete.")


def run_command_file(file_path):

    print(f"\nReading command file: {file_path}")

    parsed = parse_command_file(file_path)

    target = parsed["target"]
    commands = parsed["commands"]

    if target is None:
        target_input = input("Enter target leg (0-5) or 'all': ").strip().lower()

        if target_input == "all":
            target = "all"
        else:
            leg_number = int(target_input)

            if leg_number < 0 or leg_number > 5:
                raise ValueError("Invalid target leg")

            target = leg_number

    if target == "all":
        node_ids = [0x100 + i for i in range(6)]
    else:
        node_ids = [0x100 + target]

    if not commands:
        print("No commands found in file.")
        return

    print("\nExecuting command file...")

    for index, (choice, params) in enumerate(commands, start=1):
        print(f"\nCommand file step {index}/{len(commands)}: {choice} {params}")

        payload = build_payload(choice, params)

        if payload is None:
            print("Command file aborted due to unsupported command.")
            return

        for node_id in node_ids:
            send_isotp(node_id, payload)
            time.sleep(0.01)

        time.sleep(0.05)

    print("\nCommand file execution complete.")

# ============================================
# COMMAND BUILDERS
# ============================================

def build_single_axis_move(axis=None, position=None):

    if axis is None:
        axis = int(input("Axis index (0-2): "))

    if position is None:
        position = float(
            input("Target position: ")
        )

    payload = bytearray()

    payload.append(CMD_SINGLE_AXIS_MOVE)

    payload.append(axis)

    payload.extend(
        struct.pack("<f", position)
    )

    print("\nCOMMAND:")
    print(
        f"CMD_SINGLE_AXIS_MOVE "
        f"(0x{CMD_SINGLE_AXIS_MOVE:X})"
    )

    print(f"Axis     : {axis}")

    print(f"Position : {position}")

    return payload

# ============================================
# MAIN MENU
# ============================================

def main():

    parser = argparse.ArgumentParser(
        description="Hexapod CAN Command Tool"
    )
    parser.add_argument(
        "command_file",
        nargs="?",
        help="Optional file containing a list of commands"
    )
    args = parser.parse_args()

    print("===================================")
    print(" HEXAPOD CAN COMMAND TOOL")
    print("===================================")

    if args.command_file:
        run_command_file(args.command_file)
        return

    leg_input = input(
        "Select leg/node number "
        "(0-5), all, m for monitor, or macro for batch commands: "
    ).strip().lower()

    # ----------------------------------------
    # MONITOR MODE
    # ----------------------------------------

    if leg_input == "m":

        monitor_mode()

        return

    if leg_input == "macro":

        leg_number = int(input("Leg number for macro (0-5): "))

        node_id = 0x100 + leg_number

        run_macro(node_id)

        return

    send_to_all = leg_input == "all"

    if not send_to_all:
        leg_number = int(leg_input)

        if leg_number < 0 or leg_number > 5:
            print("Invalid leg number")

            return

        node_ids = [0x100 + leg_number]
    else:
        node_ids = [0x100 + i for i in range(6)]

    print("\nAvailable Commands:")

    print("1 -> Single Axis Move")
    print("2 -> Linear Move")
    print("3 -> Auto Tune")
    print("4 -> Quadratic Move")
    print("5 -> Rapid Move")

    choice = input("\nSelect command: ")

    payload = None

    # ----------------------------------------
    # SINGLE AXIS MOVE
    # ----------------------------------------

    if choice == "1":

        payload = build_single_axis_move()

    # ----------------------------------------
    # TODO COMMANDS
    # ----------------------------------------

    elif choice == "2":

        print("TODO: Linear move not implemented")

        return

    elif choice == "3":

        print("TODO: Auto tune not implemented")

        return

    elif choice == "4":

        print("TODO: Quadratic move not implemented")

        return

    elif choice == "5":

        print("TODO: Rapid move not implemented")

        return

    else:

        print("Invalid selection")

        return

    # ----------------------------------------
    # SEND
    # ----------------------------------------

    print("\nSending CAN command...")

    for node_id in node_ids:
        send_isotp(node_id, payload)

    if send_to_all:
        print("\nDone: command sent to legs 0-5")
    else:
        print("\nDone")

# ============================================
# ENTRY
# ============================================

if __name__ == "__main__":

    main()
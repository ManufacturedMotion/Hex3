#!/usr/bin/env python3

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

bus = can.interface.Bus(
    channel=CAN_INTERFACE,
    bustype="socketcan"
)

rx_buffer = bytearray()
expected_size = 0
rx_active = False
expected_seq = 1
latest_leg_telemetry = {}

def encode_scaled_int16(value: float) -> bytes:
    scaled = int(round(value * 10.0))
    return struct.pack("<h", scaled)

def decode_scaled_int16(data: bytes) -> float:
    return struct.unpack("<h", data)[0] / 10.0

def key_pressed():

    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return dr != []

def display_telemetry_table():

    print("\033[2J\033[H", end="")

    print("LEG | X | Y | Z | TOE")
    print("----------------------")

    for leg in sorted(latest_leg_telemetry.keys()):

        pos, toe = latest_leg_telemetry[leg]

        print(
            f"{leg} | "
            f"{pos[0]:6.1f} "
            f"{pos[1]:6.1f} "
            f"{pos[2]:6.1f} "
            f"{toe:6.1f}"
        )

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

def handle_payload(arbitration_id, payload):

    if len(payload) == 0:
        return

    cmd = payload[0]

    # ----------------------------------------
    # LEG TELEMETRY
    # ----------------------------------------

    if cmd == CMD_LEG_STATE:

        if len(payload) < 9:
            print("Invalid telemetry payload")
            return

        positions = (
            decode_scaled_int16(payload[1:3]),
            decode_scaled_int16(payload[3:5]),
            decode_scaled_int16(payload[5:7]),
        )

        toe_compression = decode_scaled_int16(
            payload[7:9]
        )

        leg_number = arbitration_id - 0x180

        latest_leg_telemetry[leg_number] = (
            positions,
            toe_compression
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
# COMMAND BUILDERS
# ============================================

def build_single_axis_move(axis=None, position=None):

    if axis is None:
        axis = int(input("Axis index (0-2): "))

    if position is None:
        position = float(input("Target position: "))

    payload = bytearray()

    payload.append(CMD_SINGLE_AXIS_MOVE)

    payload.append(axis)

    payload.extend(
        encode_scaled_int16(position)
    )

    print("\nCOMMAND:")
    print(
        f"CMD_SINGLE_AXIS_MOVE "
        f"(0x{CMD_SINGLE_AXIS_MOVE:X})"
    )

    print(f"Axis     : {axis}")
    print(f"Position : {position}")

    return payload

def build_rapid_move(
    x=None,
    y=None,
    z=None
):

    if x is None:
        x = float(input("Target X: "))

    if y is None:
        y = float(input("Target Y: "))

    if z is None:
        z = float(input("Target Z: "))

    payload = bytearray()

    payload.append(CMD_RAPID_MOVE)

    payload.extend(encode_scaled_int16(x))
    payload.extend(encode_scaled_int16(y))
    payload.extend(encode_scaled_int16(z))

    print("\nCOMMAND:")
    print(f"CMD_RAPID_MOVE (0x{CMD_RAPID_MOVE:X})")
    print(f"X : {x}")
    print(f"Y : {y}")
    print(f"Z : {z}")
    return payload

def build_linear_move(
    x=None,
    y=None,
    z=None,
    speed=None,
    relative=None
):

    if x is None:
        x = float(input("Target X: "))

    if y is None:
        y = float(input("Target Y: "))

    if z is None:
        z = float(input("Target Z: "))

    if speed is None:
        speed = float(input("Speed: "))

    if relative is None:
        relative = (
            input("Relative move? (y/n): ")
            .strip()
            .lower()
            .startswith("y")
        )

    payload = bytearray()

    payload.append(CMD_LINEAR_MOVE)

    payload.extend(encode_scaled_int16(x))
    payload.extend(encode_scaled_int16(y))
    payload.extend(encode_scaled_int16(z))
    payload.extend(encode_scaled_int16(speed))

    payload.append(
        1 if relative else 0
    )

    print("\nCOMMAND:")
    print(f"CMD_LINEAR_MOVE (0x{CMD_LINEAR_MOVE:X})")
    print(f"X        : {x}")
    print(f"Y        : {y}")
    print(f"Z        : {z}")
    print(f"Speed    : {speed}")
    print(f"Relative : {relative}")
    return payload


# ============================================
# MAIN MENU
# ============================================

def main():

    print("===================================")
    print(" HEXAPOD CAN COMMAND TOOL")
    print("===================================")

    while True:

        print("\n-----------------------------------")
        print("Enter 'm' for monitor")
        print("Enter 'exit' to quit")
        print("-----------------------------------")

        leg_input = input(
            "\nSelect leg/node number: "
        ).strip().lower()

        # ----------------------------------------
        # EXIT
        # ----------------------------------------

        if leg_input in ("exit", "quit", "q"):

            print("\nExiting...")
            break

        # ----------------------------------------
        # MONITOR MODE
        # ----------------------------------------

        if leg_input == "m":

            monitor_mode()
            continue

        # ----------------------------------------
        # LEG SELECTION
        # ----------------------------------------

        try:
            leg_number = int(leg_input)
        except ValueError:
            print("Invalid leg number")
            continue

        node_id = 0x100 + leg_number

        print("\nAvailable Commands:")
        print("1 -> Single Axis Move")
        print("2 -> Linear Move")
        print("3 -> Auto Tune")
        print("4 -> Quadratic Move")
        print("5 -> Rapid Move")

        choice = input("\nSelect command: ").strip()

        payload = None

        # ----------------------------------------
        # SINGLE AXIS MOVE
        # ----------------------------------------

        if choice == "1":

            payload = build_single_axis_move()

        elif choice == "2":

            payload = build_linear_move()

        elif choice == "3":

            print("TODO: Auto tune not implemented")
            continue

        elif choice == "4":

            print("TODO: Quadratic move not implemented")
            continue

        elif choice == "5":

            payload = build_rapid_move()

        else:

            print("Invalid selection")
            continue

        # ----------------------------------------
        # SEND
        # ----------------------------------------

        print("\nSending CAN command...")

        send_isotp(node_id, payload)

        print("\nDone")

    bus.shutdown()

# ============================================
# ENTRY
# ============================================

if __name__ == "__main__":

    main()

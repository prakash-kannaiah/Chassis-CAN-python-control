import pygame
import sys
import can
import threading

# === Setup Functions ===

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick connected.")
        sys.exit()

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Joystick: {js.get_name()}")
    return js

def initialize_can_bus():
    return can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)

def send_can_message(bus, can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(msg, timeout=0.01)
    except can.CanError as e:
        print("CAN send error:", e)

def send_idle_commands(bus):
    idle_data_drive = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    idle_data_brake = [0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
    idle_data_steer = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    send_can_message(bus, 0x100, idle_data_drive)
    send_can_message(bus, 0x101, idle_data_brake)
    send_can_message(bus, 0x102, idle_data_steer)
    print("Sent idle messages to 0x130, 0x131, and 0x132")

# === CAN Command Generators ===

def generate_brake_data(brake_active):
    return [0x01, 0x00, 0x00, 0x02 if brake_active else 0x01, 0x00, 0x00, 0x00, 0x00]

def generate_drive_data(hat_val, brake_active):
    if not brake_active:
        return [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    if hat_val == (0, 1):  # Forward
        return [0x11, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    elif hat_val == (0, -1):  # Reverse
        return [0x31, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    else:
        return [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

def generate_steer_data(axis_val, brake_active):
    if not brake_active:
        return [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    steer_value = int(axis_val * 500)
    steer_value = max(-500, min(500, steer_value))
    steer_bytes = steer_value.to_bytes(2, byteorder='little', signed=True)
    return [0x01, steer_bytes[0], steer_bytes[1], 0x00, 0x00, 0x00, 0x00, 0x00]

# === Main Loop ===

def main_loop(joystick, bus, stop_event):
    prev_brake = None
    prev_hat = None
    prev_axis = None
    clock = pygame.time.Clock()

    while not stop_event.is_set():
        pygame.event.pump()
        brake_active = joystick.get_button(4)
        hat_val = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
        axis_val = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0.0

        brake_data = generate_brake_data(brake_active)
        drive_data = generate_drive_data(hat_val, brake_active)
        steer_data = generate_steer_data(axis_val, brake_active)

        send_can_message(bus, 0x131, brake_data)
        send_can_message(bus, 0x130, drive_data)
        send_can_message(bus, 0x132, steer_data)

        if (
            brake_active != prev_brake or
            hat_val != prev_hat or
            abs(axis_val - (prev_axis if prev_axis is not None else 0.0)) > 0.05
        ):
            print(f"\nInput Changed:")
            print(f"  Brake: {'ON' if brake_active else 'OFF'}")
            print(f"  Hat: {hat_val}")
            print(f"  Axis2 (Steer): {axis_val:.2f}")
            print(f"  → Sent 0x131: {[hex(b) for b in brake_data]}")
            print(f"  → Sent 0x130: {[hex(b) for b in drive_data]}")
            print(f"  → Sent 0x132: {[hex(b) for b in steer_data]}")

            prev_brake = brake_active
            prev_hat = hat_val
            prev_axis = axis_val

        clock.tick(50)  # 20 ms

# === Entry Point ===

if __name__ == "__main__":
    stop_event = threading.Event()
    try:
        js = initialize_joystick()
        can_bus = initialize_can_bus()
        main_loop(js, can_bus, stop_event)
    except KeyboardInterrupt:
        print("\n CTRL+C received. Sending idle messages and exiting safely...")
        stop_event.set()
    finally:
        send_idle_commands(can_bus)
        pygame.quit()
        try:
            can_bus.shutdown()
        except Exception as e:
            print("CAN shutdown warning:", e)
        print("Program terminated cleanly.")

import can
import time
import struct
import json
from HUSKYLENS.huskylib import HuskyLensLibrary

hl = HuskyLensLibrary("SERIAL", "/dev/ttyUSB0", 9600)

mode = 'MANUAL'  # Default mode

# def printObjectNicely(obj):
    # count=1
    # if(type(obj)==list):
    #     for i in obj:
    #         print("\t "+ ("BLOCK_" if i.type=="BLOCK" else "ARROW_")+str(count)+" : "+ json.dumps(i.__dict__))
    #         count+=1
    # else:
    #     print("\t "+ ("BLOCK_" if obj.type=="BLOCK" else "ARROW_")+str(count)+" : "+ json.dumps(obj.__dict__))

class ZLAC8015:
    def __init__(self, bus):
        self.bus = bus
        self.tx_id = 0x601  # Fixed CAN ID for ZLAC8015D
        self.rx_id = 0x581

    def send(self, data):
        msg = can.Message(arbitration_id=self.tx_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def set_speed(self, axis, speed):
        index = 0x01 + axis
        data = [0x23, 0xFF, 0x60, index] + list(struct.pack('<i', speed))
        self.send(data)

    def request_encoder(self, axis):
        index = 0x01 + axis
        self.send([0x43, 0x64, 0x60, index] + [0x00]*4)

    def request_vbus(self):
        self.send([0x4B, 0x35, 0x20, 0x00] + [0x00]*4)

    def enable(self):
        for cmd in [0x00, 0x06, 0x07, 0x0F]:
            self.send([0x2B, 0x40, 0x60, 0x00, cmd, 0x00, 0x00, 0x00])
            time.sleep(0.002)

    def disable(self):
        self.send([0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])

# def parse_encoder(msg):
#     return struct.unpack('<i', msg.data[4:8])[0]

# def parse_vbus(msg):
#     return struct.unpack('<H', msg.data[4:6])[0] * 0.01

# def diff_drive_kinematics(vx, wz, wheel_base):
#     v_left = vx - (wz * wheel_base / 2)
#     v_right = vx + (wz * wheel_base / 2)
#     return v_left, -v_right

# def ms_to_rpm(v, wheel_radius):
#     return int((v / (2 * 3.1416 * wheel_radius)) * 60)

# def main():
#     bus = can.interface.Bus(channel='can0', interface='socketcan')
#     driver = ZLAC8015(bus)
#     # driver.enable()
#     driver_enabled = None  # or False if you want to default to disabled

#     enc = [0, 0]
#     vbus = 0.0

#     prev_enc = [0, 0]
#     prev_time = time.monotonic()
#     actual_rpm = [0.0, 0.0]

#     # Robot parameters
#     wheel_base = 0.3    # meters
#     wheel_radius = 0.107 # meters
#     speed = 0.5         # m/s
#     turn = 2.0          # rad/s
#     vx = 0.0
#     wz = 0.0

#     encoder_cpr = 4096  # Ganti sesuai encoder Anda

#     while True:
#         # --- HuskyLens Tracking Logic ---
#         obj = hl.requestAll()
#         target_block = None
#         if isinstance(obj, list):
#             for i in obj:
#                 if getattr(i, "type", "") == "BLOCK" and getattr(i, "ID", -1) == 1 and getattr(i, "learned", False):
#                     target_block = i
#                     break
#         elif getattr(obj, "type", "") == "BLOCK" and getattr(obj, "ID", -1) == 1 and getattr(obj, "learned", False):
#             target_block = obj

#         if target_block:
#             center_x = target_block.x + target_block.width // 2
#             if center_x < 100:
#                 vx = 0.2
#                 wz = 1.0
#             elif center_x > 220:
#                 vx = 0.2
#                 wz = -1.0
#             else:
#                 vx = 0.4
#                 wz = 0.0
#         else:
#             vx = 0.0
#             wz = 0.0


#         v_left, v_right = diff_drive_kinematics(vx, wz, wheel_base)

#         should_enable = not (v_left == 0.0 and v_right == 0.0)
#         if should_enable != driver_enabled:
#             if should_enable:
#                 driver.enable()
#             else:
#                 driver.disable()
#             driver_enabled = should_enable

#         rpm_left = ms_to_rpm(v_left, wheel_radius)
#         rpm_right = ms_to_rpm(v_right, wheel_radius)
#         vel = [rpm_left, rpm_right]

#         for axis in (0, 1):
#             driver.set_speed(axis, vel[axis])
#             driver.request_encoder(axis)

#         driver.request_vbus()

#         # Baca CAN message
#         end = time.monotonic() + 0.01
#         while time.monotonic() < end:
#             msg = bus.recv(timeout=0.001)
#             if msg:
#                 if msg.arbitration_id == 0x581:
#                     if list(msg.data[:4]) == [0x43, 0x64, 0x60, 0x01]:
#                         enc[0] = parse_encoder(msg)
#                     elif list(msg.data[:4]) == [0x43, 0x64, 0x60, 0x02]:
#                         enc[1] = parse_encoder(msg)
#                     elif list(msg.data[:4]) == [0x4B, 0x35, 0x20, 0x00]:
#                         vbus = parse_vbus(msg)

#         # Hitung RPM aktual dari encoder
#         curr_time = time.monotonic()
#         dt = curr_time - prev_time
#         if dt > 0:
#             for i in (0, 1):
#                 delta = enc[i] - prev_enc[i]
#                 actual_rpm[i] = (delta / encoder_cpr) / dt * 60
#                 prev_enc[i] = enc[i]
#             prev_time = curr_time

#         # Print monitoring
#         print(f"ENC0: {enc[0]} | ENC1: {enc[1]} | VBUS: {vbus:.2f}V")
#         print(f"RPM L: {rpm_left} | RPM R: {rpm_right}")
#         print(f"ACT RPM L: {actual_rpm[0]:.2f} | ACT RPM R: {actual_rpm[1]:.2f}")
#         if target_block:
#             print(f"Tracking ID 1: x={target_block.x}, y={target_block.y}, w={target_block.width}, h={target_block.height}, center_x={center_x}")
#         else:
#             print("ID 1 not found or not learned")
#         print("-" * 60)
#         time.sleep(0.1)

# if __name__ == '__main__':
#     main()
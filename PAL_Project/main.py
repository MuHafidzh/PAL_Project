# Python
import time
import threading
import queue
import RPi.GPIO as GPIO
import minimalmodbus
import can
import struct
from huskylib import HuskyLensLibrary
from ZLAC8015 import ZLAC8015

# --- GPIO Setup ---
PIN_RUNSTOP = 12
PIN_VBUS = 5
PIN_DECLARE = [6, 16]
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_RUNSTOP, GPIO.IN)
GPIO.setup(PIN_VBUS, GPIO.IN)
for pin in PIN_DECLARE:
    GPIO.setup(pin, GPIO.IN)

# --- Modbus Setup ---
PORT = '/dev/ttyS0'
SLAVE_ID = 1
laser = minimalmodbus.Instrument(PORT, SLAVE_ID)
laser.serial.baudrate = 115200
laser.serial.bytesize = 8
laser.serial.parity = minimalmodbus.serial.PARITY_NONE
laser.serial.stopbits = 1
laser.serial.timeout = 1
laser.mode = minimalmodbus.MODE_RTU

# --- HuskyLens Setup ---
hl = HuskyLensLibrary("SERIAL", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0", 9600)

def laser_thread_fn(laser_queue, stop_event):
    while not stop_event.is_set():
        for _ in range(3):  # retry 3x
            try:
                values = laser.read_registers(registeraddress=0, number_of_registers=5, functioncode=3)
                break
            except Exception:
                values = [9999]*5
        try:
            laser_queue.put_nowait(values)
        except queue.Full:
            pass
        time.sleep(0.05)

def husky_thread_fn(husky_queue, stop_event):
    while not stop_event.is_set():
        try:
            obj = hl.requestAll()
        except Exception as e:
            obj = None
        try:
            husky_queue.put_nowait(obj)
        except queue.Full:
            pass
        time.sleep(0.05)  # 20Hz

def parse_encoder(msg):
    return struct.unpack('<i', msg.data[4:8])[0]

def parse_vbus(msg):
    return struct.unpack('<H', msg.data[4:6])[0] * 0.01

def diff_drive_kinematics(vx, wz, wheel_base):
    v_left = vx - (wz * wheel_base / 2)
    v_right = vx + (wz * wheel_base / 2)
    return v_left, -v_right

def ms_to_rpm(v, wheel_radius):
    return int((v / (2 * 3.1416 * wheel_radius)) * 60)

def main_loop():
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    driver = ZLAC8015(bus)
    driver_enabled = None
    driver.enable()

    enc = [0, 0]
    vbus = 0.0

    prev_enc = [0, 0]
    prev_time = time.monotonic()
    actual_rpm = [0.0, 0.0]

    wheel_base = 0.3
    wheel_radius = 0.107
    vx = 0.0
    wz = 0.0

    encoder_cpr = 4096
    Kp = 0.04
    frame_center = 160

    run_state = False
    last_gpio = GPIO.input(PIN_RUNSTOP)
    last_vbus_gpio = GPIO.input(PIN_VBUS)
    
    # Initial display state on HuskyLens
    hl.clearText()
    hl.customText("STOP", 150, 25)  # Display STOP at top of screen
    last_display_state = False  # Track last displayed state (False = STOP, True = RUN)

    # --- Threading setup ---
    laser_queue = queue.Queue(maxsize=1)
    husky_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    t_laser = threading.Thread(target=laser_thread_fn, args=(laser_queue, stop_event), daemon=True)
    t_husky = threading.Thread(target=husky_thread_fn, args=(husky_queue, stop_event), daemon=True)
    t_laser.start()
    t_husky.start()

    try:
        probe_dist = [9999]*5
        target_block = None
        target_lost_counter = 0  # Counter for consecutive frames without target
        last_target_state = False  # Track if we had a target in previous frame
        
        # Search state tracking
        in_search_mode = False
        search_completed = False  # Flag to track if search has completed without finding target
        target_lost_time = None
        search_start_time = None
        search_direction = 1  # 1 or -1 for left/right rotation
        search_direction_change_time = None
        
        while True:
            loop_start = time.monotonic()

            # --- Latching logic for RUN/STOP ---
            gpio_now = GPIO.input(PIN_RUNSTOP)
            if gpio_now and not last_gpio:
                run_state = not run_state
                print(f"[DEBUG] RUN state toggled to: {run_state}")
                
                # Update HuskyLens display when state changes
                if run_state != last_display_state:
                    # hl.clearText()
                    if run_state:
                        hl.customText("RUN", 150, 25)  # Display RUN at top of screen
                        # Reset search state when toggling to RUN
                        search_completed = False
                    else:
                        hl.customText("STOP", 150, 25)  # Display STOP at top of screen
                        # Reset search state when toggling to STOP
                        in_search_mode = False
                        search_completed = False
                        target_lost_time = None
                        search_start_time = None
                    last_display_state = run_state
                    
            last_gpio = gpio_now

                        # --- Check VBUS GPIO for display update ---
            vbus_gpio_now = GPIO.input(PIN_VBUS)
            if vbus_gpio_now and not last_vbus_gpio:
                # PIN_VBUS just went high, update HuskyLens display with VBUS value
                print(f"[DEBUG] Displaying VBUS: {vbus:.2f}V")
                # hl.clearText()
                # Show RUN/STOP status at top
                # status_text = "RUN" if run_state else "STOP"
                # hl.customText(status_text, 150, 25)
                # Show VBUS value below
                hl.customText(f"V: {vbus:.2f}V", 10, 25)
                
                # Update last display state to match current run state
                last_display_state = run_state
                
            last_vbus_gpio = vbus_gpio_now

            # Ambil data terbaru dari queue (non-blocking)
            try:
                probe_dist = laser_queue.get_nowait()
                # print(f"[DEBUG] Laser probe readings: {probe_dist}")
            except queue.Empty:
                pass
            
            found_target_this_frame = False
            try:
                obj = husky_queue.get_nowait()
                
                # Check if we have valid target in this frame
                if isinstance(obj, list):
                    for i in obj:
                        if getattr(i, "type", "") == "BLOCK" and getattr(i, "ID", -1) == 1 and getattr(i, "learned", False):
                            target_block = i
                            found_target_this_frame = True
                            break
                elif getattr(obj, "type", "") == "BLOCK" and getattr(obj, "ID", -1) == 1 and getattr(obj, "learned", False):
                    target_block = obj
                    found_target_this_frame = True
            except queue.Empty:
                # No new data, but don't reset target yet
                pass
            
            # Update target tracking state
            if found_target_this_frame:
                target_lost_counter = 0
                target_lost_time = None  # Reset lost timer
                search_completed = False  # Reset search completed flag when target is found
                
                # Exit search mode if we were in it
                if in_search_mode:
                    print("[DEBUG] Target found during search mode. Resuming tracking.")
                    in_search_mode = False
                    search_start_time = None
                
                if not last_target_state:
                    print("[DEBUG] Target acquired.")
                    last_target_state = True
            else:
                target_lost_counter += 1
                
                # Initialize lost timer if this is the first frame of being lost
                if target_lost_counter == 1:
                    target_lost_time = time.monotonic()
                
            # Only reset target after multiple consecutive misses
            # (5 frames at 20Hz = 250ms)
            if target_lost_counter >= 5:
                if last_target_state:
                    print("[DEBUG] Target lost.")
                    last_target_state = False
                target_block = None

            if run_state:
                current_time = time.monotonic()
                
                # Check if we've lost the target for 10 seconds and should start search mode
                # Only enter search mode if we haven't completed a search yet
                if (not in_search_mode and not search_completed and 
                    not target_block and target_lost_time and 
                    (current_time - target_lost_time) >= 10.0):
                    print("[DEBUG] Target lost for 10 seconds. Entering search mode.")
                    in_search_mode = True
                    search_start_time = current_time
                    search_direction = 1  # Start rotating in one direction
                    search_direction_change_time = current_time + 3.0  # Change direction every 2 seconds
                
                # Check if search time is expired (10 seconds)
                if in_search_mode and (current_time - search_start_time) >= 15.0:
                    print("[DEBUG] Search mode completed without finding target. Stopping.")
                    in_search_mode = False
                    search_completed = True  # Mark that we've completed a search
                    search_start_time = None
                
                # Check if it's time to change search direction
                if in_search_mode and current_time >= search_direction_change_time:
                    search_direction *= -1  # Reverse direction
                    search_direction_change_time = current_time + 3.0  # Change again in 2 seconds
                    print(f"[DEBUG] Search changing direction: {search_direction}")
                
                # --- HuskyLens Tracking Logic with Proportional Control ---
                if target_block:
                    center_x = target_block.x + target_block.width // 2
                    error = frame_center - center_x

                    if any(probe_dist[i] <= 50 for i in [1, 2, 3]):
                        print("[SAFETY] Obstacle detected! Stopping robot.")
                        vx = 0.0
                        wz = 0.0
                    else:
                        # if probe_dist[2] > 50 and probe_dist[2] < 100:
                        #     vx = 0.4
                        # elif probe_dist[2] >= 100 and probe_dist[2] < 150:
                        #     vx = 0.8
                        # elif probe_dist[2] >= 150 and probe_dist[2] < 200:
                        #     vx = 1.0
                        # else:
                        #     vx = 1.2
                        if any(probe_dist[i] > 50 and probe_dist[i] < 100 for i in [1, 2, 3]):
                            vx = 0.4
                        elif any(probe_dist[i] >= 100 and probe_dist[i] < 150 for i in [1, 2, 3]):
                            vx = 0.8
                        elif any(probe_dist[i] >= 150 and probe_dist[i] < 200 for i in [1, 2, 3]):
                            vx = 1.0
                        else:
                            vx = 1.2

                        # vx = 0.4
                        wz = Kp * error

                        print(f"[DEBUG] vx: {vx:.2f} m/s, wz: {wz:.2f} rad/s, error: {error}")
                elif in_search_mode:
                    # In search mode, rotate in place while looking for target
                    vx = 0.0
                    wz = 1.0 * search_direction  # Rotate at moderate speed
                else:
                    # Either no target or search completed without finding target
                    vx = 0.0
                    wz = 0.0
            else:
                vx = 0.0
                wz = 0.0
                target_block = None
                target_lost_counter = 0
                last_target_state = False
                in_search_mode = False
                search_completed = False
                target_lost_time = None
                search_start_time = None

            v_left, v_right = diff_drive_kinematics(vx, wz, wheel_base)
            # Print debug info only when in search mode
            if in_search_mode:
                print(f"[DEBUG] SEARCHING: v_left: {v_left:.2f} m/s, v_right: {v_right:.2f} m/s, wz: {wz:.2f} rad/s")
            
            should_enable = not (v_left == 0.0 and v_right == 0.0)
            if should_enable != driver_enabled:
                if should_enable:
                    driver.enable()
                else:
                    driver.disable()
                driver_enabled = should_enable

            rpm_left = ms_to_rpm(v_left, wheel_radius)
            rpm_right = ms_to_rpm(v_right, wheel_radius)
            vel = [rpm_left, rpm_right]

            for axis in (0, 1):
                driver.set_speed(axis, vel[axis])
                driver.request_encoder(axis)

            driver.request_vbus()

            # Baca CAN message
            end = time.monotonic() + 0.01
            while time.monotonic() < end:
                msg = bus.recv(timeout=0.001)
                if msg:
                    if msg.arbitration_id == 0x581:
                        if list(msg.data[:4]) == [0x43, 0x64, 0x60, 0x01]:
                            enc[0] = parse_encoder(msg)
                        elif list(msg.data[:4]) == [0x43, 0x64, 0x60, 0x02]:
                            enc[1] = parse_encoder(msg)
                        elif list(msg.data[:4]) == [0x4B, 0x35, 0x20, 0x00]:
                            vbus = parse_vbus(msg)

            # Hitung RPM aktual dari encoder
            curr_time = time.monotonic()
            dt = curr_time - prev_time
            if dt > 0:
                for i in (0, 1):
                    delta = enc[i] - prev_enc[i]
                    actual_rpm[i] = (delta / encoder_cpr) / dt * 60
                    prev_enc[i] = enc[i]
                prev_time = curr_time

            # --- 50Hz timer ---
            loop_time = time.monotonic() - loop_start
            sleep_time = max(0, 0.02 - loop_time)
            time.sleep(sleep_time)
    finally:
        print("Shutting down CAN bus and cleaning up GPIO...")
        # Clear HuskyLens display before exit
        hl.clearText()
        stop_event.set()
        t_laser.join(timeout=1)
        t_husky.join(timeout=1)
        bus.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        print("Program terminated by user.")
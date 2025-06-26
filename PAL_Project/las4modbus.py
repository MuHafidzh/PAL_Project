import minimalmodbus
import time

# DEBUG
minimalmodbus._print_out = print

PORT = '/dev/ttyS0'
SLAVE_ID = 1

instr = minimalmodbus.Instrument(PORT, SLAVE_ID)
instr.serial.baudrate = 115200
instr.serial.bytesize = 8
instr.serial.parity = minimalmodbus.serial.PARITY_NONE
instr.serial.stopbits = 1
instr.serial.timeout = 1
instr.mode = minimalmodbus.MODE_RTU

# Bersihkan buffer dulu
instr.serial.reset_input_buffer()
instr.serial.reset_output_buffer()

try:
    while True:
        start = time.time()
        values = instr.read_registers(registeraddress=0, number_of_registers=5, functioncode=3)
        for i, val in enumerate(values):
            print(f"Probe {i+1}: {val} cm")
        print('-' * 30)
        elapsed = time.time() - start
        time.sleep(max(0, 0.02 - elapsed))  # 50 Hz
except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"Gagal membaca sensor: {e}")
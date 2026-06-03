import serial
import struct
import time

PORT  = "COM6"        # ← change to your port, e.g. "/dev/ttyUSB1" on Linux
BAUD  = 115200

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(0.1)

def send_matrix(cmd_byte, matrix_flat):
    """Send CMD + 16 signed bytes."""
    payload = bytes([cmd_byte]) + bytes([v & 0xFF for v in matrix_flat])
    ser.write(payload)
    time.sleep(0.05)

def compute():
    """Send compute command, wait for 0xAA ack."""
    ser.write(bytes([0x03]))
    ack = ser.read(1)
    if ack == b'\xAA':
        print("Compute done! Got ACK 0xAA")
    else:
        print(f"Unexpected response: {ack.hex()}")

def read_element(idx):
    """Read C[idx] as a signed 32-bit integer."""
    ser.write(bytes([0x04, idx]))
    raw = ser.read(4)
    if len(raw) < 4:
        print(f"Timeout reading element {idx}")
        return None
    return struct.unpack('<i', raw)[0]   # little-endian signed 32-bit

def read_all():
    """Read and print the full 4x4 result matrix."""
    print("\nResult matrix C = A × B:")
    for row in range(4):
        vals = [read_element(row * 4 + col) for col in range(4)]
        print("  [" + "  ".join(f"{v:6d}" for v in vals) + " ]")

# ── Test 1: Identity × Identity = Identity ─────────────────────
print("=== Test 1: I × I ===")
I = [1,0,0,0,
     0,1,0,0,
     0,0,1,0,
     0,0,0,1]
send_matrix(0x01, I)   # write A
send_matrix(0x02, I)   # write B
compute()
read_all()

# ── Test 2: Simple known values ─────────────────────────────────
print("\n=== Test 2: known values ===")
A = [1,2,3,4,
     5,6,7,8,
     1,0,0,0,
     0,1,0,0]
B = [1,0,0,0,
     0,1,0,0,
     0,0,1,0,
     0,0,0,1]
send_matrix(0x01, A)
send_matrix(0x02, B)
compute()
read_all()
# Expected: C == A (multiplying by identity)

ser.close()
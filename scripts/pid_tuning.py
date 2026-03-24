import serial
import time

# Config
PORT = 'COM3'
BAUDRATE = 115200
MONITOR_SECONDS = 3.0

# Connect
ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
time.sleep(3)  # wait for ESP32 to reset
print("Connected to ESP32\n")

# Main Loop
while True:
    try:
        # Read Motor 1 PID
        vals1 = input("Motor1 PID (kp ki kd): ").split()
        vals2 = input("Motor2 PID (kp ki kd): ").split()

        if len(vals1) != 3 or len(vals2) != 3:
            print("Please enter exactly 3 numbers for each motor.")
            continue

        # Convert to floats
        Kp1, Ki1, Kd1 = map(float, vals1)
        Kp2, Ki2, Kd2 = map(float, vals2)

        # Build command with prefix P, for PID
        command = f"P,{Kp1},{Ki1},{Kd1},{Kp2},{Ki2},{Kd2}\n"

        # Send over serial
        ser.write(command.encode())
        print(f">> Sent: {command.strip()}")
        print(f"--- Monitoring for {MONITOR_SECONDS:.0f}s (Ctrl+C to stop) ---")

        # Read incoming serial for a short period
        deadline = time.time() + MONITOR_SECONDS
        while time.time() < deadline:
            line = ser.readline()
            if line:
                print(line.decode('utf-8', errors='replace').rstrip())

        print("---\n")

    except ValueError:
        print("Invalid input, please enter numbers only.")

    except KeyboardInterrupt:
        print("\nExiting.")
        ser.close()
        break
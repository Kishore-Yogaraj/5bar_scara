import serial
import time

#Config
PORT            = 'COM3'
BAUDRATE        = 115200
MONITOR_SECONDS = 3.0

# Connect
ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
time.sleep(3)        # wait for ESP32 to finish resetting
print("Connected to ESP32\n")

# Main Loop
while True:
    try:
        angle1 = float(input("Motor 1 angle (deg): "))
        angle2 = float(input("Motor 2 angle (deg): "))
       

        command = f"A,{angle1},{angle2}\n"
        ser.write(command.encode())
        ser.flush()
        print(f"  >> Sent: {command.strip()}")
        print(f"--- Monitoring for {MONITOR_SECONDS:.0f}s (Ctrl+C to stop) ---")

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

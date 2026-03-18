import serial
import time

#Config 
PORT     = 'COM3' 
BAUDRATE = 115200

# \Connect
ser = serial.Serial(PORT, BAUDRATE)
time.sleep(3)        # wait for ESP32 to finish resetting
print("Connected to ESP32")

#Main Loop
while True:
    try:
        angle1 = float(input("Enter Motor 1 angle (degrees): "))
        angle2 = float(input("Enter Motor 2 angle (degrees): "))

        command = f"{angle1},{angle2}\n"
        ser.write(command.encode())
        ser.flush()
        print(f"Sent: {command.strip()}")

    except ValueError:
        print("Invalid input, please enter numbers only.")

    except KeyboardInterrupt:
        print("\nExiting.")
        ser.close()
        break
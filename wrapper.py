import sys
import serial
import subprocess
import time

# =======================
# Serial connection choice
# =======================
USE_USB = False  # True = USB (ttyACM0) | False = UART GPIO (ttyAMA0)

# def send_message(teensy, message):
#     """ Sends a message to the Teensy via Serial """
#     teensy.write((message + "\n").encode('utf-8'))
#     print(f"Sent to Teensy: {message}")

def main():
    if USE_USB:
        port = "/dev/ttyACM0"  # USB connection
        print("[MODE] USB selected")
    else:
        port = "/dev/serial0"  # UART GPIO connection
        print("[MODE] UART GPIO selected")

    baudrate = 115200

    # Try to open the serial port
    try:
        teensy = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to Teensy on {port}")
    except Exception as e:
        print(f"Error connecting to port {port}: {e}")
        sys.exit(1)

    print("Waiting for the 'RUN_SCRIPT' command from Teensy...")

    # Infinite loop: wait for "RUN_SCRIPT" command to launch the script
    while True:
        if teensy.in_waiting > 0:
            command = teensy.readline().decode().strip()
            if command == "RUN_SCRIPT":
                print("Teensy requested to execute test_camera_from_tag_mira220.py")
                
                # Launch the detection script in a new subprocess
                process = subprocess.Popen(
                    ["python3", "-u", "/home/Paul2901/Downloads/apriltag-main/test_find_tags/test_camera_from_tag_mira220.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    bufsize=1,
                    text=True
                )
                             
                for line in iter(process.stdout.readline, ''):
                    line = line.strip()
                    if line:
                        print(f"{line}", flush=True)
                        teensy.write((line + "\n").encode('utf-8'))

if __name__ == "__main__":
    main()

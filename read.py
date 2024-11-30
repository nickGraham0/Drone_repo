import serial

def read_from_com():
    # Replace with your COM port and baud rate
    COM_PORT = 'COM7'  # Example: 'COM3' on Windows or '/dev/ttyUSB0' on Linux/Mac
    BAUD_RATE = 57600  # Default baud rate for SiK radios

    try:
        # Open the serial port
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")

            print("Waiting for data...")
            while True:
                # Read incoming data
                response = ser.read_all().decode('utf-8')
                if response:
                    print(f"Received: {response.strip()}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")

if __name__ == "__main__":
    read_from_com()

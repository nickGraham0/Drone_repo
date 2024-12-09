import serial

def write_to_com():
    # Replace with your COM port and baud rate
    COM_PORT = 'COM6'  # Example: 'COM3' on Windows or '/dev/ttyUSB0' on Linux/Mac
    BAUD_RATE = 57600  # Default baud rate for SiK radios

    try:
        # Open the serial port
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")
            
            # Message to send
            message = "Hello SiK Radio\n"
            ser.write(message.encode('utf-8'))
            print(f"Sent: {message.strip()}")

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    write_to_com()

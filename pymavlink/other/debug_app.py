import asyncio
from PIL import Image
import io

# Function to send a 32x32 image over a socket
async def send_image(image_path):
    # Open the image and convert it to bytes
    img = Image.open(image_path)

    # Convert image to a bytes buffer
    with io.BytesIO() as img_buffer:
        img.save(img_buffer, format='PNG')  # Save the image to the buffer in PNG format
        img_bytes = img_buffer.getvalue()   # Get the byte content

    # Get the size of the image data in bytes
    img_size = len(img_bytes)

    # Connect to the server and send the image
    reader, writer = await asyncio.open_connection('localhost', 15555)

    print("Established connection")
    writer.write(f"IR \n".encode('utf8'))  # Send the size padded to 16 bytes

    # Send the image size first
    writer.write(f"{img_size:<16}".encode('utf8'))  # Send the size padded to 16 bytes

    print(len(img_bytes))

    # Send the image data itself
    writer.write(img_bytes)
    #writer.write('\0'.encode('utf8'))
    await writer.drain()

    # Close the connection
    writer.close()
    await writer.wait_closed()

# Run the client
async def main():
    # Path to the 32x32 image
    image_path = 'image.jpg'
    await send_image(image_path)

asyncio.run(main())

import asyncio
from PIL import Image
import io 

# Asynchronous function to handle a connected client
async def handle_client(reader, writer):
    
    request = None  # Initialize the request variable

    # Read the client's message (up to 255 bytes) and decode from utf8
    request = (await reader.read(255)).decode('utf8').strip()
    
    request = request.strip()

    command = str(request)

    print("found command: " + str(command))
    if command == 'wp':                 # Init Waypoint Path
        request = (await reader.read(4096)).decode('utf8')
        print("Waypoint")

        tokens = request.split()

        params = [float(x) for x in tokens]
        request = params
        print(request)
        
    if command == 'up':                  
        print("up")                 
    if command == 'down':                
        print("down")      
    if command == 'left':                  
        print("left")                 
    if command == 'right':                 
        print("right")      
                      
# Asynchronous function to start the server
async def run_server():
    print("Running Server")
    server = await asyncio.start_server(handle_client, 'localhost', 15555)
    async with server:
        await server.serve_forever()

# Entry point to run the asyncio server
asyncio.run(run_server())

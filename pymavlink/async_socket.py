import asyncio
from PIL import Image
import io 

# Asynchronous function to handle a connected client
async def handle_client(reader, writer):
    """
    This function handles a single client connection.
    It reads incoming requests from the client, evaluates the request,
    and sends the response back to the client.
    
    The connection is closed when the client sends 'quit'.
    
    Parameters:
    reader -- asyncio StreamReader to read data from the client.
    writer -- asyncio StreamWriter to send data to the client.
    """

    img_count = 0
    
    request = None  # Initialize the request variable
    while request != 'quit':  # Keep processing until 'quit' is received
        # Read the client's message (up to 255 bytes) and decode from utf8
        request = (await reader.read(2)).decode('utf8').strip()
        
        request = request.strip()

        #print(request) 

        #tokens2 = request.split()
        
        #if not tokens2:
            #continue

        command = str(request)

        print("found command: " + str(command))
        if command == 'wp':                 # Init Waypoint Path
            request = (await reader.read(4096)).decode('utf8')
            print("Waypoint")

            tokens = request.split()
            
            if not tokens:
                continue

            params = [float(x) for x in tokens]
            request = params
            print(request)
        if command == 'hm':                 # Command Home 
            print("Something")                 
        if command == 'mn':                 # Command Manual Control
            print("Something")      
                        
# Asynchronous function to start the server
async def run_server():
    """
    This function starts the asyncio server and waits for client connections.
    It uses the `handle_client` function to handle individual clients.
    
    The server will keep running forever.
    """
    
    # Start the server on localhost:15555, using the handle_client function to process clients
    server = await asyncio.start_server(handle_client, 'localhost', 15555)
    
    # Run the server indefinitely, accepting and handling clients
    async with server:
        await server.serve_forever()

# Entry point to run the asyncio server
asyncio.run(run_server())

import asyncio
import websockets
import json

class Communicator:
    def __init__(self, auto_reconnect=True, disconnect_callback=None):
        self.uri = "ws://localhost:8765"
        self.websocket = None
        self.auto_reconnect = auto_reconnect
        self.disconnect_callback = disconnect_callback
        self.running = True

    async def connect(self):
        while self.running:
            try:
                self.websocket = await websockets.connect(self.uri)
                print("Connected successfully!")
                return
            except Exception as e:
                print(f"Connection failed with error: {e}.")
                if not self.auto_reconnect:
                    break
                print("Retrying in 5 seconds...")
                await asyncio.sleep(5)

    async def listen(self):
        while self.running:
            try:
                data = await self.websocket.recv()
                data = json.loads(data)
                print(f"Received data: {data}")
            except websockets.ConnectionClosed as e:
                print(f"Connection closed with error: {e}.")
                if self.disconnect_callback:
                    self.disconnect_callback()
                if self.auto_reconnect:
                    print("Reconnecting...")
                    await self.connect()
                else:
                    break

    async def close(self):
        self.running = False
        if self.websocket:
            await self.websocket.close()
            print("Connection closed manually.")


if __name__ == "__main__":
    def handle_disconnect():
        print("Disconnected from the server. Notifying the GUI...")

    async def main():
        com = Communicator(auto_reconnect=True, disconnect_callback=handle_disconnect)
        await com.connect()
        # listen_task = asyncio.create_task(com.listen())
        asyncio.create_task(com.listen())

        # Wait for 1 second before closing the connection
        await asyncio.sleep(1)
        await com.close()

        # await listen_task

    asyncio.run(main())

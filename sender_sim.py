import asyncio
import websockets
import json

import random

async def send_robot_joints(websocket, path):
    try:
        while True:
            # joints = [-1.40780272, -1.06478243, 0.37771895, 2.16583817, 0.16887252, -4.00106307, -4.27609907]  # Example list of floats representing robot joints
            joints = [random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1]  # Example list of floats representing robot joints
            await websocket.send(json.dumps(joints))
            print(f"Sent joints: {joints}")
            await asyncio.sleep(0.001)  # Adjust the sleep duration as needed
    except websockets.ConnectionClosed as e:
        print(f"Connection closed with error: {e}")

async def main():
    async with websockets.serve(send_robot_joints, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

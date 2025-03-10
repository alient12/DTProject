import asyncio
import websockets
import json

import random
import numpy as np

async def send_robot_joints_randomly(websocket, path):
    try:
        while True:
            # joints = [-1.40780272, -1.06478243, 0.37771895, 2.16583817, 0.16887252, -4.00106307, -4.27609907]  # Example list of floats representing robot joints
            joints = [random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1, random.random()*2-1]  # Example list of floats representing robot joints
            await websocket.send(json.dumps(joints))
            print(f"Sent joints: {joints}")
            await asyncio.sleep(0.05)  # Adjust the sleep duration as needed
    except websockets.ConnectionClosed as e:
        print(f"Connection closed with error: {e}")

async def send_robot_joints(websocket, path):
    try:
        # Read joint data from CSV file
        q_data = np.loadtxt("robot_q_data.csv", delimiter=",", skiprows=1)  # Skip header row
        
        while True:
            # Forward direction: Send data from start to end
            for joints in q_data:
                
                data = {}
                data['joints_r'] = joints.tolist()[:7]
                data['joints_e'] = (data['joints_r'] + np.random.random(7)).tolist()
                data['endeff_r'] = joints.tolist()[7:]
                data['endeff_e'] = (data['endeff_r'] + 0.01 * np.random.random(3)).tolist()

                await websocket.send(json.dumps(data))  # Convert numpy array to list for JSON serialization
                print(f"Sent joints: {data}")
                await asyncio.sleep(0.05)  # Adjust the sleep duration as needed

            # Reverse direction: Send data from end to start
            for joints in reversed(q_data):

                data = {}
                data['joints_r'] = joints.tolist()[:7]
                data['joints_e'] = (data['joints_r'] + np.random.random(7)).tolist()
                data['endeff_r'] = joints.tolist()[7:]
                data['endeff_e'] = (data['endeff_r'] + 0.01 * np.random.random(3)).tolist()

                await websocket.send(json.dumps(data))
                print(f"Sent joints: {data}")
                await asyncio.sleep(0.05)  # Adjust the sleep duration as needed
    except websockets.ConnectionClosed as e:
        print(f"Connection closed with error: {e}")

async def main():
    async with websockets.serve(send_robot_joints, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

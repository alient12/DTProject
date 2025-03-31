import logging
import subprocess
import os
import time


import sys
import multiprocessing
import asyncio
import websockets

from sender_sim_ros2 import KinovaMonitorNode
from sender_sim import send_robot_joints_ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import re


def run_server(queue, stop_event):
    loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(loop)
    # com = Communicator(queue_list)
    
    # async def communication_loop():
    #     await com.connect()
    #     while not stop_event.is_set():  # Check if the stop event is set
    #         try:
    #             await asyncio.wait_for(com.listen(), timeout=1)  # Check every second
    #         except asyncio.TimeoutError:
    #             pass  # Continue checking

    #     print("Communicator stopping...")
    #     await com.close()

    async def sender_server():
        async with websockets.serve(lambda websocket, path: send_robot_joints_ros2(websocket, path, queue), "0.0.0.0", 8765):
            print("WebSocket server started on ws://0.0.0.0:8765")
            await asyncio.Future()  # Run forever
    
    loop.run_until_complete(sender_server())

def run_ros2(queue, stop_event):
    """Continuously checks the queue for new messages, stops when stop_event is set."""
    rclpy.init(args=None)
    node = KinovaMonitorNode(queue)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    print("Ros2 stopping...")

def clear_queue(queue):
    """Empties the queue before shutting down."""
    while not queue.empty():
        try:
            queue.get_nowait()  # Remove all items
        except:
            break  # Stop if queue is empty

def main():
    with multiprocessing.Manager() as manager:
        lock = manager.Lock()
        stop_event = multiprocessing.Event()
        queue = multiprocessing.Queue()

        server_process = multiprocessing.Process(target=run_server, args=(queue, stop_event))
        server_process.start()

        ros2_process = multiprocessing.Process(target=run_ros2, args=(queue, stop_event))
        ros2_process.start()

        
        ros2_process.join()
        stop_event.set()  # Ensure communicator stops when GUI exits

        print("Clearing remaining queue data before shutdown...")
        clear_queue(queue)  # Empty queue before exiting

        server_process.join(timeout=5)

        # If still alive after timeout, terminate them
        if ros2_process.is_alive():
            print("ros2_process took too long, terminating...")
            ros2_process.terminate()
            ros2_process.join()  # Ensure it's fully stopped
        
        if server_process.is_alive():
            print("server_process took too long, terminating...")
            server_process.terminate()
            server_process.join()  # Ensure it's fully stopped
        

        print("All processes stopped. Exiting program.")

if __name__ == "__main__":
    main()
from simulator import Simulator
from communication import Communicator
import logging
import subprocess
import os
import time


from PyQt5.QtWidgets import QApplication
import sys
import multiprocessing
import asyncio
from gui import MainWindow

def run_gui(shared_data, event, queue, stop_event):
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(shared_data, event, queue)
    
    def on_close():
        print("GUI closed. Notifying communicator to stop...")
        stop_event.set()  # Notify communicator and simulator processes to stop
        sys.exit()
    
    app.aboutToQuit.connect(on_close)

    window.show()
    sys.exit(app.exec_())

def run_communicator(queue_list, stop_event):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    com = Communicator(queue_list)
    
    async def communication_loop():
        await com.connect()
        while not stop_event.is_set():  # Check if the stop event is set
            try:
                await asyncio.wait_for(com.listen(), timeout=1)  # Check every second
            except asyncio.TimeoutError:
                pass  # Continue checking

        print("Communicator stopping...")
        await com.close()
    
    loop.run_until_complete(communication_loop())

def run_simulator(queue, stop_event):
    """Continuously checks the queue for new messages, stops when stop_event is set."""
    sim = Simulator(mode="online")
    sim.setup_env()
    sim.start_simulation(stop_event, queue)
    
    print("Simulatior stopping...")

def clear_queues(queue_list):
    """Empties the queue before shutting down."""
    for queue in queue_list:
        while not queue.empty():
            try:
                queue.get_nowait()  # Remove all items
            except:
                break  # Stop if queue is empty

def main():
    with multiprocessing.Manager() as manager:
        shared_data = {
            "end_eff_real": multiprocessing.Array('d', 3*20),
            "end_eff_expect": multiprocessing.Array('d', 3*20),
            "joints_real": multiprocessing.Array('d', 6*500),
            "joints_expect": multiprocessing.Array('d', 6*500),
            "lock": manager.Lock()
        }
        event = multiprocessing.Event()
        stop_event = multiprocessing.Event()
        queue_gui = multiprocessing.Queue()
        queue_sim = multiprocessing.Queue()
        queue_list = [queue_sim, queue_gui]


        comm_process = multiprocessing.Process(target=run_communicator, args=(queue_list, stop_event))
        comm_process.start()

        gui_process = multiprocessing.Process(target=run_gui, args=(shared_data, event, queue_gui, stop_event))
        gui_process.start()
        
        sim_process = multiprocessing.Process(target=run_simulator, args=(queue_sim, stop_event))
        sim_process.start()

        
        gui_process.join()
        stop_event.set()  # Ensure communicator stops when GUI exits

        print("Clearing remaining queue data before shutdown...")
        clear_queues(queue_list)  # Empty queue before exiting

        comm_process.join(timeout=5)
        sim_process.join(timeout=5)

        # If still alive after timeout, terminate them
        if gui_process.is_alive():
            print("gui_process took too long, terminating...")
            gui_process.terminate()
            gui_process.join()  # Ensure it's fully stopped
        
        if comm_process.is_alive():
            print("comm_process took too long, terminating...")
            comm_process.terminate()
            comm_process.join()  # Ensure it's fully stopped

        if sim_process.is_alive():
            print("sim_process took too long, terminating...")
            sim_process.terminate()
            sim_process.join()  # Ensure it's fully stopped
        

        print("All processes stopped. Exiting program.")

if __name__ == "__main__":
    main()
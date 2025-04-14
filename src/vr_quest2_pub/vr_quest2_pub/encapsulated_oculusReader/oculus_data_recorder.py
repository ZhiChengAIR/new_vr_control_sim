#!/usr/bin/env python3
"""
Usage:
(robodiff)$ python demo_real_robot.py -o <demo_save_dir> --robot_ip <ip_of_ur5>

Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the opencv window (make sure it's in focus).
Press "C" to start recording.
Press "S" to stop recording.
Press "Q" to exit program.
Press "Backspace" to delete the previously recorded episode.
"""

# %%
import time
from multiprocessing.managers import SharedMemoryManager
from .precise_sleep import precise_wait
from .oculus_reader.reader import OculusReader
from .oculus_data import OculusHandler

class OculusDataRecorder:
    def __init__(self, output="data/", frequency=10, command_latency=0.01) -> None:
        self.ocu_hz = 10
        self.output = output
        self.frequency = frequency
        self.command_latency = command_latency
        self.oculus_reader = OculusReader()
        self.handler = OculusHandler(self.oculus_reader, right_controller=True, hz=self.ocu_hz, use_filter=False, max_threshold=0.7/self.ocu_hz)
        self.increment = None
        self.buttons = None
        self.iter_idx = 0
        self.dt = 1/self.frequency
        time.sleep(1.0)

        # self.run()

    
    def run(self):
        # with SharedMemoryManager() as shm_manager:
        print('Ready!') 
        t_start = time.monotonic()
        stop = False
        is_recording = False
        time.sleep(1)
        while not stop:
            t_cycle_end = t_start + (self.iter_idx + 1) * self.dt
            t_sample = t_cycle_end - self.command_latency
            t_command_target = t_cycle_end + self.dt
            precise_wait(t_sample)
            increment = self.handler.get_increment()
            buttons = self.handler.get_buttons()
            right_trigger = buttons.get("rightTrig", [0])[0]
            print("increment",increment)
            print("...........")
            # print("buttons",buttons)
            # print("...........")
            # print("right_trigger",right_trigger)
            # print("...........")
            precise_wait(t_cycle_end)
            self.iter_idx += 1

if __name__ == '__main__':
    oculus_data_recorder = OculusDataRecorder()
    oculus_data_recorder.run()
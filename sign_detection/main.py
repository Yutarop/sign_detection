from queue import Queue
import threading
import rclpy
from gui import start_gui
from sign_detection2 import main as start_detection

if __name__ == "__main__":
    detection_queue = Queue()

    gui_thread = threading.Thread(target=start_gui, args=(detection_queue,))
    gui_thread.start()

    detection_thread = threading.Thread(target=start_detection, args=(None, detection_queue))
    detection_thread.start()

    gui_thread.join()
    detection_thread.join()

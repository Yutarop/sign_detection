import tkinter as tk
from queue import Queue

class SignDetectionGUI:
    def __init__(self, root, detection_queue):
        self.root = root
        self.root.title("Sign Detection")
        self.root.geometry("300x150")

        # Detection Label
        self.detection_label = tk.Label(root, text="Waiting...", font=("Times", 26), foreground='#0a0a0a')
        self.detection_label.pack(anchor="center", expand=1)

        # Queue for receiving detection signals
        self.detection_queue = detection_queue
        self.check_queue()

    def check_queue(self):
        try:
            message = self.detection_queue.get_nowait()
            if message == "Detected":
                self.detection_label.config(text="Detected!", foreground='#0a0a0a')
        except:
            pass
        self.root.after(100, self.check_queue)

def start_gui(detection_queue):
    root = tk.Tk()
    app = SignDetectionGUI(root, detection_queue)
    root.mainloop()

import tkinter as tk
from play_face_cars import FacePlayerCars

# Create the main window
window = tk.Tk()
window.title("Hello World")
window.geometry("1024x600+0+0")
window.rowconfigure(0, weight=1)
window.columnconfigure(0, weight=1)

# Create a label widget
#label = tk.Label(window, text="Hello World!")
#label.pack()
FacePlayerCars(window)

# Start the GUI event loop
window.mainloop()

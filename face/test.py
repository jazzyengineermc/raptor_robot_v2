import tkinter as tk
import time
from face.play_face_cars import FacePlayerCars

class RaptorFaceApp:
    def __init__(self, master):
        # Configure
        #self.disable_cursor = True
        self.fullscreen = False
        self.blink = True
        
        # Create the main window
        self.master = master
        master.title("Raptor")
        master.geometry("1024x600+0+0")
        master.rowconfigure(0, weight=1)
        master.columnconfigure(0, weight=1)
        
        if self.fullscreen:
            master.bind("<Escape>", self.end_fullscreen)
            master.attributes("-fullscreen",True)

        if self.blink:
            master.bind("<b>", self.blink_eyes)
            
        #if self.disable_cursor:
        #    self.config(cursor="none")

        self.build_face_page()
    
    
    def destroy_face_page(self):
        self.face_page.destroy()
        self.face_page = None
    
    
    def update_image(self):

        if self.face_page:
            self.face_page.update_image()

        return


    def end_fullscreen(self, event=None):
        self.master.attributes("-fullscreen", False)
        return "break"


    def build_face_page(self):
        self.face_page = FacePlayerCars(root)
    
    
    def blink_eyes(self, event=None):
        self.face_page.update_values(0.0, 1.0)
        self.update_image()
        #time.sleep(10)
        #self.face_page.update_values(0.0, 0.0)
        #self.update_image()


if __name__ == "__main__":
    root = tk.Tk()
    app = RaptorFaceApp(root)
    root.mainloop()
    
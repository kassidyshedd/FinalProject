import rclpy 
from rclpy.node import Node
from tkinter import Tk, Label, Frame
from PIL import Image, ImageTk
import numpy as np

class GuiNode(Node):

    def __init__(self):

        super().__init__('gui_node')

        # GUI
        self.window = Tk()
        self.window.title("Images")

        # GUI Sizing
        window_width = 1000
        window_height = 1080
        self.window.geometry(f"{window_width}x{window_height}")

        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()
        window_x = int((screen_width / 2) - (window_width / 2))
        window_y = int((screen_height / 2) - (window_height / 2))
        self.window.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")

        vertical_padding = 50

        top_frame = Frame(self.window, width=1000, height=540)
        top_frame.grid(row=0, column=0, columnspan=2, pady=(0, vertical_padding))
        top_label = Label(top_frame, text="Livestream View", font=("Arial", 16))
        top_label.pack(side="top", pady=5)


        left_label = Label(self.window, text="Top View", font=("Arial", 16))
        left_label.grid(row=1, column=0, pady=(0, 5))

        # Create a frame for the bottom half (img2 and img3) with size 480x540
        bottom_frame_left = Frame(self.window, width=500, height=540)
        bottom_frame_left.grid(row=2, column=0)

        right_label = Label(self.window, text="Side View", font=("Arial", 16))
        right_label.grid(row=1, column=1, pady=(0, 5))

        bottom_frame_right = Frame(self.window, width=500, height=540)
        bottom_frame_right.grid(row=2, column=1)

        # Open Images
        livestream = Image.open("/home/kashedd/finalproject/code/ros/ws-v2/captured_frame.png")
        top = Image.open("/home/kashedd/finalproject/pose_images/upload/fakeimg1.png")
        side = Image.open("/home/kashedd/finalproject/pose_images/upload/fakeimg2.png")

        # Resize to fit in GUI
        livestream = self.resize(livestream, (1000, 500))
        top = self.resize(top, (500, 540))
        side = self.resize(side, (500, 540))

        # Convert to photoimage
        livestream = ImageTk.PhotoImage(livestream)
        top = ImageTk.PhotoImage(top)
        side = ImageTk.PhotoImage(side)

        # Create labels
        l1 = Label(top_frame, image=livestream)
        l1.image = livestream
        l1.pack(fill="both", expand=True)

        l2 = Label(bottom_frame_left, image=top)
        l2.image = top
        l2.grid(row=0, column=0)

        l3 = Label(bottom_frame_right, image=side)
        l3.image = side
        l3.grid(row=0, column=1)

        self.window.mainloop()

    def resize(self, img, target):

        ow, oh = img.size
        tw, th = target

        scale = min(tw / ow, th / oh)

        nw = int(ow * scale)
        nh = int(oh *scale)

        return img.resize((nw, nh), Image.Resampling.LANCZOS)

def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


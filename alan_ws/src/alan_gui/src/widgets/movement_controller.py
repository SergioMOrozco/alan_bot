import tkinter as tk
from tkinter import ttk
from widgets.xbox_control_tab import XboxControlTab
from widgets.screen_control_tab import ScreenControlTab

class MovementController(tk.LabelFrame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.LabelFrame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        # create notebook with two movement control tabs
        self.notebook = tk.ttk.Notebook(self)
        self.screen_control_tab = ScreenControlTab(self.notebook,robot)
        self.xbox_control_tab = XboxControlTab(self.notebook,robot)
        self.notebook.add(self.screen_control_tab,text="On Screen")
        self.notebook.add(self.xbox_control_tab,text="Xbox Control")

        self.notebook.pack()

import tkinter as tk
import rospy
from tkinter import ttk
from std_msgs.msg import String

class ControllerLog(tk.LabelFrame):
    def __init__(self,parent,*args,**kwargs):

        self.log_sub = rospy.Subscriber("controller_logging",String,self.update_log)

        tk.LabelFrame.__init__(self,parent,*args, **kwargs)
        self.parent = parent

        self.listBox = ttk.Treeview(self,columns=('logs'), show="headings")

        self.listBox.column("logs",anchor=tk.N)
        self.listBox.pack()

    def update_log(self,log):
        ## FIFO
        if len(self.listBox.get_children()) == 10:
            self.listBox.delete(self.listBox.get_children()[0])
            self.listBox.insert("","end",value=(log.data))
        else:
            self.listBox.insert("","end",value=(log.data))






    

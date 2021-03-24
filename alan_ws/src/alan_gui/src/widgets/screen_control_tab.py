import tkinter as tk

class ScreenControlTab(tk.Frame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self._robot = robot

        ## on screen movement controls
        self.forward_button= tk.Button(self, text="W", anchor=tk.N, padx=70, pady=70)
        self.left_button= tk.Button(self, text="A", anchor=tk.W, padx=30,pady=70)
        self.backward_button= tk.Button(self, text="S", anchor=tk.S,padx=70,pady=70)
        self.right_button= tk.Button(self, text="D", anchor=tk.E,padx=30,pady=70)

        ## create and bind buttons
        self.forward_button.grid(row=0,column=1)
        self.forward_button.bind('<ButtonPress-1>',self.click_forward)
        self.forward_button.bind('<ButtonRelease-1>',self.power_button_released)


        self.left_button.grid(row=1,column=0)
        self.left_button.bind('<ButtonPress-1>',self.click_left)
        self.left_button.bind('<ButtonRelease-1>',self.steer_button_released)

        self.backward_button.grid(row=2,column=1)
        self.backward_button.bind('<ButtonPress-1>',self.click_backward)
        self.backward_button.bind('<ButtonRelease-1>',self.power_button_released)

        self.right_button.grid(row=1,column=2)
        self.right_button.bind('<ButtonPress-1>',self.click_right)
        self.right_button.bind('<ButtonRelease-1>',self.steer_button_released)

    def click_forward(self,event):
        self._robot.apply_power(1.0)

    def click_left(self,event):
        self._robot.apply_steering_power(-1.0)

    def click_backward(self,event):
        self._robot.apply_power(-1.0)

    def click_right(self,event):
        self._robot.apply_steering_power(1.0)

    def power_button_released(self,event):
        self._robot.apply_power(0)

    def steer_button_released(self,event):
        self._robot.apply_steering_power(0)


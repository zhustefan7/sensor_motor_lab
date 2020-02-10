import random
from tkinter import *
import os 
# from xls_write import xls_write
import argparse
# import imutils
# import cv2 as cv
# import PySpin
import numpy as np
import time
import datetime
# from PIL import ImageTk,Image 
import serial
import threading
import shutil,sys

EmptyGray = "gray30"
sensor_reading = '0'
motor_reading = '0'
page_bool = [True, False, False, False]
upper_dir="software_test/"  # the upper most directory under which the result of each session is stored in current path, which will be initialized later
now = datetime.datetime.now()   #for getting current time 

# Setting Up Serial Port 
ser = serial.Serial('/dev/ttyACM0',9600, writeTimeout=0, parity=serial.PARITY_NONE, rtscts=False,dsrdtr=False)
ser.bytesize=serial.EIGHTBITS
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.xonxoff = False    #disable software flow control


# def receive_msg():
#     # while page_bool[indx]:
#     global sensor_reading
#     global motor_reading
#     # print(sensor_reading)
#     reading = ser.read(4)
#     reading= reading.decode('ASCII')
#     if reading[0] == 'z':
#         motor_reading = reading[1:]
#     elif reading[0] == 's':
#         sensor_reading = reading[1:]
#     return 

def send_command_threading(msg):
    t3 = threading.Thread(target=send_command, args=[msg])
    t3.start()
    t3.join()
     
def send_command(msg):
    ser.write((msg).encode("ascii"))
    return 


class App(object):
    def __init__(self): pass
    def mousePressed(self, event): pass
    def keyPressed(self, event): pass
    def timerFired(self): pass
    def redrawAll(self): pass
    def mouseMotion(self, event): pass
    def mouseReleased(self, event): pass

    # Call app.run(width,height) to get your app started
    def run(self, width=1100, height=600):
        # create the root and the canvas
        root = Tk()

        self.width = width
        self.height = height
        self.canvas = Canvas(root, width=width, height=height)
        self.canvas.configure(background="SlateGray2")
        # self.canvas.configure(background="LightSteelBlue4")
        self.canvas.pack()


        # set up events
        def redrawAllWrapper():
            self.canvas.delete(ALL)
            self.redrawAll()
            self.canvas.update()

        def mousePressedWrapper(event):
            self.mousePressed(event)
            redrawAllWrapper()

        def mouseMotionWrapper(event):
            self.mouseMotion(event)
            redrawAllWrapper()

        def doubleClickedWrapper(event):
            self.doubleClicked(event)
            redrawAllWrapper()

        def mouseReleasedWrapper(event):
            self.mouseReleased(event)
            redrawAllWrapper()

        def keyPressedWrapper(event):
            self.keyPressed(event)
            redrawAllWrapper()

        root.bind("<Button-1>", mousePressedWrapper)
        root.bind("<Motion>", mouseMotionWrapper)
        root.bind("<ButtonRelease>", mouseReleasedWrapper)
        # root.bind("<Double-Button-1>", doubleClickedWrapper)
        root.bind("<Key>", keyPressedWrapper)


        # set up timerFired events
        self.timerFiredDelay = 1 # milliseconds
        def timerFiredWrapper():
            self.timerFired()
            redrawAllWrapper()
            # pause, then call timerFired again
            self.canvas.after(self.timerFiredDelay, timerFiredWrapper)

        # init and get timerFired running
        #self.__init__()
        t1=threading.Thread(target=timerFiredWrapper)
        t1.start()
        # t2 = threading.Thread(target=receive_msg)
        # t2.start()
        # and launch the app
        # timerFiredWrapper()
        root.mainloop()
        t1.join()
        # timerFiredWrapper()
        # t2.join()
        ser.close()  #Close the serial port when the software is closed
        print("Bye")



class Interface(App):
    def __init__(self):
        App.__init__(self)
        self.W=1100
        self.H=600
        self.Frontpage=Frontpage(self.W,self.H)
        self.stepper_motor_page=motor_page(self.W,self.H,"stepper")
        self.servo_motor_page=motor_page(self.W,self.H,"servo")
        self.dc_brushless_page=motor_page(self.W,self.H, "dc")
        self.page_list=[self.Frontpage,self.servo_motor_page,self.stepper_motor_page,self.dc_brushless_page]



    def draw_page(self,canvas):
        for i in range(len(self.page_list)):
            if page_bool[i]==True:  
                self.page_list[i].draw_page(canvas)

    def redrawAll(self):
        canvas = self.canvas
        self.draw_page(canvas)

    def mousePressed(self,event):
        for i in range(len(self.page_list)):
                if page_bool[i]==True:
                    self.page_list[i].mousePressed(event)

    def keyPressed(self,event):
        for i in range(len(self.page_list)):
                if page_bool[i]==True:
                    self.page_list[i].keyPressed(event)



class Frontpage(object):
    def __init__(self,width,height):
        self.w=width
        self.h=height
        self.user_command=''
        self.b1_loc = [self.w/2-120, self.h/2-200, self.w/2+180, self.h/2-110]
        self.b2_loc = [self.w/2-120, self.h/2-90, self.w/2+180, self.h/2]
        self.b3_loc = [self.w/2-120, self.h/2+20, self.w/2+180, self.h/2+110]
        msg = 'm%03d'%0
        # send_command(msg)
        send_command_threading(msg)
        pass

    def draw_button(self,canvas):
        canvas.create_rectangle(self.b1_loc[0], self.b1_loc[1], self.b1_loc[2], self.b1_loc[3],
                                fill='khaki', width=0)
        canvas.create_text((self.b1_loc[0]+self.b1_loc[2])/2, (self.b1_loc[1]+self.b1_loc[3])/2,fill="darkblue",font="Times 20 italic bold",text="Servo Motor")
        
        canvas.create_rectangle(self.b2_loc[0], self.b2_loc[1], self.b2_loc[2], self.b2_loc[3],
                                fill='khaki', width=0)
        canvas.create_text((self.b2_loc[0]+self.b2_loc[2])/2, (self.b2_loc[1]+self.b2_loc[3])/2,fill="darkblue",font="Times 20 italic bold",text="Stepper Motor")

        canvas.create_rectangle(self.b3_loc[0], self.b3_loc[1], self.b3_loc[2], self.b3_loc[3],
                                fill='khaki', width=0)
        canvas.create_text((self.b3_loc[0]+self.b3_loc[2])/2, (self.b3_loc[1]+self.b3_loc[3])/2,fill="darkblue",font="Times 20 italic bold",text="DC brushless Motor")

    def draw_page(self,canvas):
        # canvas.create_rectangle(0, 0, 50, 60,
        #                         fill='yellow', width=0)
        self.draw_button(canvas)


    def mousePressed(self,event):
        x=event.x
        y=event.y
        global page_bool


        if self.b1_loc[0]<=x<=self.b1_loc[2] and self.b1_loc[1]<=y<=self.b1_loc[3]:
            page_bool = [False, True, False, False]
            motor_msg = 'm%03d'%1
            # t2=threading.Thread(target=receive_msg, args=[page_bool, 1])
            # t2.start()
            # t2.join()
            print('motor mode: Servo')
            
        elif self.b2_loc[0]<=x<=self.b2_loc[2] and self.b2_loc[1]<=y<=self.b2_loc[3]:
            page_bool = [False, False, True, False]
            motor_msg = 'm%03d'%2
            # t2=threading.Thread(target=receive_msg, args=[page_bool,2])
            # t2.start()
            # t2.join()
            print('motor mode: Stepper')
            
        elif self.b3_loc[0]<=x<=self.b3_loc[2] and self.b3_loc[1]<=y<=self.b3_loc[3]:
            page_bool = [False, False, False, True]
            motor_msg = 'm%03d'%3
            # t2=threading.Thread(target=receive_msg, args=[page_bool,3])
            # t2.start()
            # t2.join()
            print('motor mode: DC brushless')
        
        # control_msg ='c%03d'%1
        # send_command(control_msg)
        send_command_threading(motor_msg)
        # send_command(motor_msg)
        # t2.join()
        return 



class motor_page(object):
    def __init__(self,width,height,motor_type):
        self.motor_type = motor_type
        self.width=width
        self.height=height
        self.button_width=150
        self.button_height=50
        self.motor_reading='0'
        self.sensor_reading = '0'
        self.user_command='0'
        self.label_box=False
        self.lengnth_box=False
        self.is_inspecting=False
        self.buffer_path='buffer/'
        self.abort=False
        self.current_blade_length='8'
        self.current_image=None
        self.abort=False
        self.control_modes = [True,False]      #indx 0: sensor control mode    indx 1: manual control mode
        # self.t3=threading.Thread(target=self.receive_msg)
        # self.t3.start()
        # self.t3.join()
        pass

    # def receive_msg(self):
    #     global sensor_reading
    #     while True:
    #         sensor_reading = ser.readline()
    #         sensor_reading= sensor_reading.decode('ASCII')
    #         sensor_reading = sensor_reading[1:]
    #     return 

    def receive_msg(self):
        reading = ser.read(4)
        reading= reading.decode('ASCII')
        if reading[0] == 'z':
            self.motor_reading = reading[1:]
        elif reading[0] == 's':
            self.sensor_reading = reading[1:]
        return 

    def draw_page(self,canvas):
        # t3=threading.Thread(target=self.receive_msg)
        # t3.start()
        # t3.join()
        # self.receive_msg()
        self.draw_button(canvas)
        self.draw_massage(canvas)
        self.draw_user_command(canvas)

        canvas.create_image(0,0,image=self.current_image,anchor=NW)



    def draw_button(self,canvas):
        button_width=self.button_width
        button_height=self.button_height


        #Sensor Control Mode
        if self.control_modes[0]:
            button1_fill="SeaGreen1"

        else:

            button1_fill="Maroon"

        canvas.create_rectangle(800, 200,800+button_width,200+button_height,
                                fill=button1_fill, width=0)
        canvas.create_text(800+button_width/2, 200+button_height/2,fill="darkblue",font="Times 10 italic bold",text="Sensor Control")
       
        #Manul Mode Button
        if self.control_modes[1]:
            button2_fill="SeaGreen1"
        else:
            button2_fill="Maroon"
        
        canvas.create_rectangle(800+button_width, 200, 800+2*button_width,200+button_height,
                                fill=button2_fill, width=0)
        canvas.create_text(800+button_width*1.5, 200+button_height/2,fill="darkblue",font="Times 10 italic bold",text="Manual Mode")


        #send command button
        text = "Send"

        canvas.create_rectangle(800+button_width ,100, 800+2*button_width,100+button_height,
                                fill="SeaGreen1", width=1)
        canvas.create_text(800+button_width*1.5, 100+button_height/2,fill="darkblue",font="Times 10 italic bold",text=text)


        #back Button
        canvas.create_rectangle(self.width-100 ,self.height-50, self.width,self.height,
                                fill="khaki", width=1)
        canvas.create_text(self.width-50,self.height-25,fill="darkblue",font="Times 10 italic bold",text="Back")

      
      
        #retrieve Button 
        text = "Get State"

        canvas.create_rectangle(800+button_width ,400, 800+2*button_width,400+button_height,
                                fill="purple", width=1)
        canvas.create_text(800+button_width*1.5, 400+button_height/2,fill="darkblue",font="Times 10 italic bold",text=text)





    def draw_massage(self,canvas):
        canvas.create_rectangle(20,self.height-90,600,self.height-20,fill="white",width=0)
        canvas.create_text(80,self.height-80,fill="darkblue",font="Times 10 italic bold",text="Sensor Reading:")
        canvas.create_text(80,self.height-30,fill="black",font="Times 10 italic bold",text=self.sensor_reading)

        canvas.create_rectangle(20,self.height-200,600,self.height-130,fill="white",width=0)
        canvas.create_text(80,self.height-190,fill="darkblue",font="Times 10 italic bold",text="Motor Angle:")
        canvas.create_text(80,self.height-180,fill="black",font="Times 10 italic bold",text=self.motor_reading)



    def mousePressed(self,event):
        x = event.x
        y = event.y
        button_width=self.button_width
        button_height=self.button_height
        global page_bool

        #Sensor Control Mode
        if 800<=x<=800+button_width and 200<=y<=200+button_height:
            self.control_modes[1] = False
            self.control_modes[0]= True
            msg = 'c%03d'%0
            send_command(msg)        
            print(self.control_modes)

        #Serial Control Mode
        if 800+button_width<=x<=800+2*button_width and 200<=y<=200+button_height:
            self.control_modes[1] = True
            self.control_modes[0]= False
            msg = 'c%03d'%1
            # send_command(msg)  
            send_command_threading(msg)
            print(self.control_modes)
            # t2=threading.Thread(target=receive_msg, args = [page_bool,1])
            # t2.start()
            # sys.stdout.flush()
            # t2.join()


        #Send Command Button
        if 800+button_width<=x<=800+2*button_width and 100<=y<=100+button_height:
            # self.hover_list[3]=True
            # self.hover_list[2]=False
            self.abort=True
            if self.control_modes[1]:
                msg = 'a%03d'% int(self.user_command)
                # send_command(msg)
                send_command_threading(msg)
                print('sent msg%s'%msg)



        #Back Button
        if self.width-100<=x<=self.width and self.height-50<=y<=self.height:
            page_bool[0]=True
            page_bool = [True, False, False, False]
            self.control_modes = [True,False] 
            msg = 'm%03d'%0
            # send_command(msg)
            send_command_threading(msg)
            print('front page')


        #Retrieve Command Button
        if 800+button_width<=x<=800+2*button_width and 400<=y<=400+button_height:
            print('here')
            # self.abort=True
            self.receive_msg()
            pass




        #User command Input Box
        if 1100-350<=x<=1100-10 and 50<=y<=90:
            self.label_box=True
        else:
            self.label_box=False


    def keyPressed(self,event):
        if self.label_box==True:
            self.user_command=self.user_command+event.char
            if event.keysym=="BackSpace":
                # print(self.user_command)
                self.user_command=self.user_command[0:len(self.user_command)-2]
     
        if self.lengnth_box==True:
            if (event.char=='1' or event.char=='2'\
            or event.char=='3'or event.char=='4'or event.char=='5'or event.char=='6' or event.char=='7' \
            or event.char=='8'or event.char=='9' or event.char=='0' ):    #making sure that user only inputs number for blade length
                self.current_blade_length=self.current_blade_length+event.char
            if event.keysym=="BackSpace":
                self.current_blade_length=self.current_blade_length[0:len(self.current_blade_length)-2]


    def draw_user_command(self,canvas):
        canvas.create_rectangle(1100-350, 50, 1100-10, 90,
                                fill='white', width=0)

        canvas.create_text(1100-350, 70,fill="darkblue",anchor='w',font="Times 11 italic bold",text="User Command:")
        canvas.create_text(1100-250, 70,fill="black",anchor='w',font="Times 11 italic bold",text=self.user_command)
        if self.label_box:
            canvas.create_line(1100-250+len(self.user_command)*7,50,1100-250+len(self.user_command)*7,80,fill='darkblue')     # this is the cursor
    
    






        # super().draw_page(canvas)

if __name__ == '__main__':
    blade_inspector=Interface()
    blade_inspector.run()


    # t1=threading.Thread(target=blade_inspector.run)
    # t1.start()
    


    # t2=threading.Thread(target=receive_msg, args = [page_bool,1])
    # t2.start()    

    # t1.join()
    # t2.join()
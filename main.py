# GUI design
import tkinter as tk
from tkinter import scrolledtext

# Communication with serial port
import serial
from serial.tools import list_ports

# Multi-threading
import threading

# Get path
import os

#mathematics
import numpy as np

#import sys
#import glob
from matplotlib import pyplot as plt

#others for calibration
from scipy import linalg
from functools import reduce
import operator

#elapsed time
import time

# Use realpath if you want the real path (symlinks are resolved)
# file_path = os.path.realpath(__file__)
FILE_PATH = os.path.abspath(__file__)
ICON_PATH = os.path.join(os.path.dirname(__file__), "icon.png")


class GUI:
    # GUI main class
    def __init__(self, title):

        self.portNamesList = []
        self.baudRatesList = [
            1200,
            2400,
            4800,
            9600,
            19200,
            38400,
            57600,
            115200,
            230400,
            460800,
            576000,
            921600,
        ]
        self.sensorsList = [
            'G',
            'A',
            'M',
            '-']
        self.isAnyPortAvailable = False
        self.isStarted = False
        self.serialPortName = None
        self.serialPortBaud = 9600

        self.serialPortManager = SerialPortManager(self.serialPortBaud)
        self.get_available_serial_ports()

        self.guiUpdateInterval = 40

        self.scroll_lock = False

        self.window = tk.Tk()
        # Title of application window
        self.window.title(title)
        # Icon of application window
        self.window.iconphoto(False, tk.PhotoImage(file=ICON_PATH))

        self.topFrame = tk.Frame(self.window, bg="#cccccc")

        self.scanButton = tk.Button(
            self.topFrame,
            text="Scan Serial Ports",
            bg="#0051ff",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#1f7cff",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.scan_button_command,
        )

        # Define a tk.StringVar for storing selected item in OptionMenu
        self.selectedPort = tk.StringVar()
        # Set default value of selectedPort
        if self.isAnyPortAvailable == False:
            self.portNamesList = ["No ports available"]
        self.selectedPort.set(self.portNamesList[0])

        self.portsOptionMenu = tk.OptionMenu(
            self.topFrame, self.selectedPort, *self.portNamesList
        )

        self.portsOptionMenu.configure(
            bg="#ffffff",
            fg="#222222",
            border=0,
            highlightbackground="#aaaaaa",
            activebackground="#eeeeee",
            activeforeground="#111111",
            direction="left",
            font=("Sans", "10", "bold"),
        )
        if self.isAnyPortAvailable == False:
            self.portsOptionMenu.configure(state="disabled")

        # Define a tk.IntVar for storing selected item in OptionMenu
        self.selectedBaudRate = tk.IntVar()
        # Set default value of selectedBaudRate
        self.selectedBaudRate.set(self.baudRatesList[3])
        self.baudRatesOptionMenu = tk.OptionMenu(
            self.topFrame, self.selectedBaudRate, *self.baudRatesList
        )

        self.baudRatesOptionMenu.configure(
            bg="#ffffff",
            fg="#222222",
            border=0,
            highlightbackground="#aaaaaa",
            activebackground="#eeeeee",
            activeforeground="#111111",
            direction="left",
            font=("Sans", "10", "bold"),
        )
        if self.isAnyPortAvailable == False:
            self.baudRatesOptionMenu.configure(state="disabled")

       # Define a tk.IntVar for storing selected item in OptionMenu
        self.selected_sensor = tk.StringVar()
        # Set default value of selected_sensor
        self.selected_sensor.set(self.sensorsList[3])
        self.selectedSensorOptionMenu = tk.OptionMenu(
            self.topFrame, self.selected_sensor, *self.sensorsList
        )

        self.selectedSensorOptionMenu.configure(
            bg="#ffffff",
            fg="#222222",
            border=0,
            highlightbackground="#aaaaaa",
            activebackground="#eeeeee",
            activeforeground="#111111",
            direction="left",
            font=("Sans", "10", "bold"),
        )

        self.connectButton = tk.Button(
            self.topFrame,
            text="Connect",
            bg="#00a832",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#3fcc69",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.start_button_command,
        )

        self.sendPEMC09Button = tk.Button(
            self.topFrame,
            text="Receive raw data from sensors",
            bg="#00a832",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#3fcc69",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.sendPEMC09_button_command,
        )

        
        self.generatePEMC12Button = tk.Button(
            self.topFrame,
            text="Calculate offsets",
            bg="#00a832",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#3fcc69",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.generatePEMC12_button_command,
        )


        self.sendPEMC14Button = tk.Button(
            self.topFrame,
            text="Send offsets to autopilot",
            bg="#00a832",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#3fcc69",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.sendPEMC14_button_command,
        )



        self.sendPEMC11CButton = tk.Button(
            self.topFrame,
            text="Save offsets sent to autopilot",
            bg="#00a832",
            fg="#ffffff",
            border=0,
            highlightbackground="#ffffff",
            highlightthickness=2,
            activebackground="#3fcc69",
            activeforeground="#ffffff",
            font=("Sans", "10", "bold"),
            command=self.sendPEMC11C_button_command,
        )

        self.scroll_lock = tk.BooleanVar()
        self.scrollLock = tk.Checkbutton(
            self.topFrame,
            text='Scroll Lock',
            variable=self.scroll_lock, onvalue=True, offvalue=False)
        
        

        self.sendPEMC09Button.configure(
            state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
        )

        self.sendPEMC14Button.configure(
            state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
        )

        self.sendPEMC11CButton.configure(
            state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
        )

        if self.isAnyPortAvailable == False:
            self.connectButton.configure(
                state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
            )
            
        self.textBox = scrolledtext.ScrolledText(
            self.topFrame,
            bg="#222222",
            fg="#eeeeee",
            border=0,
            wrap="none",
            highlightbackground="#aaaaaa",
            highlightthickness=2,
            font=("Sans", "10", "bold"),
        )

        self.figure_active = False
        
        # Start updating textbox in GUI
        self.recursive_update_textbox()

        ###############################
        ## Widgets size and position ##
        ###############################

        spacing = 2
        padding = 2
        widget_width = 17
        widget_height = 2
        window_width = 1000
        window_height = 600
        next_widget = 200
        next_widgety = 50

        # Size of application window
        self.window.geometry("{}x{}".format(window_width, window_height))
        # Don't allow resizing in the x or y direction
        self.window.resizable(False, False)

        self.topFrame.configure(padx=padding, pady=padding)
        self.topFrame.place(x=0, y=0, width=window_width, height=window_height)
        
        self.scanButton.configure(width=widget_width, height=widget_height, padx=padding, pady=padding)
        self.scanButton.place(x= spacing, y = spacing)

        self.portsOptionMenu.configure(width=widget_width, height=widget_height, padx=padding, pady=padding)
        self.portsOptionMenu.place(x= next_widget, y = spacing )

        self.baudRatesOptionMenu.configure(width=widget_width, height=widget_height, padx=padding, pady=padding)
        self.baudRatesOptionMenu.place(x= next_widget * 2, y = spacing )

        self.connectButton.configure(width=widget_width, height=widget_height, padx=padding, pady=padding)
        self.connectButton.place(x= next_widget * 3, y = spacing )

        self.sendPEMC09Button.configure(width=widget_width + 7, height=widget_height, padx=padding, pady=padding)
        self.sendPEMC09Button.place(x= 3, y = next_widgety )
        
        self.generatePEMC12Button.configure(width=widget_width + 7, height=widget_height, padx=padding, pady=padding)
        self.generatePEMC12Button.place(x= 3, y = next_widgety*2 )
        
        self.sendPEMC14Button.configure(width=widget_width + 7, height=widget_height, padx=padding, pady=padding)
        self.sendPEMC14Button.place(x= 3, y = next_widgety*3)    

        self.sendPEMC11CButton.configure(width=widget_width + 7, height=widget_height, padx=padding, pady=padding)
        self.sendPEMC11CButton.place(x= 3, y = next_widgety*4)
        
        self.selectedSensorOptionMenu.configure(width=widget_width, height=widget_height, padx=padding, pady=padding)
        self.selectedSensorOptionMenu.place(x= 3, y = next_widgety*5)

        self.scrollLock.place(x= 3, y = next_widgety*6)  

        self.textBox.configure(width=109, height=30, padx=padding, pady=padding)
        self.textBox.place(x= 207, y = 50 )
        self.textBox.tag_config("text_received", foreground="white") # plain text received
        self.textBox.tag_config("NMEA_sent", foreground="blue") # NMEA sent
        self.window.protocol("WM_DELETE_WINDOW", self.close_window)
        
        # Blocking loop for GUI (Always put at the end)
        self.window.mainloop()
        
    #connect button
    def start_button_command(self):

        if self.isStarted == False:
            self.isStarted = True
            self.connectButton.configure(
                bg="#ba0020",
                highlightbackground="#ffffff",
                activebackground="#cf324d",
                text="Disconnect",
            )
            # Get desired serial port name
            self.serialPortName = self.selectedPort.get()
            # Get desired serial port baud rate
            self.serialPortBaud = self.selectedBaudRate.get()
            # Start Serial Port Communication
            self.serialPortManager.set_name(self.serialPortName)
            self.serialPortManager.set_baud(self.serialPortBaud)
            self.serialPortManager.start()
            # Start updating textbox in GUI
            self.recursive_update_textbox()
            # Start updating 3dFigure in GUI
            self.recursive_update_3dFigure()

 
        else:
            self.isStarted = False
            self.connectButton.configure(
                bg="#00a832",
                highlightbackground="#ffffff",
                activebackground="#3fcc69",
                text="Connect",
            )
            self.serialPortManager.stop()
            self.sendPEMC09Button.configure(
                state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
            )
            self.sendPEMC14Button.configure(
                state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
            )
            self.sendPEMC11CButton.configure(
                state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa"
            )

    def enable_sendPEMCButtons(self):
        if self.serialPortManager.IsStarted == True:
            #conectar 09 y 14 verde
            self.sendPEMC09Button.configure(
                bg="#00a832",
                highlightbackground="#ffffff",
                activebackground="#3fcc69",
                state="normal",
            )
            self.sendPEMC14Button.configure(
                bg="#00a832",
                highlightbackground="#ffffff",
                activebackground="#3fcc69",
                state="normal",
            )
            self.sendPEMC11CButton.configure(
                bg="#00a832",
                highlightbackground="#ffffff",
                activebackground="#3fcc69",
                state="normal",
            )

    def sendPEMC09_button_command(self):
        self.Figure_buffer = np.array([]);
        self.serialPortManager.send_PEMC09=True
        # Set desired sensor
        self.serialPortManager.set_selected_sensor(self.selected_sensor.get())

    def generatePEMC12_button_command(self):
        # Set desired sensor
        if self.selected_sensor.get() == '-':
            sensor_list = ['G', 'A', 'M']
        else:
            sensor_list = self.selected_sensor.get()
        for x in sensor_list:
            if x == 'G':
                nmea_sentence = Gyroscope("gyro_raw.csv").run()
                f = open('gyro_offsets.txt', 'w')
                f.write(nmea_sentence)
                f.close()
            if x == 'A':
                nmea_sentence = Accel_Magnetometer("A", "accel_raw.csv").run()
                f = open('accel_offsets.txt', 'w')
                f.write(nmea_sentence)
                f.close()
            if x == 'M':
                nmea_sentence = Accel_Magnetometer("M", "magnet_raw.csv").run()
                f = open('magnet_offsets.txt', 'w')
                f.write(nmea_sentence)
                f.close()

    def sendPEMC14_button_command(self):
        self.serialPortManager.send_PEMC14 = True
        # Set desired sensor
        self.serialPortManager.set_selected_sensor(self.selected_sensor.get())

    def sendPEMC11C_button_command(self):
        self.serialPortManager.send_PEMC11C = True

    def scan_button_command(self):
        self.portNamesList = self.get_available_serial_ports()

        if len(self.portNamesList) == 0:
            self.isAnyPortAvailable = False
            self.portNamesList = ["No ports available"]
            self.portsOptionMenu.configure(state="disabled")
            self.baudRatesOptionMenu.configure(state="disabled")
            self.connectButton.configure(
            state="disabled", bg="#bbbbbb", highlightbackground="#aaaaaa")
            
        else:
            self.isAnyPortAvailable = True
            self.portsOptionMenu.configure(state="normal")
            self.baudRatesOptionMenu.configure(state="normal")
            if self.isStarted:
                self.connectButton.configure(
                    bg="#ba0020",
                    highlightbackground="#ffffff",
                    activebackground="#cf324d",
                    state="normal",
                )
                self.sendPEMC09Button.configure(
                    bg="#ba0020",
                    highlightbackground="#ffffff",
                    activebackground="#cf324d",
                    state="normal",
                )
                self.sendPEMC14Button.configure(
                    bg="#ba0020",
                    highlightbackground="#ffffff",
                    activebackground="#cf324d",
                    state="normal",
                )
                self.sendPEMC11CButton.configure(
                    bg="#ba0020",
                    highlightbackground="#ffffff",
                    activebackground="#cf324d",
                    state="normal",
                )
            else:
                self.connectButton.configure(
                    bg="#00a832",
                    highlightbackground="#ffffff",
                    activebackground="#3fcc69",
                    state="normal",
                )


        self.update_option_menu(self.portNamesList)

    def get_available_serial_ports(self):
        # Clear portNames list
        portNames = []
        # Get a list of available serial ports
        portsList = list_ports.comports()
        # Sort based on port names
        portsList = sorted(portsList)

        for port in portsList:
            portNames.append(port.device)

        return portNames

    def update_option_menu(self, portNames):
        # Remove old items
        self.portsOptionMenu["menu"].delete(0, "end")
        # Add new items
        for portName in portNames:
            self.portsOptionMenu["menu"].add_command(
                label=portName, command=tk._setit(self.selectedPort, portName)
            )
        # Set default value of selectedPort
        self.selectedPort.set(portNames[0])

    def recursive_update_textbox(self):
        serialPortBuffer_in = self.serialPortManager.read_buffer().decode("ascii")
        # Update textbox in a kind of recursive function using Tkinter after() method
        self.textBox.insert(tk.END, serialPortBuffer_in, "text_received")

        serialPortBuffer_out = self.serialPortManager.send_buffer_out()
        if serialPortBuffer_out!= "":
            self.textBox.insert(tk.END, ">>>" + serialPortBuffer_out + "\n", "NMEA_sent")
        
        # autoscroll to the bottom
        
        if self.scroll_lock.get()==False:
            self.textBox.see(tk.END)
        # Recursively call recursive_update_textbox using Tkinter after() method
        
        self.window.after(self.guiUpdateInterval, self.enable_sendPEMCButtons)
        if self.serialPortManager.isRunning:
            self.window.after(self.guiUpdateInterval, self.recursive_update_textbox)

    def close_window(self):
        if self.isStarted:
            self.serialPortManager.stop()
        self.window.destroy()

    def create_3dFigure(self):
        if self.figure_active == False:
            fig=plt.figure("Raw data received - " + self.serialPortManager.new_file)
      
            ax=fig.add_subplot(111, projection='3d');
            fig.canvas.mpl_connect('close_event', self.on_close_figure)
            plt.ion();
            plt.show();
            plt.pause(0.5)
                        
        self.figure_active = True

      
    def on_close_figure(self, event):
        self.figure_active = False;
        self.sendPEMC09_button_command();

    def recursive_update_3dFigure(self):
        if self.serialPortManager.new_sensor==True:
           self.create_3dFigure()
           self.serialPortManager.new_sensor=False 
        Figure_buffer = self.serialPortManager.read_3dFigure_buffer()
        # Update 3dFigure in a kind of recursive function using Tkinter after() method
        if self.figure_active == True and Figure_buffer.size != 0:
            plt.plot(int(Figure_buffer[0]), int(Figure_buffer[1]), int(Figure_buffer[2]), marker='o')
            plt.pause(0.01)
        # Recursively call recursive_update_textbox using Tkinter after() method
        if self.serialPortManager.isRunning:
            self.window.after(self.guiUpdateInterval, self.recursive_update_3dFigure)

class SerialPortManager:
    # A class for management of serial port data in a separate thread
    def __init__(self, serialPortBaud=9600):
        self.isRunning = False
        self.serialPortName = None
        self.serialPortBaud = serialPortBaud
        self.serialPort = serial.Serial()
        # Create a byte array to store incoming data
        self.serialPortBuffer = bytearray()
        # Create a string to store outgoing data
        self.serialPortBuffer_out = ""
        # Create a byte array to store incoming 3dFigure data
        self.Figure_buffer = np.array([]);
        #Variable to trigger PEMCXX message
        self.send_PEMC09=False
        self.send_PEMC14=False
        self.send_PEMC11C=False
        
        #Line in process
        self.line=""
        #Sensor raw data received
        self.sensor=""
        self.new_sensor = False
        #raw data accumulated
        self.data = np.array([]);
        #file to save data accumulated
        self.file = ""
        self.new_file = ""
        #elapsed time counter
        self.start_time = 0
        #sensor selected
        self.selected_sensor='-'
        self.current_sensor ='G'
        #Pilot status
        self.IsStarted = False

    def set_name(self, serialPortName):
        self.serialPortName = serialPortName

    def set_baud(self, serialPortBaud):
        self.serialPortBaud = serialPortBaud

    def set_selected_sensor(self, selected_sensor):
        self.selected_sensor = selected_sensor
        if self.selected_sensor=='-':
            self.current_sensor = 'G'
        else:
            self.current_sensor = self.selected_sensor

    def start(self):
        self.isRunning = True
        self.serialPortThread = threading.Thread(target=self.thread_handler)
        self.serialPortThread.start()

    def stop(self):
        self.isRunning = False
        self.IsStarted = False
        self.send_PEMC09=False
        self.send_PEMC14=False
        self.send_PEMC11C=False
    
    def thread_handler(self):

        while self.isRunning:

            if not self.serialPort.isOpen():

                self.serialPort = serial.Serial(
                    port=self.serialPortName,
                    baudrate=self.serialPortBaud,
                    bytesize=8,
                    timeout=2,
                    stopbits=serial.STOPBITS_ONE,
                )
            else:
                # Wait until there is data waiting in the serial buffer
                while self.serialPort.in_waiting > 0:
                    # Read only one byte from serial port
                    serialPortByte = self.serialPort.read(1)
                    self.serialPortBuffer.append(int.from_bytes(serialPortByte, byteorder='big'))
                    # Process incoming bytes
                    self.main_process(serialPortByte)
                    
                    
                # write pending sentence to serial
                if self.send_PEMC09==True:
                    s_NMEA_sentence = "PEMC,09," + self.selected_sensor #$PEMC,09,-*3F or $PEMC,09,A*53
                    s_NMEA_checksum = nmea_checksum(s_NMEA_sentence)
                    s_NMEA_sentence = "$"+s_NMEA_sentence+"*"+s_NMEA_checksum
                    self.write_buffer_out (s_NMEA_sentence)
                    self.send_PEMC09=False
                    
                if self.send_PEMC14==True:
                    #start secuence
                    if self.start_time == 0:
                        #run secuence
                        if self.current_sensor=='G': f = open('gyro_offsets.txt', 'r')
                        if self.current_sensor=='A': f = open('accel_offsets.txt', 'r')
                        if self.current_sensor=='M': f = open('magnet_offsets.txt', 'r')
                        nmea_sentence = f.read()
                        f.close()
                        self.write_buffer_out (nmea_sentence)
                        self.start_time = time.time()
                    #wait for next secuence
                    else:
                        current_time = time.time()
                        if (current_time - self.start_time)>3:
                            #next/ finish secuence
                            if self.selected_sensor=='-':
                                if self.current_sensor=='M':
                                    self.send_PEMC14=False
                                if self.current_sensor=='A':
                                    self.current_sensor='M'
                                if self.current_sensor=='G':
                                    self.current_sensor='A'
                            else:
                                self.current_sensor = self.selected_sensor
                                self.send_PEMC14=False
                            self.start_time = 0
                
                if self.send_PEMC11C==True:
                    s_NMEA_sentence = "$PEMC,11,C*58"
                    self.write_buffer_out (s_NMEA_sentence)
                    self.send_PEMC11C=False
                    
        if self.serialPort.isOpen():
            self.serialPort.close()
    
    def read_buffer(self):
        # Return a copy of serial port buffer
        i_buffer = self.serialPortBuffer
        # Clear serial port buffer
        self.serialPortBuffer = bytearray()
        return i_buffer

    def write_buffer_out (self, message):
        if self.serialPortBuffer_out != "":
            print("Buffer Overflow\n")
        self.serialPortBuffer_out = message

    def send_buffer_out(self):
        o_buffer="";
        if self.serialPort.isOpen():
            # Return a copy of serial port buffer
            o_buffer = self.serialPortBuffer_out
            self.serialPort.write(o_buffer.encode())
        
        # Clear serial port buffer
        self.serialPortBuffer_out = ""
        return o_buffer

    def read_3dFigure_buffer(self):
        # Return a copy of serial port buffer
        buffer = self.Figure_buffer
        # Clear serial port buffer
        self.Figure_buffer = np.array([])
        return buffer

    def __del__(self):
        if self.serialPort.isOpen():
            self.serialPort.close()

    def main_process(self, inputByte):
        new_line = False
        # Print the received byte in Python terminal
        try:
            character = inputByte.decode("ascii")
        except UnicodeDecodeError:
            pass
        else:
            #print(character, end="")
            
            if character =="\n":
                new_line = True
            else:
                self.line=self.line + character;
                
        if new_line == True:
            self.process_line()
            new_line = False
            self.line = ""

    def process_line(self):
        if self.line =="Start":
            self.IsStarted = True;
            
        fields = self.line.split( ',')
        if len(fields)==3:
            try:
                x = int(fields[0])+int(fields[1])+int(fields[2])
            except:
                return
            
            self.data=np.append(self.data, fields)
            self.Figure_buffer=np.append(self.Figure_buffer, fields)
            return

        if self.line[:9]=="$PEMC,12,":
            #Check if saving data is required
            if self.data.size >0:
                #print("data!=0\n")
                self.data = self.data.reshape(-1, 3)
                np.savetxt(self.new_file, self.data, fmt='%s', delimiter=',')
                print(f'File saved:{self.new_file}\n')
                self.data = np.array([])
            if self.process_PEMC12()==True:
                #reset figure
                self.new_sensor = True
                #self.data = np.array([])
                
            
        if self.sensor=="-":
            #stop process
            
            return;

    def process_PEMC12(self):
        if self.line[:10]!="$PEMC,12,O": return False;
        match self.line[11:18]:    
            case "0,1,0,0":
                #plt.clf()
                print("RECEIVING RAW DATA FROM GYROSCOPE\n")
                self.sensor="G";
                self.new_file = "gyro_raw.csv"
                return True;
            case "0,0,1,0":
                #plt.clf()
                print("RECEIVING RAW DATA FROM ACCELEROMETER\n")
                self.sensor="A";
                self.new_file = "accel_raw.csv"
                return True;
            case "0,0,0,1":
                #plt.clf()
                print("RECEIVING RAW DATA FROM MAGNETOMETER\n")
                self.sensor="M";
                self.new_file = "magnet_raw.csv"
                return True;
            case "0,0,0,0":
                #plt.clf();
                print("FINISHED RECEIVING RAW DATA FROM SENSORS\n")
                self.sensor="-";
                self.new_file = ""
                return False;
        return False;

 #https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py
 #corrected code S. James Remington, see issue #1 in above contribution.
 #required data input file: x,y,z values in .csv (text, comma separated value) format.
 # see example mag3_raw.csv, .out


class Gyroscope(object):

    def __init__(self, afile_path): 


        # initialize values
        self.b   = np.zeros([3, 1])
        self.file_path = afile_path
        self.A_1 = np.zeros([3, 3])
        self.s_sensor = "G"

    def run(self):
        print("*************************") 
        print("******* GYROSCOPE *******")
        data = np.loadtxt(self.file_path,delimiter=',')
        print("shape of data array:",data.shape)
        #print("datatype of data:",data.dtype)
        print("First 5 rows raw:\n", data[:5])
        # calculate offsets
        array_lenght = len (data)
        self.b[0] = sum(data[:,0])/ array_lenght
        self.b[1] = sum(data[:,1])/ array_lenght
        self.b[2] = sum(data[:,2])/ array_lenght
        
        # show results
        result = [] 
        for row in data: 
            # subtract the hard iron offset
            xm_cal  = row[0]-self.b[0]
            ym_cal  = row[1]-self.b[1]
            zm_cal  = row[2]-self.b[2]
            
            result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]) )#, axis=0 )
        
        result = result.reshape(-1, 3)
        fig = plt.figure(self.file_path)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(result[:,0], result[:,1], result[:,2], marker='o', color='g')
        plt.show()
        
        print("First 5 rows calibrated:\n", result[:5])

        #save corrected data to file "out.txt"
        np.savetxt('gyro_out.txt', result, fmt='%f', delimiter=' ,')


        print("*************************" )        
        print("Hardcoded. code to paste in DevICM20948.cpp:" )
        print("*************************" )
        b_string = my_string(self.b,2)
        print("float hcG_offset[3] = {", b_string.rstrip(b_string[-1]) ,"};")
        print("\n")

        print("*************************" )        
        print("NMEA Sentence for Fenix Autopilot : " )
        print("*************************" )
        print("\n")
        
        s_NMEA_sentence = "PEMC,14," + my_string(self.b,2) + my_string(self.A_1,5) + self.s_sensor
        s_NMEA_checksum = nmea_checksum(s_NMEA_sentence)
        s_NMEA_sentence = "$"+s_NMEA_sentence+"*"+s_NMEA_checksum
        print(s_NMEA_sentence, end="")
        print("\n")
        return s_NMEA_sentence



        

 
class Accel_Magnetometer(object):
    
    '''
     references :
        -  https://teslabs.com/articles/magnetometer-calibration/      
        -  https://www.best-microcontroller-projects.com/hmc5883l.html

    '''
    MField = 1000  #arbitrary norm of magnetic field vectors

    def __init__(self, asensor, afile_path, F=MField): 


        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)
        self.s_sensor = asensor
        self.file_path = afile_path

        
    def run(self):
        print("*************************") 
        if self.s_sensor == "A":
            print("***** ACCELEROMETER *****")
        if self.s_sensor == "M":
            print("***** MAGNETOMETER *****")
            
        data = np.loadtxt(self.file_path,delimiter=',')
        print("shape of data array:",data.shape)
        #print("datatype of data:",data.dtype)
        print("First 5 rows raw:\n", data[:5])
        
        # ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        
        print("\nData normalized to ",self.F)        
        print("Soft iron transformation matrix:\n",self.A_1)
        print("Hard iron bias:\n", self.b)

        plt.rcParams["figure.autolayout"] = True

        result = [] 
        for row in data: 
        
            # subtract the hard iron offset
            xm_off  = row[0]-self.b[0]
            ym_off  = row[1]-self.b[1]
            zm_off  = row[2]-self.b[2]
            
            #multiply by the inverse soft iron offset
            xm_cal = xm_off *  self.A_1[0,0] + ym_off *  self.A_1[0,1]  + zm_off *  self.A_1[0,2] 
            ym_cal = xm_off *  self.A_1[1,0] + ym_off *  self.A_1[1,1]  + zm_off *  self.A_1[1,2] 
            zm_cal = xm_off *  self.A_1[2,0] + ym_off *  self.A_1[2,1]  + zm_off *  self.A_1[2,2] 

            result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]) )#, axis=0 )

        result = result.reshape(-1, 3)
        fig = plt.figure(self.file_path)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(result[:,0], result[:,1], result[:,2], marker='o', color='g')
##        # draw sphere
##        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
##        d = 1000
##        x = np.cos(u)*np.sin(v)*d
##        y = np.sin(u)*np.sin(v)*d
##        z = np.cos(v)*d
##        ax.plot_wireframe(x, y, z, color="r")
        plt.show()
        
        print("First 5 rows calibrated:\n", result[:5])
		
        #save corrected data to file "out.txt"
        if self.s_sensor == "M":
            np.savetxt('magnet_out.txt', result, fmt='%f', delimiter=' ,')
        if self.s_sensor == "A":
            np.savetxt('accel_out.txt', result, fmt='%f', delimiter=' ,')

        print("*************************" )        
        print("Hardcoded. code to paste in DevICM20948.cpp: " )
        print("*************************" )
        b_string = my_string(self.b,2)
        if self.s_sensor == "A":
            print("float hcA_B[3] = {")
        if self.s_sensor == "M":
            print("float hcM_B[3] = {")
            
        print(b_string.rstrip(b_string[-1]) ,"};")
        print("\n")
        
        self.A_1 = np.round(self.A_1,5)
        if self.s_sensor == "A":
            print("float hcA_Ainv[3][3] = {")
        if self.s_sensor == "M":
            print("float hcM_Ainv[3][3] = {")
        print("{", self.A_1[0,0],",", self.A_1[0,1],",", self.A_1[0,2], "},")
        print("{", self.A_1[1,0],",", self.A_1[1,1],",", self.A_1[2,1], "},")
        print("{", self.A_1[2,0],",", self.A_1[2,1],",", self.A_1[2,2], "}};")
        print("\n")

        print("*************************" )        
        print("NMEA Sentence for Fenix Autopilot : " )
        print("*************************" )
        print("\n")
        
        s_NMEA_sentence = "PEMC,14," + my_string(self.b,2) + my_string(self.A_1,5) + self.s_sensor
        s_NMEA_checksum = nmea_checksum(s_NMEA_sentence)
        s_NMEA_sentence = "$"+s_NMEA_sentence+"*"+s_NMEA_checksum
        print(s_NMEA_sentence, end="")
        print("\n")
        return s_NMEA_sentence
        
    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
        M = np.array([[v_1[0], v_1[5], v_1[4]],
                      [v_1[5], v_1[1], v_1[3]],
                      [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d

def my_string(x, d):
    x = np.round(x,d)
    s_format = "{:.%df},"%d
    ret = ""
    for row in x:
        ret = ret + ''.join(map(lambda x: s_format.format(x), row))        
    return ret

def my_print(x, d):
    x = np.round(x,d)
    s_format = "{:.%df},"%d
    for row in x:
        print(''.join(map(lambda x: s_format.format(x), row)), end="")
        

def nmea_checksum(sentence: str):
    """
    This function checks the validity of an NMEA string using it's checksum
    """
    #sentence = sentence.strip("$\n")
    #nmeadata, checksum = sentence.split("*", 1)
    calculated_checksum = reduce(operator.xor, (ord(s) for s in sentence), 0)
    
    return hex(calculated_checksum)[2:].upper()


if __name__ == "__main__":

    # Create the GUI
    gui = GUI("Fenix Autopilot - Serial Terminal")


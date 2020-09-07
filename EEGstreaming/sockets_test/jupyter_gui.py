'''The following code is designed to run directly from within a jupyter cell'''

from threading import Thread
import os
import time
import datetime
import socket
import json

from matplotlib import pyplot as plt
%matplotlib notebook
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd

from IPython.core.display import display, clear_output
from IPython.display import IFrame, Javascript
import ipywidgets as ipw

import logging
logging.basicConfig(level=logging.WARNING)


####
### EEG Class ###

class EEG:
    '''The following variables are shared between a streaming and a plotting instance
    of EEG's subclasses. They run in different threads and need to access a shared variable space.'''
    
    # Data Structure
    n_channels = 8
    time = {"time":pd.Series([],dtype="int64")}
    channels = {str(i):pd.Series([],dtype="float64") for i in range(1,n_channels+1)}
    srate = {"sampling_rate":pd.Series([],dtype="float64")}
    
    data = pd.DataFrame({**time, **channels, **srate})
    plot_buffer = data
    # Preview of Data columns              #
    # time   | 0  | 1 | .. | sampling_rate # 
    
    
    # State Variables
    connected = False
    connecting = False
    connecting_stop_flag = False
    
    
    streaming = False
    streaming_stop_flag = False
    buffering = True
    
    recording = False
    plotting = False
    active_channels = [1,2,3,4,5,6,7,8]
        
    
    #Plot Parameters
    time_window = 2
    sampling_rate = 222
    xwindow = time_window*sampling_rate
    mV_window = 2000
    fps = 25
    
    
    sample_pkg =""
    
    def reset():
        n_channels = 8
        time = {"time":pd.Series([],dtype="int64")}
        channels = {str(i):pd.Series([],dtype="float64") for i in range(1,n_channels+1)}
        srate = {"sampling_rate":pd.Series([],dtype="float64")}
        EEG.data = pd.DataFrame({**time, **channels, **srate})
        #EEG.plot_buffer = EEG.data
        
    def download():
        if 'sessions' not in os.listdir():
            os.mkdir('sessions')
        time = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
        filepath = f"sessions/{time}.csv"
        EEG.data.to_csv(filepath)
        print(f"Data saved to {filepath}")
        return
        
    class Streamer: 
        
        def __init__(self, timeout=10):
            self.timeout = timeout
            self.socket = None
            self.connection = None 
            self.address = None
            
        def build_connection(self,host="",port=65432):
            '''
            Creates a socket and waits for a connection until it succeeds or 
            the user aborts the connection attempt by forcefully closing the socket.
            Binds a socket to self.socket. If a connection is succesfully established,
            the connection object is bound to self.connection.
            
            The value of EEG.connecting is managed from within this function.
            
            Parameters:
                host (string):  host address to accept connections from ("" for all)
                port (int):     port on which the socket listens
                
            '''
            EEG.connecting = True
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.bind((host, port))
            except socket.error as e:
                print(str(e))
                
            # Connection Success
            try:
                self.socket.listen()  
                self.connection, self.address = self.socket.accept()
                EEG.connected = True
                
            # Connection aborted
            except:
                self.socket = None
                self.connection = None
                EEG.connected = False
                
            EEG.connecting = False
            return

        def stream(self, buffersize=1):
            '''
            Reads and processes datastream from a connection.
            Expects incoming data to be stringified json files and turns them
            into dictionaries. 
            
            The value of EEG.streaming is managed from within this function.
            
            Requirements:
                self.socket is an actual socket 
                self.connection is an actual connection
            
            Parameters:
                buffersize (int): Number of bytes read at once from connection
            ''' 
            EEG.streaming = True
            EEG.buffering = True
            
            connection = self.connection
            socket = self.socket
            pkg = ""        # 1 pkg: 1 json object
            pkg_count = 0

            start_time = time.time()
            sampling_rate = 0 
            
            # FLUSH
            #response = connection.recv(4096)
            
            print("Entering While Loop of Stream..")
            while True:  
                if EEG.streaming_stop_flag:
                    print("Detected stop flag, stopping Stream")
                    EEG.streaming_stop_flag=False
                    break
                    
                
                # Receive data (buffer length=1byte)
                response = connection.recv(buffersize).decode("utf-8")

                # Beginning of Package
                if "{" in response:  
                    pkg += response[response.index("{"):]  # add everything after first curly

                # Content
                elif "}" not in response: 
                    pkg += response

                # End of Package 
                else:
                    pkg += response
                    pkg_count += 1
                    EEG.sample_pkg = pkg
                    json_string = pkg.rstrip("\n").strip() # remove ending break lines and white spaces

                    data = json.loads(json_string)  # parse stringified JSON as dict
                    elapsed = (time.time() - start_time)
                    sampling_rate = int(pkg_count / elapsed) if pkg_count > 1 else 0

                    self.store(elapsed,data,sampling_rate)  # add current read package for plotting 

                    pkg = pkg[pkg.index("}")+1:]  # only keep data after the end of the first package
                
            print("Set EEG.streaming to False definitely.")
            EEG.streaming=False
            return
        
        def store(self, elapsed, sample, sampling_rate):
            '''
            Formats a parsed package and appends it to the plot_buffer and data storage.
            
            
            Paramters:
                elapsed (float): elapsed time since stream start in ms
                sample (dict): voltages of all different eeg channels
                sampling_rate (int): calculated sampling rate of the packages arriving per second
            '''
            
            buffersize = 222
            
            original_time = sample["time"]
            del sample["time"]
            pkg = sample["pkg"]
            
            formatted = {**{'time':elapsed},
                       **sample,
                       **{'sampling_rate':sampling_rate},
                       **{'pkg':pkg}
                      }
            
            EEG.plot_buffer = EEG.plot_buffer.append(formatted, ignore_index=True)
            
            if EEG.recording:
                EEG.data.append(formatted,ignore_index=True)
            
            if EEG.plot_buffer.shape[0] > EEG.xwindow*2 + buffersize:
                EEG.buffering = False # Loaded buffer
                     
            return 
        
        def simulate_stream(self, interval=0.005):
            data = pd.read_csv("sessions/2020-07-15-1920.csv")            
            xwindow = EEG.xwindow
            time_window = EEG.time_window
            t = xwindow +1
            
            expected_sr = EEG.sampling_rate
            print("Entering while Loop")
            
            stop_counter = 0
            while True:
                # Only Append Data if we are "Streaming"
                if EEG.streaming:
                    
                    if t < xwindow+5:
                        print("Inside While Loop")
                        print("Appending Data")
                    # Update the plot_data
                    pkg = {**data.iloc[t]} # "Receiving from Traumschreiber"
                    EEG.plot_buffer = EEG.plot_buffer.append(pkg, ignore_index=True)
                    
                    # Keep plot_buffer <= xwindow
                    if t > xwindow*2:  
                        EEG.plot_buffer = EEG.plot_buffer[-xwindow:]
                        
                    t += 1     
                    # Wait a bit to simulate stream processing
                else:
                    if EEG.streaming_stop_flag:
                        EEG.streaming_stop_flag=False
                        return

                if t > xwindow + 10000:
                    break
            return
                    
    class Plotter:
        
        def __init__(self):
            self.fig = None
            self.ax = None
            self.lines = None
            self.animation = None
            
        def build_plot(self):
            # Preparing the Plot and extracting the lines
            EEG.plot_buffer.plot(x="time", y= [str(channel+1) for channel in range(EEG.n_channels)])
            self.fig = plt.gcf()
            self.ax = self.fig.gca()
            self.lines = [line for line in self.ax.lines]
            
            interval = 1/EEG.fps*1000 #Update time in ms
            self.animation = FuncAnimation(self.fig, self.update, interval=500, blit=True)
            
            # Setting Some Plot properties
            self.ax.set_title("EEG Streaming @ 0 Hz")
            self.ax.set_ylim(-2500,2500)
            self.ax.set_xlim(-EEG.time_window/2,EEG.time_window/2)
            self.ax.set_xlabel("time [s]")
            self.ax.set_ylabel("signal [μV]")
            self.ax.legend(loc="upper right")
            
            
            return

        def update(self, frame): 
            try:
                # Update Plot Lines
                for i,line in enumerate(self.lines):
                    line.set_data(EEG.plot_buffer.time, EEG.plot_buffer[f"{i+1}"])


                #Update x-axis
                right = EEG.plot_buffer.time.iloc[-1] # last timestamp
                left = right-EEG.time_window
                #left = EEG.plot_buffer.time.iloc[0]
                self.fig.gca().set_xlim(left,right)
                #Update y-axis
                #self.fig.gca().set_ylim(-EEG.mV_window,EEG.mV_window)
                #Update other Plot elements
                self.fig.gca().set_title(f"Traumschreiber Streaming @ {EEG.plot_buffer.sampling_rate.iloc[-1]}Hz")
            except:
                pass
            return self.lines
			
			
class GUI:
    def __init__(self):
        self.streamer = EEG.Streamer()
        self.plotter = EEG.Plotter()
        self.widgets = {}
        self.build_widgets()
        
        
        # Monitor own status
        self.aborting = False

    def build_widgets(self):
        instructions = ipw.HTML(value= "<h1>Build connection and start streaming from the app. ")
        self.widgets["instructions"] = instructions
        
        
        self.add_button("connect_btn","Connect",self.handle_connect_btn)
        self.add_button("stream_btn","Stream",self.handle_stream_btn)
        self.add_button("record_btn", "RECORD", self.handle_record_btn)
        self.add_button("save_btn","Save Data",self.handle_save_btn)
        
        self.widgets["main_buttons"] = ipw.HBox([self.widgets["connect_btn"],
                                                 self.widgets["stream_btn"],
                                                 self.widgets["record_btn"],
                                                 self.widgets["save_btn"]]
                                               )
        self.widgets["main_buttons"].layout.margin="20px 0 0 0"
    
    
        
    def display(self):
        display(self.widgets["main_buttons"])
        self.plotter.build_plot()
        return
    
    def run_scripts(self):
        Javascript('''
                document.getElementsByClassName
                ("ui-dialog-titlebar ui-widget-header ui-corner-all ui-helper-clearfix")
                [0].style.display="none";
            ''')
    
    def add_button(self,btn_name="btn_name",description="description", target=lambda x:x):
        btn = ipw.Button(description=description)
        btn.on_click(target)
        
        btn.layout.height = "80px"
        btn.layout.margin = "0 0 0 10px"
        self.widgets[btn_name] = btn
        

    def handle_connect_btn(self,b):
        # Abort Connection process
        if self.aborting:
            b.description = "Still aborting.."
        
        elif EEG.connecting:
            self.aborting = True
            try:
                self.streamer.socket.close()
            except:
                pass
            b.description = "Aborting connection.."
            b.tooltip="Please Wait"
            b.button_style="Warning"
            Thread(target=self.detect_connection_changes).start()
        
        # Close existing Connection
        elif EEG.connected:
            self.streamer.socket.close()
            EEG.connected = False
            b.description = "Connect"
            b.tooltip = "Press to connect"
            b.button_stlye=""
            
        # Create a new connection    
        else:
            Thread(target=self.streamer.build_connection, daemon=True).start()
            Thread(target=self.detect_connection_changes, daemon=True).start()
            b.description = "Connecting.. "
            b.tooltip = "Press to abort"
            b.button_style = "info"
        return
    
    def detect_connection_changes(self):
        time.sleep(3)
        while EEG.connecting:
            time.sleep(0.1)
        self.update_connection_status(EEG.connected)
        
    def update_connection_status(self, connected=EEG.connected):
        self.aborting=False
        if connected:
            self.widgets["connect_btn"].description="Connected"
            self.widgets["connect_btn"].button_style="Success"
            self.widgets["connect_btn"].tooltip = "Press to disconnect"
        else:
            self.widgets["connect_btn"].description="Connect"
            self.widgets["connect_btn"].button_style=""
            self.widgets["connect_btn"].tooltip = "Press to Connect"

    def handle_stream_btn(self,b):
        if not EEG.streaming:
            Thread(target = self.streamer.stream, daemon=True).start()
            self.plotter.animation.event_source.start()
            
            b.button_style="success"
            b.description="Streaming"
            b.tooltip="Press to stop streaming"
        else:
            
            EEG.streaming_stop_flag = True
            self.plotter.animation.event_source.stop()
            
            b.button_style=""
            b.description="Stream"
            b.tooltip="Press to start streaming"
        return
    
    def handle_record_btn(self,b):
        if not EEG.recording:
            EEG.reset()
            EEG.recording = True
            b.description="RECORDING"
            b.button_style="danger"
            b.tooltip="Click to stop recording"
        else:
            EEG.recording = False
            b.description="RECORDING"
            b.tooltip="New recording, discard old one"
            b.button_style=""
			
    def handle_save_btn(self,b):
        EEG.download()
        time = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
        filepath = f"sessions/{time}.csv"
        return
		
		

gui = GUI()
gui.display()
Javascript('''
			document.getElementsByClassName
			("ui-dialog-titlebar ui-widget-header ui-corner-all ui-helper-clearfix")
			[0].style.display="none";
		''')
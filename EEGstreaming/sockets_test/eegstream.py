import os
import time
import datetime
import socket
import json

from matplotlib import pyplot as plt
#%matplotlib notebook
from matplotlib.animation import FuncAnimation

import numpy as np
import pandas as pd


class EEG:
    '''
    The following variables are shared between a streaming and a plotting instance
    of EEG's subclasses. They run in different threads and need to access a shared variable space.
    
    Variables:
        temp_data (DataFrame): dataframe with size limit used for on the spot plotting
        stored_data (List): a list of eeg data packages in dict form.
    '''
    
    # Data Structure
    n_channels = 8
    pkg = {"pkg":pd.Series([],dtype="string")}
    time = {"time":pd.Series([],dtype="int64")}
    channels = {str(i):pd.Series([],dtype="float64") for i in range(1,n_channels+1)}
    srate = {"sampling_rate":pd.Series([],dtype="float64")}
    
    temp_data = pd.DataFrame({**pkg, **time, **channels, **srate})
    temp_data = temp_data.set_index("time")
    
    stored_data = []
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
    active_channels = [True for i in range(n_channels)]
    
    
    #Plot Parameters
    time_window = 2000 #ms
    sampling_rate = 222
    mV_window = 2000
    fps = 20

    def reset():
        data = pd.DataFrame({**EEG.pkg, **EEG.time, **EEG.channels, **EEG.srate})
        data = data.set_index("time")

    def download():
        if 'sessions' not in os.listdir():
            os.mkdir('sessions')
        time = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
        filepath = f"sessions/{time}.csv"
        EEG.data.to_csv(filepath)
        print(f"Data saved to {filepath}")
        return
        
    class Streamer: 
        
        def __init__(self, buffersize=1024):
            self.socket = None
            self.connection = None 
            self.address = ""
            self.buffersize = buffersize
            self.reftime = -1
            
        def build_connection(self,host="",port=65432):
            '''
            Creates a socket and waits for a connection until it succeeds or 
            the user aborts the connection by forcefully closing the socket.
            Binds a socket to self.socket. If a connection is succesfully established,
            the connection object is bound to self.connection.
            
            The value of EEG.connecting is managed from within this function.
            
            Parameters:
                host (string):  host address to accept connections from ("" for all)
                port (int):     port on which the socket listens
                
            '''
            if EEG.connected:
                self.socket.close()
            EEG.connecting = True
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.bind((host, port))
            except socket.error as e:
                print(str(e))
                
            # Connection succeeded
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

        def stream(self, buffersize=1024):
            '''
            Reads and processes datastream from a connection.
            Expects incoming data to be stringified json files and turns them
            into dictionaries. One json file has a size of about 170 bytes.
            
            Requirements:
                self.socket is an actual socket 
                self.connection is an actual connection
            Parameters:
                buffersize (int): Number of bytes read at once from connection
            '''
            
            if self.connection == None:
                raise Exception("No connection established yet.")
            EEG.streaming = True
            EEG.buffering = True
            self.clear_buffer()
            data = ""
            while not EEG.streaming_stop_flag:
                response = self.connection.recv(buffersize).decode("utf8")
                data += response
                
                if "\n" in data:
                    remainder = self.process_data(data)
                    data = remainder

            EEG.streaming = False
            EEG.buffering = False
            return
        
        def clear_buffer(self):
            '''
            Reads all the leftover bytes from the established connection untill
            the remaining data are small enough to be handled with the chosen
            buffersize.
            '''
            while 1:
                if len(self.connection.recv(1000000)) < self.buffersize:
                    break
                    
        def process_data(data):
            '''
            Processes a part of the datastream. Packages are delimited by linebreaks.
            The json packages are turned into dictionaries. Afterwards, they are merged into a dataframe and appended to EEG.temp_data for plotting.
            If the user records the session, the individual dict packages are appended to 
			EEG.stored_data.
            Parameters:
                data (str): String of linebreak delimited json packages sent by the EEGDroid app
            returns:
                remainder (str): incomplete package, a substring at the end of pkgs
            '''
            
            pkgs = data.split("\n")
            remainder = pkgs.pop(-1)
            pkg_dicts = [json.loads(p) for p in pkgs]
            
            if self.reftime <0:
                self.reftime = pkg_dicts[0]["time"]
            
            pkg_df = pd.DataFrame(pkg_dicts).set_index("time")
            pkg_df.index = pkg_df.index - self.reftime
            EEG.temp_data = EEG.temp_data.append(pkg_df)
                
            if EEG.temp_data.shape[0] > EEG.temp_data_limit:
                EEG.temp_data = EEG.temp_data[-EEG.temp_data_limit:]
            if EEG.recording:
                for dic in pkg_dicts:
                    EEG.stored_data.append(dic)

            return remainder
            
            
    class Plotter:
        
        def __init__(self):
            self.fig = None
            self.ax = None
            self.lines = None
            self.animation = None
            self.interval = (1/EEG.fps)*1000   #ms between plot updates
            
        def build_plot(self):
            # Preparing the Plot and extracting the lines
            EEG.temp_data.plot(y= [str(channel+1) for channel in range(EEG.n_channels)])
            self.fig = plt.gcf()
            self.ax = self.fig.gca()
            self.lines = [line for line in self.ax.lines]
            
            self.animation = FuncAnimation(self.fig, self.update, interval=self.interval, blit=True)
            
            # Setting Some Plot properties
            self.ax.set_title("EEG Streaming")
            self.ax.set_ylim(-2500,3200)
            self.ax.set_xlim(-EEG.time_window/2,EEG.time_window/2)
            self.ax.set_xlabel("time [s]")
            self.ax.set_ylabel("signal [μV]")
            self.ax.set_xticks([])
            self.ax.set_yticks([])
            self.ax.legend(loc=(0.92,0.4),fontsize=12)
            
            
            return

        def update(self, t): 
            t_a = t*self.interval
            t_b = t_a + EEG.time_window

            try:
                # Update Plot Lines
                for i,line in enumerate(self.lines):
                    line.set_data(EEG.temp_data[t_a:t_b].time,
                                  EEG.temp_data[t_a:t_b][f"{i+1}"]+i*80) #offset on y axis

                self.ax().set_xlim(t_a,t_b)
                #Update other Plot elements
            except:
                pass
            return self.lines
from IPython.core.display import display, clear_output
from IPython.display import IFrame, Javascript
import ipywidgets as ipw

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
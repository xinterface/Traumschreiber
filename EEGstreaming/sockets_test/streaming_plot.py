import itertools
import json
import matplotlib.pyplot as plt
import socket
import time

# constants
CHANNELS = 8  # number of eeg channels
PLOT_MEMO = 0.5  # plot memory in seconds
SR = 222  # expected sampling rate/frequency (Hz)
SI = 4.5  # expected sampling interval (ms)
c_SR = True  # set constant or not constant sampling rate
EXTRA_YLIM = 200  # extra amplitude for y-axis limits

# create a color iterable from a color map
color = itertools.cycle(plt.get_cmap("tab10").colors)

## Plot definiton
fig = plt.gcf()
plt.ion()  # not sure if necessary, also works without
plt.style.use("ggplot")  # use ggplot style
# prepare the lines to plot with their labels and colors
for i in range(CHANNELS):
    c = next(color)  # get next color from the palette (color map)
    plt.plot([], [], color=c, linewidth=0.6, linestyle="-", label=f"ch-{i + 1}")
ax = plt.gca()  # get axis
box = ax.get_position()  # box position to shrink axis
lines = [line for line in ax.lines]  # unpack the lines
# set visible y-axis limits
ax.set_ylim(-2000, 2000)  # can also be updated according to min/max amplitude
plt.tight_layout()
plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))  # legend position
plt.xlabel("Time (s)")
plt.ylabel("Voltage (μV)")
fig.show()

# dict to organize the data to plot into lines
stored = {f"ch-{i + 1}": {"series": []} for i in range(CHANNELS)}
stored["time"] = []
"""
Example:
    stored = {
        "time": [timestamps]
        "ch-1": { "series": [ch-1 amplitudes] }
        "ch-2": { "series": [ch-2 amplitudes] }
        ...
    }
"""

# vars to manipulate while plotting
c_time = 0
first = 0

# TODO: improve autoscale_view and relim to have fixed grid (or set xlim manually)
# TODO: separate storing than plotting data, start plotting when there's enoug data and plot @ 25-30FPS (e.g. store data dict by dict received, plot 25-30 times a second)
def update_plot(data, sr, constant=False):
    global first, lines, c_time
    # shrink current axis by 5% to place legend out of plot box
    ax.set_position([box.x0, box.y0, box.width * 0.95, box.height])
    # to store all amplitudes to check for min and max afterwards
    y_all = []
    # organize the data to plot a line per channel
    for ddict in data:  # each dict from the data stream
        if not constant:
            timestamp = ddict["time"] / 1000  # get timestamp
            if first == 0:  # first timestamp
                first = timestamp  # store it for time reference
            # calculate time relative to first timestamp (so starting at 0)
            time = timestamp - first
        else:
            # assuming constant sampling interval
            c_time += SI / 1000
            time = c_time
        stored["time"].append(time)  # add time
        for i in range(CHANNELS):  # each channel
            key = str(i + 1)  # as keys on eeg are 1-8
            series = ddict[key]  # eeg amplitude value
            stored[f"ch-{key}"]["series"].append(series)  # add eeg amplitude
            y_all.append(series)

    # visible amplitude (y-axis), for automatic adjusting
    # ymin = min(y_all)
    # ymax = max(y_all)

    times = stored["time"]  # all time values
    # select indices from the last N seconds (PLOT_MEMO)
    selected = [i for i, t in enumerate(times) if t >= time - PLOT_MEMO]
    times = times[selected[0] : selected[-1]]
    for i in range(CHANNELS):
        key = f"ch-{i+1}"
        # selected amplitude values
        series = stored[key]["series"][selected[0] : selected[-1]]
        lines[i].set_data(times, series)  # update lines time-series data

    plt.title(f"Traumschreiber EEG streaming @ {sr} Hz")
    # recompute the ax.dataLim
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()


# HOST = '127.0.0.1'  # standard loopback interface address (localhost)
# HOST = socket.gethostname()  # accept all connections from the outside world
HOST = ""  # accept all connections
PORT = 65432  # port to listen on (non-privileged ports are > 1023)

ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    # reuse address:port if on use
    # otherwise process needs to be killed if socket wasn't closed
    ss.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ss.bind((HOST, PORT))  # associate the socket with an address and port
except socket.error as e:
    print(str(e))

print("Waiting for a Connection..")
ss.listen(5)  # listening socket, ready to accept connections

c, address = ss.accept()  # wait for incoming connections
IP = address[0]  # socket sender IP
PORT = address[1]  # socket sender port
print(f"Connected to {IP}:{PORT}")

# vars to handle the receiving data loop
start = False
end = True
chunks = ""
eeg = []
no_data = 0  # no incoming data counter
# starting time reference for elapsed time and sampling rate calculations
t_start = None
pkg = 0
while True:  # loop for handling incoming data
    if not t_start:  # reference timestamp not set
        t_start = time.time()  # reference timestamp (ms)
    t_now = time.time()  # timestamp (ms)
    # calculate elapsed time
    elapsed = t_now - t_start
    # print(elapsed)
    # read and decode incoming data (buffer of 1 byte)
    response = c.recv(1).decode("utf-8")
    if len(response) <= 0:  # no incoming data
        no_data += 1  # count incoming data
        if no_data > 10000:  # sending socket stops sending
            break  # break loop
    elif not start:
        if response == "{":  # stringified JSON dict start
            chunks += response  # add chunk
            start = True  # dict start found flag
            end = False  # look for the rest flag
            no_data = 0  # reset no incoming data counter
    elif not end:
        if response != "}":  # stringified JSON dict end
            chunks += response
        else:
            chunks += response
            end = True  # dict start found flag
            start = False  # look for the start flag
            # remove ending breaklines, remove starting and ending whitespaces
            message = chunks.rstrip("\n").strip()
            chunks = ""  # clear accumulated chunks
            current = json.loads(message)  # parse stringified JSON as dict
            eeg.append(current)  # add current read package for plotting
            pkg += 1
            sampling_rate = int(pkg / elapsed)  # calculate sampling rate 
            if len(eeg) > 10:  # update plot every 10 packages
                update_plot(eeg, sampling_rate, c_SR)
                eeg.clear()  # clean incoming stored data
            # print(current)  # current received data {pkg, time, ch1-8}
            no_data = 0

ss.close()  # disconnect listening socket
print("Socket closed")
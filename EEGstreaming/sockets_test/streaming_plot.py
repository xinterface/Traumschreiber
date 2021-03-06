import itertools
import json
import matplotlib.pyplot as plt
import time
import asyncio
import websockets

# constants
CHANNELS = 8  # number of eeg channels
PLOT_MEMO = 0.5  # plot memory in seconds
SR = 222  # expected sampling rate/frequency (Hz): N/elapsed_time
SI = 4.5  # expected sampling interval (ms): 1/SR
C_SR = True  # set constant or not constant sampling rate

# color iterable from a color map
color = itertools.cycle(plt.get_cmap("tab10").colors)

# Plot definiton
fig = plt.gcf()
plt.ion()  # not sure if necessary, also works without
plt.style.use("ggplot")  # use ggplot style

# prepare the lines to plot with their labels and colors
for i in range(CHANNELS):
    c = next(color)  # get next color from the palette (color map)
    plt.plot([], [], color=c, linewidth=0.6, linestyle="-", label=f"ch{i + 1}")

ax = plt.gca()  # get axis
box = ax.get_position()  # box position to shrink axis
lines = [line for line in ax.lines]  # unpack the lines
# manually set visible y-axis limits
ax.set_ylim(-2000, 2000)  # can also be updated according to min/max amplitude
plt.tight_layout()
plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))  # legend position
plt.xlabel("Time (s)")
plt.ylabel("Voltage (μV)")
fig.show()

# dict to organize the data to plot into lines
eeg = {f"ch-{i + 1}": {"series": []} for i in range(CHANNELS)}
eeg["time"] = []
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

"""TODO:
- Improve autoscale_view and relim to have fixed grid (or set xlim manually)
- Separate storing than plotting data, start plotting when there's enough
data and plot @ 25-30FPS (e.g. store data dict by dict received, plot 25-30
times a second)
"""


def update_lines():
    global lines

    times = eeg["time"]  # all time values
    # select indices from the last N seconds (PLOT_MEMO)
    # selected = [i for i, t in enumerate(times) if t >= times[-1] - PLOT_MEMO]
    # times = times[selected[0] : selected[-1]]
    for i in range(CHANNELS):
        key = f"ch-{i+1}"
        # selected amplitude values
        # series = eeg[key]["series"][selected[0] : selected[-1]]
        series = eeg[key]["series"]
        lines[i].set_data(times, series)  # update lines time-series data

    # shrink current axis by 5% to place legend out of plot box
    ax.set_position([box.x0, box.y0, box.width * 0.95, box.height])
    # ax.set_ylim(ymin - EXTRA_YLIM, ymax + EXTRA_YLIM)
    plt.title(f"Traumschreiber EEG streaming @ {sampling_rate} Hz")
    # recompute the ax.dataLim
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view(True, True, True)
    fig.canvas.draw()
    fig.canvas.flush_events()


def store_data(ddict, constant=False):
    global first, c_time
    # organize the data to plot a line per channel
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
    eeg["time"].append(time)  # add time
    for i in range(CHANNELS):  # each channel
        key = str(i + 1)  # as keys on eeg are 1-8
        series = ddict[key]  # eeg amplitude value
        eeg[f"ch-{key}"]["series"].append(series)  # add eeg amplitude
    # call update plot when packages >= 111 (expected samples in 0.5s)
    if len(eeg["time"]) >= SR / 2:
        update_lines()


# HOST = '127.0.0.1'  # standard loopback interface address (localhost)
HOST = ""  # accept all connections
PORT = 65432  # port to listen on (non-privileged ports are > 1023)
# starting time reference for elapsed time and sampling rate calculations
t_start = None
pkg = 0
sampling_rate = 0  # calculated SR


# websocket server/listener async function
async def ws_server(websocket, path):
    global t_start, pkg, sampling_rate  # vars to update
    async for message in websocket:  # wait to get each message
        if not t_start:  # reference timestamp not set
            t_start = time.time()  # reference timestamp (ms)
        t_now = time.time()  # timestamp (ms)
        elapsed = t_now - t_start  # calculate elapsed time
        current = json.loads(message)  # parse stringified JSON as dict
        store_data(current, C_SR)  # add current read package for plotting
        pkg += 1  # update number of messages
        if elapsed >= 1:  # each second
            print(f"Elapsed: {elapsed} - Packages: {pkg}")
            sampling_rate = int(pkg / elapsed)  # calculate sampling rate
            pkg = 0  # restart messages counter
            t_start = time.time()  # restart time reference


# listen for incoming websocket connections for a specific host and port
start_server = websockets.serve(ws_server, HOST, PORT)
# make asyncronous function run non-stop
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
# await websocket.send(answer)  # for sending an answer or command back

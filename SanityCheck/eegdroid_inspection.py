
'''
The following code was intended for the usage within a jupyter notebook. It provides functions that streamline the process of reading, preprocessing and plotting
csv files that were created by the eegdroid app. 

    
# EXAMPLE USAGES
* Preprocess your EEG data - Read file, calculate regular time stamps, adjust offset, detrend, remove outliers 
data = preprocess_eeg_debugging_data(fn="eeg_data.csv") #(using a small slize parameter speeds up the process)

* Plot some specific channels 
plot_specific_channels(fn="test3_active_gnd.csv",
                       channels=[1,3,15,20],
                       start=45000,
                       end=75000,
                       eeg_labels=True,
                       custom_title="Open/Close Eyes in 30s intervals, Start: Closed, GND: Active")

* Plot a single channel and the spectral analysis right below 
plot_spectrosignal(fn="test3_active_gnd.csv")

* Custom processing before plotting
mydf = preprocess_eeg_debugging_data(fn="eeg_data.csv")
mydf = my_custom_function(mydf)
plot_spectrosignal(data=mydf)

'''


import pandas as pd
import matplotlib.pyplot as plt
%matplotlib notebook
import numpy as np
from scipy.signal import butter,filtfilt
from scipy import signal
from scipy.fft import fftshift
from scipy.signal import welch
from scipy.integrate import simps


channel_numbers = [f"ch{i}" for i in range(1,25)]
channel_names = "OZ,PZ,PO,T2,F8,FP2,F4,FZ,F2,FP1,F1,T1,T3,C3,CZ,C4,T4,T6,P4,PZ,P3,T5,O1,O2".split(",")
channel_labels = dict(zip(channel_numbers,channel_names))


########## FUNCTIONS ########
def preprocess_eeg_debugging_data(fn,slize=None, ch="", specific_channels=None, pkg_interval=6, detrend=True):
    df = pd.read_csv("data/" + fn, skiprows=2)
    df["fixed_time"] = df.index*pkg_interval
    df.set_index(df["fixed_time"])
    if(slize): df = df[slize[0]:slize[1]] 
        
    if (ch):
        df[ch] = signal.detrend(df[ch],type="constant") # Offset
        if (detrend): df[ch] = signal.detrend(df[ch])   # Remove trend
        df[ch] = df[ch].apply(lambda x: df[ch].mean() if(abs(x) > 5*df[ch].std()) else x) # Outlier removal
        
    elif (specific_channels):
        for i in specific_channels:
            ch = f"ch{i}"
            df[ch] = signal.detrend(df[ch],type="constant") # Offset
            if(detrend) df[ch] = signal.detrend(df[ch])   # Remove trend
            df[ch] = df[ch].apply(lambda x: df[ch].mean() if(abs(x) > 5*df[ch].std()) else x) # Outlier removal
    else:
        for i in range(1,25):
            ch = f"ch{i}"
            df[ch] = signal.detrend(df[ch],type="constant") # Offset
            if(detrend) df[ch] = signal.detrend(df[ch])   # Remove trend
            df[ch] = df[ch].apply(lambda x: df[ch].mean() if(abs(x) > 5*df[ch].std()) else x) # Outlier removal

    return df
    
def plot_eeg_debugging_data(fn=None,interval=6,ax=None, ylabel="muV",custom_title=None,
                            time="fixed_time",ch="ch1",slize=None,
                            params={"enc":False,
                                    "enc_f":False,
                                    "pkg_loss":False,
                                    "title":True,
                                    "legend":True,
                                    "normalize":False},eeg_labels=False, data=None):
    
    if (ax==None):
        fig, ax = plt.subplots(1,1, figsize=(9,5))
    
    if fn: data = preprocess_eeg_debugging_data(fn,interval,ch=ch,slize=slize)
    
    if(params["normalize"]):
        data[f"{ch}N"] = data[f"{ch}"]
        sd = str(round(data[f"{ch}"].std(),1))
        data[f"{ch}N"] /= data[f"{ch}"].std()
        ylabel = f"[{sd} muV]"
        data.plot(y=f"{ch}N",x=time, ax=ax, label=f"{ch} normalized")
    else:
        if eeg_labels: legend_label=channel_labels[ch]
        data.plot(y=f"{ch}",x=time, ax=ax, label=legend_label)
        
    if params["enc"]: data.plot(y="enc_ch1", x=time, ax=ax, lw="2", label=f"Bitshift 1")
    if params["enc_f"]: data.plot(y="enc_flag", x=time, ax=ax, lw="2", label=f"Encoding Flag")
    if params["pkg_loss"]: data.plot(y="pkg_loss",x=time, ax=ax, lw="2", label="Pkg Loss")

    if params["title"]: 
        if fn: ax.set_title(fn)
        else: ax.set_title(custom_title)
        
    ax.set_xlabel("ms")
    ax.set_ylabel(ylabel)
    if params["legend"]: ax.legend()
        
def plot_specific_channels(fn, channels, start,end, eeg_labels=True,custom_title=None):
    t_start = start
    t_end = end
    a = int(t_start/6)
    b = int(t_end/6)
    channels_to_plot = channels

    file = fn
    data = preprocess_eeg_debugging_data(file, specific_channels=channels_to_plot, slize=[a,b] )

    nChannels = len(channels_to_plot)
    fig, ax = plt.subplots(1, 1, figsize=(10,2*nChannels))
    for i,n in enumerate(channels_to_plot):
        data[f"ch{n}"] += data[f"ch{n}"].std() * i * 10
        plot_eeg_debugging_data(data=data, ch=f"ch{n}", eeg_labels=eeg_labels, ax=ax, custom_title=custom_title)

        
def butter_lowpass_filter(data, cutoff, fs, order,nyq):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def plot_buttersworth(fn, cutoff,ax, title="Traumschreiber", filtered ="Unfiltered", ch="ch1",a=1200,b=1300, data=None):
    if fn: data = preprocess_eeg_debugging_data(fn,6)
    # Filter requirements.
    T = 5.0         # Sample Period
    fs = 167       # sample rate, Hz
    cutoff = cutoff    # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 2       # sin wave can be approx represented as quadratic
    n = int(T * fs) # total number of samples

    y = butter_lowpass_filter(data[ch], cutoff, fs, order,nyq)
    y_slice = y[a:b]
    data_slice = data[a:b]

    data_slice.plot(y=ch,x="f_time",ax=ax, label=filtered)
    ax.plot(data_slice["f_time"],y_slice, lw=4, label=f"Filtered Fq:{cutoff}Hz")
    plt.gca().set_title(title)
    ax.legend()
    
def plot_spectrogram(fn,ch="ch1", ax=None, title=True, slize=None, data=None):
    if fn: data = preprocess_eeg_debugging_data(fn,6,ch, slize=slize)
    x = data[ch]
    f, t, Sxx = signal.spectrogram(x, 167)
    
    if ax==None:
        fig,ax = plt.subplots(1, figsize=(9,5))
        
    ax.pcolormesh(t, f, Sxx)
    ax.set_ylabel('Frequency [Hz]')
    ax.set_xlabel('Time [sec]')
    if title:
        ax.set_title("Spectrogram of "+ fn)
    plt.show()
    
def plot_spectrogram_cbar(fn=None,ch="ch1", ax=None, title=True, slize=None, data=None):
    if fn: data = preprocess_eeg_debugging_data(fn,6,ch, slize=slize)
    x = data[ch]
    f, t, Sxx = signal.spectrogram(x, 167)
    
    if ax==None:
        fig,ax = plt.subplots(1, figsize=(9,5))
        
    plt.pcolormesh(t, f, Sxx)
    
    plt.clim(np.min(Sxx),np.max(Sxx))
    plt.colorbar()
    plt.gca().set_ylabel('Frequency [Hz]')
    plt.gca().set_xlabel('Time [sec]')
    if title:
        plt.gca().set_title("Spectrogram of "+ fn)
    plt.show()
    
def compute_bandpower(data, sf, band, window_sec=None, relative=False):
    """Compute the average power of the signal x in a specific frequency band.

    Parameters
    ----------
    data : 1d-array
        Input signal in the time-domain.
    sf : float
        Sampling frequency of the data.
    band : list
        Lower and upper frequencies of the band of interest.
    window_sec : float
        Length of each window in seconds.
        If None, window_sec = (1 / min(band)) * 2
    relative : boolean
        If True, return the relative power (= divided by the total power of the signal).
        If False (default), return the absolute power.

    Return
    ------
    bp : float
        Absolute or relative band power.
        
    Source
    -------
    https://raphaelvallat.com/bandpower.html
    
    Required imports
    -------
    scipy.signal.welch
    scipy.integrate.simps
    """
    
    band = np.asarray(band)
    low, high = band

    # Define window length
    if window_sec is not None:
        nperseg = window_sec * sf
    else:
        nperseg = (2 / low) * sf

    # Compute the modified periodogram (Welch)
    freqs, psd = welch(data, sf, nperseg=nperseg)

    # Frequency resolution
    freq_res = freqs[1] - freqs[0]

    # Find closest indices of band in frequency vector
    idx_band = np.logical_and(freqs >= low, freqs <= high)

    # Integral approximation of the spectrum using Simpson's rule.
    bp = simps(psd[idx_band], dx=freq_res)

    if relative:
        bp /= simps(psd, dx=freq_res)
    return bp
    
def plot_bandpower_changes(fn=None, ch="ch1", band=[1,20], sf=167, slize=None, figsize=(9,6)):
    fig,ax= plt.subplots(1,1, figsize=figsize)
    
    if fn: data = preprocess_eeg_debugging_data(fn, ch=ch, slize=slize, detrend=True)
    data = data["ch1"]
    
    
    #Calculate a window large enough to ensure that we can compute the bandpower of our lowest frquency
    window_size = int(167*2)
    
    bandpower_changes = []
    time = np.arange(0,len(data)*6,window_size*6)
    for i in range(0, len(data)-window_size, window_size):
        bandpower = compute_bandpower(data[i:i+window_size], sf, band)
        bandpower_changes.append(bandpower)
    
    ax.plot(time[:-1], bandpower_changes)
    ax.set_ylabel("Bandpower")
    ax.set_xlabel("Time [ms]")
    ax.set_title("alpha band changes for " + fn)
    
    
def plot_spectrosignal(fn=None,ch="ch1", slize=None, figsize=(9,6), data=None):
    fig, ax = plt.subplots(2,1, figsize=figsize)
    plot_eeg_debugging_data(fn,6,ch=ch,ax=ax[0],slize=slize,data=data)
    plot_spectrogram(fn,ch=ch, ax=ax[1], title=False, slize=slize, data=data)
    

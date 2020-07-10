import requests
import random as rnd
import datetime, time
import re

import matplotlib.pyplot as plt
import numpy as np

now = datetime.datetime.now()
readfile = 'bsp.txt'
filename = "Data/" + now.strftime('%Y%m%d-%Hh%M') + "EEGrecording.csv"

time_to_record = 60#2*60
time_step = 0.004
channels = 6
data_rows = channels + 1
viewing_window = 2*250
channel_name = ["","","","","","","",""]

data = np.zeros(data_rows)


p = re.compile("INFO Attribute value changed, handle: 0x10, value \(0x\): ([0-9A-F]{2}-){19}([0-9A-F]{2})")


#we use the local copy of the 'x'-band-list
with open(readfile, 'r') as file_in:

	with open(filename, 'w') as writeFile:
		
		writeFile.write("time" + ",")
		for a in range(channels):
			writeFile.write(channel_name[a] + ",")
		writeFile.write("\n")
		
		current_time_step = 0.
		
		for line in file_in:
		
			#print(line)
			match = p.search(line)
			if match == None:
				continue
			line = match.group() #needed, because there might be corrupted lines sometimes
		
			#print('-')
			new_data = np.zeros(data_rows)
			val_arr = line.replace("INFO Attribute value changed, handle: 0x10, value (0x): ", "" ).split('-')


			writeFile.write(str(current_time_step) + ",")
			writeFile.write(str(time.time()) + ",")
			new_data[0] = current_time_step
			
			a = 1
			for i in range(1,1+channels*3,3):
				sample = int(val_arr[i],16)*256*256 + int(val_arr[i+1],16)*256 + int(val_arr[i+2],16)
				if int(val_arr[i],16) >= 128:
					sample -= 256*256*256
	
				writeFile.write(str(sample) + ",")
				new_data[a] = sample
				a += 1
			
			data = np.vstack((data,new_data))
			writeFile.write("\n")
			current_time_step += time_step
		
		#print("data reciev "+ str(len(data)))

plt.ion()
fig = plt.figure()
ax = [None]*channels
graphs = [None]*channels
for a in range(channels):
	ax[a] = fig.add_subplot(channels,1,a+1)
	graphs[a], = ax[a].plot(data[:,0],data[:,a])
	ax[a].set_ylabel("Ch" + str(a) + " - " + channel_name[a],rotation=0)
	ax[a].yaxis.set_label_coords(1.1,0.5)
fig.canvas.draw()
fig.canvas.flush_events()
plt.show()
				
input("press Enter:")				
		
		

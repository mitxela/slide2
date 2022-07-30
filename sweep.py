#!/bin/env python3

from cal import w, setpos, off
from time import sleep

import pyaudio
import numpy as np
import matplotlib.pyplot as plt

CHUNK = 8192
FORMAT = pyaudio.paFloat32
CHANNELS = 2
RATE = 48000
RECORD_SECONDS = 1

p = pyaudio.PyAudio()

info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
devID = -1

for i in range(0, numdevices):
  if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
    print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))
    if p.get_device_info_by_host_api_device_index(0, i).get('name') == "RØDE VideoMic NTG: USB Audio (hw:1,0)" or p.get_device_info_by_host_api_device_index(0, i).get('name') == "Desktop Microphone (RÃ˜DE VideoM":
      devID = i

if devID==-1:
  print("Can't find device")
  exit()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=devID,
                frames_per_buffer=CHUNK)

print("* recording")

def pitch(d):
  dc = sum(d)/len(d)
  d = d - dc

  #[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1]
  t = np.linspace(-10,10,30)
  gauss = np.exp(-0.1*t**2)
  gauss /= np.trapz(gauss)
  # plt.plot(gauss)
  # plt.show()

  d=np.convolve(d, gauss)

  #plt.plot(d)
  #plt.show()

  sign = np.sign(d[0])

  crossings=[]
  for i,s in enumerate(d):
    if (np.sign(s)!=sign):
      crossings.append(i)
      sign = np.sign(s)

  periods = np.diff(crossings)
  return RATE*len(periods)/sum(periods)


def mic_pitch():
  data=[]
  for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
      data = np.append(data,np.frombuffer(stream.read(CHUNK, exception_on_overflow=False), dtype=np.float32))

  return pitch(data)

stream.read(CHUNK, exception_on_overflow=False) # first chunk is junk



def delaySec(sec):
  len = round(sec*RATE/CHUNK)
  for i in range(len):
    stream.read(CHUNK, exception_on_overflow=False)

whistle=3

setpos(whistle,0,1000)
delaySec(0.5)
setpos(whistle,0)
delaySec(4)
print("* ready")
#print(mic_pitch())



forwards={}
backwards={}

step=10

print("Forwards")
for i in range(0, 250+step,step):
  setpos(whistle,i)
  delaySec(1)
  forwards[i]=mic_pitch()
  print(i, forwards[i])

print("Backwards")
for i in range(250, -step,-step):
  setpos(whistle,i)
  delaySec(1)
  backwards[i]=mic_pitch()
  print(i, backwards[i])

off()

x=[]
y=[]
print("Position Forwards Backwards Average")
for i in range(0, 250+step,step):
  a = (forwards[i]+backwards[i])/2.0
  print(i, forwards[i], backwards[i], a)
  x.append(i)
  y.append(a)


print("\nx =",x)
print("y =",y)


stream.stop_stream()
stream.close()
p.terminate()

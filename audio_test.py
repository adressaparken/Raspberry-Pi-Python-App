import pyaudio
import numpy as np
import audioop
import math

#CHUNK = 2**11
CHUNK = 4096
RATE = 44100

print (CHUNK)

p=pyaudio.PyAudio()
stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
              frames_per_buffer=CHUNK)

for i in range(int(10*44100/1024)): #go for a few seconds
    data = np.fromstring(stream.read(CHUNK),dtype=np.int16)
    rms = audioop.rms(data, 2)
    decibel = 20 * math.log10(rms)
    print(decibel)

stream.stop_stream()
stream.close()
p.terminate()

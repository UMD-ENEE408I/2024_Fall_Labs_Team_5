import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from scipy.signal import spectrogram

duration = 5  # seconds
fs = 44100  # sample rate

# record audio
print("Recording...")
audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float64')
sd.wait()  # wait until recording is done
print("Recording complete.")

# spectrogram time
frequencies, times, Sxx = spectrogram(audio.flatten(), fs)

# spectrogram reveal
plt.figure(figsize=(10, 6))
plt.pcolormesh(times, frequencies, 10 * np.log10(Sxx), shading='gouraud')
plt.colorbar(label='Intensity [dB]')
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [s]')
plt.title('Spectrogram of Recorded Audio')
plt.ylim(0, 20000)  # 20 kHz limit
plt.show()


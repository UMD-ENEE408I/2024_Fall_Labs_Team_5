[raw_signal, Fs] = audioread('Cafe_with_noise.wav'); 
n = length(raw_signal);
%%
sound(raw_signal, Fs)
%%
 window = hamming(n);

y = raw_signal.*window;
Y = fft(y); 
f = (0:n-1)*(Fs/n);
%%
% clean the signal to plot it and get the positive real part blah 
Y_magnitude = abs(Y/n); 
half_n = ceil(n/2);
f_half = f(1:half_n);
Y_half = Y_magnitude(1:half_n);
%%
% plot the FFT
figure(1);
semilogx(f_half, Y_half);
%plot(f_half, Y_half);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('FFT of the Signal');
%%
figure(2);
[S, F, T, P] = spectrogram(y, 1024, [], [], Fs, 'yaxis'); 
title('Spectrogram of the Signal');
%%
S_magnitude = abs(S);
max_value = max(S_magnitude(:))
[max_val_row, max_val_col] = find(S_magnitude == max_value)
F(33)
%%

%%
% low pass filter but it sucks so we used a bandstop filter instead
Fc = 600; 


lowpass_filter = designfilt('lowpassiir', 'FilterOrder', 4, ...
                            'HalfPowerFrequency', Fc, 'SampleRate', Fs);


final_signal_after_lowpass = filtfilt(lowpass_filter, raw_signal);
%%
%check the bandstop filter !!!
bandstop_filter = designfilt('bandstopiir', 'FilterOrder', 4, ...
                          'HalfPowerFrequency1', 1300, 'HalfPowerFrequency2', 1700, ...
                          'SampleRate', Fs);

final_signal = filtfilt(bandstop_filter, raw_signal);
final_signal_bandstop = real(final_signal);

%%

sound(final_signal_bandstop, Fs)
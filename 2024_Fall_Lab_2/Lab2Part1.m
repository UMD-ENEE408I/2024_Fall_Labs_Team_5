clc;
clear;

%% Human Voice

% Original File

[y, Fs] = audioread("Sound_Files/human_voice.wav");
t = linspace(0, length(y)/Fs, length(y))';
figure(1);
plot(t, y)
title("Original Human Voice Signal")
xlabel("Time, seconds")
ylabel("Frequency, Hz")

% Resample

Fs_new = 8000;
[Numer, Denom] = rat(Fs_new/Fs);
y_new = resample(y, Numer, Denom);
t_new = linspace(0, length(y_new)/Fs_new, length(y_new))';
figure(3);
plot(t_new, y_new)
title("Resampled Human Voice Signal")
xlabel("Time, seconds")
ylabel("Frequency, Hz")

% Comparison

figure(4);
subplot(1,2,1);
plot(t(24000:30000), y(24000:30000))
title("Original Audio")
xlabel("Time, seconds")
ylabel("Frequency, Hz")

subplot(1,2,2);
plot(t_new(4000:5000), y_new(4000:5000))
title("Resampled Audio")
xlabel("Time, seconds")
ylabel("Frequency, Hz")
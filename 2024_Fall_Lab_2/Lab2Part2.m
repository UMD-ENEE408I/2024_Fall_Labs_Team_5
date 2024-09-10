% Load the .wav file
[m1, fs_m1] = audioread('Sound_Files/M1.wav');

m1_rms = sqrt(mean(m1.^2));

disp(['m1 RMS Value: ', num2str(m1_rms)]);


%%%rms2
[m2, fs_m2] = audioread('Sound_Files/M2.wav');

m2_rms = sqrt(mean(m2.^2));

disp(['m2 RMS Value: ', num2str(m2_rms)]);


%%%rms1
[m3, fs_m3] = audioread('Sound_Files/M3.wav');

m3_rms = sqrt(mean(m3.^2));

disp(['m3 RMS Value: ', num2str(m3_rms)]);


% make signals the same length
N = length(m1);
m2 = m2(1:N);  

% initialize vector
corr_xy = zeros(1, 2*N-1);


% loop
for m = -(N-1):(N-1)
    sum_corr = 0;
    for n = 1:N
        if (n - m > 0) && (n - m <= N)
            sum_corr = sum_corr + m2(n) * m1(n - m);
        end
    end
    corr_xy(m + N) = sum_corr;
end

lags = -(N-1):(N-1);


% plot correlation
figure;
stem(lags, corr_xy);
title('Cross-Correlation of m1[n] and m2[n]');
xlabel('Lag (samples)');
ylabel('Cross-Correlation');

% calculate time delay from number of samples delay
[~, maxIndex] = max(abs(corr_xy));
timeDelay = lags(maxIndex);

disp(['Time delay between the signals: ', num2str(timeDelay), ' samples and ', num2str(timeDelay/fs_m1), ' seconds.']);
% @author Matteo Scordino
% @date 2018-08-04
% @version 1.0.0
% 
% @brief M file to design a lowpass Butterworth filter and get the coefficients for CMSIS DSP
% 

function coeffs= design_iir_lowpass_cmsis_butter(requested_order, f1, fs, hold_on_flag)
order = requested_order;
% Nyquist frequency, for convenience
fNyquist=fs/2;
% design filter in zeros, poles, gain format
[z,p,k] = butter(order, f1/fNyquist, 'low');
% convert it to the second order sections (used for biquads) format
[sos] = zp2sos(z,p,k);

% compute biquad coefficients
coeffs = sos(:,[1 2 3 5 6]);
% negate a1 and a2 coeffs (CMSIS expects them negated)
coeffs(:,4) = -coeffs(:,4);
coeffs(:,5) = -coeffs(:,5);

% make a linear array of it
coeffs = coeffs';
coeffs = coeffs(:);


[b,a] = butter(order,f1/fNyquist, 'low');
[h, w] = freqz (b,a);
figure(1)
if hold_on_flag==1
    hold on;
end
plot (w./pi*fNyquist, 20*log10 (abs (h)))
xlabel ("Frequency");
ylabel ("abs(H[w])[dB]");
% hold on
% plot(gca,[fs/2 fs/2],[-100 0],'-r')

set(gca,'Ylim',[-100 0])
grid on	

% print the coefficients as expected by CMSIS
coeffs

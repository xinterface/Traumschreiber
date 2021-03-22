% @author Matteo Scordino
% @date 2018-08-04
% @version 1.0.0
% 
% @brief M file to design a highpass Butterworth filter and get the coefficients for CMSIS DSP
% 

function coeffs= design_iir_highpass_cmsis_butter(requested_order, f1, fs,hold_on_flag)
order = requested_order;
% Nyquist frequency, for convenience
fNyquist=fs/2;
% design filter in zeros, poles, gain format
[z,p,k] = butter(order, f1/fNyquist, 'high');
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

plot_results = true
if (plot_results == true)
	%plot the frequency response, just for human reference
	% (we need to redesign the filter in b,a format)
	[b,a] = butter(order,f1/fNyquist, 'high');
    [h, w] = freqz (b,a,2000);
    figure(1)
    if hold_on_flag==1
        hold on;
    end
    plot (w./pi*fNyquist, 20*log10 (abs (h)))

	xlabel ("Frequency");
	ylabel ("abs(H[w])[dB]");
	
	grid on
    %axis ([0, 1, -40, 0]);
	%hold ("on");
	%x=ones (1, length (h));
	%hold ("off");
    set(gca,'Ylim',[-100 0])
    set(gca,'Xlim',[0 round(f1*5)])
end

% print the coefficients as expected by CMSIS
coeffs

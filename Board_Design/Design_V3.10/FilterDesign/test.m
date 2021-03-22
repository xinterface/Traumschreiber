%///////////////////////////////////////////////////////////////
% Example 2
% Elliptical bandpass between 150 and 600 Hz, 1dB ripple, 60dB
% stopband attenuation
%///////////////////////////////////////////////////////////////
dbstop if error

order=12
f1=1
f2=40
fs=500
plot_results=true

%design_iir_bandpass_cmsis_elliptical(order,ripple,att,f1,f2,fs,plot_results);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%///////////////////////////////////////////////////////////////
% Example 3
%& Same as frequencies sd example 2, but designed as Butterworth 
% filter
%///////////////////////////////////////////////////////////////



design_iir_bandpass_cmsis_butter(order,f1,f2,fs,plot_results);
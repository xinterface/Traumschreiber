
Order =6;
Bandwidth= 40;
Samplefrequeency = 500;
Band1 = (Samplefrequeency/6)-Bandwidth;



coeffs= design_iir_lowpass_cmsis_butter(Order, Band1, Samplefrequeency, true)

savename= ['FolterCoeff_LowPass_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1)  '_O' num2str(Order) ];
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');


Order =6;
f1   = 1;
Samplefrequeency = 500;

coeffs= design_iir_highpass_cmsis_butter(Order, f1, Samplefrequeency )

savename= ['FolterCoeff_HighPass_fs' num2str(round(Samplefrequeency)) '_L' num2str(f1)  '_O' num2str(Order) ];
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
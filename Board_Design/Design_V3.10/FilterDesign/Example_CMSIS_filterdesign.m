%see https://github.com/matteoscordino/iir-designer-cmsis-dsp

dbstop if error 
Order =6;
Bandwidth= 4;
Band1 = 50-Bandwidth;
Band2 = 50+Bandwidth; 
Samplefrequeency = 500;

coeffs= design_iir_bandstop_cmsis_butter(Order, Band1, Band2, Samplefrequeency, true)

savename= ['FolterCoeff_Notch_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1) '_H' num2str(Band2 ) '_O' num2str(Order) ];
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');








% See the examples file for usage examples. After running each of those examples, you will get the coefficients array in the format CMSIS expects.
% For Order 4 the output has 9 coefficents
% Example  will output:
% 
% coeffs =
% 
%     0.9007
%     0.4943
%     0.9007
%    -0.4241
%    -0.8993
%     1.0000
%     0.5488
%     1.0000
%    -0.6163
%    -0.9022

% 
% That array of coefficients can be directly used in ARM code like the following:
% 
% #define IIR_ORDER     2
% #define IIR_NUMSTAGES (IIR_ORDER/2)
% 
% static float32_t m_biquad_state[IIR_ORDER];
% % static float32_t m_biquad_coeffs[5*IIR_NUMSTAGES] =
% % {
%     0.9007
%     0.4943
%     0.9007
%    -0.4241
%    -0.8993
%     1.0000
%     0.5488
%     1.0000
%    -0.6163
%    -0.9022
% 
% % };
% 
% arm_biquad_cascade_df2T_instance_f32 const iir_inst = 
% {
%   IIR_ORDER/2,
%   m_biquad_state,
%   m_biquad_coeffs
% };
% 
% extern float32_t* pSrc;
% extern float32_t* pDst;
% extern uint16_t blockSize;
% 
% arm_biquad_cascade_df2T_f32(&iir_inst, pSrc, pDst, blockSize);


dbstop if error


Version     = '1_3';
Folder_Name = ['Filters_DreamMachine_V' Version '_' date];

eval('!del *.txt')
eval('!del *.zip')
eval('!del *.mat')
eval('!del *.jpg')
eval('!del *.pdf')
eval('!del *.fig')


mkdir(Folder_Name)

hold_on_flag =1;
formats.Fig =1;
formats.JPG =1;
formats.Res =150;
formats.PDF =0;
formats.eps =0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%
%%%% Low Pass for Antialising I
close all
Order =4;
Samplefrequeency = 500;
Band1 = round(45);

coeffs= design_iir_lowpass_cmsis_butter(Order, Band1, Samplefrequeency, hold_on_flag)
savename= ['FolterCoeff_LowPass_I_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1)  '_O' num2str(Order) ];
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


%%%%%%%%%%%%%%%%%%%%%%
%%%% Low Pass for Antialising II

Order =2;
Samplefrequeency = 500;
Band1 = round(45);

coeffs= design_iir_lowpass_cmsis_butter(Order, Band1, Samplefrequeency, hold_on_flag)
savename= ['FolterCoeff_LowPass_II_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1)  '_O' num2str(Order) ];
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)
legend('Filter 1','Filter 2');
Zip_file_name ='Lowpass_filters.zip';
zip(Zip_file_name,{'*.fig','*.mat','*.jpg','*.txt'});
eval(['!move *.txt ./' Folder_Name ])
eval(['!move *.fig ./' Folder_Name ])
eval(['!move *.jpg ./' Folder_Name ])
eval(['!move *.mat ./' Folder_Name ])
eval(['!move *.zip ./' Folder_Name ])
eval(['!move *.txt ./' Folder_Name ])



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% High Pass for slow Drifts I
close all
Order               = 4;
f1                  = 0.5;
Samplefrequeency    = 500/3;
coeffs              = design_iir_highpass_cmsis_butter(Order, f1, Samplefrequeency,hold_on_flag )
savename= ['FolterCoeff_HighPass_I_fs' num2str(round(Samplefrequeency)) '_L' num2str(f1)  '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


%%%%%%%%%%%%%%%%%%%%%%
%%%% High Pass for slow Drifts II

Order               = 4;
f1                  = 1.3;
Samplefrequeency    = 500/3;
coeffs              = design_iir_highpass_cmsis_butter(Order, f1, Samplefrequeency,hold_on_flag  )
savename= ['FolterCoeff_HighPass_II_fs' num2str(round(Samplefrequeency)) '_L' num2str(f1)  '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)



%%%%%%%%%%%%%%%%%%%%%%
%%%% High Pass for slow Drifts III

Order               = 2;
f1                  = 0.8;
Samplefrequeency    = 500/3;
coeffs              = design_iir_highpass_cmsis_butter(Order, f1, Samplefrequeency,hold_on_flag  )
savename= ['FolterCoeff_HighPass_III_fs' num2str(round(Samplefrequeency)) '_L' num2str(f1)  '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)

%%%%%%%%%%%%%%%%%%%%%%
%%%% High Pass for slow Drifts IV

Order               = 2;
f1                  = 1.7;
Samplefrequeency    = 500/3;
coeffs              = design_iir_highpass_cmsis_butter(Order, f1, Samplefrequeency,hold_on_flag  )
savename= ['FolterCoeff_HighPass_IV_fs' num2str(round(Samplefrequeency)) '_L' num2str(f1)  '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


legend('Filter 1','Filter 2','Filter 3','Filter 4');
Zip_file_name ='Highpass_filters.zip';
zip(Zip_file_name,{'*.fig','*.mat','*.jpg','*.txt'});
eval(['!move *.txt ./' Folder_Name ])
eval(['!move *.fig ./' Folder_Name ])
eval(['!move *.jpg ./' Folder_Name ])
eval(['!move *.mat ./' Folder_Name ])
eval(['!move *.zip ./' Folder_Name ])
eval(['!move *.txt ./' Folder_Name ])

close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Notch for 50 Hz LineNoise

Order =6;
Bandwidth= 4;
Band1 = 50-Bandwidth;
Band2 = 50+Bandwidth; 
Samplefrequeency = 500;

coeffs= design_iir_bandstop_cmsis_butter(Order, Band1, Band2, Samplefrequeency,hold_on_flag )
savename= ['FolterCoeff_Notch_I_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1) '_H' num2str(Band2 ) '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


%%%%%%%%%%%%%%%%%%%%%%
%%%% Notch for 50 Hz LineNoise II

Order =6;
Bandwidth= 2;
Band1 = 50-Bandwidth;
Band2 = 50+Bandwidth; 
Samplefrequeency = 500;

coeffs= design_iir_bandstop_cmsis_butter(Order, Band1, Band2, Samplefrequeency,hold_on_flag )
savename= ['FolterCoeff_Notch_II_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1) '_H' num2str(Band2 ) '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


%%%%%%%%%%%%%%%%%%%%%%
%%%% Notch for 50 Hz LineNoise III

Order =4;
Bandwidth= 4;
Band1 = 50-Bandwidth;
Band2 = 50+Bandwidth; 
Samplefrequeency = 500;

coeffs= design_iir_bandstop_cmsis_butter(Order, Band1, Band2, Samplefrequeency,hold_on_flag )
savename= ['FolterCoeff_Notch_III_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1) '_H' num2str(Band2 ) '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)


%%%%%%%%%%%%%%%%%%%%%%
%%%% Notch for 50 Hz LineNoiseIV

Order =4;
Bandwidth= 2;
Band1 = 50-Bandwidth;
Band2 = 50+Bandwidth; 
Samplefrequeency = 500;

coeffs= design_iir_bandstop_cmsis_butter(Order, Band1, Band2, Samplefrequeency,hold_on_flag )
savename= ['FolterCoeff_Notch_IV_fs' num2str(round(Samplefrequeency)) '_L' num2str(Band1) '_H' num2str(Band2 ) '_O' num2str(Order) ];
savename(findstr(savename,'.'))='_';
save(savename,'coeffs');
coeffs =double(coeffs)
save([savename '.txt'],'coeffs','-ascii','-single');
print_figure(formats,savename)
legend('Filter 1','Filter 2','Filter 3','Filter 4');
Zip_file_name ='LineNoise_50Hz_Noth_filters.zip';
zip(Zip_file_name,{'*.fig','*.mat','*.jpg','*.txt'});

Zip_file_name ='SourceCode.zip';
zip(Zip_file_name,{'*.m'});

eval(['!move *.txt ./' Folder_Name ])
eval(['!move *.fig ./' Folder_Name ])
eval(['!move *.jpg ./' Folder_Name ])
eval(['!move *.mat ./' Folder_Name ])
eval(['!move *.zip ./' Folder_Name ])
eval(['!move *.txt ./' Folder_Name ])

cd(Folder_Name )
Zip_file_name =['All_filter_'  Folder_Name '.zip'];
zip(Zip_file_name,{'High*.zip','Line*.zip','Low*.zip','Sour*.zip'});

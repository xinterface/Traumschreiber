clear all
close all
dbstop if error



Version     = '2c';


Parameter.Name                      = 'JackedSine10Hz_Drift_AmpChange';
Parameter.Traumschreiber_fs         =round(500/3);
Parameter.Function_generator_fs     =160; % !   160 sample per second 
Parameter.HPF                       = 1;
Parameter.Function_generator_Bit    =14;
Parameter.Function_generator_BitS   =8;
Parameter.Signal_Amp                = 0.5;
Parameter.Noise_Amp                 = 0.005*2^(Parameter.Function_generator_Bit-Parameter.Function_generator_BitS);

Folder_Name = [Parameter.Name '_' Version '_' date];

eval('!del *.txt')
eval('!del *.zip')
eval('!del *.mat')
eval('!del *.jpg')
eval('!del *.pdf')
eval('!del *.fig')
eval('!del *.csv')


mkdir(Folder_Name)

hold_on_flag =1;
formats.Fig =1;
formats.JPG =1;
formats.Res =150;
formats.PDF =0;
formats.eps =0;










opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["dataLenth", "VarName2"];
opts.VariableTypes = ["string", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["dataLenth", "VarName2"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["dataLenth", "VarName2"], "EmptyFieldRule", "auto");

% Import the data
Signal_Text = readmatrix("D:\Dropbox\EEG\Function_Generator\Signals_Temp\Cardiac.csv", opts);


opts.VariableNames = ["dataLenth", "VarName2"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
Cardiac1 = readtable("D:\Dropbox\EEG\Function_Generator\Signals_Temp\Cardiac.csv", opts);

%% Convert to output type
Signal_t = table2array(Cardiac1);


Time_vec   = Signal_t(14:end,1);
Signal_vec = Signal_t(14:end,2);
Parameter.Function_generator_samples            = length(Time_vec);
Parameter.Function_generator_signal_length_s    = Parameter.Function_generator_samples./Parameter.Function_generator_fs;


Signal_Jumps.f        = 0.009; 
Signal_Jumps.timevec  = (0:1:(Parameter.Function_generator_samples-1))/Parameter.Function_generator_fs;
Signal_Jumps.signal     = mod(round(Signal_Jumps.timevec*Signal_Jumps.f),5);
Signal_Jumps.signal     = 0.5*Parameter.Noise_Amp*Signal_Jumps.signal./max(Signal_Jumps.signal); 
kernel                  = (0:round(Parameter.Function_generator_fs/Parameter.HPF));
kernel                  = kernel./max(kernel);
kernel                  = sin(pi*kernel );
kernel                  = kernel./sum(kernel);
Signal_Jumps.signal     = conv2(Signal_Jumps.signal,kernel,'same') 

%Sin 10Hz 
Signal_sin.f        = 10;
Signal_sin.Ampmod_f = 0.015;
Signal_sin.timevec  = (0:1:(Parameter.Function_generator_samples-1))/Parameter.Function_generator_fs;
Signal_sin.timevec2 = mod(Signal_sin.timevec*Signal_sin.Ampmod_f,1).^2;
Signal_sin.timevec2 = min([Signal_sin.timevec2 ;  1-Signal_sin.timevec2])
Signal_sin.signal   = Parameter.Signal_Amp * cos((mod(Signal_sin.f *Signal_sin.timevec,1).^2)*2*pi).*Signal_sin.timevec2 ;

Signal.signal       = Signal_sin.signal;%+Signal_Jumps.signal ;

subplot(3,1,1) 
plot(Signal_sin.timevec,Signal_sin.signal);
subplot(3,1,2) 
plot(Signal_sin.timevec,Signal_Jumps.signal);

subplot(3,1,3) 
plot(Signal_sin.timevec,Signal.signal);


temp = Parameter.Function_generator_fs/length(Time_vec);
savename= [Parameter.Name ];

fileID = fopen([savename '.csv' ],'w');
fprintf(fileID,'data lenth,16384\n')
fprintf(fileID,'frequency, %9f\n', temp)
fprintf(fileID,'amp,1.000000000\n')
fprintf(fileID,'offset,0.000000000\n')
fprintf(fileID,'phase,0.000000000\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'\n')
fprintf(fileID,'xpos,value\n')

%bytes = fprintf(fileID,'%7e \n',[Time_vec' ]' )
for IDX=1:length(Time_vec)
    bytes = fprintf(fileID,'%7e,%1.5f\n',[Time_vec( IDX) ; Signal.signal( IDX)]' );
end

%bytes = fprintf(fileID,'%7e,%1.5f\n',[Time_vec' ; Signal_sin.signal]' )
fclose(fileID);


save(savename,'Signal');
print_figure(formats,savename)

Zip_file_name ='Signal_T3AFG5.zip';
zip(Zip_file_name,{'*.fig','*.mat','*.jpg','*.txt','*.m'});
eval(['!move *.txt ./' Folder_Name ])
eval(['!move *.csv ./' Folder_Name ])
eval(['!move *.fig ./' Folder_Name ])
eval(['!move *.jpg ./' Folder_Name ])
eval(['!move *.mat ./' Folder_Name ])
eval(['!move *.zip ./' Folder_Name ])
eval(['!move *.txt ./' Folder_Name ])


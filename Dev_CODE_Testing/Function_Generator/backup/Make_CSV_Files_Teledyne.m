clear all
close all

Parameter.Traumschreiber_fs     =round(500/3);
Parameter.Function_generator_fs     =160; % !60 sample per second 
Parameter.Function_generator_Bit    =14;
Parameter.Function_generator_BitS   =7;
Parameter.Signal_Amp                = 0.005;
Parameter.Noise_Amp                 = 0.005*2^(Parameter.Function_generator_Bit-Parameter.Function_generator_BitS);


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


%Sin 10Hz 
Signal_sin.f        = 10;
Signal_sin.timevec  = (0:1:(Parameter.Function_generator_samples-1))/Parameter.Function_generator_fs;
Signal_sin.signal   = Parameter.Signal_Amp * sin(Signal_sin.timevec*2*pi) ;

subplot(3,2,1) 
plot(Signal_sin.timevec,Signal_sin.signal);

temp = Parameter.Function_generator_fs/length(Time_vec);

fileID = fopen('myfile3.csv','w');
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
    bytes = fprintf(fileID,'%7e,%1.5f\n',[Time_vec( IDX) ; Signal_sin.signal( IDX)]' );
end

%bytes = fprintf(fileID,'%7e,%1.5f\n',[Time_vec' ; Signal_sin.signal]' )
fclose(fileID);


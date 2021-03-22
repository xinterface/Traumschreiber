clear all
close all
cd('D:\Dropbox\EEG\Check2\result of Used_signal')
pairwise_diff_flag =0;


opts = delimitedTextImportOptions("NumVariables", 28);

% Specify range and delimiter
opts.DataLines = [4, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7", "ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14", "ch15", "ch16", "ch17", "ch18", "ch19", "ch20", "ch21", "ch22", "ch23", "ch24", "enc_ch1", "enc_flag", "count"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
%temp   = readtable("D:\Dropbox\EEG\Check2\result of Used_signal\jackedsine10hz_drift_ampchange_2a\12-03-2021_13-40-04_jackedsine10hz_drift_ampchange_2a_HP(O8O2)_LP(L45_O4)_Notch(H54O4).csv", opts);
%temp   = readtable("D:\Dropbox\EEG\Check2\result of Used_signal\drift_ECG_2b\12-03-2021_12-34-11_ECG_2b_HP(O8O2)_LP(L45_O4)_Notch(H54O4).csv", opts);
%temp =  readtable("D:\Dropbox\EEG\Check2\result of Used_signal\Drift_ECG_2a\12-03-2021_13-35-48_ECG_2a_HP(O8O2)_LP(L45_O4)_Notch(H54O4).csv", opts);
%temp  = readtable("D:\Dropbox\EEG\Check2\result of Used_signal\jackedsine10hz_drift_2c\12-03-2021_13-46-55_jackedsine10hz_drift_2c_ampchange_HP(O8O2)_LP(L45_O4)_Notch(H54O4).csv", opts);
%temp = readtable("D:\Dropbox\EEG\check_3\new image_result of Used_signal\New Folder With Items\jackedsin10Hz_drift_ampchange_2c_HP(O8O2)_LP(L45_O4)_Notch(H54O4)_new image\15-03-2021_12-16-16_jackedsin10Hz_drift_ampchange_2c_HP(O8O2)_LP(L45_O4)_Notch(H54O4)_new image.csv", opts);
%temp = readtable("D:\Dropbox\EEG\Check2\result of Used_signal\jackedsine10hz_drift_2b\12-03-2021_13-43-57_jackedsine10hz_drift_2b_ampchange_HP(O8O2)_LP(L45_O4)_Notch(H54O4).csv", opts);
temp = readtable("D:\Dropbox\EEG\check_3\new image_result of Used_signal\New Folder With Items\jackedsin10Hz_drift_ampchange_2a_HP(O8O2)_LP(L45_O4)_Notch(H54O4)_new image\15-03-2021_12-08-53_jackedsin10Hz_drift_ampchange_2a_HP(O8O2)_LP(L45_O4)_Notch(H54O4)_new image.csv", opts);


temp   = table2array(temp);

Signal =   temp;

if pairwise_diff_flag ==1;
    for IDXCH=3:(size(Signal,2)-3)
        Signal(:,IDXCH) = Signal(:,IDXCH-1)+Signal(:,IDXCH); 
    end
end
subplot(3,1,1)
plot(Signal(:,3:end-3))
set(gca,'Xlim',[1 size(Signal,1)])
subplot(3,1,2)
plot(Signal(:,end-2:end-1))
set(gca,'Xlim',[1 size(Signal,1)])

encoding_IDX = find(diff(Signal(:,end-1))==1);
Delta_encoding_IDX = diff(encoding_IDX )
subplot(3,1,3)
plot(Delta_encoding_IDX)
set(gca,'Xlim',[1 length(Delta_encoding_IDX)])
%% Clear temporary variables

clear all
% This code demonstrates sending synchronous read & write commands% to an USB Test & Measurement Class (USBTMC) instrument using% NI-VISA%Create a VISA-USB object connected to a USB instrument 
vu = visa('ni','USB0::0xF4ED::0xEE3A::T0102C19190289::INSTR')%Open the VISA object created 
fopen(vu)%Send the string "*IDN?",asking for the device's identification. 
fprintf(vu,'*IDN?');%Request the data 
%fprintf(vu,'WVDT? user,wave1');%Request the data 

outputbuffer = fscanf(vu); 
disp(outputbuffer);%Close the VISA object 

fprintf (vu, '*RST'); 
fprintf (vu, '*CLS');

fprintf('Generating Waveform...\n\n')

sRate = 100;
ch = []; 
for t = 0:1/sRate:10 
    c = sin(t); 
    y = num2str(c); 
    s5 = sprintf(', %s',y); 
    if t==0
         ch = [ s5]; 
    else
        ch = [ch  s5]; 
    end
end

s = ch(1:100);

arbstring =sprintf('DATA VOLATILE %s', s);
fprintf('Downloading Waveform...\n\n') 
fprintf(vu, arbstring); %make instrument wait for data to download before moving on to next %command set 
fprintf(vu, '*WAI'); 
fprintf('Download Complete\n\n')


fprintf(vu,'VOLT 2'); 
fprintf(vu,'VOLT:OFFSET 4'); 
fprintf(vu,'OUTPUT:LOAD 50'); 
fprintf(vu,'FREQ 1');
fprintf(vu,'DATA:COPY TEST16385, VOLATILE'); 
fprintf(vu,'FUNC:USER VOLATILE'); 
fprintf(vu,'FUNC:SHAP USER');
fprintf(vu,'OUTPUT ON');
fprintf(vu, 'SYST:ERR?'); 
errorstr = fscanf (vu);
fclose(vu);
delete(vu);
clear vu;


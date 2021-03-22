dbstop if error
clear all

clear all


f = fgen;

availableResources  =  f.resources;
MyGnerator_Id       = availableResources{4};
f.Resource          = MyGnerator_Id;
driverlist = drivers(f)

connect(f);

f = fgen(MyGnerator_Id);


myFGen.Resource = MyGnerator_Id ;



% This code demonstrates sending synchronous read & write commands% to an USB Test & Measurement Class (USBTMC) instrument using% NI-VISA%Create a VISA-USB object connected to a USB instrument 
vu = visa('ni','USB0::0xF4ED::0xEE3A::T0102C19190289::INSTR')%Open the VISA object created 
fopen(vu);%Send the string "*IDN?",asking for the device's identification. 
fprintf(vu,'*IDN?');%Request the data 
outputbuffer = fscanf(vu); 
disp(outputbuffer);%Close the VISA object 
fclose(vu);
delete(vu);
clear vu;


%% Testing the NI Board
clearvars; close all; clc;

%% Vars
N = 1024 * 2; % Number of samples taken

%% Add Output
d = daq("ni");
addoutput(d,"Dev1","ao0","Voltage");
d.Rate = 1e+06;

amplitudePeakToPeak_ch1 = 1; % pk voltage

sineFrequency = 10e3; % 1e3 Hz
sineFrequency2 = 30e3;
sineFrequency3 = 50e3;
duration = 0.75; % length of the signal in seconds

outputSignal = [];
outputSignal(:,1) = createSine(amplitudePeakToPeak_ch1/2, sineFrequency, sineFrequency2, sineFrequency3, d.Rate, "bipolar", duration);
t = (0:(N-1)) / d.Rate;
figure;
plot(t, outputSignal(1:N))
title('Generated Output')
source = outputSignal(1:N);

preload(d,outputSignal);
start(d,"repeatoutput")

%% Add Inputs
dd = daq("ni");
dd.Rate = 1e6;
%{
ch1 = addinput(dd,"Dev1","ai0","Voltage");
ch1.TerminalConfig = "SingleEnded";
ch1.Range = [-1 1];
%}
ch2 = addinput(dd,"Dev1","ai1","Voltage");
ch2.TerminalConfig = "SingleEnded";
ch2.Range = [-5 5];
%{
ch3 = addinput(dd,"Dev1","ai2","Voltage");
ch3.TerminalConfig = "SingleEnded";
ch3.Range = [-1 1];
ch4 = addinput(dd,"Dev1","ai3","Voltage");
ch4.TerminalConfig = "SingleEnded";
ch4.Range = [-1 1];
ch5 = addinput(dd,"Dev1","ai4","Voltage");
ch5.TerminalConfig = "SingleEnded";
ch5.Range = [-1 1];
ch6 = addinput(dd,"Dev1","ai5","Voltage");
ch6.TerminalConfig = "SingleEnded";
ch6.Range = [-1 1];
ch7 = addinput(dd,"Dev1","ai6","Voltage");
ch7.TerminalConfig = "SingleEnded";
ch7.Range = [-1 1];
ch8 = addinput(dd,"Dev1","ai7","Voltage");
ch8.TerminalConfig = "SingleEnded";
ch8.Range = [-1 1];
%}
ch9 = addinput(dd,"Dev2","ai0","Voltage");
ch9.TerminalConfig = "SingleEnded";
ch9.Range = [-5 5];
%{
ch10 = addinput(dd,"Dev2","ai1","Voltage");
ch10.TerminalConfig = "SingleEnded";
ch10.Range = [-1 1];
ch11 = addinput(dd,"Dev2","ai2","Voltage");
ch11.TerminalConfig = "SingleEnded";
ch11.Range = [-1 1];
ch12 = addinput(dd,"Dev2","ai3","Voltage");
ch12.TerminalConfig = "SingleEnded";
ch12.Range = [-1 1];
ch13 = addinput(dd,"Dev2","ai4","Voltage");
ch13.TerminalConfig = "SingleEnded";
ch13.Range = [-1 1];
ch14 = addinput(dd,"Dev2","ai5","Voltage");
ch14.TerminalConfig = "SingleEnded";
ch14.Range = [-1 1];
ch15 = addinput(dd,"Dev2","ai6","Voltage");
ch15.TerminalConfig = "SingleEnded";
ch15.Range = [-1 1];
ch16 = addinput(dd,"Dev2","ai7 ","Voltage");
ch16.TerminalConfig = "SingleEnded";
ch16.Range = [-1 1];
%}

%% Add Triggers
% Use |addtrigger| to add a digital start trigger from (|'RTSI0/PFI3'|
% (source) to |'RTSI0/Dev4'| (destination)
addtrigger(dd,"Digital","StartTrigger","Dev1/RTSI0","Dev2/RTSI0");

%% Add Scan Clock
% Use |addclock| to share a scan clock using the |RTSI1| terminal
% connection.
addclock(dd, "ScanClock", "Dev1/RTSI1","Dev2/RTSI1");

%% Acquire Data with Synchronization
% Use |read| to acquire data.  
[signal,time] = read(dd, N, "OutputFormat", "Matrix");
figure();
plot(time,signal)
title('Measured Raw Data')

stop(d)

%% Verify with FFT
FFT_signal = fft(signal, N);
FFT_source = fft(source, N);
f_HZ1 = ((0:N-1)*dd.Rate)/N;
f_HZ2 = ((0:N-1)*d.Rate)/N;
figure;
hold on;
stem(f_HZ1, abs(FFT_signal)/N)
stem(f_HZ2, abs(FFT_source)/N)
hold off;
title('FFT of signal')

%% Sampling
% declare more varibles 
k = 0:N-1;          % vector of n points
f_samp = dd.Rate;   % Hz
tk = k/f_samp;      % xAxis

freq50 = 50e3;      % desired frequency in Hz
w0125 = 2*pi*freq50;     % omega???

freq30 = 30e3;
w0100 = 2*pi*freq30;

freq10 = 10e3;
w075 = 2*pi*freq10;

% matrix containing the desired frequency component of the system
E125 = [sin(w0125*tk)', cos(w0125*tk)', ones(1, length(tk))']; 
E100 = [sin(w0100*tk)', cos(w0100*tk)', ones(1, length(tk))'];
E75 = [sin(w075*tk)', cos(w075*tk)', ones(1, length(tk))'];
% Etot = [E125; E100; E75];

% inverse of E * s^t (pinv)
phi125 = E125\signal; % gives alpha, beta and offset
phi100 = E100\signal;
phi75 = E75\signal;

% seperate phi characteristics
alpha125 = phi125(1);
beta125 = phi125(2);
Cout125 = phi125(3);

alpha100 = phi100(1);
beta100 = phi100(2);
Cout100 = phi100(3);

alpha75 = phi75(1);
beta75 = phi75(2);
Cout75 = phi75(3);

% create the signal @wo
f125signal = E125 * phi125;
f100signal = E100 * phi100;
f75signal = E75 * phi75;

%% plot new signal
figure
subplot(2,1,1);
hold on;
plot(time, f125signal, 'color', 'b')
title("Matrix Generated Signal")
xlabel("Time (s)");
ylabel("Voltage (V)");
subplot(2,1,2);
hold on;
plot(time, f125signal,'.-','color', 'b')
xlim([.00001 .0001])
xlabel("Time (s)");
ylabel("Voltage (V)");

figure
subplot(2,1,1);
hold on;
plot(time, f100signal, 'color', 'b')
title("Matrix Generated Signal")
xlabel("Time (s)");
ylabel("Voltage (V)");
subplot(2,1,2);
hold on;
plot(time, f100signal,'.-','color', 'b')
xlim([.00001 .0001])
xlabel("Time (s)");
ylabel("Voltage (V)");

figure
subplot(2,1,1);
hold on;
plot(time, f75signal, 'color', 'b')
title("Matrix Generated Signal")
xlabel("Time (s)");
ylabel("Voltage (V)");
subplot(2,1,2);
hold on;
plot(time, f75signal,'.-','color', 'b')
xlim([.00001 .0001])
xlabel("Time (s)");
ylabel("Voltage (V)");

%% Create Sine Function
function sine = createSine(A, f1, f2, f3, sampleRate, type, duration)

numSamplesPerCycle = floor(sampleRate/f1);
T = 1/f1;
timestep = T/numSamplesPerCycle;
t = (0 : timestep : T-timestep)';

if type == "bipolar"
    y = A*sin(2*pi*f1*t) + A*sin(2*pi*f2*t) + A*sin(2*pi*f3*t);
elseif type == "unipolar"
    y = A*sin(2*pi*f1*t) + A*sin(2*pi*f2*t) + A*sin(2*pi*f3*t) + A;
end

numCycles = round(f1*duration);
sine = repmat(y,numCycles,1);
end
%% Control Script
%% Clear Workspace
close all; clearvars; clc;

%% Initialize Variables
numChannels = 16;                   % Number of electrodes in the system
offset= 0;                          % Offset of channels for testing
SkipN = 0;                          % Number of electrodes to skip
adcRange = 1;                       % Sets the range of the adc
currR = [10.03; 10.2];              % Resistance current is being measured over
currG = [49.72; 53.05];             % Gain for current measurment
N = 512;                            % Number of samples taken
sampleRate = 110e3;                 % Sets the rate that the NI-Boards sample at
outputRate = 1e6;                   % Sets the rate that the NI-Board output at
bufferForPreLoad = 500e3;           % sets the length of buffer for preloading
pkpV = 3;                           % pk-pk voltage per frequency
frq = [20e3];                       % array of frequencies to make up multi-freq signal
phase = [0 pi];                     % phase of signal 1 versus signal 2
dFrq = -90.00548;                   % Frequancy shift

%% Add Digital Mux Select
MuxDigiOut = daq("ni");
MuxBrd1 = addoutput(MuxDigiOut, "Dev1", "port1/line0:4","Digital"); % adds the digital mux data lines for board 1
MuxBrd2 = addoutput(MuxDigiOut, "Dev2", "port1/line0:4", "Digital"); % adds the digital mux data lines for board 2
stop(MuxDigiOut)

%% Add Digital Switch Select
SwitchSelect = daq("ni");
SWBrd1 = addoutput(SwitchSelect, "Dev1", "port2/line0:7","Digital"); % adds the digital switch data lines for board 1
SWBrd2 = addoutput(SwitchSelect, "Dev2", "port2/line0:7", "Digital"); % adds the digital switch data lines for board 2
stop(SwitchSelect)

%% Add Analog Output
dAOut = daq("ni"); % Initialize connection to the NI-Board for analog output
AnalogCH1 = addoutput(dAOut,"Dev1","ao0","Voltage"); % adds the analog output for board 1
AnalogCH2 = addoutput(dAOut,"Dev2","ao0","Voltage"); % adds the analog output for board 2
stop(dAOut) % initializes the channels to write nothing

%uncomment if using multiple boards for DAQ
% Use |addtrigger| to add a digital start trigger from (|'RTSI0/PFI3'|
% (source) to |'RTSI0/Dev4'| (destination)
addtrigger(dAOut,"Digital","StartTrigger","Dev1/RTSI0","Dev2/RTSI0");

% Use |addclock| to share a scan clock using the |RTSI1| terminal
% connection.
addclock(dAOut, "ScanClock", "Dev1/RTSI1","Dev2/RTSI1");

dAOut.Rate = outputRate; % sets the output sampling rate Hz

outputSignal = [createSine(pkpV/2, frq, phase(1), outputRate, "bipolar", bufferForPreLoad) createSine(pkpV/2, frq, phase(2), outputRate, "bipolar", bufferForPreLoad)]; % calls function to generate wave vector

% Uncomment to see signal being injected
% t = (0:(N-1)) / sampleRate;
% figure;
% plot(t, outputSignal(1:N))
% title('Generated Output')
% source = outputSignal(1:N);

preload(dAOut,outputSignal); % loads wave vector into NI-Board

%% Add ADC Inputs
dDAQ = daq("ni");
dDAQ.Rate = sampleRate; 

ch1 = addinput(dDAQ,"Dev1","ai0","Voltage");
ch1.TerminalConfig = "SingleEnded";
ch1.Range = [-adcRange adcRange];
ch2 = addinput(dDAQ,"Dev1","ai1","Voltage");
ch2.TerminalConfig = "SingleEnded";
ch2.Range = [-adcRange adcRange];
ch3 = addinput(dDAQ,"Dev1","ai2","Voltage");
ch3.TerminalConfig = "SingleEnded";
ch3.Range = [-adcRange adcRange];
ch4 = addinput(dDAQ,"Dev1","ai3","Voltage");
ch4.TerminalConfig = "SingleEnded";
ch4.Range = [-adcRange adcRange];
ch5 = addinput(dDAQ,"Dev1","ai4","Voltage");
ch5.TerminalConfig = "SingleEnded";
ch5.Range = [-adcRange adcRange];
ch6 = addinput(dDAQ,"Dev1","ai5","Voltage");
ch6.TerminalConfig = "SingleEnded";
ch6.Range = [-adcRange adcRange];
ch7 = addinput(dDAQ,"Dev1","ai6","Voltage");
ch7.TerminalConfig = "SingleEnded";
ch7.Range = [-adcRange adcRange];
ch8 = addinput(dDAQ,"Dev1","ai7","Voltage");
ch8.TerminalConfig = "SingleEnded";
ch8.Range = [-adcRange adcRange];
ch9 = addinput(dDAQ,"Dev2","ai0","Voltage");
ch9.TerminalConfig = "SingleEnded";
ch9.Range = [-adcRange adcRange];
ch10 = addinput(dDAQ,"Dev2","ai1","Voltage");
ch10.TerminalConfig = "SingleEnded";
ch10.Range = [-adcRange adcRange];
ch11 = addinput(dDAQ,"Dev2","ai2","Voltage");
ch11.TerminalConfig = "SingleEnded";
ch11.Range = [-adcRange adcRange];
ch12 = addinput(dDAQ,"Dev2","ai3","Voltage");
ch12.TerminalConfig = "SingleEnded";
ch12.Range = [-adcRange adcRange];
ch13 = addinput(dDAQ,"Dev2","ai4","Voltage");
ch13.TerminalConfig = "SingleEnded";
ch13.Range = [-adcRange adcRange];
ch14 = addinput(dDAQ,"Dev2","ai5","Voltage");
ch14.TerminalConfig = "SingleEnded";
ch14.Range = [-adcRange adcRange];
ch15 = addinput(dDAQ,"Dev2","ai6","Voltage");
ch15.TerminalConfig = "SingleEnded";
ch15.Range = [-adcRange adcRange];
ch16 = addinput(dDAQ,"Dev2","ai7","Voltage");
ch16.TerminalConfig = "SingleEnded";
ch16.Range = [-adcRange adcRange];

curCH1 = addinput(dDAQ, "Dev1", "ai8", "Voltage");
curCH1.TerminalConfig = "SingleEnded";
curCH1.Range = [-2 2];
curCH2 = addinput(dDAQ, "Dev2", "ai8", "Voltage");
curCH2.TerminalConfig = "SingleEnded";
curCH2.Range = [-2 2];

% Add Triggers and Scan Clock for Synchronization to ADC Inputs
% uncomment if using multiple boards for DAQ
% Use |addtrigger| to add a digital start trigger from (|'RTSI2/PFI3'|
% (source) to |'RTSI0/Dev4'| (destination)
addtrigger(dDAQ,"Digital","StartTrigger","Dev1/RTSI2","Dev2/RTSI2");

% Use |addclock| to share a scan clock using the |RTSI3| terminal
% connection.
addclock(dDAQ, "ScanClock", "Dev1/RTSI3","Dev2/RTSI3");

%% Collect Data Vector
% Main Loop
rawElectrodeData = zeros(numChannels, numChannels, N);
rawCurrData = zeros(numChannels, 2, N);
demodElecrodeData = zeros(numChannels^2, 1);
Epiv = computeEpiv(frq+dFrq, N, sampleRate);
start(dAOut,"repeatoutput"); % starts wave output from the NI-Board
tic

for i = 0:1:numChannels-1
    muxSet = setMux(MuxDigiOut, i+offset, SkipN, numChannels+offset);
    electrodeSet = setElectrode(SwitchSelect, i, SkipN, numChannels);
    pause(300/110e3)
    [signal,time] = read(dDAQ, N, "OutputFormat", "Matrix");

    demod = multiFreqDemod(signal, Epiv, i);
    
    rawElectrodeData(i+1,:,:) = signal(:,1:numChannels)';
    rawCurrData(i+1,:,:) = (signal(:,numChannels+1:end)./(currG.*currR)')';
    demodElecrodeData(i*numChannels+1:(i+1)*numChannels,:) = demod(1:numChannels, :);
end
toc

%% Stop Outputs
stop(dAOut);
stop(MuxDigiOut);
stop(SwitchSelect);

%% Save Data of Test
save("Saved_Data_Files/Electrode_16/Object_Left_Tank.mat", "rawElectrodeData","rawCurrData","demodElecrodeData","frq","N","sampleRate","SkipN")

%% Sine Function
function sine = createSine(A, frq, phase, sampleRate, type, bufferForPreLoad)
    timeStep = 1/sampleRate; % period of the sampling frequency
    periods = 1/eval(gcd(sym(frq))); % calculates the length of the period for a periodic
    t = (0:timeStep:periods-timeStep)'; % calculates the time vector from the period
    y = sum(A*sin(2.*pi.*frq.*t+phase),2); % calculates the signal along the time vector
    
    if type == "unipolar"
        y = y + max(y); % if the signal is unipolar add the max amplitude to make it all positive
    end
    
    numCycles = ceil(bufferForPreLoad/length(y)); % calculate the number of cycles to make a full buffer for preloading
    sine = repmat(y, numCycles, 1); % extends the data vector to match or execed the buffer length
end

%% Mux Function
function muxSet = setMux(device, currentInjection, skipNum, numChannels)
    muxSet = [1, flip(str2double(num2cell(dec2bin(mod(currentInjection, numChannels), 4)))), 1, flip(str2double(num2cell(dec2bin(mod(currentInjection + skipNum+1, numChannels), 4))))];
    write(device, muxSet);
end

%% Electrode Select Function
function electrodeSet = setElectrode(device, currentInjection, skipNum, numChannels)
    electrodeSet = [flip(str2double(num2cell(dec2bin(bitor(bitshift(1,mod(currentInjection, numChannels)), bitshift(1,mod(currentInjection+skipNum+1, numChannels))), numChannels)))) ];
    write(device, electrodeSet);
end

%% Precompute Epvi Vector
function Epiv = computeEpiv(frq, N, sampleFrq)
    tk = ((0:N-1)/sampleFrq)'; % xAxis
    
    w = 2*pi*frq.*tk;     % frequency converted to anglar frequency (rad/s)
    
    % matrix containing the desired frequency component of the system
    Etot = [sin(w), cos(w)];
    if(size(frq)==1)
        Etot = [Etot, ones(N, 1)];
    else
        for ii = 2:3:(size(Etot,2)*1.5)
            Etot = [Etot(:,1:ii), ones(1, length(tk))', Etot(:,ii+1:end)];
        end
    end

    Epiv = pinv(Etot);
end

%% Frequency Demodulation
function realAmp = multiFreqDemod(signal, Epiv, inject) % signal, frq, sampleLen, sampleFreq or signal, Etot
    phi_tot = (Epiv*signal)';
    %phi_tot = reshape(phi_tot, [], 3); % Rearrange to useful order

    amp = sqrt(phi_tot(:,1).^2+phi_tot(:,2).^2);
    phase = atan2(phi_tot(:,2), phi_tot(:,1));
    phase = phase - phase(inject+1);

    realAmp = real(amp.*exp(1i.*phase));

end
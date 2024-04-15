%% Control Script
%% Clear Workspace

close all; clearvars; clc;
%% Initialize Variables

numChannels = 8;                    % Number of electrodes in the system
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
dFrq = -90.00548;

%% Load Reconstruction Matrix
reconstruction = load('reconstruction.mat') ;

MatrixA_int = reconstruction.MatrixA_int;
mask = reconstruction.mask;
n_pixels = reconstruction.n_pixels;

clear reconstruction;
%% Add Digital Mux Select

MuxDigiOut = daq("ni");
MuxBrd1 = addoutput(MuxDigiOut, "Dev1", "port1/line0:4","Digital"); % adds the digital mux data lines for board 1
MuxBrd2 = addoutput(MuxDigiOut, "Dev2", "port1/line0:4", "Digital"); % adds the digital mux data lines for board 2
stop(MuxDigiOut)
%% Add Digital Switch Select
SwitchSelect = daq("ni");
SWBrd1 = addoutput(SwitchSelect, "Dev1", "port2/line0:7","Digital"); % adds the digital switch data lines for board 1
%SWBrd2 = addoutput(SwitchSelect, "Dev2", "port2/line0:7", "Digital"); % adds the digital switch data lines for board 2
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
%{
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
ch16 = addinput(dDAQ,"Dev2","ai7 ","Voltage");
ch16.TerminalConfig = "SingleEnded";
ch16.Range = [-adcRange adcRange];
%}
curCH1 = addinput(dDAQ, "Dev2", "ai7", "Voltage");
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
start(dAOut,"repeatoutput"); % starts wave output from the NI-Board

Epiv = computeEpiv(frq+dFrq, N, sampleRate);
EmptyTank = gatherFrame(MuxDigiOut, SwitchSelect, dDAQ, numChannels, SkipN, N, Epiv);

figure(1)
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');

figure(2)
while true
    voltVec = gatherFrame(MuxDigiOut, SwitchSelect, dDAQ, numChannels, SkipN, N, Epiv);
    
    voltage_vec_diff = (voltVec - EmptyTank);

    %% Compute Image
    imagem = real(MatrixA_int*voltage_vec_diff) .* mask;

    %% Plot Image
    imagesc(reshape(imagem', n_pixels , n_pixels)); 
    axis equal; 
    colormap jet;

    %% Stop Loop
    if ~ishandle(ButtonHandle)
        disp('Loop stopped by user');
        break;
    end
end
%% Stop Outputs
stop(dAOut);
stop(MuxDigiOut);
stop(SwitchSelect);

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
    k = 0:N-1; % vector of n points
    tk = (k/sampleFrq)'; % xAxis
    
    w = 2*pi*frq;     % frequency converted to anglar frequency (rad/s)
    
    % matrix containing the desired frequency component of the system
    Etot = [sin(w.*tk), cos(w.*tk)];
    if(size(frq)==1)
        Etot = [Etot, ones(1, length(tk))'];
    else
        for ii = 2:3:(size(Etot,2)*1.5)
            Etot = [Etot(:,1:ii), ones(1, length(tk))', Etot(:,ii+1:end)];
        end
    end
    % inverse of E * s^t (pinv)
    % phi1 = E(1)\signal; % gives alpha, beta and offset
    % phi2 = E(2)\signal;
    % phi3 = E(3)\signal;
    Epiv = pinv(Etot);
end

%% Frequency Demodulation
function realAmp = multiFreqDemod(signal, Epiv,CurrPatern) % signal, frq, sampleLen, sampleFreq or signal, Etot
    phi_tot = Epiv*signal;
    
    phi_tot = reshape(phi_tot, [], 3); % Rearrange to useful order
    realAmp = phi_tot(:, 1);
    
    [~, amplitude, phase_rad, offset, ~, ~, ~] = lms_fixed_freq_mod_JM(signal,19910,110e3,0) ;
    n = [0:512-1] ;
    raw_voltages_ref =  amplitude(1)*sin(2*pi*(19910/110e3)*n + phase_rad(1))+offset(1) ;
    
    phase_rad = phase_rad - phase_rad(CurrPatern+1);
    realAmp = real(amplitude.*exp(1i*phase_rad)).';
    %ampPhase = ampPhase(1:2,length(frq)); % cut out dc as it is not needed
end

%% Collect Frame
function voltVec = gatherFrame(MuxDigiOut, SwitchSelect, dDAQ, numChannels, SkipN, N, Epiv)
    voltVec = zeros(numChannels^2, 1);
    for i = 0:1:numChannels-1
        setMux(MuxDigiOut, i, SkipN, numChannels);
        setElectrode(SwitchSelect, i, SkipN, numChannels);
        pause(300/110e3)
        sig = read(dDAQ, N, "OutputFormat", "Matrix");
        CurrPatern = i ;
        hold = multiFreqDemod(sig, Epiv,CurrPatern);
        %plot(hold(1:numChannels,:))
        %pause;
        voltVec(i*numChannels+1:(i+1)*numChannels) = hold(1:numChannels,:);
    end
end

%% Santos Demod
function [y, amplitude, phase_rad, offset, alpha, beta, omegat] = lms_fixed_freq_mod_JM(Data_s,f0,samp_freq,k0)
% phase_rad is the phase of the returned y

Ts = 1/samp_freq; % sampling period. Value in seconds
N=size(Data_s,1);

discrete_time=[k0:(N+k0-1)]*Ts;

omegat = 2*pi*f0*discrete_time';
% compute mat_J
SINES = sin(omegat);
COSSINES = cos(omegat);
mat_J = [SINES  COSSINES ones(N,1)];

vec_p = pinv(mat_J)*Data_s;  % p is 3 by 32


alpha=vec_p(1,:);  % alpha is 1 by 32
beta=vec_p(2,:);  % beta is 1 by 32
C=vec_p(3,:);  % C is 1 by 32

n_elec = length(alpha);

amplitude = sqrt(alpha.^2 + beta.^2);
%phase_rad = -atan2(beta,alpha); % in radians!!!  Note that Raul's demod program uses -atan2, why minus?! I do believe is plus!(talles)
phase_rad= atan2(beta,alpha);
offset = C;

phi_alpha = acos(alpha./amplitude); % these agree - the diffence is pi or 0
phi_beta = asin(beta./amplitude);

y=zeros(N,n_elec);  % y is the best fit to the raw data, each column is an electrode
ysin = zeros(N,n_elec);
ycos = zeros(N,n_elec);

for k=1:N
   ycos(k,:) = beta.*cos(2*pi*f0*discrete_time(k));
   ysin(k,:) = alpha.*sin(2*pi*f0*discrete_time(k));
   y(k,:) = ysin(k,:) + ycos(k,:) + C;  
end


% JM:  alpha is V_r and beta is V_q

end
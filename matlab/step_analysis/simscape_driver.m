clc;
clear all;
close all;

% Pointer to step responses
foldernames = {'..\data\2022-11-28\2022-11-28_190714_step_exp';
    '..\data\2022-11-28\2022-11-28_190809_step_exp';
    '..\data\2022-11-28\2022-11-28_190928_step_exp';
    '..\data\2022-11-28\2022-11-28_191007_step_exp';
    '..\data\2022-11-28\2022-11-28_191241_step_exp';
    '..\data\2022-11-28\2022-11-28_191356_step_exp';
    '..\data\2022-11-28\2022-11-28_191555_step_exp'};


%simscape parameters
L_muscle = 30; %cm
L_braid = 32;
L_bottom =15;
L_pipe = 20;
L_top = 15;
T_atm = 293.15;
p_atm = 0.101325;
flow_area = 1e-04;
n=2.5;
m_sandbag = 5; %kg



for i=1:length(foldernames)
    filepath = fullfile(foldernames{i},'step_data.dat');
    data = readtable(filepath);
    
    % Extract data
    time = data.T__ms_/1000; % sec
    t_end = max(time);
    pressure_input = (data.Piezo_out__mV_-4000)*6/160000; %MPa (I think)
    %position =(125/16)*(data.V_dist__mA_-4)+25; %mm
    position = data.V_dist__mA_ *(-100.0/16.0)+125-11.2625; %mm
    pressure_reg = 12/(16)*(data.Piezo_P__mA_-4)/10; %MPa (I think)
    pressure_compr = 12/16*(data.Comp_P__mA_-4)/10; %MPa (I think)
    load_cell = 9.81*(data.LC__grams_/1000); %N

    simOut = sim('PAM_Sandbox','SimulationMode','normal',...
            'SaveState','on','StateSaveName','xout',...
            'SaveOutput','on','OutputSaveName','yout',...
            'SaveFormat', 'Dataset');

    % Plot data
    figure();
    subplot(4,1,1);
    plot(time,pressure_input);
    hold on;
    plot(simOut.logsout{1}.Values)
    xlabel('Time [s]')
    title('Pressure step input')
    ylabel('Pressure (MPa)')
    legend('Experimental','Model')
    xlim([0 t_end])

    subplot(4,1,2);
    plot(time,pressure_reg);
    hold on;
    plot(simOut.logsout{3}.Values)
    xlabel('Time [s]')
    title('Measured piezo pressure')
    ylabel('Pressure (MPa)')
    legend('Experimental','Model')
    xlim([0 t_end])
    

    subplot(4,1,3);
    plot(time,position);
    hold on;
    plot(simOut.logsout{2}.Values.Time, squeeze(simOut.logsout{2}.Values.Data))
    xlabel('Time [s]');
    title('Position sensor')
    ylabel('Position [mm]');
    legend('Experimental','Model')
    xlim([0 t_end])
    
    subplot(4,1,4);
    plot(time,load_cell);
     hold on;
    plot(simOut.logsout{4}.Values.Time,squeeze(simOut.logsout{4}.Values.Data))
    title('Load cell')
    xlabel('Time [s]');
    ylabel('Load [N]');
    legend('Experimental','Model')
    xlim([0 t_end])
end




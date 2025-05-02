%%
% Detials: This is a demo code for "A Multikernel Correntropy Approach for Invariant Extended Kalman Filtering: Theory and Application"
%         
% Description
%
% Author: Shilei Li
% Created:                         22-April-2025
%% Script to run simulation
clear all
% Reset 
clear; restoredefaultpath; clc;

% Add paths
addpath(genpath('forward_kinematics'))
addpath('example_code')

%% Open Model  
mdl = 'RIEKF_test';
open_system(mdl); % Simulation model

%% Sim Model
rng(42);



%% Set Simulation Scenario: parameters to control the algorithms and simulation scenarios

% Channel:  1= IEKF,   2= MKCIEKF,  3= MCIEKF
% Case:     1= Case 1,  2 = Case 2

Channel=3;   % Channel=1 Channel=2 Channel=3
Case=1;     % Case=1; Case=2; 
sim(mdl);

%% Plot Results
plot_results;



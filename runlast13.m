% Result and analysis for the simulink model, run this last
%
% Victor Öhman, 2016

clear
clc
close all

load('out_pos.mat')

pos_data = out_pos.Data;

North   = pos_data(:,1);
East    = pos_data(:,2);
Down    = pos_data(:,3);

plot3(North,East,Down)

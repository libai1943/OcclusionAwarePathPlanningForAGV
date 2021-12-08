% ==============================================================================
% MATLAB Source Codes for "Occlusion-aware Path Planning to Promote
% Infrared Positioning Accuracy for Autonomous Driving in a Warehouse".
% ==============================================================================
%   Copyright (C) 2022 Bai Li
%   Users are suggested to cite the following article when they use the source codes.
%   Bai Li et al., "Occlusion-aware Path Planning to Promote Infrared
%   Positioning Accuracy for Autonomous Driving in a Warehouse",
%   Electronics 2021.
% ==============================================================================
clear all; close all; clc;
global params_
InitializeParams();
LoadCase(); % Layers 1 and 2
IdentifyFrenetLowerUpperBonds();
cost = SearchViaDp(); % Layer 3
OptimizeViaQp(); % Layer 4
PlotResult(); % Output path planning result
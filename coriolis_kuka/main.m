clear all; close all; clc;
% Define path to a urdf file
path_to_urdf = 'iiwa14.urdf';

% Generate functions for dynamics based on Lagrange method
% Note that it might take some time
generate_rb_dynamics(path_to_urdf);
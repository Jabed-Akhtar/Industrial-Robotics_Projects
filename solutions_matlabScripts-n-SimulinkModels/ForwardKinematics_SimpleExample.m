% *************************************************************************
% By        : Jabed-Akhtar (github)
% date      : 16.06.2022
% *************************************************************************
% file      : ForwardKinematics_SimpleExample.m
% brief     :
% *************************************************************************
% script (this) related infos:
%   - a source used within this script: 
%   - evicences/pics can be found at location: '../docs_images__/ForwardKinematics_SimpleExample_***.png'
% *************************************************************************
% Descriptions:
%   - ---
% *************************************************************************

clc; clear; close all;

%% Initializing Robotics Toolbox ------------------------------------------
startup_rvc;

%% Defining Links and Robot -----------------------------------------------
L(1) = Link([0 0.385 0 -pi/2 0]);
L(2) = Link([0 0 0.220 0 0]);
L(3) = Link([0 0 0.220 0 0]);
L(4) = Link([0 0 0.155 0 0]);

robo = SerialLink(L, 'name', 'myRobo');

qf0 = [0 0 0 0];
qf1 = [0 -pi/2 pi/2 pi/2];
qf2 = [-pi/3 -pi/2 pi/2 0];
qf3 = [-pi/3 0 pi/2 -2*pi/3];
qf4 = [-2*pi/3 0 pi/2 0];

%% Direct Kinematics ------------------------------------------------------
T0 = robo.fkine(qf0);
% robo.plot(qf0)
% pause(0.5)
T1 = robo.fkine(qf1);
% robo.plot(qf1)
% pause(0.5)
T2 = robo.fkine(qf2);
% robo.plot(qf2)
% pause(0.5)
T3 = robo.fkine(qf3);
% robo.plot(qf3)
% pause(0.5)
T4 = robo.fkine(qf4);
% robo.plot(qf4)
% pause(0.5)

%% Robot-Plots ------------------------------------------------------------
temp = 0:0.4:10;
q = jtraj(qf0, qf1, temp);
robo.plot(q)
pause(1)
q = jtraj(qf1, qf2, temp);
robo.plot(q)
pause(1)
q = jtraj(qf2, qf3, temp);
robo.plot(q)
pause(1)
q = jtraj(qf3, qf4, temp);
robo.plot(q)
pause(1)

% *************************** END OF FILE *********************************
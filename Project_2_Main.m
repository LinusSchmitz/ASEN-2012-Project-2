%% Rocket Lab 
%Linus Schmitz

%% Housekeeping 
clc
clear all
close all

%% Declare which values will never change (ie. global variables) 
global g gamma R A_bol rho_airAmb Vol_bol P_gaug V_air0 M_air0 C_D P_amb rho_wat A_thro C_d T_air0
%% Givens 
g = 9.81;           %Gravity 
C_d = 0.8;          %Discharge coefficient 
rho_airAmb = 0.961; %Ambient Air density [kg/m^3]
Vol_bol = 0.002;    %Bottle volume (empty) [m^3]
P_amb = 83426.56;   %Ambient Pressure [Pa]
gamma = 1.4;        %Ratio of specific heats (const)
rho_wat = 1000;     %Density of water [kg/m^3]
D_thro = 0.021;     %Throat diameter [m]
D_bol = 0.105;      %Bottle diameter [m]
R = 287;            %Gas constant of air
M_bol = 0.15;       %Mass of bottle ("empty") [kg]
C_D = 0.5;          %Drag coefficient
P_gaug = 344728;    %Initial gauge pressure of air in bottle [pa]
V_wat0 = 0.001;     %Initial volume of water in bottle  [m^3]
T_air0 = 300;       %Initial temperature of air in bottle [k]
V_0 = 0;            %Initial velocity of rocket
theta0 = (pi/4);    %Initial angle of rocket[rad]
x0 = 0;             %Initial horizontal distance
z0 = 0.25;          %initial vertical distance
L_s = 0.5;          %Length of launch stand 

%% Calculated constants 
rho_air0=P_gaug/(R*T_air0);  %Density of air in the rocket initially 
V_air0=Vol_bol-V_wat0;   %Volume of air initially  
M_air0=rho_air0*V_air0;
mass_R0=M_bol+(V_wat0*rho_wat)+(M_air0); %initial mass of the rocket
A_thro=pi*0.25*(D_thro)^2;    %Area of the throat
A_bol=pi*0.25*(D_bol)^2;      %Area of the bottle 

%initials = [x0, z0, V_0, mass_R0, V_air0, theta0, M_air0]; %Declaring all of the initial values. May have to change to globals


%% 
[t locations] = ode45(@equations, [0 5], [x0, z0, V_0, mass_R0, V_air0, theta0, M_air0]);
x = locations(:,1);
z = locations(:,2);
figure(1)
hold on
plot(x,z)
hold off
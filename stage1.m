clc;
clear all;
close all;

global m0 m02 m03 m04 g0 T A Cd rh0 H0 Re hgr_turn md md2 md3 md4 tburn4
% Launch Site: Guiana Space Center
Alt = 1;              %[m] Alt above sea level

% VEGA Rocket
m_stage_gross = [96243, 26300,12000,665+540+77+1200];% 1st, 2nd,3d
            
d      = 3;           % [m]  Diameter
g0     = 9.81;        % [m/s^2] Constant at its sea-level value
m0  = 137025;         % [kg] Initial mass
A   = pi*d^2/4;       % [m^2]Frontal area
Cd  = 0.55 ;             % Drag coefficient,assumed to have the constant value
rh0 = 1.225;          % [kg/m^3]
H0 = 7500;            % [m] Density scale height
Re = 6378e3;          % [m] Earth's radius
hgr_turn = 5000;       % [m] Rocket starts the gravity turn when h = hgr_turn
% First stage(Solid Fuel)
m_prop = 88300;%65;       % [kg] Propellant mass
m_prop2=23814;
m_prop3=10000;
m_prop4=381;

Isp    = 280 ;        % [s]  Specific impulse
Isp2   = 287.5;
Isp3=298.5;
Isp4=314.6;

tburn = 109.9;        % [s] Fuell burn time, first stage
tburn2=77.1;
tburn3=119.6;
tburn4=50;%612.5;

md = (m_prop)/tburn;  % [kg/s]Propellant mass flow rate
T   = md*(Isp*g0);    % [N] Thrust (mean)
mf = m0 - m_prop;     % [kg] Final mass of the rocket(first stage is empty)
t0 = 0;               % Rocket launch time
tf = t0 + tburn;      % The time when propellant is completely burned
%and the thrust goes to zero
t_range     = [t0,tf];  % Integration interval

% Launch initial conditions:
gamma0 = 89.5/180*pi;       % Initial flight path angle
v0 = 0;   % Velocity (m/s)  % Earth's Rotation considered in eq of motion.
x0 = 0;   % Downrange distance [km]
h0 = Alt; % Launch site altitude [km]
vD0 = 0;  % Loss due to drag (Velocity)[m/s]
vG0 = 0;  % Loss due to gravity (Velocity)[m/s]
state0   = [v0, gamma0, x0, h0, vD0, vG0];
% Solve initial value problem for ordinary differential equations
[t,state] = ode45(@TE,t_range,state0) ;
v     = state(:,1)/1000;      % Velocity [km/s]
gamma = state(:,2)*180/pi;    % Flight path angle  [deg]
x     = state(:,3)/1000;      % Downrange distance [km]
h     = state(:,4)/1000;      % Altitude[km]
vD    = -state(:,5)/1000;     % Loss due to drag (Velocity)[m/s]
vG    = -state(:,6)/1000;     % Loss due to gravity (Velocity)[m/s]

figure(01)
plot(t,h,'r');grid on;grid minor;hold;set(gca,'FontSize',14);
figure(02)
plot(x,h,'r');grid on;grid minor;hold;set(gca,'FontSize',14);
figure(03)
plot(x,t,'r');grid on;grid minor;hold;set(gca,'FontSize',14);
figure(04)
plot(t,vD,'r');grid on; grid minor;hold;set(gca,'FontSize',14);

%2%%%%%%%%%%%%%%%%%%%%%%%  STAGE 2  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m02 =26300+12000+665+540+77+1200;
md2 = (m_prop2)/tburn2;  % [kg/s]Propellant mass flow rate
T   = md2*(Isp2*g0);    % [N] Thrust (mean)
mf = m02 - m_prop2;     % [kg] Final mass of the rocket(first stage is empty)
t0 = 0;               % Rocket launch time
tf = t0 + tburn2;      % The time when propellant is completely burned
%and the thrust goes to zero
t_range     = [t0,tf];  % Integration interval

state0   = [state(end,1), state(end,2), state(end,3), state(end,4), state(end,5), state(end,6)];
% Solve initial value problem for ordinary differential equations
[t2,state2] = ode45(@TE2,t_range,state0) ;
v2     = state2(:,1)/1000;      % Velocity [km/s]
gamma2 = state2(:,2)*180/pi;    % Flight path angle  [deg]
x2     = state2(:,3)/1000;      % Downrange distance [km]
h2     = state2(:,4)/1000;      % Altitude[km]
vD2    = -state2(:,5)/1000;     % Loss due to drag (Velocity)[m/s]
vG2    = -state2(:,6)/1000;     % Loss due to gravity (Velocity)[m/s]
figure(01)
plot(t2+109.9,h2,'b');
figure(02)
plot(x2,h2,'b');
figure(03)
plot(x2,t2+109.9,'b');
figure(04)
plot(t2+109.9,vD2,'b');
%%%%%%%%%%%%%%%%%%%%%%%%  STAGE 3  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m03 =12000+665+540+77+1200;
md3 = (m_prop3)/tburn3;  % [kg/s]Propellant mass flow rate
T   = md3*(Isp3*g0);    % [N] Thrust (mean)
mf = m03 - m_prop3;     % [kg] Final mass of the rocket(first stage is empty)
t0 = 0;               % Rocket launch time
tf = t0 + tburn3;      % The time when propellant is completely burned
%and the thrust goes to zero
t_range     = [t0,tf];  % Integration interval

state0   = [state2(end,1), state2(end,2), state2(end,3), state2(end,4), state2(end,5), state2(end,6)];
% Solve initial value problem for ordinary differential equations
[t3,state3] = ode45(@TE3,t_range,state0) ;
v3     = state3(:,1)/1000;      % Velocity [km/s]
gamma3 = state3(:,2)*180/pi;    % Flight path angle  [deg]
x3     = state3(:,3)/1000;      % Downrange distance [km]
h3     = state3(:,4)/1000;      % Altitude[km]
vD3    = -state3(:,5)/1000;     % Loss due to drag (Velocity)[m/s]
vG3    = -state3(:,6)/1000;     % Loss due to gravity (Velocity)[m/s]
figure(01)
plot(t3+109.9+77.1,h3,'g');
figure(02)
plot(x3,h3,'g');
figure(03)
plot(x3,t3+109.9+77.1,'g');
figure(04)
plot(t3+109.9+77.1,vD3,'g');
%%%%%%%%%%%%%%%%%%%%%%%   STAGE 4  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m04 =665+540+77+1200;
md4 = (m_prop4)/tburn4;  % [kg/s]Propellant mass flow rate
T   = md4*(Isp4*g0);    % [N] Thrust (mean)
mf = m04 - m_prop4;     % [kg] Final mass of the rocket(first stage is empty)
t0 = 0;               % Rocket launch time
tf = t0 + tburn4;      % The time when propellant is completely burned
%and the thrust goes to zero
t_range     = [t0,tf];  % Integration interval

state0   = [state3(end,1), state3(end,2), state3(end,3), state3(end,4), state3(end,5), state3(end,6)];
% Solve initial value problem for ordinary differential equations
[t4,state4] = ode45(@TE4,t_range,state0) ;
v4     = state4(:,1)/1000;      % Velocity [km/s]
gamma4 = state4(:,2)*180/pi;    % Flight path angle  [deg]
x4     = state4(:,3)/1000;      % Downrange distance [km]
h4     = state4(:,4)/1000;      % Altitude[km]
vD4    = -state4(:,5)/1000;     % Loss due to drag (Velocity)[m/s]
vG4    = -state4(:,6)/1000;     % Loss due to gravity (Velocity)[m/s]

figure(01)
plot(t4+109.9+77.1+119.6,h4,'c');
% figure(02)
% plot(x4,h4,'c');
% figure(03)
% plot(x4,t4+109.9+77.1+119.6,'c');
figure(04)
plot(t4+109.9+77.1+119.6,vD4,'c');grid on;

%
th1=x./(Re/1000);theta1=th1*180./pi;
th2=x2./(Re/1000);theta2=th2*180./pi;
th3=x3./(Re/1000);theta3=th3*180./pi;
th4=x4./(Re/1000);theta4=th4*180./pi;

% Rad1(1:length(theta1))=Re/1000;
% Rad2(1:length(theta2))=Re/1000;
% Rad3(1:length(theta3))=Re/1000;
% Rad4(1:length(theta4))=Re/1000;

theta=0:.0001:360;
radius(1:length(theta))=Re/1000;
% 
% figure(05)
% % polar(theta1,Rad1','c');hold;
% % polar(theta2,Rad2','c');
% % polar(theta3,Rad3','c');
% % polar(theta4,Rad4','c');
% polar(theta,radius,'k');hold;
% 
% polar(theta1,Re/1000+h,'r');
% polar(theta2,Re/1000+h2,'b');
% polar(theta3,Re/1000+h3,'c');
% polar(theta4,Re/1000+h4,'g');
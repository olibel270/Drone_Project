clear; clc; close all;
%Parameters
m=0.5; %mass in kg
l=0.17; %arm length in meters
Ixx=0.00365; %kg m^2
Iyy=0.00368; %kg m^2
Izz=0.00703; %kg m^2
k_F=6.11e-8; %Thrust factor N/(rpm)^2
k_M=1.5e-9; %Drag factor N m/(rpm)^2
syms F;

%theta:pitch ; phi:roll ; psi:yaw
syms theta phi psi;
syms p q r;
syms theta_dot phi_dot psi_dot ;
syms theta_ddot phi_ddot psi_ddot ;

%Position vector r
pos = [0;0;0];
pos_dot = [0;0;0];
pos_ddot = [0;0;0];

%From body to world
body_to_world_rotation_matrix = [
    cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta) -cos(phi)*sin(psi) cos(phi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
    cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta) cos(phi)*cos(psi) sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
    -cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)]

%Center of Mass acceleration equations
%m*r_ddot = [0;0;-m*g]+R*[0;0;sum_F];

%pqr_matrix contains angular velocities
pqr_matrix = [
 cos(theta) 0 -cos(phi)*sin(theta);
 0 1 sin(phi);
 sin(theta) 0 cos(phi)*cos(theta)
]*[phi_dot; theta_dot; psi_dot]

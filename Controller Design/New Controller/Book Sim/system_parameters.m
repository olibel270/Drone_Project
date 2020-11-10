%mechanical parameters
g = 9.8100; %m/s^2
d = 0.2223; %m
m = 1.0230; %kg
c_t = 1.4865e-07; %N/rpm^2
c_m = 2.9250e-09; %Nm/RMP^2
Jxx = 0.0095; %kg.m^2
Jyy = 0.0095; %kg.m^2
Jzz = 0.0186; %kg.m^2
J = [Jxx 0 0; 0 Jyy 0; 0 0 Jzz];
T_m = 0.076; %s
C_r = 80.5840; %rpm
omega_b = 976.2000; %rpm

rpm_to_forces = [c_t c_t c_t c_t; 0 -d*c_t 0 d*c_t; d*c_t 0 -d*c_t 0; c_m -c_m c_m -c_m];

global f_max 
    f_max = 8.55*4; %N
global f_min 
    f_min = 6;
global tau_max
    tau_max = 400^2*d*c_t;

%Simulation_parameters
time_interval = 0.01;
xref = 0;
yref = 0;
zref = 0;
psiref = 0;
g = 9.81;
m=0.5; %mass in kg
l=0.17; %arm length in meters
Ixx=0.00365; %kg m^2
Iyy=0.00368; %kg m^2
Izz=0.00703; %kg m^2
k_F=6.11e-8; %Thrust factor N/(rpm)^2
k_M=1.5e-9; %Drag factor N m/(rpm)^2
j = diag([6.4, 6.4, 12.5])*10^(-3); %rotor inertia m^2kg
Jxx = 0.0064;
Jyy = 0.0064;
Ax = 0.1; %kg/s Air drag coefficient in x


Ay = 0.1; %kg/s Air drag coefficient in y
Az = 0.1; %kg/s Air drag coefficient in z
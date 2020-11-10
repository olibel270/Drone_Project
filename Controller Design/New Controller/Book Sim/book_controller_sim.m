clc;clear;close all;

%Parameters
system_parameters
controller_parameters

%Define trajectory
trajectory

%initial_state
omega = [4000 4000 4000 4000].'; %RPM
f = 20;
p = [0 0 0].'; %m
v = [0 0 0].'; %m/s
attitude = [0 0 0].'; %degrees
attitude_vel = [0 0 0].'; %deg/s

%initialize errors to zero
e_vh = 0;
e_vhi = 0;
e_vz = 0;
e_vzi = 0;
e_omega = [0; 0; 0];
e_omega_i = [0;0;0];

%track_position
x_track = zeros(1,length(desired_pos));
y_track = zeros(1,length(desired_pos));
z_track = zeros(1,length(desired_pos));
phi_track = zeros(1,length(desired_pos));
theta_track = zeros(1,length(desired_pos));
psi_track = zeros(1,length(desired_pos));
f_track =  zeros(1,length(desired_pos));

iterations = length(desired_pos);
for iter=1:iterations
    %horizontal_position error tracking
    p_h = p(1:2);
    p_hd = desired_pos(1:2,iter);
    K_ph = K_pos(1:2);
    v_h = v(1:2);
    
    e_vhmin = e_vh;
    
    v_hd = diag(K_ph)*(p_hd-p_h);
    e_vh = v_hd - v_h;
    e_vhd = (e_vh-e_vhmin)/time_interval;
    e_vhi = e_vhi + (e_vh)*time_interval ;
    %horizontal position controller
    psi = attitude(3);
    R_psi = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    A_psi = R_psi*[0 1; -1 0];
    
    orientation_desired = sat_horizontal_angles( ...
        (1/g)*inv(A_psi)*(diag(K_vp(1:2))*e_vh+diag(K_vi(1:2))*e_vhi + diag(K_vd(1:2))*e_vhd) ...
    );

    %attitude controller
    attitude_desired = [orientation_desired ; desired_pos(4,iter)];
    e_att = attitude - attitude_desired;
    omega_d = -diag(K_att)*e_att;
    
    e_omega_min = e_omega;
    e_omega = e_omega_sat(attitude_vel-omega_d);
    e_omega_i = e_omega_i + e_omega_i*time_interval;
    e_omega_d = (e_omega-e_omega_min)/time_interval;
    
    moment_d = moment_sat(...
        -diag(K_omega_p)*e_omega - diag(K_omega_i)*e_omega_i - diag(K_omega_d)*e_omega_d...        
    );

    %altitude controller
    K_pz = K_pos(3);
    
    K_vpz = K_vp(3);
    K_viz = K_vi(3);
    K_vdz = K_vd(3);
    
    p_z = p(3);
    p_zd = desired_pos(3,iter);
    v_z = v(3);
    v_zd = -K_pz*(p_z-p_zd);
    e_vzmin = e_vz;
    
    e_vz = v_z - v_zd;
    e_vzi = e_vzi + (e_vz)*time_interval;
    e_vzd = (e_vz-e_vzmin)/time_interval;
    
    thrust_desired = sat_thrust( ...
        m*(g+ K_vpz*e_vz + K_viz*e_vzi + K_vdz+e_vzd))...
        +(f_min+f_max)/2;
    
    %quadcopter plant
    phi = attitude(1);
    theta = attitude(2);
    psi = attitude(3);
   
    %Get required omegas
    omega_req = inv(rpm_to_forces)*[thrust_desired; moment_d];
    omega_req(omega_req<0) = 0;
    omega_req = sqrt(omega_req);
    
    if iter>=1500
        disp('beep')
    end
    %Find actual omegas response
    omega = omega + (omega_req-omega)*(1-exp((-1/T_m)*0.01));
    %set f to thrust from omegas found
    f = c_t*sum(omega.^2);
    f_track(iter) = f;
    
    
    x_ddot = (f/m)*(sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi));
    x_dot = v(1) + x_ddot*time_interval;
    x = p(1) + x_dot*time_interval + (1/2)*(x_ddot)*time_interval^2;
    x_track(iter) = x;
    p(1) = x;
    v(1) = x_dot;
    
    y_ddot = (f/m)*(-cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi));
    y_dot = v(2) + y_ddot*time_interval;
    y = p(2) + y_dot*time_interval + (1/2)*(y_ddot)*time_interval^2;
    y_track(iter) = y;
    p(2) = y;
    v(2) = y_dot;
    
    z_ddot = g-(f/m)*cos(theta)*cos(phi);
    z_dot = v(3) + z_ddot*time_interval;
    z = p(3) + z_dot*time_interval + (1/2)*(z_ddot)*time_interval^2;
    z_track(iter) = z;
    p(3) = z;
    v(3) = z_dot;

    phi_ddot = moment_d(1)/Jxx;
    phi_dot = attitude_vel(1) + phi_ddot*time_interval;
    phi = attitude(1) + phi_dot*time_interval + (1/2)*(phi_ddot)*time_interval^2;
    phi_track(iter) = phi;
    attitude(1) = phi;
    attitude_vel(1) = phi_dot;
    
    theta_ddot = moment_d(2)/Jyy;
    theta_dot = attitude_vel(2) + theta_ddot*time_interval;
    theta = attitude(2) + theta_dot*time_interval + (1/2)*(theta_ddot)*time_interval^2;
    theta_track(iter) = theta;
    attitude(2) = theta;
    attitude_vel(2) = theta_dot;

    psi_ddot = moment_d(3)/Jzz;
    psi_dot = attitude_vel(3) + psi_ddot*time_interval;
    psi = attitude(3) + psi_dot*time_interval + (1/2)*(psi_ddot)*time_interval^2;
    psi_track(iter) = psi;
    attitude(3) = psi;
    attitude_vel(3) = psi_dot;
end

tiledlayout(6,1)
nexttile
plot(x_track)
nexttile
plot(y_track)
nexttile
plot(z_track)
nexttile
plot(phi_track)
nexttile
plot(theta_track)
nexttile
plot(psi_track)

figure
plot(f_track)
figure 
plot3(x_track,y_track,z_track)

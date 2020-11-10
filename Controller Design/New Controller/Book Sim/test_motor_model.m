clear;
omega = [4000 4000 4000 4000]; %RPM
omega_track = zeros(1,1000);
thrust_track = zeros(1,1000);
c_t = 1.4865e-07; %N/rpm^2
Tm = 0.076; %s
Cr =  80.5840;
omega_b = 976.2000; %rpm
new_f_desired = 5;

for i=1:1000
   current_trust =  c_t*sum(omega.^2);
   
   if i>500
       omega_required = sqrt(new_f_desired/(4*c_t));
       omega = omega + (omega_required-omega)*(1-exp((-1/Tm)*0.01));   
   end
   
end

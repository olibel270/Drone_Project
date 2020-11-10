function [outputArg1] = sat_thrust(inputArg1)
%SAT_THRUST 
global f_max;
global f_min;
outputArg1 = min((f_max-f_min)/2, max(-(f_max-f_min)/2, inputArg1)); 
end


function [outputArg1] = e_omega_sat(inputArg1)
%E_OMEGA_SAT
outputArg1 = min(10, max(-10, inputArg1));
end


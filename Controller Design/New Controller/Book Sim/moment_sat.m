function [outputArg1] = moment_sat(inputArg1)
global tau_max;
outputArg1 = min(tau_max, max(-tau_max, inputArg1));
end


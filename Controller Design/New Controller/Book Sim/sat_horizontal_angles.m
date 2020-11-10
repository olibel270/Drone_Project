function [outputArg1] = sat_horizontal_angles(inputArg1)
%saturates theta or phi to +- 0.7 rads
outputArg1 = min(0.5, max(-0.5, inputArg1)); 
end

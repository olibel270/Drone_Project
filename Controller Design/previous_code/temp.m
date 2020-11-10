clear
syms k l m;
syms u1 u2 u3 u4;
fbf = [k k k k; 0 k*l 0 -k*l; -k*l 0 k*l 0; m -m m -m]
U = [u1; u2; u3; u4]
inv_fbf = inv(fbf)
W = inv_fbf*U

clear;
clc;
% a form of expression of quaternions [cos(theta/2) sin(theta/2)*[kx ky kz]]
syms kx ky kz; 
syms theta;
q = [kx ky kz];
q = sin(theta/2) * q;

q_invers = [cos(theta/2) -q];
q = [cos(theta/2) q];

disp(q);

syms x y z;
v = [0 x y z];

v_trans = quaternion_multip(quaternion_multip(q, v),q_invers);
assume(kx^2+ky^2+kz^2==1)
disp(collect(simplify(v_trans(1)),[x y z]));
disp(collect(simplify(v_trans(2)),[x y z]));
disp(collect(simplify(v_trans(3)),[x y z]));
disp(collect(simplify(v_trans(4)),[x y z]));

simplify(quaternion_multip(q, q_invers))

% another form of expression of quaternions [w1 w2 w3 w4]
w = sym('w', [1, 4]);
w_invers = -w;
w_invers(1) = w(1);
assume(w(1)^2 + w(2)^2 + w(3)^2 + w(4)^2 == 1);
v_trans = quaternion_multip(quaternion_multip(w, v),w_invers);
assume(kx^2+ky^2+kz^2==1)
disp(collect(simplify(v_trans(1)),[x y z]));
disp(collect(simplify(v_trans(2)),[x y z]));
disp(collect(simplify(v_trans(3)),[x y z]));
disp(collect(simplify(v_trans(4)),[x y z]));

simplify(quaternion_multip(w, w_invers))

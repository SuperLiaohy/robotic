clear;
clc;
q = sym('q', [1, 6]);
assume(q, 'real');
% assume(q <= pi & q >= -pi);
% a(i-1) alpha(i-1) d(i) theta(i)
mdh_para = [
    % [0 0 332.55 q(1)]
    [0 0 0 q(1)]
    [0 -pi/2 0 q(2)-pi/2]
    [320 0 0 q(3)+pi/2]

    [0 pi/2 325.5 q(4)]
    [0 -pi/2 0 q(5)]
    [0 pi/2 122 q(6)]
];
trans=[
[1 0 0 0]    
[0 1 0 0]
[0 0 1 0]
[0 0 0 1]
];
for i = 1:4
    trans = trans * mdh_trans(mdh_para(i,:));
end
disp(trans);
position = trans * [0 0 0 1]';
position(4) = [];
position = simplify(position);
disp((position))

% syms x y z real
% express = [x y z]' == position;
% q_answer = [q(1) q(2) q(3)];
% assume(q(1)==0);
% assume(q(4)==0);
% assume(y==0)
% assume(q(1) == atan2(y, x))
% express = subs(express, q(1), atan2(y, x)); % 代入 q1
% s = solve(express, q_answer);
% display(s.q1);
% display(s.q2);
% display(s.q3);


x = position(1);
y = position(2);
z = position(3);

disp(y,x);
% q1 = atan2(x,y) or q1 = atan2(x,y) - pi

disp(vpa(y/sin(q(1))))
disp(vpa(z))

syms r1 r2 r3 t real;
expr1 = x/cos(q(1)) == r1;
expr2 = z == r3;

s = solve([expr1 expr2], [q(2) q(3)]);
disp(s.q2);
disp(s.q3);

simple_q2 = simplify(subs(s.q2, r1^2+r3^2, t));
simple_q3 = simplify(subs(s.q3, r1^2+r3^2, t));
disp(simple_q2);
disp(simple_q3);

% eul 1.r 2.p  3.y
syms r p y real
rpy = rot_eul(r,p,y) * [0 0 1]';
disp(rpy);
line = mdh_para(6,3)* rpy;

rot1_3 = eye(4);
for i = 1:3
    rot1_3 = rot1_3 * mdh_trans(mdh_para(i,:));
end
rot1_3(4, :) = [];
rot1_3(:, 4) = [];
rinit3_4 = [
[1 0 0]
[0 0 -1]
[0 1 0]
];
rot4_6 = rinit3_4' * rot1_3' * rot_eul(r,p,y);


expr_q5 = atan2(sqrt(rot4_6(1,3)^2+rot4_6(2,3)^2),rot4_6(3,3));
% q5 = atan2(-sqrt(rot4_6(1,3)^2+rot4_6(2,3)^2),rot4_6(3,3)); 
expr_q4 = atan2(rot4_6(2,3)/sin(expr_q5),rot4_6(1,3)/sin(expr_q5));
expr_q6 = atan2(rot4_6(3,2)/sin(expr_q5),-rot4_6(3,1)/sin(expr_q5));

% clc;
% x = 383.177;
% y = 336.461;
% z = 91.830;
% q(1) = atan2(y,x);
% disp(vpa(q(1)/pi*180));
% t=x/cos(q(1));
% vpa(subs(s.q2,[r1, r3], [t, z])/pi*180)
% vpa(subs(s.q3,[r1, r3], [t, z])/pi*180)


clc;
pos_x = 303.796;
pos_y = 433.197;
pos_z = 424.628;
roll = 94.1;
pitch = 97.2;
yaw = 79.1;

disp(line);
line = subs(line,[r, p, y], [roll/180*pi, pitch/180*pi, yaw/180*pi]);
disp(vpa(line));
equ_pos = vpa([pos_x pos_y pos_z] - line');
disp(equ_pos)

q1 = atan2(equ_pos(2),equ_pos(1));
t = equ_pos(1)/cos(q1);

q1 = vpa(q1/pi*180);
q2 = vpa(subs(s.q2,[r1, r3], [t, equ_pos(3)])/pi*180);
q3 = vpa(subs(s.q3,[r1, r3], [t, equ_pos(3)])/pi*180);

q5 = vpa(subs(expr_q5,[r,p,y,q(1),q(2),q(3)], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(1)/180*pi,q3(1)/180*pi])/pi*180);
q4 = vpa(subs(expr_q4,[r,p,y,q(1),q(2),q(3),expr_q5], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(1)/180*pi,q3(1)/180*pi,q5/180*pi])/pi*180);
q6 = vpa(subs(expr_q6,[r,p,y,q(1),q(2),q(3),expr_q5], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(1)/180*pi,q3(1)/180*pi,q5/180*pi])/pi*180);

r1_3 = vpa(subs(rot1_3,[q(1),q(2),q(3)], [q1/180*pi,q2(1)/180*pi,q3(1)/180*pi]));
display(r1_3);
cac = r1_3 * rot_eul(q4/180*pi,q5/180*pi,q6/180*pi);
disp(vpa(cac));
disp(rot_eul(roll/180*pi, pitch/180*pi, yaw/180*pi));

rot1_6 = eye(4);
for i = 1:6
    rot1_6 = rot1_6 * mdh_trans(mdh_para(i,:));
end
disp(rot1_6);
rot1_6(4, :) = [];
rot1_6(:, 4) = [];

r1_6 = vpa(subs(rot1_6,q, [q1/180*pi,q2(1)/180*pi,q3(1)/180*pi,q4/180*pi,q5/180*pi,q6/180*pi]));

display(r1_6)

display(q1);
display(q2);
display(q3);
display(q4);
display(q5);
display(q6);





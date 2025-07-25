clear;
clc;
q = sym('q', [1, 6]);
assume(q, 'real');
% assume(q <= pi & q >= -pi);
% 定义MDH参数的列表顺序如下
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
% 初始化变换矩阵
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
% 计算出前三个连杆的末端位置
position = trans * [0 0 0 1]';
% 删除多余维度
position(4) = [];
position = simplify(position);
disp((position))

% 计算前三个连杆末端处于(x,y,z)坐标下前三轴的关节的旋转角度。
clc;
x = position(1);
y = position(2);
z = position(3);

disp(y);
disp(x);
% q1 = atan2(y,x) or q1 = atan2(y,x) - pi

% 注意选择这里没有选择y/sin(q(1))=r2是因为q(1)在我们的坐标系下非常容易为0。若cos(q(1))容易为0则应该选则前项。
syms r1 r2 r3 t real;
expr1 = x/cos(q(1)) == r1;
expr2 = z == r3;

s = solve([expr1 expr2], [q(2) q(3)]);
disp(s.q2);
disp(s.q3);

% 观察前项可以发现有些地方可以进一步化简运算。
simple_q2 = simplify(subs(s.q2, r1^2+r3^2, t));
simple_q3 = simplify(subs(s.q3, r1^2+r3^2, t));

disp(simple_q2);
disp(simple_q3);

% 通过末端姿态反推出前三轴的末端位置。
clc;
% 考虑车体的imu姿态角(只考虑其中的pitch轴的运动)的变化。
syms imu_pitch real;
% assume(imu_pitch==0);
imu_trans = [cos(imu_pitch) 0 sin(imu_pitch);
0 1 0;
-sin(imu_pitch) 0 cos(imu_pitch);];
% eul 1.z 2.y 3.z

syms r p y real
% 计算初始末端连杆单位向量经过姿态角的旋转后的单位向量。
rpy =  (imu_trans' * rot_eul(r,p,y)) * [0 0 1]';
% 使用末端位置减去末端连杆的向量便可以得到前三个连杆的末端位置。
line = mdh_para(6,3)* rpy;
disp(line)
disp(simplify(line));

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
% 计算出从基坐标系到第三个连杆末端的旋转矩阵。rot1_3*rinit3_4。
% 使用末端的旋转矩阵乘上基坐标系到第三个连杆末端的旋转矩阵的逆，就可以得出后三个关节所需要旋转形成的旋转矩阵。
rot4_6 = rinit3_4' * rot1_3' * (imu_trans' * rot_eul(r,p,y));


% 使用欧拉角zyz求解法求出旋转矩阵的三个有效量。
% 计算出旋转矩阵中需要使用的中间变量。
display(simplify(rot4_6(1,3)));
display(simplify(rot4_6(2,3)));
display(simplify(rot4_6(3,3)));
display(simplify(rot4_6(3,2)));
display(simplify(rot4_6(3,1)));

display(simplify(rot4_6(2,2)));
display(simplify(rot4_6(2,1)));

disp(rot4_6)

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

% 带入具体的值进行验算。
clc;
pos_x = 307.897;
pos_y = -146.052;
pos_z = 123.937;
roll = 1;
pitch = 180;
yaw = 1;

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

q5 = vpa(subs(expr_q5,[r,p,y,q(1),q(2),q(3)], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(2)/180*pi,q3(2)/180*pi])/pi*180);
q4 = vpa(subs(expr_q4,[r,p,y,q(1),q(2),q(3),expr_q5], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(2)/180*pi,q3(2)/180*pi,q5/180*pi])/pi*180);
q6 = vpa(subs(expr_q6,[r,p,y,q(1),q(2),q(3),expr_q5], [roll/180*pi, pitch/180*pi, yaw/180*pi,q1/180*pi,q2(2)/180*pi,q3(2)/180*pi,q5/180*pi])/pi*180);

display(simplify(q1));
display(simplify(q2));
display(simplify(q3));
display(simplify(q4));
display(simplify(q5));
display(simplify(q6));





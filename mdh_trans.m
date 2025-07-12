function [trans] = mdh_trans(mdh)
% a(i-1) alpha(i-1) d(i) theta(i)
a = mdh(1);
alpha = mdh(2);
d = mdh(3);
theta = mdh(4);

rotx = [
[1 0 0 0]    
[0 cos(alpha) -sin(alpha) 0]
[0 sin(alpha) cos(alpha) 0]
[0 0 0 1]
];

movex = [
[1 0 0 a]    
[0 1 0 0]
[0 0 1 0]
[0 0 0 1]
];

rotz = [
[cos(theta) -sin(theta) 0 0]    
[sin(theta) cos(theta) 0 0]
[0 0 1 0]
[0 0 0 1]
];

movez = [
[1 0 0 0]
[0 1 0 0]
[0 0 1 d]
[0 0 0 1]
];
% step by step transform 
trans = rotx * movex * rotz * movez;

end
function [result] = rot_eul(roll,pitch,yaw)
%ROT_EUL Summary of this function goes here
%   Detailed explanation goes here

result = rot_z(roll)*rot_y(pitch)*rot_z(yaw);

end

function [result] = rot_x(t)
result = [
[1 0 0]    
[0 cos(t) -sin(t)]
[0 sin(t) cos(t)]
];
end

function [result] = rot_y(t)
result = [
[cos(t) 0 sin(t)]    
[0 1 0]
[-sin(t) 0 cos(t)]
];
end

function [result] = rot_z(t)
result = [
[cos(t) -sin(t) 0]    
[sin(t) cos(t) 0]
[0 0 1]
];
end
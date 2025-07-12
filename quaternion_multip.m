function [result] = quaternion_multip(q1,q2)
%QUATER Summary of this function goes here
%   Detailed explanation goes here

result = [];
q1_real = q1(1);
q2_real = q2(1);
q1(1) = [];
q2(1) = [];
result = [result q1_real*q2_real-q1*transpose(q2) ];
result = [result q1*q2_real+q1_real*q2+cross(q2,q1)];

end
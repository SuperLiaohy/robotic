clear;
clc;
q = sym("q",[6 1],"real");
dq= sym("dq",[6 1],"real");
ddq= sym("ddq",[6 1],"real");

mdh_para = [
    % [0 0 332.55 q(1)]
    [0 0 0 q(1)]
    [0 -pi/2 0 q(2)-pi/2]
    [320 0 0 q(3)+pi/2]

    [0 pi/2 325.5 q(4)]
    [0 -pi/2 0 q(5)]
    [0 pi/2 122 q(6)]
];

m = [];
for i = 1:6
    m(:,i) = 1;
end

[F,N] = forward_recursive(dq,ddq,mdh_para,m);
disp(F);

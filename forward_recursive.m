function [F,N] = forward_recursive(dq,ddq,mdh_param,m)
%FORWARD_RECURSIVE Summary of this function goes here
%   Detailed explanation goes here

% w = [];v = [];
% dw = [];a = [];
    
    [t_w, t_v] = w_v_forward_recursive([0 0 0]', [0 0 0]', dq(1), mdh_param(1,:));
    
    [t_dw, t_a] = dw_a_forward_recursive([0 0 0]', [0 0 0]', t_w, dq(1), ddq(1), mdh_param(1,:));
    
    w(:,1) = t_w;
    dw(:,1)= t_dw;
    v(:,1) = t_v;
    a(:,1) = t_a;
    
    F(:,1) = m(1) * a(:,1);
    for i = 2:1:6
        [t_w, t_v] = w_v_forward_recursive(w(:,i-1), v(:,i-1), dq(i), mdh_param(i,:));
        [t_dw, t_a] = dw_a_forward_recursive(dw(:,i-1), v(:,i-1), t_w, dq(i), ddq(i), mdh_param(i,:));
        
        w(:,i) = t_w;
        dw(:,i)= t_dw;
        v(:,i) = t_v;
        a(:,i) = t_a;
        
        F(:,i) = m(i) * a(:,i);
    end
    N = [];
end





function [P] = mdh_org(mdh_param)

P = mdh_trans(mdh_param) * [0 0 0 1]';
P(4,:) = [];

end


function [org_mat] = mdh_org_mat(mdh_param)

p = mdh_trans(mdh_param) * [0 0 0 1]';
p(4,:) = [];

org_mat = [0 -p(3) p(2);
           p(3) 0 -p(1);
           -p(2) p(1) 0];

end




function [rot] = mdh_rot(mdh_param)

rot = mdh_trans(mdh_param);
rot(4,:) = [];
rot(:,4) = [];
end



function [w_2,v_2] = w_v_forward_recursive(w_1, v_1, dq, mdh_param)
    disp(mdh_param);
    rot = mdh_rot(mdh_param)';
    jacobi = [rot (-rot*mdh_org_mat(mdh_param));
              zeros(3) rot];
    result = jacobi * [v_1' w_1']';

    w_2 = result(4:6) + [0 0 dq]';
    v_2 = result(1:3);
end

function [dw_2,a_2] = dw_a_forward_recursive(dw_1, a_1, w_1, dq, ddq, mdh_param)
    rot = mdh_rot(mdh_param)';

    dw_2 = rot*dw_1 + cross(rot*w_1,[0 0 dq]') + [0 0 ddq]';
    a_2 = rot*a_1 + cross(rot*w_1, mdh_org(mdh_param)) + cross(rot*w_1,cross(w_1,mdh_org(mdh_param)));

end

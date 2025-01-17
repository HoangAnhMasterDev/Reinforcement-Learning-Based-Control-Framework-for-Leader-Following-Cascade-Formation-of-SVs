function a = J_dot_dot(eta)
    %need to add eta_dot
    J_dot_dot=[-cos(eta(3)),sin(eta(3)),0;
    -sin(eta(3)),-cos(eta(3)),0;
    0,0,0];
    a = J_dot_dot;
end
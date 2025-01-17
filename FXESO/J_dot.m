function a = J_dot(eta)
    J_dot=[-sin(eta(3)),-cos(eta(3)),0;
    cos(eta(3)),-sin(eta(3)),0;
    0,0,0];
    a = J_dot;
end
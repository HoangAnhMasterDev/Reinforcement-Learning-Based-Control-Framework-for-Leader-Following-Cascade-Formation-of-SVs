function a = J(eta)
    J=[cos(eta(3)),-sin(eta(3)),0;
    sin(eta(3)),cos(eta(3)),0;
    0,0,1];
    a = J;
end
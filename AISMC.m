%An enhanced tracking control of marine surface vessels based on adaptive integral sliding mode control and disturbance observer
%Add disturbance observer later

clear;
clc;
close all;
T_end = 500;     % run 500s simulation
step = 0.001;   % step time = 0.001s
t = 0:step:T_end;
timeTest = T_end/step;

M = [25.8,0,0;0,33.8,1.0948;0,1.0948,2.76];

% Controller parameters
gamma = 1;
deadzoneSize = 0.01;
rho = 100;
w_hat = 1;
c = 0.1;
k1 = 20;
k2 = 20;
delta_s = diag([5,5,5]);
K = 20;
epsi1 = 12;
epsi2 = 10;
slidingGain = 20;

% Initial conditions

eta_1(:,1) = [0;0.5;0];
eta_2(:,1) = [-3.5;5;0];
eta_3(:,1) = [-7;10;0];
eta_4(:,1) = [-10.5;15;0.2];
eta_5(:,1) = [-14;20;0.4];

v_1(:,1) = [0;0;0];
v_2(:,1) = [0;0;0];
v_3(:,1) = [0;0;0];
v_4(:,1) = [0;0;0];
v_5(:,1) = [0;0;0];

s_1(:,1) = [0;0;0];
s_2(:,1) = [0;0;0];
s_3(:,1) = [0;0;0];
s_4(:,1) = [0;0;0];
s_5(:,1) = [0;0;0];


sigma_1(:,1) = zeros(3,1);
sigma_2(:,1) = zeros(3,1);
sigma_3(:,1) = zeros(3,1);
sigma_4(:,1) = zeros(3,1);
sigma_5(:,1) = zeros(3,1);

s_hat_1(:,1) = zeros(3,1);
s_hat_2(:,1) = zeros(3,1);
s_hat_3(:,1) = zeros(3,1);
s_hat_4(:,1) = zeros(3,1);
s_hat_5(:,1) = zeros(3,1);

k_hat_1(:,1) = [0.01;0.01;0.01];
k_hat_2(:,1) = [0.01;0.01;0.01];
k_hat_3(:,1) = [0.01;0.01;0.01];
k_hat_4(:,1) = [0.01;0.01;0.01];
k_hat_5(:,1) = [0.01;0.01;0.01];


delta_hat_1(:,1) = zeros(3,1);
delta_hat_2(:,1) = zeros(3,1);
delta_hat_3(:,1) = zeros(3,1);
delta_hat_4(:,1) = zeros(3,1);
delta_hat_5(:,1) = zeros(3,1);


var1_1(:,1) = zeros(3,1);
var1_2(:,1) = zeros(3,1);
var1_3(:,1) = zeros(3,1);
var1_4(:,1) = zeros(3,1);
var1_5(:,1) = zeros(3,1);


lamda0_1(:,1) = zeros(3,1);
lamda0_2(:,1) = zeros(3,1);
lamda0_3(:,1) = zeros(3,1);
lamda0_4(:,1) = zeros(3,1);
lamda0_5(:,1) = zeros(3,1);


%desired trajectory parameters
d = 6;theta = 2*pi/3;beta = 0;
eta_d=[d*cos(theta);d*sin(theta);beta];


% Cost function parameters
gamma2 = 0.01;
Q = eye(3);
R = eye(3);

%% Main loop 

for k = 1:timeTest
    k
    

    if k < (40/step)
        eta_d_1(:,k) = [0.5*t(k);0;0];
    else
        eta_d_1(:,k) = [0.5*40 + 50*sin(0.01*(t(k)-40));50*(1-cos(0.01*(t(k)-40)));0.01*(t(k)-40)];
    end
    
    if k < (40/step)
        eta_d_dot_1 = [0.5;0;0];
    else
        eta_d_dot_1 = [0.5*cos(0.01*(t(k)-40));0.5*sin(0.01*(t(k)-40));0.01];
    end

    if k < (40/step)
        eta_d_dot_dot_1 = [0;0;0];
    else
        eta_d_dot_dot_1 = [-0.5*0.01*sin(0.01*(t(k)-40));0.5*0.01*cos(0.01*(t(k)-40));0];
    end

    Delta_1 = J(eta_1(:,k))*inv(M);
    Delta_2 = J(eta_2(:,k))*inv(M);
    Delta_3 = J(eta_3(:,k))*inv(M);
    Delta_4 = J(eta_4(:,k))*inv(M);
    Delta_5 = J(eta_5(:,k))*inv(M);


    eta_dot_1 = J(eta_1(:,k))*v_1(:,k);
    eta_dot_2 = J(eta_2(:,k))*v_2(:,k);
    eta_dot_3 = J(eta_3(:,k))*v_3(:,k);
    eta_dot_4 = J(eta_4(:,k))*v_4(:,k);
    eta_dot_5 = J(eta_5(:,k))*v_5(:,k);

    if k ==1
        v_dot_1 = zeros(3,1);
        v_dot_2 = zeros(3,1);
        v_dot_3 = zeros(3,1);
        v_dot_4 = zeros(3,1);
        v_dot_5 = zeros(3,1);
    end
    eta_dot_dot_1 = J_dot(eta_1(:,k))*eta_dot_1(3)*v_1(:,k) + J(eta_1(:,k))*v_dot_1;
    eta_dot_dot_2 = J_dot(eta_2(:,k))*eta_dot_2(3)*v_2(:,k) + J(eta_2(:,k))*v_dot_2;
    eta_dot_dot_3 = J_dot(eta_3(:,k))*eta_dot_3(3)*v_3(:,k) + J(eta_3(:,k))*v_dot_3;
    eta_dot_dot_4 = J_dot(eta_4(:,k))*eta_dot_4(3)*v_4(:,k) + J(eta_4(:,k))*v_dot_4;
    eta_dot_dot_5 = J_dot(eta_5(:,k))*eta_dot_5(3)*v_5(:,k) + J(eta_5(:,k))*v_dot_5;


    eta_d_2(:,k) = eta_1(:,k) + J(eta_1(:,k))*eta_d;
    eta_d_3(:,k) = eta_2(:,k) + J(eta_2(:,k))*eta_d;
    eta_d_4(:,k) = eta_3(:,k) + J(eta_3(:,k))*eta_d;
    eta_d_5(:,k) = eta_4(:,k) + J(eta_4(:,k))*eta_d;


    eta_d_dot_2 = eta_dot_1 + J_dot(eta_1(:,k))*eta_dot_1(3)*eta_d;
    eta_d_dot_3 = eta_dot_2 + J_dot(eta_2(:,k))*eta_dot_2(3)*eta_d;
    eta_d_dot_4 = eta_dot_3 + J_dot(eta_3(:,k))*eta_dot_3(3)*eta_d;
    eta_d_dot_5 = eta_dot_4 + J_dot(eta_4(:,k))*eta_dot_4(3)*eta_d;


    eta_d_dot_dot_2 = eta_dot_dot_1 + J_dot_dot(eta_1(:,k))*(eta_dot_1(3)^2)*eta_d +...
        J_dot(eta_1(:,k))*eta_dot_dot_1(3)*eta_d;
    eta_d_dot_dot_3 = eta_dot_dot_2 + J_dot_dot(eta_2(:,k))*(eta_dot_2(3)^2)*eta_d +...
        J_dot(eta_2(:,k))*eta_dot_dot_2(3)*eta_d;
    eta_d_dot_dot_4 = eta_dot_dot_3 + J_dot_dot(eta_3(:,k))*(eta_dot_3(3)^2)*eta_d +...
        J_dot(eta_3(:,k))*eta_dot_dot_3(3)*eta_d;
    eta_d_dot_dot_5 = eta_dot_dot_4 + J_dot_dot(eta_4(:,k))*(eta_dot_4(3)^2)*eta_d +...
        J_dot(eta_4(:,k))*eta_dot_dot_4(3)*eta_d;
    

    Gamma_1 = Delta_1*(l(v_1(:,k)) + J_dot(eta_1(:,k))*eta_dot_1(3)*v_1(:,k));
    Gamma_2 = Delta_2*(l(v_2(:,k)) + J_dot(eta_2(:,k))*eta_dot_2(3)*v_2(:,k));
    Gamma_3 = Delta_3*(l(v_3(:,k)) + J_dot(eta_3(:,k))*eta_dot_3(3)*v_3(:,k));
    Gamma_4 = Delta_4*(l(v_4(:,k)) + J_dot(eta_4(:,k))*eta_dot_4(3)*v_4(:,k));
    Gamma_5 = Delta_5*(l(v_5(:,k)) + J_dot(eta_5(:,k))*eta_dot_5(3)*v_5(:,k));

    z1_1(:,k) = eta_1(:,k) - eta_d_1(:,k);
    z1_2(:,k) = eta_2(:,k) - eta_d_2(:,k);
    z1_3(:,k) = eta_3(:,k) - eta_d_3(:,k);
    z1_4(:,k) = eta_4(:,k) - eta_d_4(:,k);
    z1_5(:,k) = eta_5(:,k) - eta_d_5(:,k);


    z1_dot_1 = eta_dot_1 - eta_d_dot_1;
    z1_dot_2 = eta_dot_2 - eta_d_dot_2;
    z1_dot_3 = eta_dot_3 - eta_d_dot_3;
    z1_dot_4 = eta_dot_4 - eta_d_dot_4;
    z1_dot_5 = eta_dot_5 - eta_d_dot_5;

    alpha1_1(:,k) = -k1*z1_1(:,k) + eta_d_dot_1;
    alpha1_2(:,k) = -k1*z1_2(:,k) + eta_d_dot_2;
    alpha1_3(:,k) = -k1*z1_3(:,k) + eta_d_dot_3;
    alpha1_4(:,k) = -k1*z1_4(:,k) + eta_d_dot_4;
    alpha1_5(:,k) = -k1*z1_5(:,k) + eta_d_dot_5;


    z2_1(:,k) = eta_dot_1 - alpha1_1(:,k);
    z2_2(:,k) = eta_dot_2 - alpha1_2(:,k);
    z2_3(:,k) = eta_dot_3 - alpha1_3(:,k);
    z2_4(:,k) = eta_dot_4 - alpha1_4(:,k);
    z2_5(:,k) = eta_dot_5 - alpha1_5(:,k);


    u0_1 = inv(Delta_1)*(-Gamma_1 + eta_d_dot_dot_1 - k1*z1_dot_1 - z1_1(:,k) - k2*z2_1(:,k));
    u0_2 = inv(Delta_2)*(-Gamma_2 + eta_d_dot_dot_2 - k1*z1_dot_2 - z1_2(:,k) - k2*z2_2(:,k));
    u0_3 = inv(Delta_3)*(-Gamma_3 + eta_d_dot_dot_3 - k1*z1_dot_3 - z1_3(:,k) - k2*z2_3(:,k));
    u0_4 = inv(Delta_4)*(-Gamma_4 + eta_d_dot_dot_4 - k1*z1_dot_4 - z1_4(:,k) - k2*z2_4(:,k));
    u0_5 = inv(Delta_5)*(-Gamma_5 + eta_d_dot_dot_5 - k1*z1_dot_5 - z1_5(:,k) - k2*z2_5(:,k));
    
    delta_hat_1(:,k) = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    delta_hat_2(:,k) = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    delta_hat_3(:,k) = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    delta_hat_4(:,k) = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    delta_hat_5(:,k) = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    % delta_hat_5(:,k) = [1.3 + 2*sin(0.02*t(k)) + 1.5*sin(0.1*t(k));
    %                 -0.9 + 2*sin(0.02*t(k) - pi/6) + 1.5*sin(0.3*t(k));
    %                 -sin(0.09*t(k) + pi/3) - 4*sin(0.01*t(k))];



    % % ADDING DISTURBANCE OBSERVER
    % e_phi_hat_dot_1 = -delta_s*(Delta_1*u0_1 + l(v_1(:,k)) - eta_d_dot_dot_1 + K*z1_dot_1);
    % e_phi_hat_dot_2 = -delta_s*(Delta_2*u0_2 + l(v_2(:,k)) - eta_d_dot_dot_2 + K*z1_dot_2);
    % e_phi_hat_dot_3 = -delta_s*(Delta_3*u0_3 + l(v_3(:,k)) - eta_d_dot_dot_3 + K*z1_dot_3);
    % e_phi_hat_dot_4 = -delta_s*(Delta_4*u0_4 + l(v_4(:,k)) - eta_d_dot_dot_4 + K*z1_dot_4);
    % e_phi_hat_dot_5 = -delta_s*(Delta_5*u0_5 + l(v_5(:,k)) - eta_d_dot_dot_5 + K*z1_dot_5);
    % 
    % 
    % s_hat_dot_1 = Delta_1*u0_1 + l(v_1(:,k)) - eta_d_dot_dot_1 + K*z1_dot_1 + delta_hat_1(:,k);
    % s_hat_dot_2 = Delta_2*u0_2 + l(v_2(:,k)) - eta_d_dot_dot_2 + K*z1_dot_2 + delta_hat_2(:,k);
    % s_hat_dot_3 = Delta_3*u0_3 + l(v_3(:,k)) - eta_d_dot_dot_3 + K*z1_dot_3 + delta_hat_3(:,k);
    % s_hat_dot_4 = Delta_4*u0_4 + l(v_4(:,k)) - eta_d_dot_dot_4 + K*z1_dot_4 + delta_hat_4(:,k);
    % s_hat_dot_5 = Delta_5*u0_5 + l(v_5(:,k)) - eta_d_dot_dot_5 + K*z1_dot_5 + delta_hat_5(:,k);
    % 
    % 
    % s_1(:,k) = z1_dot_1 + K*z1_1(:,k);
    % s_2(:,k) = z1_dot_2 + K*z1_2(:,k);
    % s_3(:,k) = z1_dot_3 + K*z1_3(:,k);
    % s_4(:,k) = z1_dot_4 + K*z1_4(:,k);
    % s_5(:,k) = z1_dot_5 + K*z1_5(:,k);
    % 
    % 
    % theta_1 = s_1(:,k) - s_hat_1(:,k);
    % theta_2 = s_2(:,k) - s_hat_2(:,k);
    % theta_3 = s_3(:,k) - s_hat_3(:,k);
    % theta_4 = s_4(:,k) - s_hat_4(:,k);
    % theta_5 = s_5(:,k) - s_hat_5(:,k);
    % 
    % lamda0_dot_1 = -epsi1*sqrt(abs(theta_1 - lamda0_1(:,k))).*sign(theta_1-lamda0_1(:,k)) + var1_1(:,k);
    % lamda0_dot_2 = -epsi1*sqrt(abs(theta_2 - lamda0_2(:,k))).*sign(theta_2-lamda0_2(:,k)) + var1_2(:,k);
    % lamda0_dot_3 = -epsi1*sqrt(abs(theta_3 - lamda0_3(:,k))).*sign(theta_3-lamda0_3(:,k)) + var1_3(:,k);
    % lamda0_dot_4 = -epsi1*sqrt(abs(theta_4 - lamda0_4(:,k))).*sign(theta_4-lamda0_4(:,k)) + var1_4(:,k);
    % lamda0_dot_5 = -epsi1*sqrt(abs(theta_5 - lamda0_5(:,k))).*sign(theta_5-lamda0_5(:,k)) + var1_5(:,k);
    % 
    % 
    % var1_dot_1 = -epsi2*sign(var1_1(:,k) - lamda0_dot_1);
    % var1_dot_2 = -epsi2*sign(var1_2(:,k) - lamda0_dot_2);
    % var1_dot_3 = -epsi2*sign(var1_3(:,k) - lamda0_dot_3);
    % var1_dot_4 = -epsi2*sign(var1_4(:,k) - lamda0_dot_4);
    % var1_dot_5 = -epsi2*sign(var1_5(:,k) - lamda0_dot_5);
    % 
    % 
    % lamda1_1 = lamda0_dot_1;
    % lamda1_2 = lamda0_dot_2;
    % lamda1_3 = lamda0_dot_3;
    % lamda1_4 = lamda0_dot_4;
    % lamda1_5 = lamda0_dot_5;
    % 
    % 
    % delta_hat_dot_1 = e_phi_hat_dot_1 + delta_s*s_hat_dot_1 + slidingGain.*sign(lamda1_1);
    % delta_hat_dot_2 = e_phi_hat_dot_2 + delta_s*s_hat_dot_2 + slidingGain.*sign(lamda1_2);
    % delta_hat_dot_3 = e_phi_hat_dot_3 + delta_s*s_hat_dot_3 + slidingGain.*sign(lamda1_3);
    % delta_hat_dot_4 = e_phi_hat_dot_4 + delta_s*s_hat_dot_4 + slidingGain.*sign(lamda1_4);
    % delta_hat_dot_5 = e_phi_hat_dot_5 + delta_s*s_hat_dot_5 + slidingGain.*sign(lamda1_5);


    ud_1 = -inv(Delta_1)*delta_hat_1(:,k);
    ud_2 = -inv(Delta_2)*delta_hat_2(:,k);
    ud_3 = -inv(Delta_3)*delta_hat_3(:,k);
    ud_4 = -inv(Delta_4)*delta_hat_4(:,k);
    ud_5 = -inv(Delta_5)*delta_hat_5(:,k);
    
    sigma_dot_1 = -(rho + w_hat)*sign(sigma_1(:,k)) + delta_hat_1(:,k);
    sigma_dot_2 = -(rho + w_hat)*sign(sigma_2(:,k)) + delta_hat_2(:,k);
    sigma_dot_3 = -(rho + w_hat)*sign(sigma_3(:,k)) + delta_hat_3(:,k);
    sigma_dot_4 = -(rho + w_hat)*sign(sigma_4(:,k)) + delta_hat_4(:,k);
    sigma_dot_5 = -(rho + w_hat)*sign(sigma_5(:,k)) + delta_hat_5(:,k);


    if abs(sigma_1(:,k)) > deadzoneSize
        k_hat_dot_1 = abs(sigma_1(:,k))./gamma;
    else
        k_hat_dot_1 = [0;0;0];
    end
    if abs(sigma_2(:,k)) > deadzoneSize
        k_hat_dot_2 = abs(sigma_2(:,k))./gamma;
    else
        k_hat_dot_2 = [0;0;0];
    end
    if abs(sigma_3(:,k)) > deadzoneSize
        k_hat_dot_3 = abs(sigma_3(:,k))./gamma;
    else
        k_hat_dot_3 = [0;0;0];
    end
    if abs(sigma_4(:,k)) > deadzoneSize
        k_hat_dot_4 = abs(sigma_4(:,k))./gamma;
    else
        k_hat_dot_4 = [0;0;0];
    end
    if abs(sigma_5(:,k)) > deadzoneSize
        k_hat_dot_5 = abs(sigma_5(:,k))./gamma;
    else
        k_hat_dot_5 = [0;0;0];
    end


    us_1 = -inv(Delta_1)*(k_hat_1(:,k) + w_hat).*sign(sigma_1(:,k));
    us_2 = -inv(Delta_2)*(k_hat_2(:,k) + w_hat).*sign(sigma_2(:,k));
    us_3 = -inv(Delta_3)*(k_hat_3(:,k) + w_hat).*sign(sigma_3(:,k));
    us_4 = -inv(Delta_4)*(k_hat_4(:,k) + w_hat).*sign(sigma_4(:,k));
    us_5 = -inv(Delta_5)*(k_hat_5(:,k) + w_hat).*sign(sigma_5(:,k));


    tau_1(:,k) = u0_1 + ud_1 + us_1;
    tau_2(:,k) = u0_2 + ud_2 + us_2;
    tau_3(:,k) = u0_3 + ud_3 + us_3;
    tau_4(:,k) = u0_4 + ud_4 + us_4;
    tau_5(:,k) = u0_5 + ud_5 + us_5;


    v_dot_1 = M\(tau_1(:,k) -l(v_1(:,k)));
    v_dot_2 = M\(tau_2(:,k) -l(v_2(:,k)));
    v_dot_3 = M\(tau_3(:,k) -l(v_3(:,k)));
    v_dot_4 = M\(tau_4(:,k) -l(v_4(:,k)));
    v_dot_5 = M\(tau_5(:,k) -l(v_5(:,k)));


    v_1(:,k+1) = v_1(:,k) + step*v_dot_1;
    v_2(:,k+1) = v_2(:,k) + step*v_dot_2;
    v_3(:,k+1) = v_3(:,k) + step*v_dot_3;
    v_4(:,k+1) = v_4(:,k) + step*v_dot_4;
    v_5(:,k+1) = v_5(:,k) + step*v_dot_5;


    eta_1(:,k+1) = eta_1(:,k) + step*eta_dot_1;
    eta_2(:,k+1) = eta_2(:,k) + step*eta_dot_2;
    eta_3(:,k+1) = eta_3(:,k) + step*eta_dot_3;
    eta_4(:,k+1) = eta_4(:,k) + step*eta_dot_4;
    eta_5(:,k+1) = eta_5(:,k) + step*eta_dot_5;



    sigma_1(:,k+1) = sigma_1(:,k) + step*sigma_dot_1;
    sigma_2(:,k+1) = sigma_2(:,k) + step*sigma_dot_2;
    sigma_3(:,k+1) = sigma_3(:,k) + step*sigma_dot_3;
    sigma_4(:,k+1) = sigma_4(:,k) + step*sigma_dot_4;
    sigma_5(:,k+1) = sigma_5(:,k) + step*sigma_dot_5;


    k_hat_1(:,k+1) = k_hat_1(:,k) + step*k_hat_dot_1;
    k_hat_2(:,k+1) = k_hat_2(:,k) + step*k_hat_dot_2;
    k_hat_3(:,k+1) = k_hat_3(:,k) + step*k_hat_dot_3;
    k_hat_4(:,k+1) = k_hat_4(:,k) + step*k_hat_dot_4;
    k_hat_5(:,k+1) = k_hat_5(:,k) + step*k_hat_dot_5;


    costFunction_1(:,k) = cumsum(exp(-gamma2*t(k))*(z1_1(:,k)'*Q*z1_1(:,k) + tau_1(:,k)'*R*tau_1(:,k)));
    costFunction_2(:,k) = cumsum(exp(-gamma2*t(k))*(z1_2(:,k)'*Q*z1_2(:,k) + tau_2(:,k)'*R*tau_2(:,k)));
    costFunction_3(:,k) = cumsum(exp(-gamma2*t(k))*(z1_3(:,k)'*Q*z1_3(:,k) + tau_3(:,k)'*R*tau_3(:,k)));
    costFunction_4(:,k) = cumsum(exp(-gamma2*t(k))*(z1_4(:,k)'*Q*z1_4(:,k) + tau_4(:,k)'*R*tau_4(:,k)));
    costFunction_5(:,k) = cumsum(exp(-gamma2*t(k))*(z1_5(:,k)'*Q*z1_5(:,k) + tau_5(:,k)'*R*tau_5(:,k)));


    % %DO things
    % delta_hat_1(:,k+1) = delta_hat_1(:,k) + step*delta_hat_dot_1;
    % delta_hat_2(:,k+1) = delta_hat_2(:,k) + step*delta_hat_dot_2;
    % delta_hat_3(:,k+1) = delta_hat_3(:,k) + step*delta_hat_dot_3;
    % delta_hat_4(:,k+1) = delta_hat_4(:,k) + step*delta_hat_dot_4;
    % delta_hat_5(:,k+1) = delta_hat_5(:,k) + step*delta_hat_dot_5;
    % 
    % 
    % var1_1(:,k+1) = var1_1(:,k) + step*var1_dot_1;
    % var1_2(:,k+1) = var1_2(:,k) + step*var1_dot_2;
    % var1_3(:,k+1) = var1_3(:,k) + step*var1_dot_3;
    % var1_4(:,k+1) = var1_4(:,k) + step*var1_dot_4;
    % var1_5(:,k+1) = var1_5(:,k) + step*var1_dot_5;
    % 
    % 
    % lamda0_1(:,k+1) = lamda0_1(:,k) + step*lamda0_dot_1;
    % lamda0_2(:,k+1) = lamda0_2(:,k) + step*lamda0_dot_2;
    % lamda0_3(:,k+1) = lamda0_3(:,k) + step*lamda0_dot_3;
    % lamda0_4(:,k+1) = lamda0_4(:,k) + step*lamda0_dot_4;
    % lamda0_5(:,k+1) = lamda0_5(:,k) + step*lamda0_dot_5;
    % 
    % 
    % s_hat_1(:,k+1) = s_hat_1(:,k) + step*s_hat_dot_1;
    % s_hat_2(:,k+1) = s_hat_2(:,k) + step*s_hat_dot_2;
    % s_hat_3(:,k+1) = s_hat_3(:,k) + step*s_hat_dot_3;
    % s_hat_4(:,k+1) = s_hat_4(:,k) + step*s_hat_dot_4;
    % s_hat_5(:,k+1) = s_hat_5(:,k) + step*s_hat_dot_5;
end
figure(1)
plot(eta_1(1,1:timeTest),eta_1(2,1:timeTest))
hold on
plot(eta_2(1,1:timeTest),eta_2(2,1:timeTest))
plot(eta_3(1,1:timeTest),eta_3(2,1:timeTest))
plot(eta_4(1,1:timeTest),eta_4(2,1:timeTest))
plot(eta_5(1,1:timeTest),eta_5(2,1:timeTest))

%% Save as txt file 

trackingError_dat = [z1_1(1,1:100:timeTest)',z1_2(1,1:100:timeTest)',z1_3(1,1:100:timeTest)',z1_4(1,1:100:timeTest)',z1_5(1,1:100:timeTest)',...
    z1_1(2,1:100:timeTest)',z1_2(2,1:100:timeTest)',z1_3(2,1:100:timeTest)',z1_4(2,1:100:timeTest)',z1_5(2,1:100:timeTest)',...
    z1_1(3,1:100:timeTest)',z1_2(3,1:100:timeTest)',z1_3(3,1:100:timeTest)',z1_4(3,1:100:timeTest)',z1_5(3,1:100:timeTest)',t(1:100:timeTest)'];
header = {'etae1x','etae2x','etae3x','etae4x','etae5x',...
    'etae1y','etae2y','etae3y','etae4y','etae5y',...
    'etae1phi','etae2phi','etae3phi','etae4phi','etae5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_trackingError.txt');
writematrix (trackingError_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_trackingError.txt','WriteMode','Append');

tau_dat = [tau_1(1,1:100:timeTest)',tau_2(1,1:100:timeTest)',tau_3(1,1:100:timeTest)',tau_4(1,1:100:timeTest)',tau_5(1,1:100:timeTest)',...
    tau_1(2,1:100:timeTest)',tau_2(2,1:100:timeTest)',tau_3(2,1:100:timeTest)',tau_4(2,1:100:timeTest)',tau_5(2,1:100:timeTest)',...
    tau_1(3,1:100:timeTest)',tau_2(3,1:100:timeTest)',tau_3(3,1:100:timeTest)',tau_4(3,1:100:timeTest)',tau_5(3,1:100:timeTest)',t(1:100:timeTest)'];
header = {'tau1x','tau2x','tau3x','tau4x','tau5x',...
    'tau1y','tau2y','tau3y','tau4y','tau5y',...
    'tau1phi','tau2phi','tau3phi','tau4phi','tau5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_tau.txt');
writematrix (tau_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_tau.txt','WriteMode','Append');

costFunction_dat = [costFunction_1(1:100:timeTest)',costFunction_2(1:100:timeTest)',costFunction_3(1:100:timeTest)',costFunction_4(1:100:timeTest)',...
    costFunction_5(1:100:timeTest)',t(1:100:timeTest)'];
header = {'costFunction1','costFunction2','costFunction3','costFunction4','costFunction5','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_costFunction.txt');
writematrix (costFunction_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_costFunction.txt','WriteMode','Append');

% %% Function 
% 
% function a = C(v)
%     C = [0,0,-33.8*v(2)-1.0948*v(3);...
%         0,0,25.8*v(1);...
%         33.8*v(2)+1.0948*v(3),-25.8*v(1),0];
%     a = C;
% end
% function a = D(v)
%     D = [0.7225 + 1.3274*abs(v(1)) + 5.8664*v(1)^2,0,0;...
%          0,0.8612+36.2823*abs(v(1)) + 0.805*abs(v(3)),-0.1079 + 0.845*abs(v(1)) + 3.45*abs(v(3));...
%          0,-0.1052 - 5.0437*abs(v(1)) - 0.13*abs(v(3)),1.9-0.08*abs(v(2))+0.75*abs(v(3))];
%     a = D;
% end
% function a = l(v)
%     l = C(v)*v + D(v)*v;
%     a = l;
% end
% function a = J(eta)
%     J=[cos(eta(3)),-sin(eta(3)),0;
%     sin(eta(3)),cos(eta(3)),0;
%     0,0,1];
%     a = J;
% end
% function a = J_dot(eta)
%     J_dot=[-sin(eta(3)),-cos(eta(3)),0;
%     cos(eta(3)),-sin(eta(3)),0;
%     0,0,0];
%     a = J_dot;
% end
% function a = J_dot_dot(eta)
%     %need to add eta_dot
%     J_dot_dot=[-cos(eta(3)),sin(eta(3)),0;
%     -sin(eta(3)),-cos(eta(3)),0;
%     0,0,0];
%     a = J_dot_dot;
% end
% function a = sig(beta,X)
%     sig = [sign(X(1))*abs(X(1))^beta;
%                 sign(X(2))*abs(X(2))^beta;
%                 sign(X(3))*abs(X(3))^beta];
%     a = sig;
% end

%Fixed-time extended state observer-based trajectory tracking and point
%stabilization control for marine surface vessels with uncertainties and
%disturbances

%

%% Parameters
clear;
clc;
close all;
T_end = 500;     % run 40s simulation
step = 0.001;   % step time = 0.001s
t = 0:step:T_end;
timeTest = T_end/step;

%Controller parameters
miu1 = 10;
epsi1= 10;
miu2=20;
epsi2 = 20;
miu3 = 50;
epsi3 = 50;
alpha1 = 0.8;
alpha2 = 0.6;
alpha3 = 0.4;
beta1 = 2.2;
beta2 = 3.4;
beta3 = 2.6;
k1 = 80;
k2 = 80;
gamma1 = 0.3;
gamma2 = 2*gamma1/(1+gamma1);
Gamma = 5;

% Initial condition 
eta_hat_1(:,1) = [0;0.5;0];
eta_hat_2(:,1) = [-3.5;5;0];
eta_hat_3(:,1) = [-7;10;0];
eta_hat_4(:,1) = [-10.5;15;0.2];
eta_hat_5(:,1) = [-14;20;0.4];


w_hat_1(:,1) = [0;0;0];
w_hat_2(:,1) = [0;0;0];
w_hat_3(:,1) = [0;0;0];
w_hat_4(:,1) = [0;0;0];
w_hat_5(:,1) = [0;0;0];


X_hat_1(:,1) = [0;0;0];
X_hat_2(:,1) = [0;0;0];
X_hat_3(:,1) = [0;0;0];
X_hat_4(:,1) = [0;0;0];
X_hat_5(:,1) = [0;0;0];

noise = zeros(3,1);

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

w_1(:,1) = [0;0;0];
w_2(:,1) = [0;0;0];
w_3(:,1) = [0;0;0];
w_4(:,1) = [0;0;0];
w_5(:,1) = [0;0;0];

M = [25.8,0,0;0,33.8,1.0948;0,1.0948,2.76];

%desired trajectory parameters
d = 6;theta = 2*pi/3;beta = 0;
eta_d=[d*cos(theta);d*sin(theta);beta];

%cost function parameters
gamma = 0.01;
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

    eta_dot_1 = J(eta_1(:,k))*v_1(:,k);
    eta_dot_2 = J(eta_2(:,k))*v_2(:,k);
    eta_dot_3 = J(eta_3(:,k))*v_3(:,k);
    eta_dot_4 = J(eta_4(:,k))*v_4(:,k);
    eta_dot_5 = J(eta_5(:,k))*v_5(:,k);
    
    % eta_dot_dot_1 = J_dot(eta_1(:,k))*eta_dot_1(3)*v_1(:,k) + J(eta_1(:,k))*v_dot_1;
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
    

    eta_e_1(:,k) = eta_1(:,k) - eta_d_1(:,k);
    eta_e_2(:,k) = eta_2(:,k) - eta_d_2(:,k);
    eta_e_3(:,k) = eta_3(:,k) - eta_d_3(:,k);
    eta_e_4(:,k) = eta_4(:,k) - eta_d_4(:,k);
    eta_e_5(:,k) = eta_5(:,k) - eta_d_5(:,k);


    w_d_1(:,k) = eta_d_dot_1;
    w_d_2(:,k) = eta_d_dot_2;
    w_d_3(:,k) = eta_d_dot_3;
    w_d_4(:,k) = eta_d_dot_4;
    w_d_5(:,k) = eta_d_dot_5;


    v_d_1(:,k) = J(eta_1(:,k))'*w_d_1(:,k);
    v_d_2(:,k) = J(eta_2(:,k))'*w_d_2(:,k);
    v_d_3(:,k) = J(eta_3(:,k))'*w_d_3(:,k);
    v_d_4(:,k) = J(eta_4(:,k))'*w_d_4(:,k);
    v_d_5(:,k) = J(eta_5(:,k))'*w_d_5(:,k);

  
    w_d_dot_1 = eta_d_dot_dot_1;
    w_d_dot_2 = eta_d_dot_dot_2;
    w_d_dot_3 = eta_d_dot_dot_3;
    w_d_dot_4 = eta_d_dot_dot_4;
    w_d_dot_5 = eta_d_dot_dot_5;

    
    e2_hat_1(:,k) = w_hat_1(:,k) - w_d_1(:,k);
    e2_hat_2(:,k) = w_hat_2(:,k) - w_d_2(:,k);
    e2_hat_3(:,k) = w_hat_3(:,k) - w_d_3(:,k);
    e2_hat_4(:,k) = w_hat_4(:,k) - w_d_4(:,k);
    e2_hat_5(:,k) = w_hat_5(:,k) - w_d_5(:,k);


    eta_hat_dot_1 = w_hat_1(:,k) + miu1*sig(alpha1, eta_1(:,k) - eta_hat_1(:,k)) + epsi1*sig(beta1,eta_1(:,k)-eta_hat_1(:,k));
    eta_hat_dot_2 = w_hat_2(:,k) + miu1*sig(alpha1, eta_2(:,k) - eta_hat_2(:,k)) + epsi1*sig(beta1,eta_2(:,k)-eta_hat_2(:,k));
    eta_hat_dot_3 = w_hat_3(:,k) + miu1*sig(alpha1, eta_3(:,k) - eta_hat_3(:,k)) + epsi1*sig(beta1,eta_3(:,k)-eta_hat_3(:,k));
    eta_hat_dot_4 = w_hat_4(:,k) + miu1*sig(alpha1, eta_4(:,k) - eta_hat_4(:,k)) + epsi1*sig(beta1,eta_4(:,k)-eta_hat_4(:,k));
    eta_hat_dot_5 = w_hat_5(:,k) + miu1*sig(alpha1, eta_5(:,k) - eta_hat_5(:,k)) + epsi1*sig(beta1,eta_5(:,k)-eta_hat_5(:,k));

    if k < 15/step
        noise = 0.1*[(10*sin(25*t(k)) + 16*sin(45*(t(k)-1)) + 15*sin(20*(t(k)-0.5)) + 10*sin(35*(t(k)-1.5)));
         (50*sin(25*t(k)) + 50*sin(45*(t(k)-1)) + 40*sin(20*(t(k)-0.5)) + 50*sin(35*(t(k)-1.5)));
        (40*sin(25*t(k)) + 30*sin(45*(t(k)-1)) + 45*sin(20*(t(k)-0.5)) + 40*sin(35*(t(k)-1.5)))];
    else
        noise = zeros(3,1);
    end

    
    tau_1(:,k) = M*inv(J(eta_1(:,k)))*(-k1*sig(gamma1,eta_e_1(:,k)) - k2*sig(gamma2,e2_hat_1(:,k)) - X_hat_1(:,k) + w_d_dot_1);
    tau_2(:,k) = M*inv(J(eta_2(:,k)))*(-k1*sig(gamma1,eta_e_2(:,k)) - k2*sig(gamma2,e2_hat_2(:,k)) - X_hat_2(:,k) + w_d_dot_2);
    tau_3(:,k) = M*inv(J(eta_3(:,k)))*(-k1*sig(gamma1,eta_e_3(:,k)) - k2*sig(gamma2,e2_hat_3(:,k)) - X_hat_3(:,k) + w_d_dot_3);
    tau_4(:,k) = M*inv(J(eta_4(:,k)))*(-k1*sig(gamma1,eta_e_4(:,k)) - k2*sig(gamma2,e2_hat_4(:,k)) - X_hat_4(:,k) + w_d_dot_4);
    tau_5(:,k) = M*inv(J(eta_5(:,k)))*(-k1*sig(gamma1,eta_e_5(:,k)) - k2*sig(gamma2,e2_hat_5(:,k)) - X_hat_5(:,k) + w_d_dot_5);
    
    w_hat_dot_1 = J(eta_1(:,k))*inv(M)*tau_1(:,k) + X_hat_1(:,k) + miu2*sig(alpha2,eta_1(:,k)-eta_d_1(:,k)) + epsi2*sig(beta2,eta_1(:,k)-eta_hat_1(:,k));
    w_hat_dot_2 = J(eta_2(:,k))*inv(M)*tau_2(:,k) + X_hat_2(:,k) + miu2*sig(alpha2,eta_2(:,k)-eta_d_2(:,k)) + epsi2*sig(beta2,eta_2(:,k)-eta_hat_2(:,k));
    w_hat_dot_3 = J(eta_3(:,k))*inv(M)*tau_3(:,k) + X_hat_3(:,k) + miu2*sig(alpha2,eta_3(:,k)-eta_d_3(:,k)) + epsi2*sig(beta2,eta_3(:,k)-eta_hat_3(:,k));
    w_hat_dot_4 = J(eta_4(:,k))*inv(M)*tau_4(:,k) + X_hat_4(:,k) + miu2*sig(alpha2,eta_4(:,k)-eta_d_4(:,k)) + epsi2*sig(beta2,eta_4(:,k)-eta_hat_4(:,k));
    w_hat_dot_5 = J(eta_5(:,k))*inv(M)*tau_5(:,k) + X_hat_5(:,k) + miu2*sig(alpha2,eta_5(:,k)-eta_d_5(:,k)) + epsi2*sig(beta2,eta_5(:,k)-eta_hat_5(:,k));

    X_hat_dot_1 = miu3*sig(alpha3, eta_1(:,k)-eta_hat_1(:,k)) + epsi3*sig(beta3, eta_1(:,k)-eta_hat_1(:,k)) + Gamma*sign(eta_1(:,k)-eta_hat_1(:,k));
    X_hat_dot_2 = miu3*sig(alpha3, eta_2(:,k)-eta_hat_2(:,k)) + epsi3*sig(beta3, eta_2(:,k)-eta_hat_2(:,k)) + Gamma*sign(eta_2(:,k)-eta_hat_2(:,k));
    X_hat_dot_3 = miu3*sig(alpha3, eta_3(:,k)-eta_hat_3(:,k)) + epsi3*sig(beta3, eta_3(:,k)-eta_hat_3(:,k)) + Gamma*sign(eta_3(:,k)-eta_hat_3(:,k));
    X_hat_dot_4 = miu3*sig(alpha3, eta_4(:,k)-eta_hat_4(:,k)) + epsi3*sig(beta3, eta_4(:,k)-eta_hat_4(:,k)) + Gamma*sign(eta_4(:,k)-eta_hat_4(:,k));
    X_hat_dot_5 = miu3*sig(alpha3, eta_5(:,k)-eta_hat_5(:,k)) + epsi3*sig(beta3, eta_5(:,k)-eta_hat_5(:,k)) + Gamma*sign(eta_5(:,k)-eta_hat_5(:,k));
    
    w_hat_1(:,k+1) = w_hat_1(:,k) + step*w_hat_dot_1;
    w_hat_2(:,k+1) = w_hat_2(:,k) + step*w_hat_dot_2;
    w_hat_3(:,k+1) = w_hat_3(:,k) + step*w_hat_dot_3;
    w_hat_4(:,k+1) = w_hat_4(:,k) + step*w_hat_dot_4;
    w_hat_5(:,k+1) = w_hat_5(:,k) + step*w_hat_dot_5;


    X_hat_1(:,k+1) = X_hat_1(:,k) + step*X_hat_dot_1;
    X_hat_2(:,k+1) = X_hat_2(:,k) + step*X_hat_dot_2;
    X_hat_3(:,k+1) = X_hat_3(:,k) + step*X_hat_dot_3;
    X_hat_4(:,k+1) = X_hat_4(:,k) + step*X_hat_dot_4;
    X_hat_5(:,k+1) = X_hat_5(:,k) + step*X_hat_dot_5;

    
    v_dot_1 = M\(tau_1(:,k) + noise -l(v_1(:,k)));
    v_dot_2 = M\(tau_2(:,k) + noise -l(v_2(:,k)));
    v_dot_3 = M\(tau_3(:,k) + noise -l(v_3(:,k)));
    v_dot_4 = M\(tau_4(:,k) + noise -l(v_4(:,k)));
    v_dot_5 = M\(tau_5(:,k) + noise -l(v_5(:,k)));


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



    eta_hat_1(:,k+1) = eta_hat_1(:,k) + step*eta_hat_dot_1;
    eta_hat_2(:,k+1) = eta_hat_2(:,k) + step*eta_hat_dot_2;
    eta_hat_3(:,k+1) = eta_hat_3(:,k) + step*eta_hat_dot_3;
    eta_hat_4(:,k+1) = eta_hat_4(:,k) + step*eta_hat_dot_4;
    eta_hat_5(:,k+1) = eta_hat_5(:,k) + step*eta_hat_dot_5;
    
    costFunction_1(:,k) = cumsum(exp(-gamma*t(k))*(eta_e_1(:,k)'*Q*eta_e_1(:,k) + tau_1(:,k)'*R*tau_1(:,k)));
    costFunction_2(:,k) = cumsum(exp(-gamma*t(k))*(eta_e_2(:,k)'*Q*eta_e_2(:,k) + tau_2(:,k)'*R*tau_2(:,k)));
    costFunction_3(:,k) = cumsum(exp(-gamma*t(k))*(eta_e_3(:,k)'*Q*eta_e_3(:,k) + tau_3(:,k)'*R*tau_3(:,k)));
    costFunction_4(:,k) = cumsum(exp(-gamma*t(k))*(eta_e_4(:,k)'*Q*eta_e_4(:,k) + tau_4(:,k)'*R*tau_4(:,k)));
    costFunction_5(:,k) = cumsum(exp(-gamma*t(k))*(eta_e_5(:,k)'*Q*eta_e_5(:,k) + tau_5(:,k)'*R*tau_5(:,k)));

end

% figure(1)
% plot(eta_1(1,1:timeTest),eta_1(2,1:timeTest))
% hold on
% plot(eta_2(1,1:timeTest),eta_2(2,1:timeTest))
% plot(eta_3(1,1:timeTest),eta_3(2,1:timeTest))
% plot(eta_4(1,1:timeTest),eta_4(2,1:timeTest))
% plot(eta_5(1,1:timeTest),eta_5(2,1:timeTest))

% figure(2)
% plot(t(1:timeTest),e2_hat(1,1:timeTest))
% hold on
% plot(t(1:timeTest),e2_hat(2,1:timeTest))
% plot(t(1:timeTest),e2_hat(3,1:timeTest))

%% Save as txt file 

trackingError_dat = [eta_e_1(1,1:100:timeTest)',eta_e_2(1,1:100:timeTest)',eta_e_3(1,1:100:timeTest)',eta_e_4(1,1:100:timeTest)',eta_e_5(1,1:100:timeTest)',...
    eta_e_1(2,1:100:timeTest)',eta_e_2(2,1:100:timeTest)',eta_e_3(2,1:100:timeTest)',eta_e_4(2,1:100:timeTest)',eta_e_5(2,1:100:timeTest)',...
    eta_e_1(3,1:100:timeTest)',eta_e_2(3,1:100:timeTest)',eta_e_3(3,1:100:timeTest)',eta_e_4(3,1:100:timeTest)',eta_e_5(3,1:100:timeTest)',t(1:100:timeTest)'];
header = {'etae1x','etae2x','etae3x','etae4x','etae5x',...
    'etae1y','etae2y','etae3y','etae4y','etae5y',...
    'etae1phi','etae2phi','etae3phi','etae4phi','etae5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_trackingError.txt');
writematrix (trackingError_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_trackingError.txt','WriteMode','Append');

tau_dat = [tau_1(1,1:100:timeTest)',tau_2(1,1:100:timeTest)',tau_3(1,1:100:timeTest)',tau_4(1,1:100:timeTest)',tau_5(1,1:100:timeTest)',...
    tau_1(2,1:100:timeTest)',tau_2(2,1:100:timeTest)',tau_3(2,1:100:timeTest)',tau_4(2,1:100:timeTest)',tau_5(2,1:100:timeTest)',...
    tau_1(3,1:100:timeTest)',tau_2(3,1:100:timeTest)',tau_3(3,1:100:timeTest)',tau_4(3,1:100:timeTest)',tau_5(3,1:100:timeTest)',t(1:100:timeTest)'];
header = {'tau1x','tau2x','tau3x','tau4x','tau5x',...
    'tau1y','tau2y','tau3y','tau4y','tau5y',...
    'tau1phi','tau2phi','tau3phi','tau4phi','tau5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_tau.txt');
writematrix (tau_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_tau.txt','WriteMode','Append');

costFunction_dat = [costFunction_1(1:100:timeTest)',costFunction_2(1:100:timeTest)',costFunction_3(1:100:timeTest)',costFunction_4(1:100:timeTest)',...
    costFunction_5(1:100:timeTest)',t(1:100:timeTest)'];
header = {'costFunction1','costFunction2','costFunction3','costFunction4','costFunction5','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_costFunction.txt');
writematrix (costFunction_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_costFunction.txt','WriteMode','Append');


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
%     %need to add eta_dot
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


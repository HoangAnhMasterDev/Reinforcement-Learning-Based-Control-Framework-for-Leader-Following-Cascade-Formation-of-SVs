% Get data from simulink and convert into txt file
% Run after actor-critic-based controller simulink file

T_end = 500;     
step = 0.001; 
t = 0:step:T_end;
timeTest = T_end/step;


% SV 1
eta_r_1 = logsout.getElement('eta_r_1');
eta_d_1 = eta_r_1.Values.Data;
eta_d_1 = squeeze(eta_d_1)';
eta_1 = logsout.getElement('eta_1');
eta_1 = eta_1.Values.Data;
tau1 = logsout.getElement('tau1');
tau_1 = tau1.Values.Data;
costFunction1 = logsout.getElement('Jsig1');
costFunction_1 = costFunction1.Values.Data;
eta_e_1 = eta_1 - eta_d_1;


% SV 2
eta_r_2 = logsout.getElement('eta_r_2');
eta_d_2 = eta_r_2.Values.Data;
eta_d_2 = squeeze(eta_d_2)';
eta_2 = logsout.getElement('eta_2');
eta_2 = eta_2.Values.Data;
tau2 = logsout.getElement('tau2');
tau_2 = tau2.Values.Data;
costFunction2 = logsout.getElement('Jsig2');
costFunction_2 = costFunction2.Values.Data;
eta_e_2 = eta_2 - eta_d_2;


% SV 3
eta_r_3 = logsout.getElement('eta_r_3');
eta_d_3 = eta_r_3.Values.Data;
eta_d_3 = squeeze(eta_d_3)';
eta_3 = logsout.getElement('eta_3');
eta_3 = eta_3.Values.Data;
tau3 = logsout.getElement('tau3');
tau_3 = tau3.Values.Data;
tau_3 = squeeze(tau_3)';
costFunction3 = logsout.getElement('Jsig3');
costFunction_3 = costFunction3.Values.Data;
eta_e_3 = eta_3 - eta_d_3;



% SV 4
eta_r_4 = logsout.getElement('eta_r_4');
eta_d_4 = eta_r_4.Values.Data;
eta_d_4 = squeeze(eta_d_4)';
eta_4 = logsout.getElement('eta_4');
eta_4 = eta_4.Values.Data;
tau4 = logsout.getElement('tau4');
tau_4 = tau4.Values.Data;
tau_4 = squeeze(tau_4)';
costFunction4 = logsout.getElement('Jsig4');
costFunction_4 = costFunction4.Values.Data;
eta_e_4 = eta_4 - eta_d_4;



% SV 5
eta_r_5 = logsout.getElement('eta_r_5');
eta_d_5 = eta_r_5.Values.Data;
eta_d_5 = squeeze(eta_d_5)';
eta_5 = logsout.getElement('eta_5');
eta_5 = eta_5.Values.Data;
tau5 = logsout.getElement('tau5');
tau_5 = tau5.Values.Data;
tau_5 = squeeze(tau_5)';
costFunction5 = logsout.getElement('Jsig5');
costFunction_5 = costFunction5.Values.Data;
eta_e_5 = eta_5 - eta_d_5;


% Wa1 and Wc1
Wa1 = logsout.getElement('Wa1');
Wa1 = Wa1.Values.Data;

Wc1 = logsout.getElement('Wc1');
Wc1 = Wc1.Values.Data;
% Convert data


trackingError_dat = [eta_e_1(1:100:timeTest,1),eta_e_2(1:100:timeTest,1),eta_e_3(1:100:timeTest,1),eta_e_4(1:100:timeTest,1),eta_e_5(1:100:timeTest,1),...
    eta_e_1(1:100:timeTest,2),eta_e_2(1:100:timeTest,2),eta_e_3(1:100:timeTest,2),eta_e_4(1:100:timeTest,2),eta_e_5(1:100:timeTest,2),...
    eta_e_1(1:100:timeTest,3),eta_e_2(1:100:timeTest,3),eta_e_3(1:100:timeTest,3),eta_e_4(1:100:timeTest,3),eta_e_5(1:100:timeTest,3),t(1:100:timeTest)'];
header = {'etae1x','etae2x','etae3x','etae4x','etae5x',...
    'etae1y','etae2y','etae3y','etae4y','etae5y',...
    'etae1phi','etae2phi','etae3phi','etae4phi','etae5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_trackingError.txt');
writematrix (trackingError_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_trackingError.txt','WriteMode','Append');

tau_dat = [tau_1(1:100:timeTest,1),tau_2(1:100:timeTest,1),tau_3(1:100:timeTest,1),tau_4(1:100:timeTest,1),tau_5(1:100:timeTest,1),...
    tau_1(1:100:timeTest,2),tau_2(1:100:timeTest,2),tau_3(1:100:timeTest,2),tau_4(1:100:timeTest,2),tau_5(1:100:timeTest,2),...
    tau_1(1:100:timeTest,3),tau_2(1:100:timeTest,3),tau_3(1:100:timeTest,3),tau_4(1:100:timeTest,3),tau_5(1:100:timeTest,3),t(1:100:timeTest)'];
header = {'tau1x','tau2x','tau3x','tau4x','tau5x',...
    'tau1y','tau2y','tau3y','tau4y','tau5y',...
    'tau1phi','tau2phi','tau3phi','tau4phi','tau5phi','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_tau.txt');
writematrix (tau_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_tau.txt','WriteMode','Append');

costFunction_dat = [costFunction_1(1:100:timeTest),costFunction_2(1:100:timeTest),costFunction_3(1:100:timeTest),costFunction_4(1:100:timeTest),...
    costFunction_5(1:100:timeTest),t(1:100:timeTest)'];
header = {'costFunction1','costFunction2','costFunction3','costFunction4','costFunction5','t'};
writecell(header, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_costFunction.txt');
writematrix (costFunction_dat, 'C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_costFunction.txt','WriteMode','Append');


% Draw comparison graph
close all

trackingErrFXESO = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_trackingError');
tauFXESO = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_tau');
costFuncFXESO = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\FXESO_costFunction');

trackingErrAISMC = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_trackingError');
tauAISMC = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_tau');
costFuncAISMC = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\AISMC_costFunction');

trackingErrRL = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_trackingError');
tauRL = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_tau');
costFuncRL = readtable('C:\Users\PC\OneDrive\Máy tính\SimulationPaper\RL_costFunction');


t = trackingErrFXESO.t;

for k = 1:length(t)
    e_up(:,k)= [1.4*exp(-0.05*t(k))+0.1;0.9*exp(-0.04*t(k))+0.1;exp(-0.1*t(k))+0.05];
    e_down(:,k) = -[0.9*exp(-0.05*t(k))+0.1;1.4*exp(-0.04*t(k))+0.1;exp(-0.1*t(k))+0.05];
end

% tracking error FXESO 
trackingErr_1_x_FXESO = trackingErrFXESO.etae1x;
trackingErr_2_x_FXESO = trackingErrFXESO.etae2x;
trackingErr_3_x_FXESO = trackingErrFXESO.etae3x;
trackingErr_4_x_FXESO = trackingErrFXESO.etae4x;
trackingErr_5_x_FXESO = trackingErrFXESO.etae5x;

trackingErr_1_y_FXESO = trackingErrFXESO.etae1y;
trackingErr_2_y_FXESO = trackingErrFXESO.etae2y;
trackingErr_3_y_FXESO = trackingErrFXESO.etae3y;
trackingErr_4_y_FXESO = trackingErrFXESO.etae4y;
trackingErr_5_y_FXESO = trackingErrFXESO.etae5y;

trackingErr_1_phi_FXESO = trackingErrFXESO.etae1phi;
trackingErr_2_phi_FXESO = trackingErrFXESO.etae2phi;
trackingErr_3_phi_FXESO = trackingErrFXESO.etae3phi;
trackingErr_4_phi_FXESO = trackingErrFXESO.etae4phi;
trackingErr_5_phi_FXESO = trackingErrFXESO.etae5phi;


% tracking error AISMC
trackingErr_1_x_AISMC = trackingErrAISMC.etae1x;
trackingErr_2_x_AISMC = trackingErrAISMC.etae2x;
trackingErr_3_x_AISMC = trackingErrAISMC.etae3x;
trackingErr_4_x_AISMC = trackingErrAISMC.etae4x;
trackingErr_5_x_AISMC = trackingErrAISMC.etae5x;

trackingErr_1_y_AISMC = trackingErrAISMC.etae1y;
trackingErr_2_y_AISMC = trackingErrAISMC.etae2y;
trackingErr_3_y_AISMC = trackingErrAISMC.etae3y;
trackingErr_4_y_AISMC = trackingErrAISMC.etae4y;
trackingErr_5_y_AISMC = trackingErrAISMC.etae5y;

trackingErr_1_phi_AISMC = trackingErrAISMC.etae1phi;
trackingErr_2_phi_AISMC = trackingErrAISMC.etae2phi;
trackingErr_3_phi_AISMC = trackingErrAISMC.etae3phi;
trackingErr_4_phi_AISMC = trackingErrAISMC.etae4phi;
trackingErr_5_phi_AISMC = trackingErrAISMC.etae5phi;


%tracking Error RL

trackingErr_1_x_RL = trackingErrRL.etae1x;
trackingErr_2_x_RL = trackingErrRL.etae2x;
trackingErr_3_x_RL = trackingErrRL.etae3x;
trackingErr_4_x_RL = trackingErrRL.etae4x;
trackingErr_5_x_RL = trackingErrRL.etae5x;

trackingErr_1_y_RL = trackingErrRL.etae1y;
trackingErr_2_y_RL = trackingErrRL.etae2y;
trackingErr_3_y_RL = trackingErrRL.etae3y;
trackingErr_4_y_RL = trackingErrRL.etae4y;
trackingErr_5_y_RL = trackingErrRL.etae5y;

trackingErr_1_phi_RL = trackingErrRL.etae1phi;
trackingErr_2_phi_RL = trackingErrRL.etae2phi;
trackingErr_3_phi_RL = trackingErrRL.etae3phi;
trackingErr_4_phi_RL = trackingErrRL.etae4phi;
trackingErr_5_phi_RL = trackingErrRL.etae5phi;


%tau FXESO 

tau_1_x_FXESO = tauFXESO.tau1x;
tau_2_x_FXESO = tauFXESO.tau2x;
tau_3_x_FXESO = tauFXESO.tau3x;
tau_4_x_FXESO = tauFXESO.tau4x;
tau_5_x_FXESO = tauFXESO.tau5x;

tau_1_y_FXESO = tauFXESO.tau1y;
tau_2_y_FXESO = tauFXESO.tau2y;
tau_3_y_FXESO = tauFXESO.tau3y;
tau_4_y_FXESO = tauFXESO.tau4y;
tau_5_y_FXESO = tauFXESO.tau5y;

tau_1_phi_FXESO = tauFXESO.tau1phi;
tau_2_phi_FXESO = tauFXESO.tau2phi;
tau_3_phi_FXESO = tauFXESO.tau3phi;
tau_4_phi_FXESO = tauFXESO.tau4phi;
tau_5_phi_FXESO = tauFXESO.tau5phi;


%tau AISMC
tau_1_x_AISMC = tauAISMC.tau1x;
tau_2_x_AISMC = tauAISMC.tau2x;
tau_3_x_AISMC = tauAISMC.tau3x;
tau_4_x_AISMC = tauAISMC.tau4x;
tau_5_x_AISMC = tauAISMC.tau5x;

tau_1_y_AISMC = tauAISMC.tau1y;
tau_2_y_AISMC = tauAISMC.tau2y;
tau_3_y_AISMC = tauAISMC.tau3y;
tau_4_y_AISMC = tauAISMC.tau4y;
tau_5_y_AISMC = tauAISMC.tau5y;

tau_1_phi_AISMC = tauAISMC.tau1phi;
tau_2_phi_AISMC = tauAISMC.tau2phi;
tau_3_phi_AISMC = tauAISMC.tau3phi;
tau_4_phi_AISMC = tauAISMC.tau4phi;
tau_5_phi_AISMC = tauAISMC.tau5phi;


%tau RL
tau_1_x_RL = tauRL.tau1x;
tau_2_x_RL = tauRL.tau2x;
tau_3_x_RL = tauRL.tau3x;
tau_4_x_RL = tauRL.tau4x;
tau_5_x_RL = tauRL.tau5x;

tau_1_y_RL = tauRL.tau1y;
tau_2_y_RL = tauRL.tau2y;
tau_3_y_RL = tauRL.tau3y;
tau_4_y_RL = tauRL.tau4y;
tau_5_y_RL = tauRL.tau5y;

tau_1_phi_RL = tauRL.tau1phi;
tau_2_phi_RL = tauRL.tau2phi;
tau_3_phi_RL = tauRL.tau3phi;
tau_4_phi_RL = tauRL.tau4phi;
tau_5_phi_RL = tauRL.tau5phi;


% cost function
costFunction_1_FXESO = costFuncFXESO.costFunction1;
costFunction_2_FXESO = costFuncFXESO.costFunction2;
costFunction_3_FXESO = costFuncFXESO.costFunction3;
costFunction_4_FXESO = costFuncFXESO.costFunction4;
costFunction_5_FXESO = costFuncFXESO.costFunction5;

costFunction_1_AISMC = costFuncAISMC.costFunction1;
costFunction_2_AISMC = costFuncAISMC.costFunction2;
costFunction_3_AISMC = costFuncAISMC.costFunction3;
costFunction_4_AISMC = costFuncAISMC.costFunction4;
costFunction_5_AISMC = costFuncAISMC.costFunction5;

costFunction_1_RL = costFuncRL.costFunction1;
costFunction_2_RL = costFuncRL.costFunction2;
costFunction_3_RL = costFuncRL.costFunction3;
costFunction_4_RL = costFuncRL.costFunction4;
costFunction_5_RL = costFuncRL.costFunction5;


% Draw tracking error
% figure(1)
% plot(t,tau_1_x_FXESO)
% hold on
% plot(t,tau_1_y_FXESO)
% plot(t,tau_1_phi_FXESO)
% 
% 
% 
% figure(2)
% plot(t,tau_1_x_AISMC)
% hold on
% plot(t,tau_1_y_AISMC)
% plot(t,tau_1_phi_AISMC)
% 
% 
% figure(3)
% plot(t,tau_1_x_RL)
% hold on
% plot(t,tau_1_y_RL)
% plot(t,tau_1_phi_RL)

% figure(4)
% plot(t,trackingErr_4_x_AISMC)
% hold on
% plot(t,trackingErr_4_x_FXESO)
% plot(t,trackingErr_4_x_RL)
% plot(t,e_up(1,:))
% plot(t,e_down(1,:))
% 
% 
% figure(5)
% plot(t,trackingErr_4_y_AISMC)
% hold on
% plot(t,trackingErr_4_y_FXESO)
% plot(t,trackingErr_4_y_RL)
% plot(t,e_up(2,:))
% plot(t,e_down(2,:))
% 
% figure(6)
% plot(t,trackingErr_4_phi_AISMC)
% hold on
% plot(t,trackingErr_4_phi_FXESO)
% plot(t,trackingErr_4_phi_RL)
% plot(t,e_up(3,:))
% plot(t,e_down(3,:))


% figure(5)
% plot(t,trackingErr_3_y_FXESO)
% hold on
% plot(t,trackingErr_3_y_AISMC)
% plot(t,trackingErr_3_y_RL)
% 
% figure(6)
% plot(t,trackingErr_3_phi_FXESO)
% hold on
% plot(t,trackingErr_3_phi_AISMC)
% plot(t,trackingErr_3_phi_RL)
% 
figure(7)
plot(t,costFunction_1_FXESO-costFunction_1_RL)
hold on
plot(t,costFunction_2_FXESO-costFunction_2_RL)
plot(t,costFunction_3_FXESO-costFunction_3_RL)
plot(t,costFunction_4_FXESO-costFunction_4_RL)
plot(t,costFunction_5_FXESO-costFunction_5_RL)

figure(8)
plot(t,costFunction_1_AISMC-costFunction_1_RL)
hold on
plot(t,costFunction_2_AISMC-costFunction_2_RL)
plot(t,costFunction_3_AISMC-costFunction_3_RL)
plot(t,costFunction_4_AISMC-costFunction_4_RL)
plot(t,costFunction_5_AISMC-costFunction_5_RL)





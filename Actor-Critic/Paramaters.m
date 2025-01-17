clc;

close all;
% %%
INT_1=[0;0.5;0];
INT_2=[-3.5;6;0];
INT_3=[-7.1;10;0.0];
INT_4=[-10.5;15;0.2];
INT_5=[-14;20;0.4];
d=6;theta=2*pi/3;beta=0;


RL = 0; 
sim('Multiagent_1030_ActorCritic');

%Get data of the cost function
Jsig1 = logsout.getElement('Jsig1');
t = Jsig1.Values.Time;
Jsig1_off = Jsig1.Values.Data;

Jsig2 = logsout.getElement('Jsig2');
t = Jsig2.Values.Time;
Jsig2_off = Jsig2.Values.Data;

Jsig3 = logsout.getElement('Jsig3');
t = Jsig3.Values.Time;
Jsig3_off = Jsig3.Values.Data;

Jsig4 = logsout.getElement('Jsig4');
t = Jsig4.Values.Time;
Jsig4_off = Jsig4.Values.Data;

Jsig5 = logsout.getElement('Jsig5');
t = Jsig5.Values.Time;
Jsig5_off = Jsig5.Values.Data;


RL = 1;
sim('Multiagent_1030_ActorCritic');
Jsig1 = logsout.getElement('Jsig1');
t = Jsig1.Values.Time;
Jsig1_on = Jsig1.Values.Data;

Jsig2 = logsout.getElement('Jsig2');
t = Jsig1.Values.Time;
Jsig2_on = Jsig2.Values.Data;

Jsig3 = logsout.getElement('Jsig3');
t = Jsig3.Values.Time;
Jsig3_on = Jsig3.Values.Data;

Jsig4 = logsout.getElement('Jsig4');
t = Jsig4.Values.Time;
Jsig4_on = Jsig4.Values.Data;

Jsig5 = logsout.getElement('Jsig5');
t = Jsig5.Values.Time;
Jsig5_on = Jsig5.Values.Data;

Wa1 = logsout.getElement('Wa1');
Wc1 = logsout.getElement('Wc1');
Wa1 = Wa1.Values.Data;
Wc1 = Wc1.Values.Data;
% error =logsout.getElement('error');
% errorUpper =logsout.getElement('errorUpper');
% errorDown =logsout.getElement('errorDown');

% Draw cost function difference
figure(1)

plot(t(1:30000), -Jsig1_on(1:30000) + Jsig1_off(1:30000),'LineWidth',2);
hold on
plot(t(1:30000), +Jsig2_on(1:30000) - Jsig2_off(1:30000),'LineWidth',2);
plot(t(1:30000), -Jsig3_on(1:30000) + Jsig3_off(1:30000),'LineWidth',2);
plot(t(1:30000), -Jsig4_on(1:30000) + Jsig4_off(1:30000),'LineWidth',2);
plot(t(1:30000), -Jsig5_on(1:30000) + Jsig5_off(1:30000),'LineWidth',2);

ax = gca; ax.LineWidth = 1; ax.FontSize = 14;
grid on;
title('$J_{\Sigma off} - J_{\Sigma on}$', 'Interpreter','latex','FontSize',24);
xlabel('Time (seconds)');

figure(2)
plot(t, Wa1(:,1:15));

figure(3)
plot(t, Wc1(:,1:15));


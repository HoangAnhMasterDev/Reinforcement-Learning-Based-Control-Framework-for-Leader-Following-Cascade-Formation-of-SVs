%Main file for Actor-critic-based cascade controller


clc;
clear all;
close all;
% %%
INT_1=[0;0.5;0];
INT_2=[-3.5;6;0];
INT_3=[-7.1;10;0.0];
INT_4=[-10.5;15;0.2];
INT_5=[-14;20;0.4];


d=6;theta=2*pi/3;beta=0;

%%
RL = 1;
sim('Multiagent_1030_ActorCritic');




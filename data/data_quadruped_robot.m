clear; clc; close all;

% system('run_win.bat');

filename_FL = 'data_FL.csv';
filename_FR = 'data_FR.csv';
filename_RL = 'data_RL.csv';
filename_RR = 'data_RR.csv';
filename_TRUNK = 'data_trunk.csv';


T_FL = readtable(filename_FL); %check T.Properties
T_FR = readtable(filename_FR); %check T.Properties
T_RL = readtable(filename_RL); %check T.Properties
T_RR = readtable(filename_RR); %check T.Properties
T_TRUNK = readtable(filename_TRUNK); %check T.Properties


Arr_FL = table2array(T_FL);
Arr_FR = table2array(T_FR);
Arr_RL = table2array(T_RL);
Arr_RR = table2array(T_RR);
Arr_TRUNK = table2array(T_TRUNK);

%아직 이유는 못찾았는데 Trunk의 size가 leg랑 동일하게 나옴. 이거 FAKE다. Trunk 배열 몇개인지는 파일을 열어보삼.
%그럼 알게 됩니다.
[m1,n1] = size(Arr_FL);
[m2,n2] = size(Arr_TRUNK);

% DATA 순서가 어떻게 되는지는 파일들 한번 열어보면 이름이 적혀 있다. 진송쿤!
% t = zeros(m,1);
t           = Arr_FL(:,1);

r_ref_FL       = Arr_FL(:,2);
r_ref_FR       = Arr_FR(:,2);
r_ref_RL       = Arr_RL(:,2);
r_ref_RR       = Arr_RR(:,2);

thr_ref_FL       = Arr_FL(:,3);
thr_ref_FR       = Arr_FR(:,3);
thr_ref_RL       = Arr_RL(:,3);
thr_ref_RR       = Arr_RR(:,3);

r_act_FL       = Arr_FL(:,4);
r_act_FR       = Arr_FR(:,4);
r_act_RL       = Arr_RL(:,4);
r_act_RR       = Arr_RR(:,4);

% theta_r_FL = Arr_FL(:,5);
% theta_r_FR = Arr_FR(:,5);
% theta_r_RL = Arr_RL(:,5);
% theta_r_RR = Arr_RR(:,5);
% 
% 
% grf_x_FL       = Arr_FL(:,8);
% grf_x_FR       = Arr_FR(:,8);
% grf_x_RL       = Arr_RL(:,8);
% grf_x_RR       = Arr_RR(:,8);
% 
% PD_output_F_r_FL = Arr_FL(:,10);
% PD_output_F_r_FR = Arr_FR(:,10);
% PD_output_F_r_RL= Arr_RL(:,10);
% PD_output_F_r_RR = Arr_RR(:,10);
% 
% PD_output_F_thr_FL    = Arr_FL(:,11);
% PD_output_F_thr_FR   = Arr_FR(:,11);
% PD_output_F_thr_RL   = Arr_RL(:,11);
% PD_output_F_thr_RR   = Arr_RR(:,11);
% 
% q_bi_FL = Arr_FL(:,14);
% q_bi_FR = Arr_FR(:,14);
% q_bi_RL = Arr_RL(:,14);
% q_bi_RR = Arr_RR(:,14);
% 
% tau_dis_m_FL    = Arr_FL(:,15);
% tau_dis_m_FR   = Arr_FR(:,15);
% tau_dis_m_RR  = Arr_RR(:,15);
% tau_dis_m_RL   = Arr_RL(:,15);
% 
% tau_dis_b_FL    =  Arr_FL(:,16);
% tau_dis_b_FR   =  Arr_FR(:,16);
% tau_dis_b_RR   = Arr_RR(:,16);
% tau_dis_b_RL   =  Arr_RL(:,16);
% 
% r_acc_act_FL        =  Arr_FL(:,17);
% r_acc_act_FR       =  Arr_FR(:,17);
% r_acc_act_RL       =  Arr_RL(:,17);
% r_acc_act_RR      =  Arr_RR(:,17);
% 
% thr_acc_act_FL         = Arr_FL(:,18);
% thr_acc_act_FR        = Arr_FR(:,18);
% thr_acc_act_RL        = Arr_RL(:,18);
% thr_acc_act_RR       = Arr_RR(:,18);
% 
% r_Dist_hat_FL  = Arr_FL(:,19);
% r_Dist_hat_FR  = Arr_FR(:,19);
% r_Dist_hat_RL  = Arr_RL(:,19);
% r_Dist_hat_RR  = Arr_RR(:,19);
% 
% thr_Dist_hat_FL  = Arr_FL(:,20);
% thr_Dist_hat_FR  = Arr_FR(:,20);
% thr_Dist_hat_RL  = Arr_RL(:,20);
% thr_Dist_hat_RR  = Arr_RR(:,20);
% 
% delta_pos_FL = Arr_FL(:,21);
% delta_pos_FR = Arr_FR(:,21);
% delta_pos_RL = Arr_RL(:,21);
% delta_pos_RR = Arr_RR(:,21);
% 
% 
% touch_FL       = Arr_TRUNK(:,2);
% touch_FR       = Arr_TRUNK(:,3);
% touch_RL       = Arr_TRUNK(:,4);
% touch_RR       = Arr_TRUNK(:,5);
% 
% 
% 
% %GRF sensor estimation
% grf_x_sensor_FL = Arr_TRUNK(:,6);
% grf_x_sensor_FR = Arr_TRUNK(:,7);
% grf_x_sensor_RL = Arr_TRUNK(:,8);
% grf_x_sensor_RR = Arr_TRUNK(:,9);
% 
% grf_y_sensor_FL = Arr_TRUNK(:,10);
% grf_y_sensor_FR = Arr_TRUNK(:,11);
% grf_y_sensor_RL = Arr_TRUNK(:,12);
% grf_y_sensor_RR = Arr_TRUNK(:,13);
% 
% %Trunk state estimation
% Trunk_acc_x    = Arr_TRUNK(:,14);
% Trunk_acc_y    = Arr_TRUNK(:,15);
% Trunk_acc_z    = Arr_TRUNK(:,16);
% 
% Trunk_vel_x    = Arr_TRUNK(:,17);
% Trunk_vel_y    = Arr_TRUNK(:,18);
% Trunk_vel_z    = Arr_TRUNK(:,19);
% 
% Trunk_vel_roll    = Arr_TRUNK(:,20);
% Trunk_vel_pitch    = Arr_TRUNK(:,21);
% Trunk_vel_yaw    = Arr_TRUNK(:,22);
% 
% tau_m_FL   = Arr_TRUNK(:,23);
% tau_m_FR   = Arr_TRUNK(:,25);
% tau_m_RL   = Arr_TRUNK(:,27);
% tau_m_RR   = Arr_TRUNK(:,29);
% 
% tau_b_FL   = Arr_TRUNK(:,24);
% tau_b_FR   = Arr_TRUNK(:,26);
% tau_b_RL   = Arr_TRUNK(:,28);
% tau_b_RR   = Arr_TRUNK(:,30);
% 
% 
% cartesian_grf_x_FL = grf_x_sensor_FL .* cos(pi - q_bi_FL) - grf_y_sensor_FL .* sin(pi - q_bi_FL);
% cartesian_grf_x_FR = grf_x_sensor_FR .* cos(pi - q_bi_FR) - grf_y_sensor_FR .* sin(pi - q_bi_FR);
% cartesian_grf_x_RL = grf_x_sensor_RL .* cos(pi - q_bi_RL) - grf_y_sensor_RL .* sin(pi - q_bi_RL);
% cartesian_grf_x_RR = grf_x_sensor_RR .* cos(pi - q_bi_RR) - grf_y_sensor_RR .* sin(pi - q_bi_RR);
% 
% cartesian_grf_y_FL = grf_x_sensor_FL .* sin(pi - q_bi_FL) + grf_y_sensor_FL .* cos(pi - q_bi_FL);
% cartesian_grf_y_FR = grf_x_sensor_FR .* sin(pi - q_bi_FR) + grf_y_sensor_FR .* cos(pi - q_bi_FR);
% cartesian_grf_y_RL = grf_x_sensor_RL .* sin(pi - q_bi_RL) + grf_y_sensor_RL .* cos(pi - q_bi_RL);
% cartesian_grf_y_RR = grf_x_sensor_RR .* sin(pi - q_bi_RR) + grf_y_sensor_RR .* cos(pi - q_bi_RR);
% 
% grf_r_FL = cartesian_grf_y_FL .* cos(theta_r_FL - pi / 2) + cartesian_grf_x_FL .* sin(theta_r_FL - pi / 2);
% grf_r_FR = cartesian_grf_y_FR .* cos(theta_r_FR - pi / 2) + cartesian_grf_x_FR .* sin(theta_r_FR - pi / 2);
% grf_r_RL = cartesian_grf_y_RL .* cos(theta_r_RL - pi / 2) + cartesian_grf_x_RL .* sin(theta_r_RL - pi / 2);
% grf_r_RR = cartesian_grf_y_RR .* cos(theta_r_RR - pi / 2) + cartesian_grf_x_RR .* sin(theta_r_RR - pi / 2);

%%

%%%%%%%%%%%%%%%% Data Ploting %%%%%%%%%%%%%%%%%%%%%%%%%
%time cutting
% 시작 시간과 끝 시간 설정

start_time = 1; % 시작 시간
end_time = 7;   % 끝 시간

% t 벡터를 기준으로 시작 시간과 끝 시간에 해당하는 인덱스 찾기
start_idx = find(t >= start_time, 1);
end_idx = find(t >= end_time, 1);
t = t(start_idx:end_idx);

% 찾은 인덱스를 확인하기 위해 disp 사용
disp(start_idx);
disp(end_idx);

%Data ploting 관련 Parameter
lw =1.2;   %Line Width
FT = 18; %Title Fonte Size
Faxis = 8; %Axis Fonte Size
fl =8 ; % Legend Fonte Size


% figure(1)
% plot(t,Trunk_vel_x(start_idx:end_idx),'b','LineWidth', lw);
% xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
% ylabel('Velocity (m/s)','FontSize', Faxis); % y축 레이블
% legend('Compare','Rotating','FontSize',fl);
% title("Compare Trunk Velocity(x direction)")
% grid on

% figure(2)
% plot(t,Trunk_vel_y(start_idx:end_idx),'b','LineWidth', lw);
% xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
% ylabel('velocity (m/s)','FontSize', Faxis); % y축 레이블
% legend('Compare','Rotating','FontSize',fl);
% title("Compare Trunk Velocity(y direction)")
% grid on

figure(3)
plot(t,r_act_FL(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_act_FR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_act_RR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_act_RL(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_ref_RL(start_idx:end_idx),'LineWidth', lw);
xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
ylabel('Position (m)','FontSize', Faxis); % y축 레이블
legend('FL','FR','RR','RL','ref','FontSize',fl);
title("Rotating Pos act(r direction)")
grid on

figure(4)
plot(t,grf_r_FL(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,grf_r_FR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,grf_r_RR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,grf_r_RL(start_idx:end_idx),'LineWidth', lw);
hold on
xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
ylabel('F_z (N)','FontSize', Faxis); % y축 레이블
legend('FL','FR','RR','RL','FontSize',fl);
title("Ground Reaction Force(r direction)")
grid on

% figure(5)
% plot(t,PD_output_F_r_FL(start_idx:end_idx),'LineWidth',lw);
% hold on
% plot(t,PD_output_F_r_FR(start_idx:end_idx),'LineWidth',lw);
% hold on
% plot(t,PD_output_F_r_RR(start_idx:end_idx),'LineWidth',lw);
% hold on
% plot(t,PD_output_F_r_RL(start_idx:end_idx),'LineWidth',lw);
% hold on
% xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
% ylabel('F_r (N)','FontSize', Faxis); % y축 레이블
% legend('FL','FR','RR','RL','FontSize',fl);
% title("PD output (r direction)")
% grid on

figure(6)
plot(t,r_ref_FL(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_ref_FR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_ref_RR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,r_ref_RL(start_idx:end_idx),'LineWidth', lw);
hold on
xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
ylabel('Position (m)','FontSize', Faxis); % y축 레이블
legend('FL','FR','RR','RL','FontSize',fl);
title("Rotating Pos Reference(r direction)")
grid on

figure(7)
plot(t,delta_pos_FL(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,delta_pos_FR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,delta_pos_RR(start_idx:end_idx),'LineWidth', lw);
hold on
plot(t,delta_pos_RL(start_idx:end_idx),'LineWidth', lw);
hold on
plot (t,touch_RL(start_idx:end_idx)/10000000,'LineWidth', lw);
xlabel('Time (seconds)','FontSize', Faxis); % x축 레이블
ylabel('Position (m)','FontSize', Faxis); % y축 레이블
legend('FL','FR','RR','RL','touch','FontSize',fl);
title("Delta by admittance(r direction)")
grid on






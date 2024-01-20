clear all; close all; clc; 

addpath('C:\Users\flora\OneDrive\Desktop\casadi-3.6.4-windows64-matlab2018b')
import casadi.*

gp_model = gpCallback('model');

% MPC variables 
N = 6;    % MPC Horizon
dt = 0.1; % sample time
M = 2; % No of autonomous vehicles
 
% constraints
a_max = 4;  
a_min = -4; 
v_max = 37;  % max vel, 133.2km/h for WLTP testing profile 
v_min = 0; % min vel
delta = 10;  % safe distance between vehicles

% initializations
v0 = zeros(1,M); % velocity varibales of AVs
p0 = zeros(1,M); % position variables of AVs
% a0 = zeros(1,M); % acceleration variables of AVs

% initialize AV positions
for m = 2:M
    p0(m) = p0(m-1)-1.2*delta; % start 2 delta behind car in front
end
p0_h  = p0(M)- 1.2*delta; % HV starts 2 delta behind the last AV
v0_h = 0; % velocity initials of HV
v0_h_gp = v0_h;
p0_h_gp = p0_h;
p0_h_sigma = 0; % position variance initials of HV 

% past 4 samples (set to zero for simplicity) of velocity for the last AV
v_M_m1 = 0; % i-1, i-2, i-3 and i-4
v_M_m2 = 0;
v_M_m3 = 0;
v_M_m4 = 0;
% past 4 samples (set to zero for simplicity) of velocity for the HV
v_h_m1 = 0;
v_h_m2 = 0;
v_h_m3 = 0;
v_h_m4 = 0;

Q = 5; % MPC weight for velocity tracking 
R = 10; % weight for control 
% prob_desired = 0.95; % desired probability for tighenting distance constraint
prob_desired = 0.95; % 2\sigma

%% Simulation 
sim_tim = 130; % simulation time, max simulation is 320s with lei's laptop

t0 = 0; % initial time step
TIME = sim_tim / dt; % simualtion steps

ET = zeros(TIME, 1); % mpc runing time

t = zeros(M, 1);
t(1) = t0;

k = 0;  % system simulation time step k
av_accel = zeros(TIME,M); % accelerations hisotry from MPC
vv0 = [v0;zeros(TIME,M)]; % velocity history of AVs
vv0_h = [v0_h;zeros(TIME-1,1)]; % velocity history of HV for ARX norminal model updates
vv0_h_gp = [v0_h;zeros(TIME-1,1)]; % velocity history of HV with GP corrections

pp0 = [p0;zeros(TIME,M)]; % position history of AVs
pp0_h = [p0_h;zeros(TIME,1)]; % position history of HV with ARX norminal model 
pp0_h_gp = [p0_h;zeros(TIME,1)]; % position history of HV with ARX + GP
pp0_h_sigma = [p0_h_sigma;zeros(TIME,1)]; % position variance history of HV 

AV_accel = zeros(N,M); % MPC acceleration output
AV_vel = zeros(N+1,M);

humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each
H_vel = zeros(N+1,M);
H_vel_gp = zeros(N+1,M);

gp_mean = zeros(N+1,M);
gp_sigma = zeros(N+1,M);

% Initialize the reference velocity history array
v_reference = zeros(TIME+1,1);

% % Generate the entire WLTP normalized velocity profile for the simulation duration
% wltp_ref_velocity = wltp_velocity_profile(sim_tim,dt);

while k < TIME
    % init variables for past and limit them to be equal to their measured vals
    lbvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    ubvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    lbvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    ubvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    
    % % Set the reference velocity for the current time step
    % v_ref_c = wltp_ref_velocity(k+1);
    
    % reference velocity 
    if k< 4*TIME/13
        v_ref_c = 35; 
    elseif k< 8*TIME/13
        v_ref_c = 20;
    elseif k < 10*TIME/13
        v_ref_c = 10;
    elseif k < 12*TIME/13
        v_ref_c = 2;
    else
        v_ref_c = 0;
    end

    % % reference velocity 
    % if k< 1*TIME/3
    %     v_ref_c = 35; 
    % elseif k< 2*TIME/3
    %     v_ref_c = 20;
    % else
    %     v_ref_c = 2;
    % end

    % Save the current reference velocity in the history array
    v_reference(k+1) = v_ref_c;

    % Ensure v_ref is a column vector of size (N+1,1) for the prediction horizon
    v_ref = v_ref_c * ones(N+1,1);

    % MPC
    tic

    [sol] = GP_MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
                           v_min, v_max, a_max, a_min, ...
                           p0, v0, v0_h_gp, p0_h_gp, p0_h_sigma, prob_desired, ...
                           lbvM_, ubvM_, lbvh_, ubvh_, ...
                           gp_mean, gp_sigma);
    

    % update the history for next time steps
    % update k-2, k-3, k-4 
    % Note: every time to call the MPC loop, we need to update the initials,
    % including the k-1, k-2, k-3, k-4 at the begining
    v_M_m4 = v_M_m3; 
    v_M_m3 = v_M_m2;
    v_M_m2 = v_M_m1;
    v_h_m4 = v_h_m3; 
    v_h_m3 = v_h_m2;
    v_h_m2 = v_h_m1;

    % for k-1 assign the current vel to v_M_m1 for the next iteration
    v_M_m1 = v0(end); % for last AV in platoon
    v_h_m1 = v0_h_gp; % v0_h_gp is the actual "measured" states from the plant
                      % we use the arx+gp model to represent the real plant
                      % behaviors. The model propagations must be handled
                      % by the arx model, gp is the corrections to make
                      % the whole system model more accurate to the real
                      % plant.  
    
    % retrive results from the solution
    % x = [p(:);v(:);a(:);ph(:);vh(:);vh_gp;vh_(:);vM_(:);ph_sigma(:)];
    for m = 1:M
%        AV_pos(:,m) = full(sol.x(1+(N+1)*(m-1):(N+1)*m));
       AV_vel(:,m) = full(sol.x((N+1)*M+1+(N+1)*(m-1):(N+1)*m+(N+1)*M));
       AV_accel(:,m) = full(sol.x((N+1)*2*M+1+(N)*(m-1):(N)*m+(N+1)*2*M));
    end
    
    humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each
%     H_pos = full(sol.x(humanvar_offset+1:humanvar_offset+N+1)); 
    H_vel = full(sol.x(humanvar_offset+N+1+1:humanvar_offset+N+1+(N+1))); 
    H_vel_gp = full(sol.x(humanvar_offset+N+1+(N+1)+1:humanvar_offset+N+1+(N+1)+(N+1))); 
 
    t(k+1) = t0;

    for i = 1:N+1
        x_i = [H_vel_gp(i), AV_vel(i,M)];
        [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
        gp_mean_ = gp_pred{1};
        gp_sigma_ = gp_pred{2};
        gp_mean_ = full(gp_mean_);
        gp_sigma_ = full(gp_sigma_);

        gp_mean(i) = gp_mean_;
        gp_sigma(i) = gp_sigma_;

    end
    
    toc
    ET(k+1) = toc;

    % calculate the new initial values for MPC 
    [t0, p0, v0, v0_h, v0_h_gp, p0_h_gp] = GP_sysmodel(dt, k+1, t0, p0, vv0, vv0_h, vv0_h_gp, pp0_h_gp, M, AV_accel);
    % convert casadi.DM to double 
    v0_h = full(v0_h);
    v0_h_gp = full(v0_h_gp);
    p0_h_gp = full(p0_h_gp);

    % save variable history 
    vv0(k+2, :) = v0;
    vv0_h(k+1,:) = v0_h;
    vv0_h_gp(k+1, :) = v0_h_gp;
    pp0(k+2, :) = p0;
    pp0_h_gp(k+2, :) = p0_h_gp;
    av_accel(k+1, :)= AV_accel(1,:); % the first elements of accel 

    k = k + 1;
end

%% Reconstruct stuff from solver
AV_positions = pp0;
AV_velocities = vv0;
AV_accelerations = av_accel;
H_positions_gp = pp0_h_gp;
H_positions_gp_var = pp0_h_sigma;

H_velocities = vv0_h;
H_velocities_gp = vv0_h_gp;
t = [t; t(end)+dt];
%% Plot 
figure; 
subplot(411) %plot velocities
plot(t,v_reference,'k--');hold on;grid on;
plot(t,AV_velocities(:,1),'b'); hold on;grid on
for m = 2:M
  plot(t,AV_velocities(:,m),'r-.'); 
end
plot(t(1:length(t)-1),H_velocities,'y');hold on;grid on;
plot(t(1:length(t)-1),H_velocities_gp,'g');hold on;grid on;
legend(["velocity ref", "leading AV velocity", "AVs velocity", "arx HV vel", "arx+GP HV vel"],'Location','best');

xlabel('Time steps');ylabel('Velocities');
xlim([0 t(end)])

subplot(412)
plot(t,AV_positions(:,1),'b'); hold on;grid on;
for m = 2:M
  plot(t,AV_positions(:,m),'r-.'); 
end
plot(t,H_positions_gp,'g');hold on;grid on;
legend(["leading AV position", "AVs position", "arx HV position"],'Location','best');

xlabel('Time steps');ylabel('Positions');
xlim([0 t(end)])

subplot(413)
plot(t,AV_positions(:,1) - AV_positions(:,2),'b');hold on;grid on;
if(M>2) 
   for m = 2:M-1
      plot(t,AV_positions(:,m) - AV_positions(:,m+1),'r-.'); 
   end
end
plot(t,AV_positions(:,M) - H_positions_gp,'g');hold on;grid on;
legend(["lead-follower AV relative pos", "AV-HV relative pos"],'Location','best');

xlabel('Time steps');ylabel('Relative distance');
xlim([0 t(end)])

subplot(414)
plot(t(1:length(t)-1),AV_accelerations(:,1),'b');hold on;grid on;

for m = 2:M
  plot(t(1:length(t)-1),AV_accelerations(:,m),'r-.'); 
end
legend(["leading AV accel", "AV accel"],'Location','best');

xlabel('Time steps');ylabel('Accl');
xlim([0 t(end)])

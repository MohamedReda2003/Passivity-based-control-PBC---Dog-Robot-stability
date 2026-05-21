%% QuadrupedSim Init File with PBC for Standing + Walking
%  Run this script BEFORE opening the Simulink model
%  Loads all parameters into the base workspace

%% 1. Add Contact Library
addpath(genpath(fullfile(pwd,'contat_lib')));

%% 2. Ground Plane Parameters
ground.stiff = 5e4;
ground.damp = 1000;
ground.height = 0.4;

%% 3. Robot Body Geometry
body.x_length = 0.6;
body.y_length = 0.3;
body.z_length = 0.15;
body.shoulder_size = 0.07;
body.upper_length = 0.16;
body.lower_length = 0.25;
body.foot_radius = 0.035;
body.shoulder_distance = 0.2;
body.max_stretch = body.upper_length + body.lower_length;
body.knee_damping = 0.1;

%% 4. Low-Level Leg Control Gains
ctrl.pos_kp = 10;
ctrl.pos_ki = 0;
ctrl.pos_kd = 0;
ctrl.vel_kp = 4.4;
ctrl.vel_ki = 0;
ctrl.vel_kd = 0.0;

%% 5. High-Level Planner Parameters
planner.touch_down_height = body.foot_radius;

planner.stand_s = 0;
planner.stand_u = 30*pi/180;
planner.stand_k = -60*pi/180;
stance_pos = forward_kinematics(planner.stand_s, planner.stand_u, planner.stand_k, body);
planner.stand_height = stance_pos(3);

planner.flight_height = 1.1*planner.stand_height;

%% 6. Gait Control Parameters
planner.time_circle = 3;
planner.swing_ang = 12/180*pi;
planner.init_shake_ang = 3/180*pi;

planner.tgt_body_ang = 0;
planner.tgt_body_vx = 0.3;
planner.Ts = 0.1;
planner.Kv = 0.3;

planner.y_Ts = 0.21;
planner.y_Kv = -0.34;

%% 7. State Transition Thresholds
planner.state0_vel_thres = 0.05;
planner.state0_trans_thres = 300;
planner.state0_swing_ang = 10*pi/180;
planner.state0_swing_T = 1200;
planner.state12_trans_speed = 0.2;
planner.leg_swing_time = 70;

%% 8. Robot Mass Properties
body_weight = 600*body.x_length*body.y_length*body.z_length;
leg_density = 660;
should_weight = leg_density*0.07^3;
upperleg_weight = leg_density*0.04*0.04*body.upper_length;
lowerleg_weight = leg_density*0.04*0.04*body.lower_length;
foot_weight = 1000*4/3*pi*body.foot_radius^3;

total_weight = body_weight + 4*(should_weight+upperleg_weight+lowerleg_weight+foot_weight);

body_inertia = diag([0.151875; 0.516375; 0.6075]);

%% 9. PBC Controller Parameters for Standing + Walking
pbc_params = struct();

% Energy shaping gains
pbc_params.Kp = 8.0;           % Potential energy shaping
pbc_params.Kv = 2.0;           % Kinetic energy shaping
pbc_params.Kd = 1.5;           % Damping injection
pbc_params.tau_max = 50;       % [Nm] max control torque
pbc_params.stand_height = planner.stand_height;  % ADD THIS
% Leg command mapping
pbc_params.K_stiff = 0.8;      % [deg/Nm] torque-to-angle stiffness

% Standing pose joint angles [deg] (2 joints per leg)
pbc_params.hind_upper = 45;    % Hind leg hip/upper angle
pbc_params.hind_knee = 90;     % Hind leg knee angle
pbc_params.front_upper = 120;  % Front leg hip/upper angle (raised)
pbc_params.front_knee = 30;    % Front leg knee angle (tucked)

% Phase timing
pbc_params.stand_time = 2.0;   % [s] Stand-up duration
pbc_params.balance_time = 3.0; % [s] Balance before walk

% Gravity compensation
pbc_params.m_body = body_weight;     % [kg]
pbc_params.g = 9.81;                 % [m/s^2]
pbc_params.L_body = body.z_length;   % [m] COM height approx

% Leg link lengths (needed for swing IK)
pbc_params.upper_length = body.upper_length;  % [m]
pbc_params.lower_length = body.lower_length;  % [m]

% Gait parameters for walking
pbc_params.gait_period = 1.0;      % [s] Trot gait period
pbc_params.duty_factor = 0.6;      % 60% stance, 40% swing
pbc_params.swing_height = 0.08;    % [m] Foot lift height during swing
pbc_params.step_length = 0.15;     % [m] Step length
pbc_params.Kv_fp = 0.3;            % Foot placement velocity gain
pbc_params.stand_height = planner.stand_height;
% Load into base workspace for Simulink
assignin('base', 'ground', ground);
assignin('base', 'body', body);
assignin('base', 'ctrl', ctrl);
assignin('base', 'planner', planner);
assignin('base', 'body_weight', body_weight);
assignin('base', 'total_weight', total_weight);
assignin('base', 'body_inertia', body_inertia);
assignin('base', 'pbc_params', pbc_params);

disp(' ');
disp('============================================================');
disp('  QuadrupedSim + PBC Parameters Loaded Successfully');
disp('============================================================');
disp(['  Body weight: ' num2str(body_weight) ' kg']);
disp(['  Total weight: ' num2str(total_weight) ' kg']);
disp(['  PBC Kp: ' num2str(pbc_params.Kp)]);
disp(['  PBC Kd: ' num2str(pbc_params.Kd)]);
disp(['  Gait period: ' num2str(pbc_params.gait_period) ' s']);
disp(['  Duty factor: ' num2str(pbc_params.duty_factor)]);
disp('============================================================');
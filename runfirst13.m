% Data for the simulink model, run this first
%
% Victor �hman, 2016

close all
clear all
clc

rad_2_deg = 180/pi;
deg_2_rad = 1/rad_2_deg;

    %% Import control surf positions
    
    
    % further down temporarily to make fake matrices
    

    %% --- % Reference condition
    
    g           = 9.82;     % Gravity constant
    rho_ref     = 1.225;    % Air density [kg/m^3]
    V_ref       = 40.0;     % Airspeed [m/s]
    alpha_ref   = 2.0;      % Angle of attack [rad] 
    beta_ref    = 0.0;      % Side-slip [rad] 
    Cd0_ref     = 0.010;    %
    h_ref       = 0;        % 
        
    %% --- % Atmosphere at ground
    
    T_g     = 293; % Temperature [K]
    P_g     = 100000; % Pressure [Pa]
    R       = 287; %  Characteristic gas constant [J/Kg/K]
    gamma   = 1.4; % Ratio of specific heats 
    
    %% --- % Reference

    Xe_0    = 0;        % Initial position North [m]
    Ye_0    = 0;        % Initial position East  [m]
    Ze_0    = -100;      % Initial position Down  [m]
    U_0     = 10;       % Initial body velocity u [m/s]
    v_0     = 0;        % Initial body velocity v [m/s]
    w_0     = 0;        % Initial body velocity w [m/s]
    uw_0    = 1;        % Initial wind velocity u [m/s]
    vw_0    = 1;        % Initial wind velocity v [m/s]
    ww_0    = 0;        % Initial wind velocity w [m/s]
    phi_0     = 0.0;      % Euler roll rotation  [rad]
    theta_0   = 0.02;      % Euler pitch rotation [rad]
    psi_0     = 0.02;      % Euler yaw rotation   [rad]
    p_0       = 0.0;      % roll rotation rate  [rad/s]
    q_0       = 0.5;      % Pitch rotation rate [rad/s]
    r_0       = 0.0;      % Yaw rotation rate   [rad/s]

    heading_0   = 0;            % Direction of flat Earth x-axis (degrees clockwise from north)
    LL0         = [1 0];        % Initial geodetic latitude and longitude [deg]
    f           = 1/196.877360; % Flattening
    
    pos3_0  = [Xe_0 Ze_0];      % For 3-DoF
    pos_0   = [Xe_0 Ye_0 Ze_0]; % For 6-DoF
    Vb3_0   = [U_0 w_0];        % For 3-DoF
    Vb_0    = [U_0 v_0 w_0];    % For 6-DoF
    Vw_0    = [uw_0 vw_0 ww_0];
    eul_0   = [phi_0 theta_0 psi_0];
    rot_0   = [p_0 q_0 r_0];

%% --- % Pilot Input

    z_act       = 0.57;   % zeta damping ratio
    wn_act      = 1.7;   % omega natural frequenzy
    maxdef_act  = 2.0;   % max deflection
    mindef_act  = -2.0;   % min deflection
    act_0       = 0.3;   % initial pos
    maxrate_act = inf;   % maximum rate 
    
    load('log_pilotcmd.mat')
    cmd_time = log_pilotcmd.time(:);
    cmd_time(:) = cmd_time(:)-cmd_time(1); 

    canard_port_ini = 0; % 
    canard_star_ini = 0; % 
    elevon_port_ini = 0; % 
    elevon_star_ini = 0; % 
    flap_port_ini   = 0; % 
    flap_star_ini   = 0; % 
    throttle_ini = 0.5; % 
    F_0 = [100 0 0];
    
%% --- % Design

    L_fuse  = 2.4;      % Fuselage length [m]
    x_noz   = 2.400;    % Engine nozzle position
    S_ref   = 0.747;    % Reference wing area [m^2]
    S_redbw = 0.612;    % Reference wing 1 + body area S [m^2]
    b       = 1.365;    % Reference wing 1 span [m]
    MAC     = 0.645;    % Mean aerodynamic chord [m]
    xa_ref  = 1.185;    % Aerodynamic reference point from nose tip
    cbar   = 0.5; % godtyckligt
    
%% --- % Weight & Inertia
    
    W_oew   = 14.95;    % Operation empty weight, OEW [kg]
    W_if    = 2.9;      % Internal fuel weight [kg]
    W_cargo = 2.15;     % Max cargo weight [kg]
    W_load  = 17.905;   % Loaded weight [kg]
    W_mtow  = 20;       % Max takeoff weight, MTOW [kg]
 
    mass    = W_oew;    % A/C mass (using W_oew)
    mass_f  = mass+W_if;     % Full mass
    mass_e  = 0;        % Empty mass
    m_dot   = 0.2;     % Mass flow
    
    Xcg     = 1.185 ;   % reference c.g. location Xcg.
    Ycg     = 0;       % reference c.g. location Ycg.
    Zcg     = 0.024 ;   % reference c.g. location Zcg.
    
    CG      = [Xcg Ycg Zcg];    % 
    CP      = [xa_ref 0 0];     % OBS! Temp sol 

    Ix0 = 0.0201;   % Roll moment of inertia  Ix0 { Ix/(mass*b^2) }
    Iy0 = 0.8494;   % Pitch moment of inertia Iy0 { Iy/(mass*MAC^2) }
    Iz0 = 0.8944;   % Yaw moment of inertia   Iz0 { Iz/(mass*MAC^2) }
    
    Ix  = 0.56 ;    % Roll moment of inertia  Ix
    Iy  = 5.28 ;    % Pitch moment of inertia Iy
    Iz  = 5.56 ;    % Yaw moment of inertia   Iz

%     Ixx = 0.02;        % Placeholder
    Ixy = 0;        % Product moment of inertia Ixy
    Ixz = 0;        % Product moment of inertia Ixz
    Iyx = 0;        % 
%     Iyy = 0.02;      % Placeholder 
    Iyz = 0 ;       % Product moment of inertia Iyz
    Izx = 0;        % 
    Izy = 0;        %     
%     Izz = 0.02;        % Placeholder

    I = [Ix0 -Ixy -Ixz;-Iyx Iy0 -Iyz;-Izx -Izy Iz0];
    I_dot   = [0 0 0;0 0 0;0 0 0];
    
    Iyy_f = 0; % Full intertia matrix
    Iyy_e = 0; % Empty interia matrix
    Iyy=0.01; 
    
    
%% --- % Aerodynamic derivatives
    
    CD_alpha	= 0.173;    %
    CY_alpha    = 0;        %
    CL_alpha	= 3.137;    %
    Cl_alpha	= 0;        %
    Cm_alpha	= -0.083;   %
    Cn_alpha	= 0;        %
    
    CD_beta     = 0; % 
    CY_beta     = -0.351;   % 
    CL_beta     = 0; % 
    Cl_beta     = 0.046;    %
    Cm_beta     = 0; % 
    Cn_beta     = 0; % 
    
    Cl_P    = -0.235;   % 
    Cy_P    = -0.046;   %
    Cn_P    = -0.035;   %
    
    Cm_Q    = 0.001;    % Made up number
    
    Cy_R    = -0.437;   %
    Cl_R    = 0.063;    %
    Cn_R    = -0.259;   %
    
    %     Canard
    
    CD_ca   = 0; % 
    CL_ca	= 0.000;  % Lift coefficient
    CY_ca	= 0.000;  % Side coefficient
    Cl_ca	= 0.000;  % Rolling moment coefficient
    Cm_ca	= -0.456; % Pitching moment coefficient
    Cn_ca   = 0.000;  % Yawing moment coefficient
    
    %     Elevons
    
    CD_el   = 0; % 
    CL_el	= 0.000; % dubbelkolla
    CY_el	= 0.049; % dubbelkolla
    Cl_el	= 0.111; %
    Cm_el	= 0.000; %
    Cn_el	= 0.037; %
    
    %     Rudder
    
    CD_ru	= 0.033; %
    CY_ru	= 0.000; %
    CL_ru	= 0.532; %
    Cl_ru	= 0.000; %
    Cm_ru	= -0.233; %
    Cn_ru	= 0.000; %
    
    
    %% Import control surf positions
    
    rowder_ca = [CD_ca CY_ca CL_ca Cl_ca Cm_ca Cn_ca];
    rowder_elp = [CD_el CY_el CL_el Cl_el Cm_el Cn_el];
    rowder_els = [CD_el CY_el CL_el Cl_el Cm_el Cn_el];
    rowder_ru = [CD_ru CY_ru CL_ru Cl_ru Cm_ru Cn_ru];
    
    rownumber=length(cmd_time(:));
    mder_ca = zeros(rownumber,6);
    mder_elp = zeros(rownumber,6);
    mder_els = zeros(rownumber,6);
    mder_ru = zeros(rownumber,6);

    
     for i = 1:rownumber
         mder_ca(i,:) = rowder_ca;
         mder_elp(i,:) = rowder_elp;
         mder_els(i,:) = rowder_els;
         mder_ru(i,:) = rowder_ru;
     end
    

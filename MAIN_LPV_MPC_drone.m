clear all
close all
clc
echo off
warning off

%% Main file for controllering the drone

% The controller consists of the position controller (state feedback
% linearization) - outer loop AND attitude controller (LPV-MPC) - inner
% loop with faster dynamics

% The relevant function files to this main file are the following:
    % initial_constants.m
    % LPV_cont_discrete.m
    % MPC_simplification.m
    % nonlinear_drone_model.m
    % trajectory_generator.m
    % pos_controller.m

%% Load the constant values
constants=initial_constants();
Ts=constants('Ts');
controlled_states=constants('controlled_states'); % number of controlled states in this script
innerDyn_length=constants('innerDyn_length'); % Number of inner control loop iterations

%% Generate the reference signals
t = 0:Ts*innerDyn_length:100;
t_angles=(0:Ts:t(end))';
r = 2;
f=0.025;
height_i=2;
height_f=5;
[X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,psi_ref]=trajectory_generator(t,r,f,height_i,height_f);
plotl=length(t); % Number of outer control loop iterations

%% Load the initial state vector

ut=0;
vt=0;
wt=0;
pt=0;
qt=0;
rt=0;
xt=0;%X_ref(1,2); % Initial translational position
yt=-1;%Y_ref(1,2); % Initial translational position
zt=0;%Z_ref(1,2); % Initial translational position
phit=0;    % Initial angular position
thetat=0;  % Initial angular position
psit=psi_ref(1,2);    % Initial angular position

states=[ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit];
states_total=states;

% Assume that first Phi_ref, Theta_ref, Psi_ref are equal to the first
% phit, thetat, psit
ref_angles_total=[phit,thetat,psit];
velocityXYZ_total=[X_dot_ref(1,2),Y_dot_ref(1,2),Z_dot_ref(1,2)];
%% Initial drone state

omega1=110*pi/3; % rad/s at t = -1 s
omega2=110*pi/3; % rad/s at t = -1 s
omega3=110*pi/3; % rad/s at t = -1 s
omega4=110*pi/3; % rad/s at t = -1 s

ct = constants('ct');
cq = constants('cq');
l  = constants('l');

U1=ct*(omega1^2+omega2^2+omega3^2+omega4^2); % Input at t = -1 s
U2=ct*l*(omega2^2-omega4^2); % Input at t = -1 s
U3=ct*l*(omega3^2-omega1^2); % Input at t = -1 s
U4=cq*(-omega1^2+omega2^2-omega3^2+omega4^2); % Input at t = -1 s

UTotal=[U1,U2,U3,U4];% 4 inputs

global omega_total
omega_total=omega1-omega2+omega3-omega4;

%% Constraints
omega_min=constants('omega_min');
omega_max=constants('omega_max');

U1_min=ct*4*omega_min.^2;
U1_max=ct*4*omega_max.^2;

U2_min=ct*l*(omega_min.^2-omega_max.^2);
U2_max=ct*l*(omega_max.^2-omega_min.^2);

U3_min=ct*l*(omega_min.^2-omega_max.^2);
U3_max=ct*l*(omega_max.^2-omega_min.^2);

U4_min=cq*(-2*omega_max.^2+2*omega_min.^2);
U4_max=cq*(-2*omega_min.^2+2*omega_max.^2);

y_max=[U2_max;U3_max;U4_max];
y_min=[U2_min;U3_min;U4_min];

%% Start the global controller

for i_global = 1:plotl-1


    %% Implement the position controller (state feedback linearization)

    [phi_ref, theta_ref, U1]=pos_controller(X_ref(i_global+1,2),X_dot_ref(i_global+1,2),X_dot_dot_ref(i_global+1,2),Y_ref(i_global+1,2),Y_dot_ref(i_global+1,2),Y_dot_dot_ref(i_global+1,2),Z_ref(i_global+1,2),Z_dot_ref(i_global+1,2),Z_dot_dot_ref(i_global+1,2),psi_ref(i_global+1,2),states);


    Phi_ref=phi_ref*ones(innerDyn_length+1,1);
    Theta_ref=theta_ref*ones(innerDyn_length+1,1);
    
    %% Constraints
    % Check the boundaries for U1
    if U1<U1_min
        U1=U1_min;
    end
    
    if U1>U1_max
        U1=U1_max;
    end
    
    
    % Make Psi_ref increase continuosly in a linear fashion per outer loop
    Psi_ref=zeros(innerDyn_length+1,1);
    for yaw_step = 1:(innerDyn_length+1)
        Psi_ref(yaw_step)=psi_ref(i_global,2)+(psi_ref(i_global+1,2)-psi_ref(i_global,2))/(Ts*innerDyn_length)*Ts*(yaw_step-1);
    end
    
    ref_angles_total=[ref_angles_total;Phi_ref(2:end) Theta_ref(2:end) Psi_ref(2:end)];

    %% Create the reference vector

    refSignals=zeros(length(Phi_ref(:,1))*controlled_states,1);
    % Format: refSignals=[Phi_ref;Theta_ref;Psi_ref;Phi_ref; ... etc] x inner
    % loop frequency per one set of position controller outputs
    k_ref_local=1;
    for i = 1:controlled_states:length(refSignals)
       refSignals(i)=Phi_ref(k_ref_local,1);
       refSignals(i+1)=Theta_ref(k_ref_local,1);
       refSignals(i+2)=Psi_ref(k_ref_local,1);
       k_ref_local=k_ref_local+1;
    end

    k_ref_local=1; % for reading reference signals
    hz = constants('hz'); % horizon period
    for i =1:innerDyn_length
        %% Generate discrete LPV Ad, Bd, Cd, Dd matrices
        [Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phit, phi_dot, thetat, theta_dot, psit, psi_dot]=LPV_cont_discrete(states);
        velocityXYZ_total=[velocityXYZ_total;[x_dot, y_dot, z_dot]];


        %% Generating the current state and the reference vector
        x_aug_t=[phit;phi_dot;thetat;theta_dot;psit;psi_dot;U2;U3;U4];

        k_ref_local=k_ref_local+controlled_states;

        % Start counting from the second sample period:
        % r=refSignals(Phi_ref_2;Theta_ref_2;Psi_ref_2;Phi_ref_3...) etc.
        if k_ref_local+controlled_states*hz-1 <= length(refSignals)
            r=refSignals(k_ref_local:k_ref_local+controlled_states*hz-1);
        else
            r=refSignals(k_ref_local:length(refSignals));
            hz=hz-1;
        end

        %% Generate simplification matrices for the cost function
        [Hdb,Fdbt,Cdb,Adc,C_cm_g,y_max_global,y_min_global] = MPC_simplification(Ad,Bd,Cd,Dd,hz,y_max,y_min);
   
        % Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du
        ft=[x_aug_t',r']*Fdbt;        
        %% Constraints
        CC=C_cm_g*Cdb;
        G=[CC;-CC];
        CAX=C_cm_g*Adc*x_aug_t;
        h1=y_max_global-CAX;
        h2=-y_min_global+CAX;
        ht=[h1;h2]';

        %% Calling the optimizer (quadprog)

        % Hdb must be positive definite for the problem to have finite minimum.
        % Check if matrix Hdb in the cost function is positive definite.
        [~,p] = chol(Hdb);
        if p~=0
           disp('Hdb is NOT positive definite');
        end

        % Call the solver
        options = optimoptions('quadprog','Display', 'off','LinearSolver','dense');

        [du,fval]=quadprog(Hdb,ft,G,ht,[],[],[],[],[],options);
        
        % Update the real inputs
        U2=U2+du(1);
        U3=U3+du(2);
        U4=U4+du(3);

        UTotal=[UTotal;U1,U2,U3,U4];

        % Compute the new omegas based on the new U-s.
        U1C=U1/ct;
        U2C=U2/(ct*l);
        U3C=U3/(ct*l);
        U4C=U4/cq;
        
        omega_Matrix=[1 1 1 1;0 1 0 -1;-1 0 1 0;-1 1 -1 1];
        UC_vector=[U1C;U2C;U3C;U4C];
        omegas_vector=inv(omega_Matrix)*UC_vector;

        omega1=sqrt(omegas_vector(1));
        omega2=sqrt(omegas_vector(2));
        omega3=sqrt(omegas_vector(3));
        omega4=sqrt(omegas_vector(4));

        % Compute the total omega
        omega_total=omega1-omega2+omega3-omega4;

        % Simulate the new states
        T = (Ts)*(i-1):(Ts)/30:Ts*(i-1)+(Ts);
        [T,x]=ode45(@(t,x) nonlinear_drone_model(t,x,[U1,U2,U3,U4]),T,states);
        states=x(end,:);
        states_total=[states_total;states];

        imaginary_check=imag(states)~=0;
        imaginary_check_sum=sum(imaginary_check);
        if imaginary_check_sum~=0
            disp('Imaginary part exists - something is wrong');
        end
    end
end

%% Plot the trajectory

% Trajectory
figure;
plot3(X_ref(:,2),Y_ref(:,2),Z_ref(:,2),'--b','LineWidth',2)
hold on
plot3(states_total(1:innerDyn_length:end,7),states_total(1:innerDyn_length:end,8),states_total(1:innerDyn_length:end,9),'r','LineWidth',1)
grid on;
xlabel('x-position [m]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
zlabel('z-position [m]','FontSize',15)
legend({'position-ref','position'},'Location','northeast','FontSize',15)

%% Plot the positions and velocities individually

% X and X_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),X_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,7),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('x-position [m]','FontSize',15)
legend({'x-ref','x-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),X_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,1),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('x-velocity [m/s]','FontSize',15)
legend({'x-dot-ref','x-velocity'},'Location','northeast','FontSize',15)

% Y and Y_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),Y_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,8),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
legend({'y-ref','y-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),Y_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,2),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('y-velocity [m/s]','FontSize',15)
legend({'y-dot-ref','y-velocity'},'Location','northeast','FontSize',15)

% Z and Z_dot
figure;
subplot(2,1,1)
plot(t(1:plotl),Z_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,1)
plot(t(1:plotl),states_total(1:innerDyn_length:end,9),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('z-position [m]','FontSize',15)
legend({'z-ref','z-position'},'Location','northeast','FontSize',15)
subplot(2,1,2)
plot(t(1:plotl),Z_dot_ref(1:plotl,2),'--b','LineWidth',2)
hold on
subplot(2,1,2)
plot(t(1:plotl),velocityXYZ_total(1:innerDyn_length:end,3),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('z-velocity [m/s]','FontSize',15)
legend({'z-dot-ref','z-velocity'},'Location','northeast','FontSize',15)

%% Plot the angles individually

% Phi
figure;
subplot(3,1,1)
plot(t_angles(1:length(ref_angles_total(:,1))),ref_angles_total(:,1),'--b','LineWidth',2)
hold on
subplot(3,1,1)
plot(t_angles(1:length(states_total(:,10))),states_total(:,10),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('phi-angle [rad]','FontSize',15)
legend({'phi-ref','phi-angle'},'Location','northeast','FontSize',15)

% Theta
subplot(3,1,2)
plot(t_angles(1:length(ref_angles_total(:,2))),ref_angles_total(:,2),'--b','LineWidth',2)
hold on
subplot(3,1,2)
plot(t_angles(1:length(states_total(:,11))),states_total(:,11),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('theta-angle [rad]','FontSize',15)
legend({'theta-ref','theta-angle'},'Location','northeast','FontSize',15)

% Psi
subplot(3,1,3)
plot(t_angles(1:length(ref_angles_total(:,3))),ref_angles_total(:,3),'--b','LineWidth',2)
hold on
subplot(3,1,3)
plot(t_angles(1:length(states_total(:,12))),states_total(:,12),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',15)
ylabel('psi-angle [rad]','FontSize',15)
legend({'psi-ref','psi-angle'},'Location','northeast','FontSize',15)


%% Plot the inputs

figure
subplot(4,1,1)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,1))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U1 [N]','FontSize',15)
subplot(4,1,2)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,2))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U2 [Nm]','FontSize',15)
subplot(4,1,3)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,3))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U3 [Nm]','FontSize',15)
subplot(4,1,4)
plot(t_angles(1:length(states_total(:,10))),UTotal(:,4))
grid on
xlabel('time [s]','FontSize',15)
ylabel('U4 [Nm]','FontSize',15)

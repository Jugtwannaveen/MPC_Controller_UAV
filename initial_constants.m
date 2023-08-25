function constants=initial_constants()
    
    % Constants 
    Ix = 0.0034; %kg*m^2
    Iy = 0.0034; %kg*m^2
    Iz  = 0.006; %kg*m^2
    m  = 0.698; %kg
    g  = 9.81; %m/s^2
    Jtp=1.302*10^(-6); %N*m*s^2=kg*m^2
    Ts=0.1; %s
    
    % Matrix weights for the cost function (They must be diagonal)
    Q=[10 0 0;0 10 0;0 0 10]; % weights for outputs (output x output)
    S=[10 0 0;0 10 0;0 0 10]; % weights for the final horizon outputs (output x output)
    R=[10 0 0;0 10 0;0 0 10]; % weights for inputs (input x input)
    
    ct = 7.6184*10^(-8)*(60/(2*pi))^2; %N*s^2
    cq = 2.6839*10^(-9)*(60/(2*pi))^2; %N*m^2
    l = 0.171; %m;
    
    controlled_states=3;
    hz = 4; % horizon period
    
    innerDyn_length=4; % Number of inner control loop iterations
    
    px=[-1 -2];
    py=[-1 -2];
    pz=[-1 -2];
    
    % Choose your trajectory (1,2,3,4,5)
    trajectory=1;
    
    %% Constraints
    omega_min=110*pi/3; % [rad/s]
    omega_max=860*pi/3; % [rad/s]
    
    % Constraint matrix for extracting desired outputs for constraints
    C_cm=[0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1];
    
    keySet={'Ix','Iy','Iz','m','g','Jtp','Ts','Q','S','R','ct','cq','l','controlled_states','hz','innerDyn_length','px','py','pz','trajectory','omega_min','omega_max','C_cm'};
    constants_list={Ix Iy Iz m g Jtp Ts Q S R ct cq l controlled_states hz innerDyn_length px py pz trajectory omega_min omega_max C_cm};
    constants=containers.Map(keySet,constants_list);

end

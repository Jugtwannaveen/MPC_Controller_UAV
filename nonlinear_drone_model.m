function dx = nonlinear_drone_model(t, states, U)
    % In this simulation, the body frame and its transformation is used
    % instead of a hybrid frame. That is because for the solver ode45, it
    % is important to have the nonlinear system of equations in the first
    % order form. 
    
    % Constants
    constants = initial_constants();
    Ix = constants('Ix'); %kg*m^2
    Iy = constants('Iy'); %kg*m^2
    Iz = constants('Iz'); %kg*m^2
    m  = constants('m'); %kg
    g  = constants('g'); %m/s^2
    Jtp=constants('Jtp'); %N*m*s^2=kg*m^2
    
    % States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]
    u = states(1);
    v = states(2);
    w = states(3);
    p = states(4);
    q = states(5);
    r = states(6);
    x = states(7);
    y = states(8);
    z = states(9);
    phi = states(10);
    theta = states(11);
    psi = states(12);
    

    % Inputs:
    
    U1  = U(1);
    U2  = U(2);
    U3  = U(3);
    U4  = U(4);
    
    
    % Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
    R_matrix=[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    
    % Transformation matrix that relates p,q,r with phi_dot,theta_dot,psi_dot
    T_matrix=[1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
        0, cos(phi), -sin(phi); ...
        0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
    
    global omega_total
    
    % The nonlinear equation describing the dynamics of the drone
    dx(1,1)=(v*r-w*q)+g*sin(theta); %u_dot
    dx(2,1)=(w*p-u*r)-g*cos(theta)*sin(phi); %v_dot
    dx(3,1)=(u*q-v*p)-g*cos(theta)*cos(phi)+U1/m; %w_dot
    dx(4,1)=q*r*(Iy-Iz)/Ix-Jtp/Ix*q*omega_total+U2/Ix; %p_dot
    dx(5,1)=p*r*(Iz-Ix)/Iy+Jtp/Iy*p*omega_total+U3/Iy; %q_dot
    dx(6,1)=p*q*(Ix-Iy)/Iz+U4/Iz; %r_dot
    dx(7,1)=R_matrix(1,:)*[u;v;w]; %x_dot
    dx(8,1)=R_matrix(2,:)*[u;v;w]; %y_dot
    dx(9,1)=R_matrix(3,:)*[u;v;w]; %z_dot
    dx(10,1)=T_matrix(1,:)*[p;q;r]; %phi_dot
    dx(11,1)=T_matrix(2,:)*[p;q;r]; %theta_dot
    dx(12,1)=T_matrix(3,:)*[p;q;r]; %psi_dot
    
end


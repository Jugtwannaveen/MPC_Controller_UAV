function [Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot] = LPV_cont_discrete(states)
    % This is an LPV model concerning the three rotational axis.

    % Get the constants from the general pool of constants
    constants = initial_constants();
    Ix = constants('Ix'); %kg*m^2
    Iy = constants('Iy'); %kg*m^2
    Iz = constants('Iz'); %kg*m^2
    Jtp=constants('Jtp'); %N*m*s^2=kg*m^2
    Ts=constants('Ts'); %s


    % Assign the states
    % States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]
    u = states(1);
    v = states(2);
    w = states(3);
    p = states(4);
    q = states(5);
    r = states(6);
    phi = states(10);
    theta = states(11);
    psi = states(12);

    global omega_total;

    %%

    % Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
    R_matrix=[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

    x_dot=R_matrix(1,:)*[u;v;w]; %x_dot
    y_dot=R_matrix(2,:)*[u;v;w]; %y_dot
    z_dot=R_matrix(3,:)*[u;v;w]; %z_dot

    % To get phi_dot, theta_dot, psi_dot, you need the T matrix

    % Transformation matrix that relates p,q,r with phi_dot,theta_dot,psi_dot
    T_matrix=[1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
        0, cos(phi), -sin(phi); ...
        0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

    phi_dot=T_matrix(1,:)*[p;q;r]; %phi_dot
    theta_dot=T_matrix(2,:)*[p;q;r]; %theta_dot
    psi_dot=T_matrix(3,:)*[p;q;r]; %psi_dot

    A12=1;
    A24=-omega_total*Jtp/Ix;
    A26=theta_dot*(Iy-Iz)/Ix;
    A34=1;
    A42=omega_total*Jtp/Iy;
    A46=phi_dot*(Iz-Ix)/Iy;
    A56=1;
    A62=(theta_dot/2)*(Ix-Iy)/Iz;
    A64=(phi_dot/2)*(Ix-Iy)/Iz;

    A = [0  A12   0   0     0   0;
         0  0     0   A24   0   A26;
         0  0     0   A34   0   0;
         0  A42   0   0     0   A46;
         0  0     0   0     0   A56;
         0  A62   0   A64   0   0];


    B = [ 0       0     0    ;
          1/Ix    0     0    ;
          0       0     0    ;
          0       1/Iy  0    ;
          0       0     0    ;
          0       0     1/Iz];

    C = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];

    D=zeros(3);


    % Discretize the system

    % Forward Euler
    Ad=eye(length(A(1,:)))+Ts*A;
    Bd=Ts*B;
    Cd=C;
    Dd=D;


end

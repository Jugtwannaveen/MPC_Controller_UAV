function [Phi_ref, Theta_ref, U1]=pos_controller(X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,Psi_ref,states)


%% Load the constants
constants=initial_constants();
m  = constants('m');
g  = constants('g');
px=constants('px');
py=constants('py');
pz=constants('pz');

%% Assign the states
% States: [u,v,w,p,q,r,x,y,z,phi,theta,psi]

u = states(1);
v = states(2);
w = states(3);
x = states(7);
y = states(8);
z = states(9);
phi = states(10);
theta = states(11);
psi = states(12);

% Rotational matrix that relates u,v,w with x_dot,y_dot,z_dot
R_matrix=[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
    cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
    -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

x_dot=R_matrix(1,:)*[u;v;w]; %x_dot
y_dot=R_matrix(2,:)*[u;v;w]; %y_dot
z_dot=R_matrix(3,:)*[u;v;w]; %z_dot

%% Compute the errors
ex=X_ref-x;
ex_dot=X_dot_ref-x_dot;
ey=Y_ref-y;
ey_dot=Y_dot_ref-y_dot;
ez=Z_ref-z;
ez_dot=Z_dot_ref-z_dot;

%% Compute the the constants K1, K2, and the values vx, vy, vz to stabilize the position subsystem
kx1=(px(1)-(px(1)+px(2))/2).^2-(px(1)+px(2)).^2/4;
kx2=px(1)+px(2);
kx1=real(kx1);
kx2=real(kx2);

ky1=(py(1)-(py(1)+py(2))/2).^2-(py(1)+py(2)).^2/4;
ky2=py(1)+py(2);
ky1=real(ky1);
ky2=real(ky2);

kz1=(pz(1)-(pz(1)+pz(2))/2).^2-(pz(1)+pz(2)).^2/4;
kz2=pz(1)+pz(2);
kz1=real(kz1);
kz2=real(kz2);

% Compute the values vx, vy, vz for the position controller
ux=kx1*ex+kx2*ex_dot;
uy=ky1*ey+ky2*ey_dot;
uz=kz1*ez+kz2*ez_dot;
%% Continue here!
vx=X_dot_dot_ref-ux;
vy=Y_dot_dot_ref-uy;
vz=Z_dot_dot_ref-uz;

%% Compute phi, theta, U1
a=vx/(vz+g);
b=vy/(vz+g);
c=cos(Psi_ref);
d=sin(Psi_ref);
tan_theta=a*c+b*d;
Theta_ref=atan(tan_theta);

if Psi_ref>=0
    Psi_ref_singularity=Psi_ref-floor(abs(Psi_ref)/(2*pi))*2*pi;
else
    Psi_ref_singularity=Psi_ref+floor(abs(Psi_ref)/(2*pi))*2*pi;
end


if or(or(abs(Psi_ref_singularity)<pi/4,abs(Psi_ref_singularity)>7*pi/4), ...
        and(abs(Psi_ref_singularity)>3*pi/4,abs(Psi_ref_singularity)<5*pi/4))
    tan_phi=cos(Theta_ref)*(tan(Theta_ref)*d-b)/c;
else
    tan_phi=cos(Theta_ref)*(a-tan(Theta_ref)*c)/d;
end

Phi_ref=atan(tan_phi);
U1=(vz+g)*m/(cos(Phi_ref)*cos(Theta_ref));

end
%% Position trajectory generation
function [X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,psi_ref]=trajectory_generator(t,r,f,height_i,height_f)

constants = initial_constants();
Ts=constants('Ts'); %s
innerDyn_length=constants('innerDyn_length'); % Number of inner control loop iterations
trajectory=constants('trajectory');

alpha=2*pi*f.*t;
d_height=height_f-height_i;

if trajectory==1
    x = r.*cos(alpha);
    y = r.*sin(alpha);
    z = height_i+d_height/t(end)*t;
elseif trajectory==2
    x = (r/10.*t+2).*cos(alpha);
    y = (r/10.*t+2).*sin(alpha);
    z = height_i+d_height/t(end)*t;
elseif trajectory==3
    x = 2.*t/20+1;
    y = 2.*t/20-2;
    z = height_i+d_height/t(end)*t;
elseif trajectory==4
    x = r.*cos(alpha);
    y = 2.*t;
    z = height_i+d_height/t(end)*t;
else
    x = r.*cos(alpha);
    y = r.*sin(alpha);
    z = height_i+50*d_height/t(end)*sin(t);
end

dx=[x(2)-x(1),x(2:end)-x(1:end-1)];
dy=[y(2)-y(1),y(2:end)-y(1:end-1)];
dz=[z(2)-z(1),z(2:end)-z(1:end-1)];

x_dot=round(dx.*(1/(Ts*innerDyn_length)),8);
y_dot=round(dy.*(1/(Ts*innerDyn_length)),8);
z_dot=round(dz.*(1/(Ts*innerDyn_length)),8);

%% Generate the reference accelerations
ddx=[x_dot(2)-x_dot(1),x_dot(2:end)-x_dot(1:end-1)];
ddy=[y_dot(2)-y_dot(1),y_dot(2:end)-y_dot(1:end-1)];
ddz=[z_dot(2)-z_dot(1),z_dot(2:end)-z_dot(1:end-1)];

x_dot_dot=round(ddx.*(1/(Ts*innerDyn_length)),8);
y_dot_dot=round(ddy.*(1/(Ts*innerDyn_length)),8);
z_dot_dot=round(ddz.*(1/(Ts*innerDyn_length)),8);
%% Generate the reference yaw angles

psi=zeros(1,length(x));
psiInt=psi;
psi(1)=atan2(y(1),x(1))+pi/2;
psi(2:end)=atan2(dy(2:end),dx(2:end));

dpsi=psi(2:end)-psi(1:end-1);

psiInt(1)=psi(1);
for i = 2:length(psiInt)
    if dpsi(i-1)<-pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)+2*pi);
    elseif dpsi(i-1)>pi
        psiInt(i)=psiInt(i-1)+(dpsi(i-1)-2*pi);
    else
        psiInt(i)=psiInt(i-1)+dpsi(i-1);
    end
end
psiInt=round(psiInt,8);

X_ref = [t' x'];
X_dot_ref = [t' x_dot'];
X_dot_dot_ref = [t' x_dot_dot'];
Y_ref = [t' y'];
Y_dot_ref = [t' y_dot'];
Y_dot_dot_ref = [t' y_dot_dot'];
Z_ref = [t' z'];
Z_dot_ref = [t' z_dot'];
Z_dot_dot_ref = [t' z_dot_dot'];
psi_ref = [t' psiInt'];

end

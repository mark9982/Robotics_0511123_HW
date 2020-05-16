function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);
R_d=eye(3);
Omega_d=[0;0.1;1];
%xd=[0,0,2];
J=[P.Jxx,0,0;0,P.Jyy,0;0,0,P.Jzz];

e3 = [0;0;1];
e_R=0.5*vee((R_d'*R-R'*R_d));
e_Omega=Omega-R'*R_d*Omega_d;
e_x=x-xd;

%e_v=v-xd_dot;
%xd_double_dot=

R_dot=R*hat(Omega);
R_d_dot=R_d*hat(Omega_d);

%attitude
M=-P.kR*e_R-P.kOmega*e_Omega+cross(Omega,J*Omega);
f=(P.kx*(x(3)-xd(3))+P.mass*P.gravity)/dot(e3,R*e3);



%position
%M=-P.kR*e_R-P.kOmega*e_Omega+cross(Omega,J*Omega);

%f=dot(P.kx*e_x+P.mass*P.gravity*e3,R*e3);
%f=dot(P.kx*e_x+P.kv*e_v+P.mass*P.gravity*e3-P.mass*xd_double_dot,R*e3);



out = [f;M;e_R];
end
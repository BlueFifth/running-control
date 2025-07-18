%[text] ## Deriving the full dynamics of Baleka's legs
%[text] Using the general equation as follows:
%[text] $M(q)\\ddot{q} + C(q, \\dot{q})\\dot{q} + g(q) + f\_d(\\dot{q}) + f\_c(q, \\tau) = \\tau - J^TF\_{ext}$
%[text] where $M(q)\n$is the mass matrix, $C(q, \\dot{q})\n$ is Coriolis force $G(q)$ is gravity, $f\_d(\\dot{q})$ is damping friction, $f\_c(q, \\tau)$ is torque dependent Coulomb friction, $\\tau$ is the torque applied at the motors, $J$ is the Jacobian matrix and $F\_{ext}$ is the external reaction force exerted on the foot in Cartesian space.
%[text] 
%[text] For now we're assuming that Baleka's hips are locked, so the body cannot rotate.
%[text] 
%[text] This code is adapted from Heather Wimberly's Kemba monoped scripts
%%
%[text] ### Declare variables
% Generalized coordinates
% C.O.M x, y; left hip angle, right hip angle, left knee angle, right knee
% angle
% Defined as per image (TODO: ADD PPT image)


syms x y th1 th2 ph1 ph2 'real'
syms dx dy dth1 dth2 dph1 dph2 'real'
syms ddx ddy ddth1 ddth2 ddph1 ddph2 'real'

q = [x;y;th1;th2;ph1;ph2];
dq = [dx;dy;dth1;dth2;dph1;dph2];
ddq = [ddx;ddy;ddth1;ddth2;ddph1;ddph2];

% System constants
syms th3 grav 'real'

% mass: body, l1, l2, l3, l4
syms mb ml1 ml2 ml3 'real'
% Inertias: body, l1, l2, l3, l4
syms Ib Il1 Il2 Il3 'real'

%Lengths
syms l1 l2 l3 l4 l5 foot_offset 'real'

% Center of mass (along the body)
syms cby cl1x cl2x cl2y cl3x 'real' %

%%
%[text] ## Rotation Matrices
% Idk what Heather did here - coming back to it
%[text] 
%%
%[text] ## Position and velocity
% done in terms of global [x;y] - starting from the body's initial position

p_body = [x;y]; % body centre position

% left leg
p_hip_left = p_body - [l5/2; 0];
p_knee_left = p_hip_left + trotz(th1)*[l1; 0];
p_joint_left = p_knee_left + trotz(ph1 + th1)*[l2;0];


% right leg
p_hip_right = p_body + [l5/2; 0];
p_knee_right = p_hip_right + trotz(th2)*[l1;0];
p_joint_right = p_knee_right + trotz(th2+ph2)*[l3;0];

p_foot = p_joint_left + trotz(th1 + ph1 + th3)*[l4;0] - [0;foot_offset];


% Position of COMs:
r_body = p_body + [0;cby] %[output:2d1ea9a1]
r_upperLink_left = p_hip_left + trotz(th1)*[cl1x;0];
r_upperLink_right = p_hip_right + trotz(th2)*[cl1x;0];
r_lowerLink_left = p_knee_left +trotz(th1+ph1)*[cl2x;cl2y]; % Includes foot
r_lowerLink_right = p_knee_right + trotz(th2+ph2)*[cl3x;0];

% velocities of bodies
v_body = getVel(r_body, q, dq);
v_upperLink_left = getVel(r_upperLink_left, q, dq);
v_upperLink_right = getVel(r_upperLink_right, q, dq);
v_lowerLink_left = getVel(r_lowerLink_left, q, dq);
v_lowerLink_right = getVel(r_lowerLink_right, q, dq);

v_foot = getVel(p_foot, q, dq);
% Rotational velocities
w_body = 0;
w_upperLink_left = dth1;
w_upperLink_right = dth2;
w_lowerLink_left = dth1 + dph1;
w_lowerLink_right = dth2 + dph2;

%%
%[text] ## Energy
% Kinetic energy
T_body = getT(mb,v_body, Ib, w_body);
T_upperLink_left = getT(ml1, v_upperLink_left, Il1, w_upperLink_left);
T_upperLink_right = getT(ml1, v_upperLink_right, Il1, w_upperLink_right);
T_lowerLink_left = getT(ml2, v_lowerLink_left, Il2, w_lowerLink_left);
T_lowerLink_right = getT(ml3, v_lowerLink_right, Il3, w_lowerLink_right);

T = simplify(T_body + T_upperLink_left + T_upperLink_right +T_lowerLink_left + T_lowerLink_right);

% Potential energy
V_body = grav*mb*r_body(2);
V_upperLink_left = grav*ml1*r_upperLink_left(2);
V_upperLink_right = grav*ml1*r_upperLink_right(2);
V_lowerLink_left = grav*ml2*r_lowerLink_left(2);
V_lowerLink_right = grav*ml3*r_lowerLink_right(2);

V = simplify(V_body + V_upperLink_right + V_upperLink_left + V_lowerLink_right + V_lowerLink_left);

%%
%[text] ## Manipulator Equation
% Mass matrix
M = hessian(T, dq);

% Define dM
dM = sym(zeros(length(M), length(M)));
for i=1:length(M)
    for j=1:length(M)
        dM(i,j) = jacobian(M(i,j), q)*dq;
    end
end
dM = simplify(dM);

% Define Gravity Matrix
G = jacobian(V,q);
G = simplify(transpose(G));

% Define Coriolis Matrix
C = dM*dq - transpose(jacobian(T,q));
C = simplify(C);
%%
%[text] ## Generalised Forces
% Torque at motors (ignoring damping for now)
syms t1 t2 b 'real'

torque = [0;0;t1;t2;0;0];
B = diag([0, 0, b, b, 0, 0]);


Q_t = torque - B*dq;

% Closed loop constraint forces
syms f_cx f_cy 'real'

h = p_joint_left - p_joint_right; % Connector joints must be in the same place

H = jacobian(h, q);

F_c = [f_cx; f_cy];

Q_Fc = transpose(H)*F_c;

% Ground contact forces
syms grf_n grf_t 'real'

GRF = [grf_t; grf_n];

Q_GRF = simplify(transpose(jacobian(p_foot, q))*GRF);

Q = simplify(Q_t + Q_Fc + Q_GRF);

%%
%[text] ## Substitution
BalekaParams;

grav = 9.81;


th3 = FootLink.th3;

mb = HalfBody.m;
ml1 = UpperLink.m;
ml2 = FootLink.m;
ml3 = LowerLink.m;

Ib = HalfBody.I;
Il1 = UpperLink.I;
Il2 = FootLink.I;
Il3 = UpperLink.I;

l1 = UpperLink.l;
l2 = FootLink.l2;
l3 = LowerLink.l;
l4 = FootLink.l4;
l5 = HalfBody.l;
foot_offset = FootLink.offset;
cby = HalfBody.cmy;
cl1x = UpperLink.cmx;
cl2x = FootLink.cmx;
cl2y = FootLink.cmy;
cl3x = LowerLink.cmx;

b = Damping;

clear FootLink LowerLink HalfBody UpperLink
H = simplify(subs(H));
J = simplify(subs(simplify(transpose(jacobian(p_foot, q)))));
M = simplify(subs(M));
% G = simplify(subs(G));
C = simplify(subs(C));
Q = simplify(subs(Q));
%%
%[text] ## System Acceleration Function
% comment this section after running once as it takes a while to run
% get the inverse of M using a proxy to speed up computation
N = 6;
syms prox [N N]
proxinv = inv(prox); 
proxinv_M = subs(proxinv, prox, M);


iM = inv(M);
% formula for acceleration of each coordinate
accel = proxinv_M*(-G-C+Q);
isequal(M, proxinv_M) %[output:07eb40c2]
% % export acceleration to callable function
% accel_Func = matlabFunction(accel,"File","SimFunctions/accel_Func","Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2 t1 t2 f_cx f_cy grf_t grf_n]);
%%
%[text] ## Foot kinematics function
% % get the position and velocity of the foot to use when determining grf
% kinematics_foot = [subs(p_foot), subs(v_foot)];
% % export to a callable function
% foot_Func = matlabFunction(kinematics_foot,"File","SimFunctions/foot_Func","Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2]);
%%
%[text] ## Constraint forces
% % determine the equation for the constraint of the system
% dH = sym(zeros(height(H),width(H)));
% for i=1:height(H)
%     for j=1:width(H)
%         dH(i,j) = jacobian(H(i,j),q)*dq;
%     end
% end
% dH = simplify(dH);
% 
% phi_eqn = Q_t  + Q_GRF;
% 
% % create a proxy inverse matrix to be used in final force calculation
% N = 2;
% syms prox [N N]
% proxinv = inv(prox);
% divisor = H*proxinv_M*transpose(H);%,"IgnoreAnalyticConstraints",true);
% inverse = subs(proxinv,prox,divisor);%,"IgnoreAnalyticConstraints",true);
% 
% % rearranged manipulator equation
% constraint = -inverse*(H*proxinv_M*(phi_eqn - C - G) + dH*dq);
% constraint = subs(constraint);
% % constraint = simplify(subs(constraint),"IgnoreAnalyticConstraints",true);
% 
% % export constraint equation as callable function
% constraint_force = matlabFunction(constraint,"File","SimFunctions/constraint_Func","Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2 t1 t2 grf_t grf_n]);
%%
%[text] ## Full feedback
% Ht_calc_func = matlabFunction(transpose(H), "File","SimFunctions/Ht_calc_func", "Vars",[x y th1 th2 ph1 ph2]);
% J_calc_func = matlabFunction(J, "File","SimFunctions/J_calc_func", "Vars",[x y th1 th2 ph1 ph2]);
% M_calc_func = matlabFunction(M, "File","SimFunctions/M_calc_func", "Vars",[x y th1 th2 ph1 ph2]);
% C_calc_func = matlabFunction(C, "File","SimFunctions/C_calc_func", "Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2]);
% G_calc_func = matlabFunction(G, "File","SimFunctions/G_calc_func", "Vars",[x y th1 th2 ph1 ph2]);
%%
% ff_torque_extra = M*ddq + C + G;% + Q_Fc; % + Jt*(-Kpx)
% ff_torque_extra = ff_torque_extra(3:4);
% ff_torque_func = matlabFunction(subs(ff_torque_extra), "File","SimFunctions/ff_torque_func", "Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2 ddx ddy ddth1 ddth2 ddph1 ddph2 f_cx f_cy])
%%
%[text] ## Mapping of Angles
% % finding ph1 and ph2
% a = -l1*cos(th1) + l5 + l1*cos(th2); 
% b = -l1*sin(th1) + l1*sin(th2);
% B = atan2(b, a);
% c = sqrt(a^2 + b^2);
% th4 =  (acos((l2^2+c^2-l3^2)/(2*l2*c)) - B); %Cos rule of triangle L2 L3 c minus angle B
% 
% ph1 = 2*pi - th4 - th1;
% dph1 = getVel(ph1, q, dq);
% ph1_calc = matlabFunction(ph1,"File","SimFunctions/ph1_calc","Vars",[th1 th2]);
% dph1_calc = matlabFunction(dph1,"File","SimFunctions/dph1_calc","Vars",[th1 th2 dth1 dth2]);
% 
% th5 = asin(l2*sin(th4+B)/l3) + B; % Sine rule of triangle L2 L3 c plus angle B
% ph2 = pi - th2  + th5;
% dph2 = getVel(ph1, q, dq);
% ph2_calc = matlabFunction(ph2,"File","SimFunctions/ph2_calc","Vars",[th1 th2]);
% dph2_calc = matlabFunction(dph2,"File","SimFunctions/dph2_calc","Vars",[th1 th2 dth1 dth2]);
%%
%[text] ## Mapping to template
% % x, y
% legXY = p_foot - p_body;
% 
% %r
% r = sqrt(legXY(1)^2 + legXY(2)^2);
% th = atan2(legXY(2), legXY(1));
% 
% dr = getVel(r, q, dq);
% dth = getVel(th, q, dq);
% r_calc = matlabFunction(subs(r),"File","SimFunctions/r_calc","Vars",[th1 th2]);
% dr_calc = matlabFunction(subs(dr),"File","SimFunctions/dr_calc","Vars",[th1 th2 dth1 dth2]);
% th_calc = matlabFunction(subs(th),"File","SimFunctions/th_calc","Vars",[th1 th2]);
% dth_calc = matlabFunction(subs(dth),"File","SimFunctions/dth_calc","Vars",[th1 th2 dth1 dth2]);
%%
%[text] ## 
%%
%[text] ## Functions
function rot = rotz(th)
    rot = [cos(th),sin(th);-sin(th),cos(th)];
end
function rot = trotz(th)
    rot = transpose(rotz(th));
end
function vel = getVel(r, q, dq)
    vel = simplify(jacobian(r, q)*dq);
end
function T = getT(m, v, I, w)
    T = simplify(0.5*m*transpose(v)*v + 0.5*I*w^2);
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":15.4}
%---
%[output:2d1ea9a1]
%   data: {"dataType":"symbolic","outputData":{"name":"r_body","value":"\\left(\\begin{array}{c}\nx\\\\\n\\mathrm{cby}+y\n\\end{array}\\right)"}}
%---
%[output:07eb40c2]
%   data: {"dataType":"textualVariable","outputData":{"header":"logical","name":"ans","value":"   0\n"}}
%---

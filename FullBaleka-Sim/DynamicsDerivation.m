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


syms x y th1 th2 th3 th4 ph1 ph2 ph3 ph4 'real'
syms dx dy dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4 'real'
syms ddx ddy ddth1 ddth2 ddth3 ddth4 ddph1 ddph2 ddph3 ddph4 'real'

q = [x;y;th1;th2;th3;th4;ph1;ph2;ph3;ph4];
dq = [dx;dy;dth1;dth2;dth3;dth4;dph1;dph2;dph3;dph4];
ddq = [ddx;ddy;ddth1;ddth2;ddth3;ddth4;ddph1;ddph2;ddph3;ddph4];

% System constants
syms thl grav 'real'

% mass: body, l1, l2, l3, l4
syms mb ml1 ml2 ml3 'real'
% Inertias: body, l1, l2, l3, l4
syms Ib Il1 Il2 Il3 'real'

%Lengths
syms l1 l2 l3 l4 l5 foot_offset 'real'

% Center of mass (along the body)
syms cby cl1x cl2x cl2y cl3x 'real' %
%%
%[text] ## Position and velocity
% done in terms of global [x;y] - starting from the body's initial position

p_body = [x;y]; % body centre position
% Front leg
% left
p_f_hip_left = p_body - [l5/2; 0];
p_f_knee_left = p_f_hip_left + trotz(th1)*[l1; 0];
p_f_joint_left = p_f_knee_left + trotz(ph1 + th1)*[l2;0];


% right
p_f_hip_right = p_body + [l5/2; 0];
p_f_knee_right = p_f_hip_right + trotz(th2)*[l1;0];
p_f_joint_right = p_f_knee_right + trotz(th2+ph2)*[l3;0];

p_f_foot = p_f_joint_left + trotz(th1 + ph1 + thl)*[l4;0] - [0;foot_offset];

% Back leg
% left
p_b_hip_left = p_body - [l5/2; 0];
p_b_knee_left = p_b_hip_left + trotz(th3)*[l1; 0];
p_b_joint_left = p_b_knee_left + trotz(ph3 + th3)*[l2;0];


% right
p_b_hip_right = p_body + [l5/2; 0];
p_b_knee_right = p_b_hip_right + trotz(th4)*[l1;0];
p_b_joint_right = p_b_knee_right + trotz(th4+ph4)*[l3;0];

p_b_foot = p_b_joint_left + trotz(th3 + ph3 + thl)*[l4;0] - [0;foot_offset];


% Position of COMs:
r_body = p_body + [0;cby] %[output:32ef12f1]

r_f_upperLink_left = p_f_hip_left + trotz(th1)*[cl1x;0];
r_f_upperLink_right = p_f_hip_right + trotz(th2)*[cl1x;0];
r_f_lowerLink_left = p_f_knee_left +trotz(th1+ph1)*[cl2x;cl2y]; % Includes foot
r_f_lowerLink_right = p_f_knee_right + trotz(th2+ph2)*[cl3x;0];

r_b_upperLink_left = p_b_hip_left + trotz(th3)*[cl1x;0];
r_b_upperLink_right = p_b_hip_right + trotz(th4)*[cl1x;0];
r_b_lowerLink_left = p_b_knee_left +trotz(th3+ph3)*[cl2x;cl2y]; % Includes foot
r_b_lowerLink_right = p_b_knee_right + trotz(th4+ph4)*[cl3x;0];

% velocities of bodies
v_body = getVel(r_body, q, dq);

v_f_upperLink_left = getVel(r_f_upperLink_left, q, dq);
v_f_upperLink_right = getVel(r_f_upperLink_right, q, dq);
v_f_lowerLink_left = getVel(r_f_lowerLink_left, q, dq);
v_f_lowerLink_right = getVel(r_f_lowerLink_right, q, dq);
v_f_foot = getVel(p_f_foot, q, dq);

v_b_upperLink_left = getVel(r_b_upperLink_left, q, dq);
v_b_upperLink_right = getVel(r_b_upperLink_right, q, dq);
v_b_lowerLink_left = getVel(r_b_lowerLink_left, q, dq);
v_b_lowerLink_right = getVel(r_b_lowerLink_right, q, dq);
v_b_foot = getVel(p_b_foot, q, dq);

% Rotational velocities
% NOTE: Should we be doing parallel axis theorem??
w_body = 0;
w_f_upperLink_left = dth1;
w_f_upperLink_right = dth2;
w_f_lowerLink_left = dth1 + dph1;
w_f_lowerLink_right = dth2 + dph2;

w_b_upperLink_left = dth3;
w_b_upperLink_right = dth4;
w_b_lowerLink_left = dth3 + dph3;
w_b_lowerLink_right = dth4 + dph4;
%%
%[text] ## Energy
% Kinetic energy
% NOTE: Should we be doing parallel axis theorem??
T_body = getT(mb,v_body, Ib, w_body);

T_f_upperLink_left = getT(ml1, v_f_upperLink_left, Il1, w_f_upperLink_left);
T_f_upperLink_right = getT(ml1, v_f_upperLink_right, Il1, w_f_upperLink_right);
T_f_lowerLink_left = getT(ml2, v_f_lowerLink_left, Il2, w_f_lowerLink_left);
T_f_lowerLink_right = getT(ml3, v_f_lowerLink_right, Il3, w_f_lowerLink_right);

T_b_upperLink_left = getT(ml1, v_b_upperLink_left, Il1, w_b_upperLink_left);
T_b_upperLink_right = getT(ml1, v_b_upperLink_right, Il1, w_b_upperLink_right);
T_b_lowerLink_left = getT(ml2, v_b_lowerLink_left, Il2, w_b_lowerLink_left);
T_b_lowerLink_right = getT(ml3, v_b_lowerLink_right, Il3, w_b_lowerLink_right);

T = simplify(T_body + T_f_upperLink_left + T_f_upperLink_right +T_f_lowerLink_left ...
    + T_f_lowerLink_right + T_b_upperLink_left + T_b_upperLink_right +T_b_lowerLink_left ...
    + T_b_lowerLink_right);

% Potential energy
V_body = grav*mb*r_body(2);

V_f_upperLink_left = grav*ml1*r_f_upperLink_left(2);
V_f_upperLink_right = grav*ml1*r_f_upperLink_right(2);
V_f_lowerLink_left = grav*ml2*r_f_lowerLink_left(2);
V_f_lowerLink_right = grav*ml3*r_f_lowerLink_right(2);

V_b_upperLink_left = grav*ml1*r_b_upperLink_left(2);
V_b_upperLink_right = grav*ml1*r_b_upperLink_right(2);
V_b_lowerLink_left = grav*ml2*r_b_lowerLink_left(2);
V_b_lowerLink_right = grav*ml3*r_b_lowerLink_right(2);

V = simplify(V_body + V_f_upperLink_right + V_f_upperLink_left + V_f_lowerLink_right + ...
    V_f_lowerLink_left + V_b_upperLink_right + V_b_upperLink_left + V_b_lowerLink_right + ...
    V_b_lowerLink_left);

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
syms t1 t2 t3 t4 b 'real'

torque = [0;0;t1;t2;t3;t4;0;0;0;0];
B = diag([0, 0, b, b, b, b, 0, 0, 0, 0]);


Q_t = torque - B*dq;

% Constraint forces (Closed loop)
syms f_fcx f_fcy f_bcx f_bcy 'real'

h_f = p_f_joint_left - p_f_joint_right; % Connector joints must be in the same place
h_b = p_b_joint_left - p_b_joint_right;
h = [h_f;h_b];

H = jacobian(h, q);

F_c = [f_fcx; f_fcy;f_bcx; f_bcy];

dH = sym(zeros(height(H), width(H)));
for i=1:height(H)
    for j=1:width(H)
        dH(i,j) = jacobian(H(i,j), q)*dq;
    end
end
dH = simplify(dH);

% Ground contact forces
syms grf_fy grf_fx grf_bx grf_by 'real'

GRF = [grf_fx; grf_fy; grf_bx; grf_by];

J_f = simplify(jacobian(p_f_foot, q));
J_b = simplify(jacobian(p_b_foot, q));
J_fb = simplify(jacobian([p_f_foot;p_b_foot], q));

dJ_f = sym(zeros(height(J_f), width(J_f)));
for i=1:height(J_f)
    for j=1:width(J_f)
        dJ_f(i,j) = jacobian(J_f(i,j), q)*dq;
    end
end
dJ_f = simplify(dJ_f);

dJ_b = sym(zeros(height(J_b), width(J_b)));
for i=1:height(J_b)
    for j=1:width(J_b)
        dJ_b(i,j) = jacobian(J_b(i,j), q)*dq;
    end
end
dJ_b = simplify(dJ_b);

dJ_fb = sym(zeros(height(J_fb), width(J_fb)));
for i=1:height(J_fb)
    for j=1:width(J_fb)
        dJ_fb(i,j) = jacobian(J_fb(i,j), q)*dq;
    end
end
dJ_fb = simplify(dJ_fb);

%%
%[text] ## Substitution
BalekaParams;

grav = 9.81;


thl = FootLink.th3;

mb = FullBody.m;
ml1 = UpperLink.m;
ml2 = FootLink.m;
ml3 = LowerLink.m;

Ib = FullBody.I;
Il1 = UpperLink.I;
Il2 = FootLink.I;
Il3 = UpperLink.I;

l1 = UpperLink.l;
l2 = FootLink.l2;
l3 = LowerLink.l;
l4 = FootLink.l4;
l5 = FullBody.l;
foot_offset = FootLink.offset;
cby = FullBody.cmy;
cl1x = UpperLink.cmx;
cl2x = FootLink.cmx;
cl2y = FootLink.cmy;
cl3x = LowerLink.cmx;

b = Damping;

clear FootLink LowerLink HalfBody UpperLink


H = simplify(subs(H));
dH = simplify(subs(dH));

J_f = simplify(subs(J_f));
dJ_f = simplify(subs(dJ_f));

J_b = simplify(subs(J_b));
dJ_b = simplify(subs(dJ_b));

J_fb = simplify(subs(J_fb)); 
dJ_fb = simplify(subs(dJ_fb));

M = simplify(subs(M));
G = simplify(subs(G));
C = simplify(subs(C));
Q_t = simplify(subs(Q_t));
% Q_contact_f = simplify(subs(Q_t + J'*GRF + H_f'*F_c));
% Q_flight = simplify(subs(Q_t + H_f'*F_c));


%%
%[text] ## Manipulator Functions
M_calc_func = matlabFunction(M, "File","SimFunctions/GeneratedFuncs/M_calc_func", "Vars",[x y th1 th2 th3 th4 ph1 ph2 ph3 ph4]);
C_calc_func = matlabFunction(C, "File","SimFunctions/GeneratedFuncs/C_calc_func", "Vars",[x y th1 th2 th3 th4 ph1 ph2 ph3 ph4 dx dy dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4]);
G_calc_func = matlabFunction(G, "File","SimFunctions/GeneratedFuncs/G_calc_func", "Vars",[x y th1 th2 th3 th4 ph1 ph2 ph3 ph4]);
Q_t_calc_func = matlabFunction(Q_t, "File","SimFunctions/GeneratedFuncs/Q_t_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4 dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4 t1 t2 t3 t4]);

H_calc_func = matlabFunction(H, "File","SimFunctions/GeneratedFuncs/H_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4]);
dH_calc_func = matlabFunction(dH, "File","SimFunctions/GeneratedFuncs/dH_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4 dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4]);

J_f_calc_func = matlabFunction(J_f, "File","SimFunctions/GeneratedFuncs/J_f_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4]);
dJ_f_calc_func = matlabFunction(dJ_f, "File","SimFunctions/GeneratedFuncs/dJ_f_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4 dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4]);

J_b_calc_func = matlabFunction(J_b, "File","SimFunctions/GeneratedFuncs/J_b_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4]);
dJ_b_calc_func = matlabFunction(dJ_b, "File","SimFunctions/GeneratedFuncs/dJ_b_calc_func", "Vars",[th1 th2 th3 th4 ph1 ph2 ph3 ph4 dth1 dth2 dth3 dth4 dph1 dph2 dph3 dph4]);


%%
%[text] ## Foot kinematics function
% get the position and velocity of the foot to use when determining grf
kinematics_foot = [subs(p_f_foot), subs(v_f_foot)];
% export to a callable function
foot_Func = matlabFunction(kinematics_foot,"File","SimFunctions/GeneratedFuncs/foot_Func","Vars",[x y th1 th2 ph1 dx dy dth1 dth2 dph1]);
%%
%[text] ## Mapping of Angles
% finding ph1 and ph2 (and hence ph3 and ph4)
a = -l1*cos(th1) + l5 + l1*cos(th2); 
b = -l1*sin(th1) + l1*sin(th2);
B = atan2(b, a);
c = sqrt(a^2 + b^2);
th4 =  (acos((l2^2+c^2-l3^2)/(2*l2*c)) - B); %Cos rule of triangle L2 L3 c minus angle B

ph1 = 2*pi - th4 - th1;
dph1 = simplify(getVel(ph1, q, dq));
ph1_calc = matlabFunction(ph1,"File","SimFunctions/GeneratedFuncs/ph1_calc","Vars",[th1 th2]);
dph1_calc = matlabFunction(dph1,"File","SimFunctions/GeneratedFuncs/dph1_calc","Vars",[th1 th2 dth1 dth2]);

th5 = asin(l2*sin(th4+B)/l3) + B; % Sine rule of triangle L2 L3 c plus angle B
ph2 = pi - th2  + th5;
dph2 = simplify(getVel(ph2, q, dq));
ph2_calc = matlabFunction(ph2,"File","SimFunctions/GeneratedFuncs/ph2_calc","Vars",[th1 th2]);
dph2_calc = matlabFunction(dph2,"File","SimFunctions/GeneratedFuncs/dph2_calc","Vars",[th1 th2 dth1 dth2]);
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
%   data: {"layout":"onright","rightPanelPercent":26.8}
%---
%[output:32ef12f1]
%   data: {"dataType":"symbolic","outputData":{"name":"r_body","value":"\\left(\\begin{array}{c}\nx\\\\\n\\mathrm{cby}+y\n\\end{array}\\right)"}}
%---

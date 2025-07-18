%[text] ## Deriving the full dynamics of a basic monoped hopper
%[text] Using the general equation as follows:
%[text] $M(q)\\ddot{q} + C(q, \\dot{q})\\dot{q} + g(q)= \\tau - J^TF\_{ext}$
%[text] where $M(q)\n$is the mass matrix, $C(q, \\dot{q})\n$ is Coriolis force $G(q)$ is gravity, $f\_d(\\dot{q})$ is damping friction, $f\_c(q, \\tau)$ is torque dependent Coulomb friction, $\\tau$ is the torque applied at the motors, $J$ is the Jacobian matrix and $F\_{ext}$ is the external reaction force exerted on the foot in Cartesian space.
%[text] 
%[text] 
%[text] This code is adapted from Heather Wimberly's Kemba monoped scripts
%%
%[text] ### Declare variables
% Generalized coordinates
% C.O.M x, y; hip angle, knee angle
% angle
% Defined as per image (TODO: ADD PPT image)


syms x y th1 th2 'real'
syms dx dy dth1 dth2 'real'
syms ddx ddy ddth1 ddth2 'real'

q = [x;y;th1;th2];
dq = [dx;dy;dth1;dth2];
ddq = [ddx;ddy;ddth1;ddth2];

% System constants
syms grav 'real'

% mass: body, l1, l2
syms mb ml1 ml2 'real'
% Inertias: body, l1, l2
syms Ib Il1 Il2 'real'

%Lengths
syms l1 l2 'real'

% Center of mass (along the body)
syms cby cl1x cl2x 'real' %

%%
%[text] ## Position and velocity
% done in terms of global [x;y] - starting from the body's initial position


p_hip = [x;y]; % body centre position
p_knee= p_hip + trotz(th1)*[l1; 0];
p_foot = p_knee + trotz(th1 + th2)*[l2;0];


% Position of COMs:
r_body = p_hip + [0;cby] %[output:4e04e562]
r_upperLink = p_hip + trotz(th1)*[cl1x;0];
r_lowerLink = p_knee +trotz(th1 + th2)*[cl2x;0]; %

% velocities of bodies
v_body = getVel(r_body, q, dq);
v_upperLink = getVel(r_upperLink, q, dq);
v_lowerLink = getVel(r_lowerLink, q, dq);

v_foot = getVel(p_foot, q, dq);
% Rotational velocities
w_body = 0;
w_upperLink= dth1;
w_lowerLink = dth1 + dth2;

%%
%[text] ## Energy
% Kinetic energy
T_body = getT(mb,v_body, Ib, w_body);
T_upperLink = getT(ml1, v_upperLink, Il1, w_upperLink);
T_lowerLink = getT(ml2, v_lowerLink, Il2, w_lowerLink);

T = simplify(T_body + T_upperLink + T_lowerLink);

% Potential energy
V_body = grav*mb*r_body(2);
V_upperLink = grav*ml1*r_upperLink(2);
V_lowerLink = grav*ml2*r_lowerLink(2);

V = simplify(V_body + V_upperLink + V_lowerLink);

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
syms fx fy t1 t2 b 'real'

torque = [fx;fy;t1;t2];
B = diag([0, 0, b, b]);


Q_t = torque - B*dq;

% Ground contact forces
syms grf_n grf_t 'real'

GRF = [grf_t; grf_n];
J = simplify(jacobian(p_foot, q));
Q_GRF = J'*GRF;
dJ = sym(zeros(height(J), width(J)));
for i=1:height(J)
    for j=1:width(J)
        dJ(i,j) = jacobian(J(i,j), q)*dq;
    end
end
dJ = simplify(dJ);
Q = simplify(Q_t);

%%
%[text] ## Substitution
LegParams;
dJ = subs(dJ);
% Q_t = subs(Q_t);
J = simplify(subs(J));
M = simplify(subs(M));
G = simplify(subs(G));
C = simplify(subs(C));
Q = simplify(subs(Q));
%%
%[text] ## System Acceleration Function
% comment this section after running once as it takes a while to run
% get the inverse of M using a proxy to speed up computation
% N = 4;
% syms prox [N N]
% proxinv = inv(prox); 
% proxinv_M = subs(proxinv, prox, M);


% formula for acceleration of each coordinate
% accel_GRF = proxinv_M*(-G-C+Q);
% accel = proxinv_M*(-G-C+Q);
% export acceleration to callable function
% accel_Func = matlabFunction(accel,"File","SimFunctions/accel_Func","Vars",[x y th1 th2 dx dy dth1 dth2 fx fy t1 t2]);
% accel_GRF_Func = matlabFunction(accel_GRF,"File","SimFunctions/accel_GRF_Func","Vars",[x y th1 th2 dx dy dth1 dth2 t1 t2 grf_t grf_n]);
%%
%[text] ## Foot kinematics function
% get the position and velocity of the foot to use when determining grf
kinematics_foot = [subs(p_foot), subs(v_foot)];
% export to a callable function
foot_Func = matlabFunction(kinematics_foot,"File","SimFunctions/foot_Func","Vars",[x y th1 th2 dx dy dth1 dth2]);
%%
%[text] ## Hard Contact equations
% For impact:
% lambda = -(J*(proxinv_M*J'))\J*dq;
% dq_impact = simplify(dq + proxinv_M*J'*lambda);
% dq_impact_Func = matlabFunction(dq_impact,"File","SimFunctions/dq_impact_Func","Vars",[x y th1 th2 dx dy dth1 dth2]);
% 
% % for contact
% lambda = (J*proxinv_M*J')\(J*proxinv_M*(C + G - Q) - dJ*dq);
% syms GRFx GRFy
% lambda_calc_Func = matlabFunction(simplify(lambda(1)), simplify(lambda(2)),"File","SimFunctions/lambda_calc_Func","Vars",[x y th1 th2 dx dy dth1 dth2 fx fy t1 t2], 'Outputs',{'GRFx','GRFy'});
% 
% lambda = [GRFx;GRFy];
% ddq_contact = simplify(proxinv_M*(-C-G + J'*lambda + Q));
% ddq_contact_Func = matlabFunction(ddq_contact,"File","SimFunctions/ddq_contact_Func","Vars",[x y th1 th2 dx dy dth1 dth2 fx fy t1 t2 GRFx GRFy]);
%%
%[text] ## Manipulator Functions
dJ_calc_func = matlabFunction(dJ, "File","SimFunctions/dJ_calc_func", "Vars",[th1 th2 dth1 dth2]);
J_calc_func = matlabFunction(J, "File","SimFunctions/J_calc_func", "Vars",[th1 th2]);
M_calc_func = matlabFunction(M, "File","SimFunctions/M_calc_func", "Vars",[x y th1 th2]);
C_calc_func = matlabFunction(C, "File","SimFunctions/C_calc_func", "Vars",[x y th1 th2 dx dy dth1 dth2]);
G_calc_func = matlabFunction(G, "File","SimFunctions/G_calc_func", "Vars",[x y th1 th2]);
Q_calc_func = matlabFunction(Q, "File","SimFunctions/Q_calc_func", "Vars",[fx fy t1 t2 dth1 dth2]);
%%
% ff_torque_extra = M*ddq + C + G - Q_Fc; % + Jt*(-Kpx)
% ff_torque_func = matlabFunction(subs(ff_torque_extra), "File","SimFunctions/ff_torque_func", "Vars",[x y th1 th2 ph1 ph2 dx dy dth1 dth2 dph1 dph2 ddx ddy ddth1 ddth2 ddph1 ddph2 f_cx f_cy grf_n grf_t])
%%
%[text] ## Mapping to template
% x, y
legXY = subs(p_foot - p_hip);

%r
r = sqrt(legXY(1)^2 + legXY(2)^2);
ph = atan2(legXY(2), legXY(1));
dr = simplify(jacobian(r, q)*dq);
dph = simplify(jacobian(ph, q)*dq);

ForKinRTh = [r, ph];
matlabFunction(ForKinRTh(1),ForKinRTh(2),'File', 'SimFunctions/KinematicsRth',... 
    'Vars',[th1 th2],... 
    'Outputs',{'R','Th'},... 
    'Optimize',0); 

J = jacobian(ForKinRTh, [th1; th2]); 
matlabFunction(J(1,1),J(1,2),J(2,1),J(2,2),'File','SimFunctions/JacobianRth',...
    'Vars',[th1 th2],... 
    'Outputs',{'J11','J12','J21','J22'},...
    'Optimize',1);
ForKinXY = legXY;
matlabFunction(ForKinXY(1),ForKinXY(2),'File', 'SimFunctions/KinematicsXY',... 
    'Vars',[th1 th2],... 
    'Outputs',{'X','Y'},... 
    'Optimize',0); 

J = jacobian(ForKinXY, [th1; th2]); 
matlabFunction(J,'File','SimFunctions/JacobianXY',...
    'Vars',[th1 th2]);

r_calc = matlabFunction(subs(r),"File","SimFunctions/r_calc","Vars",[th1 th2]);
dr_calc = matlabFunction(subs(dr),"File","SimFunctions/dr_calc","Vars",[th1 th2 dth1 dth2]);
ph_calc = matlabFunction(subs(ph),"File","SimFunctions/ph_calc","Vars",[th1 th2]);
dph_calc = matlabFunction(subs(dph),"File","SimFunctions/dph_calc","Vars",[th1 th2 dth1 dth2]);
%%
%[text] ## Derivation of Raibert equations
% dx to dth conversion
syms th(t) x(t) r(t) dth(t) dx(t) dr(t)
eqn = sin(th) == x/r;
solve(eqn, x) %[output:5aa71050]
deqn = diff(eqn, t) %[output:098b551e]
deqn_2 = cos(th)*dth == dx/r - x*dr/r^2 %[output:2e159b92]
dth_sol = solve(deqn_2, dth) %[output:1db41799]
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
%   data: {"layout":"onright","rightPanelPercent":25.3}
%---
%[output:4e04e562]
%   data: {"dataType":"symbolic","outputData":{"name":"r_body","value":"\\left(\\begin{array}{c}\nx\\\\\n\\mathrm{cby}+y\n\\end{array}\\right)"}}
%---
%[output:5aa71050]
%   data: {"dataType":"symbolic","outputData":{"name":"ans","value":"\\sin \\left(\\mathrm{th}\\left(t\\right)\\right)\\,r\\left(t\\right)"}}
%---
%[output:098b551e]
%   data: {"dataType":"symbolic","outputData":{"name":"deqn(t)","value":"\\cos \\left(\\mathrm{th}\\left(t\\right)\\right)\\,\\frac{\\partial }{\\partial t}\\;\\mathrm{th}\\left(t\\right)=\\frac{\\frac{\\partial }{\\partial t}\\;x\\left(t\\right)}{r\\left(t\\right)}-\\frac{x\\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;r\\left(t\\right)}{{r\\left(t\\right)}^2 }"}}
%---
%[output:2e159b92]
%   data: {"dataType":"symbolic","outputData":{"name":"deqn_2(t)","value":"\\cos \\left(\\mathrm{th}\\left(t\\right)\\right)\\,\\mathrm{dth}\\left(t\\right)=\\frac{\\mathrm{dx}\\left(t\\right)}{r\\left(t\\right)}-\\frac{\\mathrm{dr}\\left(t\\right)\\,x\\left(t\\right)}{{r\\left(t\\right)}^2 }"}}
%---
%[output:1db41799]
%   data: {"dataType":"symbolic","outputData":{"name":"dth_sol","value":"\\frac{\\mathrm{dx}\\left(t\\right)\\,r\\left(t\\right)-\\mathrm{dr}\\left(t\\right)\\,x\\left(t\\right)}{\\cos \\left(\\mathrm{th}\\left(t\\right)\\right)\\,{r\\left(t\\right)}^2 }"}}
%---

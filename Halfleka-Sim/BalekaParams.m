
Damping = 0.0;


% Masses

HalfBody.m = 2.97031; %kg
UpperLink.m = 0.15381; %kg
LowerLink.m = 0.34645; %kg
FootLink.m = 0.49995; %kg

% Center of mass
HalfBody.cmy = 0.00517; %m, positive y
UpperLink.cmx = 0.04114; %m along the link (x)
LowerLink.cmx = 0.13220; %m along the link
FootLink.cmx = 0.19040; %m
FootLink.cmy = 0.00163; %m

% Lengths
UpperLink.l = 0.1745; %m
LowerLink.l = 0.300; % m
FootLink.l2 = 0.2955; %m
FootLink.l4 = 0.02625; %m
FootLink.th3 = 130*pi/180 +pi; %rad
HalfBody.l = 0.12; %m
FootLink.offset = 0.01625; % m rubber foot radius (orientation doesn't matter for this)

% Moments of inertia
HalfBody.I = 0.01701235507;% kg*m^2
UpperLink.I = 0.00060093368; % kg*m^2
LowerLink.I = 0.00465965863; % kg*m^2
FootLink.I = 0.00736996850; % kg*m^2
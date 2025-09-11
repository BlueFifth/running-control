Damping = 0.01;


% Masses
FullBody.m = 5.50815; %kg
HalfBody.m = 2.55985; %kg
UpperLink.m = 0.15381; %kg
LowerLink.m = 0.34645; %kg
FootLink.m = 0.49995; %kg

% Center of mass
FullBody.cmy = 6.81e-3; %m
HalfBody.cmy = 6.39e-3; %m, positive y
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
FullBody.l = 0.12; %m
FootLink.offset = 0.01625; % m rubber foot radius (orientation doesn't matter for this)

% Moments of inertia
FullBody.I = 0.03217458052; % kg*m^2
HalfBody.I = 0.01491024833;% kg*m^2
UpperLink.I = 0.00060093368; % kg*m^2
LowerLink.I = 0.00465965863; % kg*m^2
FootLink.I = 0.00736996850; % kg*m^2
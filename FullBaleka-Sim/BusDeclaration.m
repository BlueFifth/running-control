
% generalised coordinates
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'th1';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'th2';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'th3';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'th4';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'dx';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'dy';
elems(8).DataType = 'double'; 

elems(9) = Simulink.BusElement;
elems(9).Name = 'dth1';
elems(9).DataType = 'double';

elems(10) = Simulink.BusElement;
elems(10).Name = 'dth2';
elems(10).DataType = 'double'; 

elems(11) = Simulink.BusElement;
elems(11).Name = 'dth3';
elems(11).DataType = 'double';

elems(12) = Simulink.BusElement;
elems(12).Name = 'dth4';
elems(12).DataType = 'double';


Coordinate = Simulink.Bus;
Coordinate.Elements = elems;

clear elems;



% Constraint Forces
elems(1) = Simulink.BusElement;
elems(1).Name = 'f_fcx';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'f_fcy';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'f_bcx';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'f_bcy';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'grf_fx';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'grf_fy';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'grf_bx';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'grf_by';
elems(8).DataType = 'double';

ConstraintForces = Simulink.Bus;
ConstraintForces.Elements = elems;
clear elems

% Foot
elems(1) = Simulink.BusElement;
elems(1).Name = 'foot_fx';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'foot_fy';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'foot_fdx';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'foot_fdy';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'foot_bx';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'foot_by';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'foot_bdx';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'foot_bdy';
elems(8).DataType = 'double';

Foot = Simulink.Bus;
Foot.Elements = elems;
clear elems;

% controller
elems(1) = Simulink.BusElement;
elems(1).Name = 't1';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 't2';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'r_f';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'th_f';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'dr_f';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'dth_f';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'Fr_f';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Tth_f';
elems(8).DataType = 'double';

elems(9) = Simulink.BusElement;
elems(9).Name = 't3';
elems(9).DataType = 'double';

elems(10) = Simulink.BusElement;
elems(10).Name = 't4';
elems(10).DataType = 'double';

elems(11) = Simulink.BusElement;
elems(11).Name = 'r_b';
elems(11).DataType = 'double';

elems(12) = Simulink.BusElement;
elems(12).Name = 'th_b';
elems(12).DataType = 'double';

elems(13) = Simulink.BusElement;
elems(13).Name = 'dr_b';
elems(13).DataType = 'double';

elems(14) = Simulink.BusElement;
elems(14).Name = 'dth_b';
elems(14).DataType = 'double';

elems(15) = Simulink.BusElement;
elems(15).Name = 'Fr_b';
elems(15).DataType = 'double';

elems(16) = Simulink.BusElement;
elems(16).Name = 'Tth_b';
elems(16).DataType = 'double';

elems(17) = Simulink.BusElement;
elems(17).Name = 'state';
elems(17).DataType = 'Enum: States';

Cmd = Simulink.Bus;
Cmd.Elements = elems;

clear elems;

% accelerations
elems(1) = Simulink.BusElement;
elems(1).Name = 'ddx';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'ddy';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'ddth1';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'ddth2';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'ddth3';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ddth4';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'ddph1';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'ddph2';
elems(8).DataType = 'double';

elems(9) = Simulink.BusElement;
elems(9).Name = 'ddph3';
elems(9).DataType = 'double';

elems(10) = Simulink.BusElement;
elems(10).Name = 'ddph4';
elems(10).DataType = 'double';

Accel = Simulink.Bus;
Accel.Elements = elems;
clear elems;

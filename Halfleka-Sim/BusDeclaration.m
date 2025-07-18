% Busses for CPG output
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
elems(5).Name = 'dx';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'dy';
elems(6).DataType = 'double'; 

elems(7) = Simulink.BusElement;
elems(7).Name = 'dth1';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'dth2';
elems(8).DataType = 'double'; 

Coordinate = Simulink.Bus;
Coordinate.Elements = elems;

clear elems;



% forces
elems(1) = Simulink.BusElement;
elems(1).Name = 'grf_t';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'grf_n';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'foot_x';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'foot_y';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'dfoot_x';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'dfoot_y';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'damping_th1';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'damping_th2';
elems(8).DataType = 'double';

Forces = Simulink.Bus;
Forces.Elements = elems;

clear elems;

% controller
elems(1) = Simulink.BusElement;
elems(1).Name = 't1';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 't2';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'r';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'th';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'dr';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'dth';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 't1_extra';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 't2_extra';
elems(8).DataType = 'double';

elems(9) = Simulink.BusElement;
elems(9).Name = 'state';
elems(9).DataType = 'Enum: States';

Cmd = Simulink.Bus;
Cmd.Elements = elems;

clear elems;

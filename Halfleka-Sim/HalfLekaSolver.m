%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88];
dq0 = [0,0,0,0];
X0 = [q0, dq0];
Sim.time = 0.5;
tspan = [0 Sim.time];
options = odeset('RelTol',1e-10, 'AbsTol',1e-10, 'Events',@(t, X) collision_event(t, X),'MaxStep', 1e-5);
tic

Animate = 1; % 1 = drawnow

t_total = [];
x_total = [];
te_total = [];
count = 0;
% Using ode113
while tspan(1) <Sim.time %[output:group:0a16c001]

    % solve (with event detection)
    [t, x, te, xe, ie] = ode113(@(t, X) halfleka_dynamics(t, X), tspan, X0, options); %[output:209cd0e0]

    % Store results
    t_total = [t_total(1:end-1);t];
    x_total = [x_total(1:end-1, :);x];
    te_total = [te_total(1:end);te];
    if ~isempty(te) % Impact
        count = count + 1; % impact/break counts - for diagnosing hard contacts

        % calculate post-impact velocity
        dq_post =dq_impact_func(x(end, 1:4), x(end, 5:8));
        
        % update initial conditions to restart solver
        X0 = [x(end, 1:4), dq_post'];
        tspan = [te(end) Sim.time];
    else
        break
    end
end %[output:group:0a16c001]
toc %[output:67c211e0]

% Import important coords
BalekaParams

l1 = UpperLink.l;
l2 = FootLink.l2;
l3 = LowerLink.l;
l4 = FootLink.l4;
l5 = HalfBody.l;
foot_offset = FootLink.offset;
th3 = FootLink.th3;

clear FootLink LowerLink HalfBody UpperLink % declutter workspace

%%
%[text] ## Display generalized Coordinates
% display state of all variables
plot(t_total,x_total(:,1)); %[output:2c5f46d0]
hold on %[output:2c5f46d0]
plot(t_total,x_total(:,2)); %[output:2c5f46d0]
plot(t_total,x_total(:,3)); %[output:2c5f46d0]
plot(t_total,x_total(:,4)); %[output:2c5f46d0]
legend('x','y','th1','th2'); %[output:2c5f46d0]
title("Generalised Coordinates"); %[output:2c5f46d0]
xlabel('time (s)'); %[output:2c5f46d0]
hold off %[output:2c5f46d0]

% display derivative state of all variables
plot(t_total,x_total(:,5)); %[output:2091a621]
hold on %[output:2091a621]
plot(t_total,x_total(:,6)); %[output:2091a621]
plot(t_total,x_total(:,7)); %[output:2091a621]
plot(t_total,x_total(:,8)); %[output:2091a621]
legend('dx','dy','dth1','dth2'); %[output:2091a621]
title("Derivative of Generalised Coordinates"); %[output:2091a621]
xlabel('time (s)'); %[output:2091a621]
hold off %[output:2091a621]
%%
%[text] ## Forces
coords_total = zeros(8, length(t_total));
foot_total = zeros(4,length(t_total));
accel_total = zeros(6, length(t_total));

contact_total = zeros(1, length(t_total));

controller_total = zeros(9, length(t_total));

controller_total(9, :) = repmat(States.Compression,1, length(t_total));
constraint_total = zeros(4,length(t_total));


for i = 1:length(t_total)
    % States
    coords_total(:, i) = x_total(i, 1:8);
    
    % ph1 = ph1_calc(x_total(i,3),x_total(i,4));
    % dph1 = dph1_calc(x_total(i,3),x_total(i,4),x_total(i,7),x_total(i,8));
    % ph2 = ph2_calc(x_total(i,3),x_total(i,4));
    % dph2 = dph2_calc(x_total(i,3),x_total(i,4),x_total(i,7),x_total(i,8));
    % th1 = x_total(i, 3);
    % th2 = x_total(i, 4);
    % dth1 = x_total(i, 7);
    % dth2 = x_total(i, 8);
    % dx = x_total(i, 5);
    % dy = x_total(i, 6);
    % q = [x_total(i, 1); x_total(i, 2); x_total(i, 3); x_total(i, 4); ph1; ph2];
    % dq = [x_total(i, 5); x_total(i, 6); x_total(i, 7); x_total(i, 8); dph1; dph2];
    
    out = SolverFuncRaibert(t_total(i), x_total(i, 1:8));
      
    accel_total(:, i) = out.ddq;
    foot_total(:, i) = out.foot;
    controller_total(:, i) = out.controller;
    constraint_total(:, i) = out.lambda;
    contact_total(:, i) = out.contact;
end
%%
plot(t_total, contact_total) %[output:1cc47b25]


%[text] ## 
%%
%[text] ## Load into Simulink
time = t_total;

coords_sols.time = transpose(time);
coords_sols.signals.values = transpose(coords_total);

constraint_forces_sols.time = transpose(time);
constraint_forces_sols.signals.values = transpose(constraint_total);

cmd_sols.time = transpose(time);
cmd_sols.signals.values = transpose(controller_total);

foot_sols.time = transpose(time);
foot_sols.signals.values = transpose(foot_total);

accel_sols.time = transpose(time);
accel_sols.signals.values = transpose(accel_total);

sim("data_inspector_Baleka.slx");
%%
%[text] ## Animate (drawnow)
if Animate ==1 %[output:group:97dcda85]
    fps = 1000; % Desired frame rate
    dt = 1 / fps; % Time interval between frames
    t_interp = t_total(1):dt:t_total(end); % New time vector
    
    % Interpolate all states in x_total
    solsy_interp = interp1(t_total, x_total, t_interp, 'linear')';
    
    
    x_sim = solsy_interp(1,:);
    y_sim = solsy_interp(2,:);
    th1_sim = solsy_interp(3,:);
    th2_sim = solsy_interp(4,:);
    ph1_sim =ph1_calc(th1_sim, th2_sim);
    ph2_sim = ph2_calc(th1_sim, th2_sim);
    
    x_hip_left =x_sim - l5/2; % y same as body
    x_hip_right = x_sim + l5/2; % y same as body
    
    
    x_knee_left = x_hip_left + l1*cos(th1_sim);
    y_knee_left = y_sim + l1*sin(th1_sim);
    
    x_knee_right = x_hip_right + l1*cos(th2_sim);
    y_knee_right = y_sim + l1*sin(th2_sim);
    
    x_joint_left = x_knee_left + l2*cos(ph1_sim + th1_sim); % calculated from left leg segment
    y_joint_left = y_knee_left + l2*sin(ph1_sim + th1_sim); % calculated from left leg segment
    
    x_joint_right = x_knee_right + l3*cos(ph2_sim + th2_sim); 
    y_joint_right = y_knee_right + l3*sin(ph2_sim + th2_sim); 
    
    x_foot = x_joint_left + l4*cos(ph1_sim + th1_sim + th3);
    y_foot = y_joint_left + l4*sin(ph1_sim + th1_sim +th3) + foot_offset;
    fig = figure; %[output:5f7dd01a]
    
    ax = axes('Parent',fig); %[output:5f7dd01a]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:5f7dd01a]
    u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:5f7dd01a]
    u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:5f7dd01a]
    l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:5f7dd01a]
    l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:5f7dd01a]
    f = animatedline(ax,'Color','r','LineWidth',0.5); %[output:5f7dd01a]
    
    axes(ax); %[output:5f7dd01a]
    
    axis equal; %[output:5f7dd01a]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:5f7dd01a]
    
    hold on; %[output:5f7dd01a]
    tic
    drawnow
    
    for i = 1:length(solsy_interp)
    
        clearpoints(b);
        clearpoints(u_l);
        clearpoints(u_r);
        clearpoints(l_l);
        clearpoints(l_r);
        clearpoints(f);
    
        % axis equal;
        % axis([-1 11 -0.5 1]);
        % body
        addpoints(b,x_sim(i),y_sim(i)); %[output:5f7dd01a]
        % upper left
        addpoints(u_l,[x_hip_left(i), x_knee_left(i)],[y_sim(i), y_knee_left(i)]);
        % upper right
        addpoints(u_r,[x_hip_right(i), x_knee_right(i)],[y_sim(i), y_knee_right(i)]);
        % lower left
        addpoints(l_l,[x_knee_left(i), x_joint_left(i)],[y_knee_left(i), y_joint_left(i)]);
        % lower right
        addpoints(l_r,[x_knee_right(i), x_joint_right(i)],[y_knee_right(i), y_joint_right(i)]);
        % foot
        addpoints(f,[x_joint_left(i), x_foot(i)],[y_joint_left(i), y_foot(i)]);
    
        axis(ax,[(x_sim(i)-1) (x_sim(i)+1) -0.1 1]);
    
        drawnow
        pause(dt);
    
    end
    drawnow
    toc %[output:3719e395]
    hold off; %[output:5f7dd01a]
    clear fps b f l_l l_r u_l u_r ax fig l1 l2 l3 l4 l5
end %[output:group:97dcda85]

%%
%[text] ## Functions
function out = halfleka_dynamics(time, q)
    solv = SolverFuncRaibert(time, q);
    out = [q(5:8); solv.ddq(1:4)];
    persistent threshold;
        if (time == 0)
            threshold = 0;
        end
    
        if (time>threshold)
            if (threshold == 0)
                fprintf("Time = %.4f",time);
            else
                if (time<=10)
                    fprintf("\b\b\b\b\b\b%.4f",time);
                else
                    fprintf("\b\b\b\b\b\b\b%.4f",time);
                end
            end
            threshold = threshold + 0.001;
        end
end
function [value, isterminal, direction] = collision_event(t, q)
    persistent contact % contact flag
    persistent groundheight % log ground heigh for when contact is found
    if isempty(contact)
        contact = false; % start in the air
        groundheight = 0;
    end
    isterminal = 1;
    direction = -1;
    ph1 = ph1_calc(q(3), q(4));
    dph1 = dph1_calc(q(3),q(4), q(7), q(8));
    foot = foot_Func(q(1), q(2), q(3), q(4), ph1, q(5), q(6), q(7), q(8), dph1);
    if ~contact && foot(2)<=0 && foot(4) <0
        contact = true;
        groundheight = foot(2) +1e-5;
        value = foot(2);
        return;
    end
    if contact
        if (foot(2) + 1e-5)<groundheight
            groundheight = foot(2) +1e-5;
        end
        if foot(4)>-1e-2
            value = 0;
            return
        end
        if foot(4)>1e-3 && foot(2)>groundheight
            contact = false;
        end
    end
    value = 1;

end
%%
clear Damping dph2 X0 dph1 dq dq0 dq_post dx dy dt dth1 dth2 ph1 ph2 q q0 th1 th2 th3

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":27.4}
%---
%[output:209cd0e0]
%   data: {"dataType":"text","outputData":{"text":"Time = 0.4990","truncated":false}}
%---
%[output:67c211e0]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 103.423981 seconds.\n","truncated":false}}
%---
%[output:2c5f46d0]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUsAAADHCAYAAACQsnGJAAAAAXNSR0IArs4c6QAAH3NJREFUeF7tnX2MV9WZx49QseIIjVVwhhdDqkvcthqdCrYqarJmNyNhhSB0ZzeRrGCkE7IvmR02ZhYinWUz4+yaLGGLBbLQP6hOV+0Swpr1j7K0viAFa2y3pbJBeZnZQWtWoNig4Oa5M8+PM4f7cs7vd869z\/39vjcxOL\/73HPP+TznfO9z3u697JprrvlM4QABEAABEEglcBnEEjUEBEAABLIJQCyzGcECBEAABBTEsuBKMHfuXNXT06OampoqORkcHFSdnZ3qyJEjBefu0ttv27ZNzZo1S23cuFG99957Ud5PnTrlPb+rVq1S7e3tau\/evWr16tWxHJYsWaI6OjrUhAkTKud37NihNmzYkBs3YtHf368mTZqkuru7o\/uGYpJboXCjWAIQywIrBgvCoUOH1LJly6KcsHjS\/1Pj27dvX4E5lCuWcWLa29ur5s2blyqwvmGGFkuqDytXrlRPPvmkyIenb56S04NYFuQdFsUQUVnIIumR5cDAQLBbpUWWzO7EiROVh0ywjGQkbIqlz4dbWetIUb4IfV+IZWjCCenbdDP5Um6QLS0t0U96JMrR1P79+9Wtt94adUnNbjwJ3OzZs6Nr9XPcGM+dOxedo6EA6l5PnTo16gLzod8vqxuu34uu17vFZrdZP6eXkfLz1ltvqTvuuCM2SnRhZ95TLwvlL+08n\/vggw+ibjZH+3PmzKnweffddyPmSd1wKgMNFRB3siMfUvmIMz9s2IfMm4YeNm3aFHXv2ef6NUn+pOvT+BdU1evmthDLglzJDSRrjM2MXG644Yao8b3++uvRWB6nwyJgphsnbhyRsVhSI+bGywJhps\/5TBPL+fPnRyJCtrt27Rozlsf5pnFYGnKgfN55552V++r55mtJKOLGLG3ZcVn4nmakxkKWdZ6EiodEzDS4zGfOnIkds+R7cBomBxZ+G76UzzR\/6mmZ\/H1GvAU1mcJvC7EsyAVmgzejR47KhoeHI3HkBs12dJ4mgR5\/\/PFonI4bmx51vfDCC5dMNlBjmzZtWuZkBKfDeGwaMwuHGf3S32Z5dVHmKIqjM2rYadGjrViagsz5YJFubW0dI9jmefpbZ09\/m\/nKGrPMEmSexNOjS44izQm0KVOmpPpTj3jNCLqgal5Xt4VYFuROWzFgsdRnfCnLHMksWrQoUyz1mXa6NqkxUsNlESM7ijZJUHQxToss6Rq960h\/c7efRd3ETY36O9\/5ziUikMbHthseN76qCy2VjWf2zS4xPRzMB1Wc6NcqlmZkSpM5SasNWCyT\/ElDMUn8Ja6sKKjpVX1biGXV6Gq70IwQ9cqsN+i4Bqvf2YyysiJL\/dq4CQQzPZtufdwklT4WSF1pOnTR1fMRN0niY4KnDJHl2rVrxwh22sMoLrJMqoUm\/6TlV7XV4sa6GmJZoL\/jlg7p3V\/bsb+kbjg1ENfGpy+\/4W48RTI23fCkIYG4KC1NlLPGLOO6w3rUx+OcvsYseQiE7lHtmGXSuCiJJU2+6Yw4qudIkf7mdbdp\/kzjn+fa0wKbVNBbQyyD4s1OPG5ROl2VNFOsd22pAaZFlhxN6DOk3H2nccG4yNIcO6XZXppo4QmfrNnwuJldzoc582xO3nA+s2bDmarNonQfs+G6WNK99TRtZ8OTxNLsWtOwBIsnCZzOU39g8eoG3Z\/6A4MZpS3qz66dsNAJQCxRH0AABEDAggDE0gISTEAABEAAYok6AAIgAAIWBCCWFpBgAgIgAAKlEsvm5ubCPXb5uI8qeZjRMrHy\/zOnXTUmbzOmXTxHJ2YafxdekDrPwNETZ2NLeGz096MnfhudPzZ4Vn1yYXKd06iP4g0NDRVakNKIJQklbTm7\/fbbvQFj4fvcuFOK\/\/\/yyz5Sl487Fd3jc6PCSH\/rIpmVgaTG98mFkf3FOMITYB\/qd8ryIfuN\/PTphcmK\/v3kM\/p3svp01HcQ1vC+S7rDwYMHo80LRYmmOLGk5SN08CvLGByJJO0oyYLF0R5FchzN3XXHdRejwWkTIzGMazgcjVSij8GR6IN+599G\/h75nRoSH9S4bI\/bbrtNLV++PLMstukVaVfWslQehJeN9BSoZ0C9AXooPzT\/bvX2z\/ZEv3MPQe8ZkGAeO\/FbdX1zs3r2h++pGdOuUq+88X5k\/8r+D4p0x5h7l9U3cQC5LLRjDWKp7buN29dKYvnslnb1ny99v8JyZsvFri9V6riuLgvgK\/tHKnNUod8YqdB6Vyyp2xai5lODbGtrU1u3bg2RfK5p1lNZCFxSeahu0YOYRZVs7x59CN8159oxzLkuff+H743Wt2KEtJ58w8ESxHJ0Z0RXV1dUuU6fPh0bWe7+3q3qjZ\/+T6ViHh2N\/EaE72L0RyJIY1F5CmCuCoWbiSOg92I4GiUxTRJSenhT\/ZQYkYqDq1Q0\/EY9S4jl6Hv4aB80vUuRjqRuOL0FhsYucIBAmQiQmPJwEIlpkpDScM9PRntBJKSSuvVF8aYImf6DWI5uH1u4cKF64oknFO2VTRNLOrdly5a66MIWVflwXzkEzHH1P3nohmiclH6nyJP+5WEj6tbTORLRRuo5Pfroo9EYPx0NHVnSXuT169erF198MXpztM0ED0WWRQ3yymlmyEm9E2Ah\/eZDN0QTjCSkcd36eo9GeeyVBLOhxTLuZQjUCMxJHgljFvXeOFG+fAlUu244ijxbJqq75oys8rj7jmujGXn9oNl6On6y\/4MoEqWJKere66s68i1t+t2ygp85X\/uS+tdn\/lb9wfw1hQVKpVs6VOSTRVLlQl7KTSDEuuEyE4lbQ8krEFZ3\/H4UUdOSrTl\/+FxhE7cQyzLXMOS9tARs1w2XtoAOGec1lH+x6s\/Ul6Z\/GgkjLQukf6MVA\/vfV4fe+6JqX\/ZkY3fDbZmiG25LCnZlIID6fNFLzGLW1d+NNouwQNLEFq9VlcBLXGSZVNElwCpDI0Qey0EA9flSsfzHf3hM\/dvOt2MdKIEXxLIcbQu5rDMCEhq\/FKQ2LGxsQpcHYhmaMNIHgRgCEhq\/FMfYsLCxCV0eiGVowkgfBCzEctzk4l8\/mOaoCx+NfT2avh6aPrK3ePHiaIcNf1LYxek2Qmhj43LPamwhltVQwzUgUCMBs\/FPvGeFmnjPYzWmGu7ysz\/+rjr7482VG\/Bmkpdfflk98MADlU0l1eTARghtbKq5t8s1EEsXWrAFAU8EzMZPkeX4yS2eUvefzPmPBpUZXfLXQWmNZC3fJbcRQhsb\/6UemyLEMjRhpA8CFt3wMkIyv8tebRlshNDGptr7214HsbQlBTsQ8EhAQuOvpTj6Ox3o3awHDhxQ9J3zag4bFjY21dzb5RqIpQst2IKAJwISGn8tRXn++efV4cOHo+43RZgrVqxQmzdvxgRPLVB9XVv2yuWLA9KpDwKozxf9aMPCxiZ0zUBkGZow0geBOh2z9OVYGyG0sfGVn6R0IJahCSN9EIBYptYBGyG0sQld0SCWoQkjfRCAWEIs82wFEp4seZYX96pvAqjPGLMMVsNRuYKhRcIFEEB9hlgGq3aoXMHQIuECCJS9PtPecPoaKy8deuSRR9S6devUvn37nGnasLCxcb6x4wUYs3QEBnMQ8EHAbPzTrx7vI9lgaRw\/fX5M2vTyjNbW1uiT1b29vdEnrM3PV9tmxkYIbWxs71etHcSyWnK4DgRqIGA2\/r+6\/Sr1161NNaQY9tJ\/OnBGPX1w5CNodNC+8K6uLtXX16dWrlyJHTxh8bulLuHJ4pZjWINAMoG4yHJGk9zo8tiZ88qMLqkrfuTIEXXLLbdEollNF5wI2bRtG5vQ9Q2RZWjCSB8EYghIaPy1Ooa2OS5dulSdPn266i44xLJWL9Rp5QqABUmWlEA9iCW9TKO\/v1\/t2bOn6pdoQCwDVOB6qFwBsCDJkhKoh\/pMkWUts+DsOhsWNjahqwK64aEJI30QqMOeEs2Gt7e3q71799b04l9ElgGah4QnS4BiIckGJYD6fNHxNixsbEJXJUSWoQkjfRCow8jSp1NthNDGxmee4tKCWIYmjPRBAGKZWgdshNDGJnRFg1iGJoz0QaAOxZIWpa9Zs0Zt37492r2zYMEC1d3dHbvWMmvW3EYIbWxCVzSIZWjCSB8E6lAs9U9JpIklfwGyqalJ7dixI3aJkY0Q2tiErmgQy9CEkT4IWIjlzGkTRXM6euLsmPzRN3haWlrU4OCgevPNN9W9996rTp06Vfmts7Mzsl+7dq3avXt39I2enTt3Qizz8LKEJ0se5cQ9GoOAWZ+7Om5WqztuFlv43o2\/VH0bf1nJnxlZ0jIiihzfeOMN1dPTM0YYObqEWObkXohlTqBxm1wImPWZIssZLXKjy2ODZ5UeXSZ1w0+ePHnJrh6IZS5V6uJNIJY5A8ftghIoe32GWAatHrUlXvbKVVvpcXW9ESh7fYZYCq6RZa9cgtEiawUQKHt95q41TerwBA8tHUI3vIDKZN6y7JVLAEJkQRAB1Ge3ITYJvLB0SFADQlYah4CExi+Ftg0LG5vQ5YFYhiaM9EEghoCExi\/FMTYsbGxCl0eEWPLrnriwcSv9JcAK7Qyk3zgEUJ\/RDXeu7fqHj+gbHiSccftMUbmc0eICwQTKXp9t9obznnDa6UPHoUOHYj8\/YcPCxia0u0VElnohkxawSoAV2hlIv3EIlL0+2+wN1z+Ry+364MGDl7ws2IaFjU3o2iNOLHUnDAwMVMovAVZoZyD9xiFQ9vpsszecvvyoH\/Q1yOHhYYilj2rOYfvhw4cTgXZ0dCh6OuEAgTITMMVyypUzRBfn5MfHxuTPZW84Xah32\/UgiM5lPTiam5sV\/bdx40a1aNEiNTQ0VAgrMZElC2XSZzUZKFHasmWL2rp1ayHAcFMQ8EHAFIilN3WqpTf9jY+kg6Tx3DtPqefe6a+k7bKDp9b3WT766KNq+fLl0b0bXizTxjPYO1y56I0mFFkW9XQJUhORaMMRiIssrxMcXb7\/8TGlR5e2YpkWUZptO0kIKapsa2uLBLOhxTKt6623oKxQveFaGwpcagJlr882Ykmva+vq6lJ9fX2xb1C3FUubrnoelaHwbjhBp3HICRMmjCmvuday7JUrD2fiHuUhUPb6bLM3vLW1Vc2ePXuMU+I+nWvDwsYmtPcLF0vbAkqAZZtX2IFAFgHU54uEbFjY2GQxr\/U8xLJWgrgeBKogIKHxV5HtIJfYsLCxCZI5LVGIZWjCSB8EYghw46chqEafrLRZFgSxdGhGEmA5ZBemIJBKgASC3v9I9RqHila40IMj6ZDQ\/hFZoqaCQEEEeLF1QbcXdVuKrtMibIilg7skwHLILkxBAAQ8EpDQ\/hFZenQokgIBEAhDAGLpwFUCLIfswhQEQMAjAQntH5GlR4ciKRAAgTAEIJYOXCXAcsguTEEABDwSkND+EVl6dCiSAgEQCEMAYunAVQIsh+zCFARAwCMBCe0fkaVHhyIpEACBMAQglg5cJcByyC5MQQAEPBKQ0P4RWXp0KJICARAIQwBi6cBVAiyH7MIUBEDAIwEJ7R+RpUeHIikQAIEwBCCWDlwlwHLILkxBAAQ8EpDQ\/hFZenQokgIBEAhDAGLpwFUCLIfswhQEQMAjAQntH5GlR4ciKRAAgTAEIJYOXCXAcsguTEEABDwSkND+EVl6dCiSAgEQCEMAYunAlWBt6l2rOr7VoYb+d8jhynjT46fP15wGEgABEMiHAMTSgTPB+vfWEw5XVG+aJKTHzowVWNPumCHAx89cqGRCt+V0INjV+whXNhYBiKWDvwnWlu6Vqqenx9unQ6dfPT42B9ObxsX+PsOwN6+f0XQxvaS0k4qcJqa6CLMAsz2E16ESwbS0BCCWDq6TAMshu5eY6uIZJ6q6QLMom9fYCHCciJpiq9sguq3Fq7g2LwIS2j8mePLytuf7sHCy8PLfLLqm4JJdmtjGCSiLLEWzfP61oXOeS4LkQCCbAMQym1HFQgIsh+yKNtWFNk5k486bBTLFlYTVHCKAsIquBqXKnIT2j8iyVFWmuMyaAqpHsFniqgvra4MjkSlHqzTmiqGA4vxaljtDLB08JQGWQ3Yb2lTv7n+9eYJiYf16y4SIS9KQAIkmiycPAbw+9El0DaLUhq5SSkL7R2TZ2HWw0NLHRas01sq\/k9DGdf9NQf3BO7+LBBiCWqg7g94cYumAVwIsh+zC1BMBEk6KOFk472y+XLGgxkWoaV1+iKknpxSQjIT2j8iyAMfjln4J6BEqiSkd3OUnkSUBZRtdTH\/w699FQwTo6vv1R4jUIJYOVCXAcsguTAURcBVT7ua\/NvQJlkwJ8aOE9o\/IUkhlQDaKI6CPkVKkyd18c8xUX2tKQkoH\/YbufXjfQSwdGEuA5ZBdmNYRARJTGh+lLj4teXr49z5fmdHnLn7cWCl17yGkfiqChPaPyNKPL5FKgxJgIaV\/KSqlsdK4iScWTV5nCiF1qzAQSwdeEmA5ZBemIFCZVHr4ps9nRqQkpvraUkSkYyuQhPYvJrLctm2bmj17tjp37pzauHGjGhgYGENLAiy0fxDwRYDHSUlI6YiLSLlrP\/Drj0fGR0f36DeikEpo\/yLEsre3V914442qs7NTzZ8\/Xy1YsEB1d3erffv2VeqmBFi+GgrSAYE0ArqQ8hhp0mRTowiphPYvQiwpqhweHlarV69Wc+fOjd5ZuXPnTrVhwwaIJXQFBDQC+mRTVkRKS6D0MdIy78OHWCqlZs2apfr7+9WePXsiceS\/Dx8+HIknHwRr7VMb1bc6Ory9\/LesrfDCR7V\/VqOsZUe+kwmkzdrrV\/EefPOlJq7d+3GTmxMzM35yS+y5cV9Iuyb+3LjRtB58sE3tXfdQYe2\/8MjSjCTTxPLoHz2DtiKMwPkchfvC\/w0GKb2PMlz4KEzezAKzcLiAGD+5OZpsovWjLeeHVfP5k6rlwrBqOX9Sfe2Tt8ckxeOkg+Onqp9e\/lVFgnhgwlcjG\/o75JHkh6lXfqaam0eE9Ff9f9y4YukSWT6+7l9Uz7f9fVYipONDp532hA59b06fGmGeRzVCYZM\/X+UY94X4aMomD7Y2rg8MmwfB9R\/+LLo9C2jz+eHEhflkR0I6OG6KOn7mfGUGn7aOzrh6nHptcGSx\/vmEh0c1vSISyra2NrV8+XK1aNGixhVLAosxS9umAjsQyJ+A\/jITXk9q8zIT7tbTkijef0+5r2bsFGOWo37HbHj+DQB3BAGfBOL23+uCSvcyP2uS9tE980uplFbbgw+q+5\/e09iRJUeXWGfps\/oiLRCQRcD8AB+\/ISrpA3167j+9eopqvr5Z3bPlvyGWWW6VEIZn5RHnQQAEwhCQ0P4Lnw23RSsBlm1eYQcCIOCXgIT2D7H061OkBgIgEIAAxNIBqgRYDtmFKQiAgEcCEto\/IkuPDkVSIAACYQhALB24SoDlkF2YggAIeCQgof0jsvToUCQFAiAQhgDE0oGrBFgO2YUpCICARwIS2j8iS48ORVIgUM8Eplw5o1K860b\/f8rEi7\/x+SlXzrTGcPLjo+rnv3lVUTonzx5Tv\/jw1dhrIZbWSJWSAMshuzAFgVIQIIE7+fEx9eVrvhEJFh1fueauMaJIv+tCGVcwSkM\/SPhsj6988RsVU0rnF795Vf38w1fUj44\/V\/ldQvtHZGnrUdiBQAkJsMh9eVSQSAgpKnz\/42Pq\/ulLI6HUhZD+JqGj83xQ9MdiyCJI502BrAUP54HyRJEp5+1Hx5+NRHP6zddFn5tp+LcO2UCW8GSxySdsQCBvArogclRIgqhHbJQnFrfn3nkqih4peot+HxVHn+JXKwMqEwnm\/dO\/GYn5r87\/l5r3l7PUw3++AHvDs+BCLLMI4Xw9E9AFkf6foi8SRO46c3eaGFA3lg5dDJPGAsvAbOlNnRXR\/LvXFyaOa4YuC7rhoQkjfRBwIGBGiSyI\/DtHf9Q9JcEkQZQYGToU2cqUgqX+x76n\/vTJBxBZZhFDZJlFCOfLQiBpHJG6zTyGqI8d8phh2mxxWcpebT4ltP9SRZZPd21V39Y+K+F7kLlaR+I6EDAJJAmiObPMs7\/cbW6EKLGa2gKxdKBGsLqv\/4\/EK\/TB6ZEn8MjgdVQJR8dwIK4OwGGaScBWEHkdIY0lIkrMxBprALF04Eaw\/vnvn1Ed2qdw9YWx+vIHffA7aX2YvhRiRESPRrkJvUTCocgwLZAA1Rt9AkWfZY5bd6g\/kPUoscwTKwXiv+TWEEsHb9QKi0XTHDC3EVbOphm98u\/mmjSzWFlLMlwW8DogyzTVd19kGteBQdwOE3MnStzDlYWQEFCPRZ9YGfktftdJHSATU4Ra27+PgpRqzDKvRanmti59eQZD17d0cYOjc6YAZe188OFEpJFNIGmHSdriawzbZHPNywJi6UBaAiyH7DqZFiWoWRGvUyFgDAIBCUho\/4gsAzoYSYMACPghALF04CgBlkN2YQoCIOCRgIT2j8jSo0ORFAiAQBgCEEsHrhJgOWQXpiAAAh4JSGj\/iCw9OhRJgQAIhCEAsXTgKgGWQ3ZhCgIg4JGAhPaPyNKjQ5EUCIBAGAIQSweuEmA5ZBemIAACHglIaP+ILD06FEmBAAiEIQCxdOAqAZZDdmEKAiDgkYCE9o\/I0qNDkRQIgEAYAhBLB64SYDlkF6YgAAIeCUho\/4gsPToUSYEACIQhALF04CoBlkN2YQoCIOCRgIT2j8jSo0ORFAiAQBgCEEsHrhJgOWQXpiAAAh4JSGj\/iCw9OhRJgQAIhCEAsRzlumrVKtXe3l6hvGPHDrVhw4Yx1CXAClMNkCoIgEAWAQntv\/DIcu7cuaqrq0v19fWpffv2KRLOBQsWqO7u7uhvPiTAynIozoMACIQhIKH9Fy6WJloSz56eHrVz584x0aUEWL6qQXNzs2pra1O7d+9WQ0NDvpItJJ16KgsBrKfy1FNZJLR\/cWK5ZMkStWLFCrV582Y1MDBwSWSpfze8EHXwcFOqxPSlSpTFA0zPScA3noF6So79smjRosICDFFiOWvWLNXf368OHz6sVq9ePQYzwaKuOT1hcIAACDQegYMHD0YBRlFH7mKpT+acOXOmMjbJQnn69Gm1bNmyWB4kmPQfDhAAgcYjQENWRQ5b5S6WcS7mcUp6cpgRZeNVCZQYBEBAIoHCxTKt6y0RGPIEAiDQmAQKF0ua0KFxiAkTJozxQNxay8Z0EUoNAiAggUDhYikBAvIAAiAAAlkEIJZZhHAeBEAABJRSpRBLvat+6NChxNlyaR51zfe2bdvU8PCw2Ekum\/LwGHRLS0vkjsHBQdXZ2amOHDkiyj02ZaEMk09mz54d5X3v3r2l9o3ugKT1zBKcVI1v8vCPeLHUd\/Ts2rUrcR2mBCfreXDNNzdKqQ3Stjy9vb0RBlrVIHXyzrYstMyttbU1ejgn7SyTUO9sy8N5Zb9ce+210eYIffNH0eWxLQuXYc+ePZe8RyJUGcSLJT1lHnnkEbVu3bporzg1xhtvvFFktGI+uW3yra8vpeulRpbV+kGiv6oti9TI37U85JOZM2cqEktzp1woobFN17YsJKpr1qxR27dvz03sxYslPd3vu+++ijgmvWjD1hl52VWTb6mNkZhVUx7uxtK\/SRsN8vKHfp9qyiI5snQpD4vRSy+9FL2wRppY2pbFXEWTx3CPeLE0I5OyiGU1+ZYsltWUR6qvXMtC9vPmzRM7\/upSHqpjBw4ciHowce9gKOLhpd\/Ttix63Tp58mQ0PJe2+89HucSLpe2TxgcMn2lUk2\/JYulaHrJfvHixuDGxWqJkiUMKLuXRx2ClTvC41jNus3k8mMWLpe0Yhk+h85FWNfmWLJYu5SFRoReemO8k9cHVRxouZTG773HvWvWRp1rSsC2PPrPP9zt37pyoB5ptWUxepsjWwjPpWvFiaTs7FgJOLWlWk2\/JYmlbnjye8LX4ha61LQuJ\/tSpUyvjreQfaeOvLuXRuUmNLKvxTV6rLsSLJTnYdt1VrY3I9\/VJ+U4SRclimeYHPd9x0Useg++uvrP1jV4eieXgctuWR7eXOGZpW8\/ITvdNHuuvSyGWrg0B9iAAAiDgmwDE0jdRpAcCIFCXBCCWdelWFAoEQMA3AYilb6JIDwRAoC4JQCzr0q0oFAiAgG8CEEvfRJFeLIE8lxTxLpUNGzYkesPGBq4EAZ0AxBL1IQgBcy91XmKp71JJKxjlr6urS\/X19UUvaMEBAlkEIJZZhHC+KgJFvHiCFievX79evfjii1ZvotFfJ1dVIXFRQxGAWDaUu\/MprPkCYPqeEh28VXDlypXqiiuuiL67RC8Jpk8i09tvaJF0U1NT9Le+VZJfZEFppC0Mp4XZCxcuVE888UTlZcNpi8rz2CKXD3HcJQ8CEMs8KDfgPdK64SSWJKj04tn9+\/dHb4yZNGlSJJB09PT0KP4sstl9T9tyaG5PNMXQ3CFVxDsRG7Aq1E2RIZZ140pZBckSS8otv+NSF0Bzn68pcOaLFvRSm7Yklu3t7SrpS6FFDBXI8hJy40IAYulCC7bWBHyI5aZNm6Kok7\/nwzdPelNO3N56Fky+VhfOIj5NYA0QhuIIQCzFuaQ+MuRDLOk7Pi4vF8myNV8dh8iyPupaXqWAWOZFusHu40sszTHLtHdlmmOWcW\/d1j9RgjHLBquUNRYXYlkjQFyeTED\/YuXx48fHzIbbjlmSnT4bnvay2qzZcPNazIaj9roQgFi60IKtaAKu6yyzuu2iC4vM5U4AYpk7ctwwJAHs4AlJt7HThlg2tv\/rsvQ2+75tbOoSDgpVNYH\/Bxnjyp+XQw2oAAAAAElFTkSuQmCC","height":199,"width":331}}
%---
%[output:2091a621]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUsAAADHCAYAAACQsnGJAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ+QVtV5\/x9ZXf+AoKDQXVCkYhhrqhWCGE0IOmVsKEMiIUCorTSika6YaWYL+RmKf7KSLt1qm3UbEyEDTQYFqaRbapzJpFIaqgiso4lNEPxR\/u2WhRBhV8RNwM5zlufN2bP3z7n3vfe9573v984w7Pu+5557zuc553uf8\/+coUOHfki4QAAEQAAEAgmcA7FECQEBEACBcAIQy3BGCAECIAAClEuxbGxspMmTJ\/cz79q1a6m5uTm22SXeYuPxSsDq1atpzJgx1NLSQuvXr4+dxqg3Tpo0iRoaGmjQoEHU3d1NS5cupW3btvWLhtM3bty4wvc9PT0lT6tt3mbPnk11dXW0d+9emj9\/PqVlN2F34sQJqq+vV8\/zs63Orr29PTC8bT6jhDMZpMUkSprKLWyuxVIXNSkcW7ZsoSVLlsSyU5IFjCv07bffTvfff79KS1ZiGZYnEQROoy6kIp5pvDhiGUe7yRWx5JdfU1MTDR48uMDOj2exeQ67P22x5PgPHjxYlDMSloesf68YsfQquFnBNytzVung54aJZVYiXgyTUvEN8yyZ7c033+yEBx5m52J4pxl3MelK+t6KEUsRBr3wSqWqrq5WXHUvSUSCm0xXXXUVsUfKFzfvOdyECRNo5MiR\/TwGaZJxWPYqamtr1X3SbN23b1+h2cvfS9N34cKFqhn+7LPP0tSpU9U90rQzK78Iv8S9a9cu1dz0u\/QmtN58NrsrzHjCxMB8nv4cvampx8PPZ54m76i2ePrppz35chdGmGdpPstsFvvlQ3+58N9vvPEGXX311eTVDI\/ycta7Qjher2a6nw05vFdZfeGFF\/p0r7zzzjt0ww03FMq4LnCvvfaaCsv58LOPmUYptzfddBPNmzevUBSkDAXZM4x\/0iKXVHwVJ5YidocPH+7Xr6ULqRROvdmuFzA2ABcS+X3RokXqswgu3y9iymGlMLIATpw4sc+z9QLPfZYsxJJO7mPVPZTt27f3adqNHj1axfXqq696di+YnmGU5lgUD01\/jrwQDh06pERcKhq\/lDh\/8ruIjMnD9Mi8bBGFr57nTZs2KX7yMpo+fXofOwblw0+EvcTN9kVjhpOXrN50D7OhHx\/pA9df0FI+vcTSzz6SJmHmZS+9vHpxkrol5dePf1LClkY8FSuWo0aN6iNIYmARHa\/mp9\/bmAXw4Ycf7uNpsrFMD1AqVZhY8r0yQPHoo4\/2qdzmvfIM3ROVgiIVUUSLvze\/C2pCeYmllzf6rW99q8\/LgAc6gsTMFISZM2dGtkUUvl5iyV65OUjlJXB6Psx0BgmirViaL1nOl\/6deH1BNjTLqtezg16SumfJZdlLsPWyw4OBfIkXGfYC1uuW3iJweZDQS2wrTizlDSfemwlFCkCYWOoe34YNG2jGjBmkF2h523N8ppiEiSU3JaWStra20qxZswqeo9mEkfR7jWR7iZ1ZkXiASfcKdB5BFV6PW\/InlUjiMLsezC4K8Z5EhKLYIgpfszKLGOnPY49LRMMvH9OmTeszYyGIj20z3OtlJenjVsvOnTv7tULM5\/KLWp9JEfSSC\/Is\/ezT2dlZ6PLwSpOfWPrZ049\/MTNV0vAkzTgrRizNwmt6CSYYG7GUQslCNXTo0EIT3CzMw4cPj9QMZ7GUAnXs2DE1rUemFEVpGhfrWTITv0EKL7H0m0IT1tSMaouofIO8ZxFdtuH3v\/99uuuuuzz7IIWF\/mIJ8x5tBnjKwbOUrh6ZihXWJxxlwEfn7zdtrRRCaPOMihFL\/W3NU4fCDG4jlnoz22tAgw3ABUDEQMKIeJqeqO4d6B3q+sCLKfpmQQ4T\/bAmk3m\/11QXr7R59fWJeJovC7OZZ+bBTKNfM9OPb1Cfml+TU7oxdC\/Npm+VxdNv3qTN1CF+rt6f7dUEtumzNOfoetmDX7pxPEuxz9GjR9WgIzPieaNSLk3BD6pbYfz95qraiFnaYXItliY8c06g2aTVB3NsxFL3Nsz5m3pTg70WFg6z054LnDRVzSYexy1vXTPdfn2hNpOizX4iWy9AH40VrubouR5G7xYI8yx5EnxUWwTxDRNfv5FdmYzvlw\/Ot\/7coNFwveyZ7ExxTWo0XF\/QoJcRtoXtaLhfn6XeV82tHb5OnTqlxFNehizGZr+8zDTR60cY\/7RFL278mYglFx4ejdYnh0uBKrdO37jgcR8IgEB5ESi5WHpNc+C31tixY9Vbiqdy8GCJ6\/0X5WVmpBYEQKBYAiUTS2kWdHV1qTTrnqXuaYqLzqPAro+OFQsf94MACJQPgZKJpdmHI2IpIrp582YljvJ5z549sddwlw9+pBQEQKBcCGQulqYnGSSWNTU15cIV6QQBEEiYQEdHR8IxRosuc7G09SxZKLkfc\/z48dFyiNAgAAK5INDW1qamWWUlmpmLJVvRps+SRZKnRmQJK6kSd+ONN9KCBQuQl6SAJhgPbJMgzASjErvwnOWKFkub0XARyyxhJWV79pJ5XuWqVauSijKzePKUF4aYp\/zkKS8u1H8nPEvxLvVJ2uZu4S7AykyR8GAQqHACLtT\/TMQyjt1dgBUn3bgHBECgeAIu1H+IZfF2RAwgAAIpE4BYRgDsAqwIyUVQEACBBAm4UP\/hWSZoUEQFAn4EMEc4uGyEjXBDLCPULRdgRUgugoJAgQDmCIcXhrA5lFz\/\/\/m70+mhR9bQsz\/YFx5hCiHgWaYAFVGCgE4gT3OE07CszRzKWydeRq1rJtOMu7fQ1u1H00hGaJwQy1BECAACxRFAqyiYnw2fp5ZPoM9M\/wTddMf6ypqUHqfo2QCNEy\/uAYG0CaDsFieWV468iF7\/0R\/Rgffm0qdnfBliGVZgUeDCCOF3VwnkqeyaezkkwTyMzxc+O5qebLid9nbdp45oCRsMSiJNXnGgGZ4WWcQLAmcJhIlBOYEqtViKV7lr3zA655IvQixtCkueCpxNfhEmPwTMsjtgiNtbDZ453n8rNDnhQM7feemll4iPk+Zr\/vz56mwiPrJZPwfI1oJBdVsGdh75h26aN\/9RiKUNVIilDSWEcZGAWXYv+uS9dNEn73MxqSpNJ\/\/zO3TyP58ppI+FUI56kcPgNmzYQJs2baLly5fTj370I5o6dSpt3LiRzD0dbDIZVLdbV09WUTzyzW4lxGiGWxCFWFpAQhAnCXh5llVDap1MKyfq9PF20r1L9irFgzSb4bJ5N8+T1A8gjJI5v7otTfAHHtpJu\/YPg1jaQoVY2pJCONcIlHvZDRJL84zwOOz9+PDAzuK6a+nGqS+pTb\/hWVrSdQGWZVIRDAT6ECj3shvWDOfmN+\/PunPnzliHDPrxYaH8xMTLacb8LRDLKHWq3AtclLwibL4I5KHsmgM8r7zyCvHKGzlYkD3Me++9l5555pnI\/ZZefPQmOC9vdIEhpg7lq14iNw4ScKGiO4ilkCQvPjIKPuz3XlDhXGAIsXS5FCFtuSDgQkV3GaQXHxkF5yY4xDKi9VDgIgJDcGcIoOwGm8LkI03wxpaf04qWn0Mso5ZkFLioxBDeFQIou9HEkpvgTy3\/GD3w0I7CDkMuMEQz3JUahXTkloALFT0puKVY7qhPGZJ0u8DQCbGUuVrV1dWKTXd3Ny1dupS2bdsW2AmcVAFAPCCQJgEXKnpS+SuFWHJ\/5f7294gno0MsDcvxPK4pU6ZQfX097d2719OueSpwSRVcxFMeBPJQdv3Whh8+fFit3GGH5+6776bHHnusj5NjYyGdz3kDjqvt2MxNfl1g6IRn2djYSCNGjFAL8v0uF2DZGB5hQMAkYJbdURdXOQ3pYNfpPunzm5TOgXgzDa63NnXYpm7\/7qhfqx3RZcoQPEuDmry15Ou1a9f2WwkgBa6uro54HSouECgXAqZY\/uX4gfSVCYOcTf4TO7vpybb3CunzW+742muv0eLFi2nFihW0cOHCRFbw\/OnMS4j7LHmJo1x8hhH\/q\/jljtIHIisB\/LZ6kgLHAFeuXEmrVq1ytrAhYSCgE\/DyLK8Y5K53eaD7NOneZdDacP6Nu86uv\/56JZr6OINtKdD5fPsb19BPth8pTBniOO655x5asGCBig67DmlUTfE03fCGhgblWWa1W7JtAUA4EHCpCVmMNfya4c3Nzaqvcs6cOdTV1RXYjRb0fBHLLy+6i374Tzf0669kr5LXnrNgQiw9xHLz5s19muLosyymuOPeLAnkoex6DfCwWCYxOi58\/u4b99G3\/+aafv2VbDsXGGY+wMP74S1btozWrFmjFuDrbzFMHcqyiuPZSRFwoaInlRcznmJGwU3P+9Uff42uHvUbtcuQebnAMHOxZCj6PMuenh7PreldgJVWgUO8+SaQ17LLjs28efNoy5YtsTf+1b3Gc088Sdt2vtNnfqVLXRlOiKVNVclrgbPJO8KUNwGU3WD7CZ8xF3+HPvfn\/1pY4qjf5QJDiGV510OkvgwIuFDRXcYEsUzYOihwCQNFdCUjgLJr51lyM3ziHes9A7vAEJ5lyaoMHlSpBFyo6Emx10e\/eVK6DM7yCjw5AdJrrmXQqLnwaf\/F4\/S5P98EsSzWWHkqcMWywP3lRSBPZVcXPV4XLkdJBImlnAA5aNAgClqdx1OHNrT+FGJZbPHOU4ErlgXuLy8CZtnlzW1dvvYfOtkveX5n8NTW1lJ7ezu9\/vrr9KlPfYpOnDhB8h1vjMPXww8\/TC+++KIS1tbWVt+lzDwp\/bUd70Asiy0cEMtiCeL+UhEYfuEVdN2wW+ijQ2+l20bNoerR79OwP+sorD7hUwuX1F1bquREfo6+Qznf7LeCx\/QseRoRe47cPOeVdrowincZJJZBq3NcqP\/os4xclHADCPQnIAJ528i59NFht9DPfvlfKtBbx7ZS9ehT9Bdf\/5OCWLJneUWtu97lgfaTpHuXfmvD\/ZrhnZ2d1NTURPoqPIhlCWuNC2+WEmYXjyoDAiyQ7DleN\/RWJZCd7x+gt375X\/TvB5+jt471iiVf5V52IZa9doRnWQaVEkl0i8B1Q2+h20fNVULJHuSR9w\/Qut1\/q8TS6yp3sbRthstoODzLjMtruRe4jPHh8UUSMJvZLIwskC8fXBcacx7KrtdGGtI3yYM6MsDDx8FALEOLRLoB8lDg0iWE2NMgICI555q\/Iv6bxdFsZoc9F2U3mJANH5swYXYo9vdcNsO5A33uZ0fTJyZeTlcY0zQOHDqpNhfl\/5\/9wb5i+eH+HBPgZjaLJF9efZG2WXehotumNYtwNnxswqSd9lyJJW9Hz\/9EILduP6L46SN7LKC33nRZn+9ZNLe+dsRzAX\/aBkD87hFgD\/KB67+pBm3Ykwzqj7RJvQsV3SadWYWx4WMTJu3050IsxZNkoWSBfHbjvlDhk+kbt950eWHOG4sqC+dzP9jXR2DTNgLid4OANLkXXf9NNXDDIqmPasdNpQsVPW7aS3GfDR+bMGmntezFkkXvqcc\/przJBx7aESqSfkBFcHmysHiiLJwrWn6etg0Qf8YEdJHkgZuXDz5H63Y3JZYqFyp6UpmJujZcwvOqHr527drV7\/gJGz42YZLKo188ZS2WLHB8xjCLm34aXLHQOF5eZcGeKl+8ogHN9GKpunm\/NLmHX8SDN8mKpOTYhYqeFP2oa8P1I3JlYjqfocVnjUfh4wLDshbL1tWTlUeZpFDqhUpv3vPf7GmyMMPbTKrqZRePTCjnARz2Jpe9eqfvPMliU+lCRS82D3HXhvPJj\/rF8fDKH4hlsRYJuF8vcOcNOF5oeqcllGZS9GY6\/yb9mxDOFI2eUtQ8qXzRDd9UsdvOlSwmKaZYslC7fJmT620npQetDef8mudtwbNMqRToBU4OYv\/M\/C2ZDMRwE53FU5rpvaPpRzEVKSXbJxWteJO3jZpLnScP0FNvPpiaN6mn2RTLOdfUF6YkJZW3JOPhF4jeZ5vEckeb\/SyxkYalFcXNDzuwTI7LNHdGsXxMosFYMG+deLkSTZmOxKJpHhKf6EMRWSwCujeZVt+kX8K8PMvLHfYuefmm7l0WK5Z+HmUUz3LiNaPohSm\/ptmbfkWvdPTEKgPF3uREnyV3Ao8dO5Z4\/7vp06d77rjMBe4731pOH777Xero+F\/P4zKLhVHM\/fpEeH0eJ+ZwFkM1mXvFk+PpQKXyJoM8y2RyVbpYbJvhXmvDeUnk4sWLacWKFeS1gzrnQn+ZHD67lWbVkFoacEkNVQ2poVEXV9GawRtowCW19Nl\/OUIdHR2ly7z2JCfEUu\/09dvKiYF+75mF1HPsn4uaIlQKyiKc\/Cx930LxOrF6qBRWILU88bGbN6qHldqbzJNYcl7irg2fMGECjRs3ro\/BzaNzRSzv+XEVdZ48pxD29PFeUVz57v+j8cMH0Ms3PU5f+8qXKlcszb4M+bxnz54+I2Y3fexq4tHvDa1vep4rXJrqF+8pfuKpz+fkmHl6krmXYLwnVvZdpRzptiGdh9Fwm3zGDSN8vvAItxo76My7HXT6eDudOd5Bn\/\/IhfSV8QPpe11X058u\/fvCnqBxn1XMfZl7lqYn6SeW3C\/Y\/NWP0veb36RX3z6P2quGq3y3DxhR+FveRHGAsIGSutjQQdeVIweqAaJbJ\/Yuu+RVRB\/\/yK\/73PL2zy5Un3f\/9ALa336Sdv+09\/OpXw2l4RdeWQhr9n3xfMFiLx78CLu4X8v26nx\/v23QRMLxAA5fWXqTkpEBQ2roD66uoZaWlkwreiJgU4rE72Xy8ZpqeuJTg+nZjovpJ12XZs4wc7G09Sx5UwNehjZx5PW+JmuvGnFWQIfTjvN+X\/3dUTWc+Pv2Ab3\/u3QN6zmXhvVU0WU959JHui+gkQOP02U9VTTwsHc6L7j0l4XkXzC092\/9O\/nx\/LO\/xc3rB8eGhd566lfhYfRITgXEeei9IaHPsw3ADEdMfIWeONpDm\/b+tknH9we9TM+EvOAGDOldgWJe3KemX9yvxpf+\/e8P+5CW33Ka6urqMmtC2vLLIlxNTf+XCfdTPv\/Hl6rBnLfGfp4WLFigkhY0Yp522jMXS86gTZ\/lHbd+hr405Du04cg31OYGVd2dis0Vg6rU\/wx31KAB6u+P11b3\/l\/T+79+Hew6rT6yEV7p6PXm+Dv2AJIaZePOafNij4+9QG4i8gYN11aNIRbLX1b\/Rv3Po4\/s0bHH9rNjW9Xt4uG9PeiDfvGNHHS88N3Ige9S1dlKyl+yYES9bATrULe\/qDE\/vmziiZo22\/AiaM8t+IXy1Bf\/cDyte\/m3HrspbHq8InJBzzrzbv8WgynAIrr69yMuJFryZ59WAxm4vAnwqh5+mfDFdZk9Sq7bH3\/uKLGYTps2TQlmxYul7Wj443esp\/ffuJhmvhjNQ2T4DN4UVPnOT0wPdJ2mg91nlJiKwNoWdv3QKhZJ\/VwWjkNEkQUxic0abNNVKeHkUDAXppgxc67w\/K8SLz5v6At3jlbdTjK4Oerd+cp5eOrNLysk3Fcpo9x\/OX4gzea+yv84UXBgXOj3dcKzZFi28yw7vv67yrNsfvPBxModiygLIncm696piCn\/xmH44r8PdJ\/u\/f+siD6\/+5QS43feHdHnTBYOL+eysLfIeyL6HT2QWGYQUYEA93M\/tXyCWriQ1QKGSjaHzEMWG+gb00i3mun4cB1kr\/KJnd30ZNt7BXwQywglSWA1P7iOJg+6i5rfeLBkHpkIJTfrdTEdP\/wq6vlNLR09+Rnq+mCiyk31uYfo\/Kp2uvj8HbR8+\/cKOWRxTaqZHwFbxQflCvsvqycrj6ax5b9j70pV8SAjADB38PLavYsXCXz95o10\/8sfKzgQXM9emXsZPf\/2+8qr1C+IZQQD6LAWXvmPxH2ADDqLS9\/SS\/cedx9\/nq4d1qaSpPebmp4p\/y7C+fzbpwpZEI81izzl\/Zns3bCnyRX3gYd25j27meRP33iGExC2xeEL0w6rFiK3FKWfku\/jVTrmBbGMYFId1ul3z6Wnb9uhNmhdtu3OCLHED6qvK+a\/Zd9DTkNYn6N4ptxUv7nmPJWIK3hA6uIqNQiliyn\/Zg5CsTf76tnBKHin8W3IfWatayarZnkxe5\/GT0E+77TxJL1yznVYpnd59VPCs4xZXsw3i7jxaQomiyJ3QvM2XjJAwwMzUQ+sssmyPgjF4Vkg2Tv1GoQSQRVPlEf1Iag2lEnNb+XNonlJqiuDP3Ypdy9UXJGUnPBUQL52HKmn9dMv7ddPCbGMaXMvNzwNwdRHsfVzoXmAxubY05jZC71NBqHYEzW9U336lEQk3qk+GMXeKX\/WvdfQB+c0gIyWY6u96Ab2Esk4R7GwWP7R6E\/S0IFTVbn0an5L6tAMj2AnP1gimNwsjjvoIx7k7aPmqtFsvqKcCx0hG6kGDWvumw+XkX3+\/pX2HjVNyvRaU01wxpGblR5N82CDJCWS8hSua\/U3flWJ5ef\/7VeF7ievVEAsI1SWIFj6aXw2Iiebr1437Ba6beRc1cQWgeQ+FPYg8zrFR5r7nF\/xUG0m8fObnwWVL+k\/zcuAlN40581OHvjajkz2SY1QHUoaNGmRlMSzV3nfdRuoYcckauv8n8A8QSwjmNwGluxZKGIocxzlMbKOWsSR+zv5evnQc5gDqdnCbxI\/BwlbFSUT+cVDLacBKR4tl42duT8zTtMyQpF2PmhaIskZl2lC2w+9WRgRDwJiU\/\/TBurMpPSwjEaBJf2OvDqGB2fkks0fuP8RK2fCiIf\/ruadRlhmag5IyWR+1wRVDqtjsWBPs9LmZ+oH9hXbp6vPUe592Z5XmAXCdv\/Fke\/SW8e2hp6mGaX+h5fceCFyKZbxUOCuJAnoA1IiqDJdKmiEX4RTPFT+zOGzEFRdNIsVjSTZphGXueu\/HMzHcyXDLr2vvHc6XK8gmnY2lw0\/0fYe3TnmSRV92Io8iGWYFbTfXYAVIbkIakEgaEAqbN0+Ry9TptL2UL08rbw00c28bd1+hJ7duK\/fSidbQZSX2m+F8ddq4MbvZccj4jx2ELbAxIX6D8\/SolIjSHYEpJL2NuF6l5sGeaheU6b43iQEVfe++AhmvtjzKjfhNI9AYS\/yJz8+qETywxMfFLpWdM6cV7GF6e3zS0vfMyFKafFbI27GAbGMQNUFWBGSi6AlJGD2i4UJKidNF1UZ5dd3mAob6fc7Gpl3u9+6\/WgJcx\/+KE4rX3M\/O5o+MfFyNSGfRfH0weP0m7c6af\/Pj6nfzc1ihJO++1bS3SFea8S9cuRC\/YdnGV7WECIHBMxmpGyI4uc96VnWhVUXEAkzYMgFat35VZNq6JzB56uvpY8zCfHUvWuO22sRgrwg1O8jB6o0jL72Uhp97VA69\/d6TxUQgfyfbR3qCBPpV9bFMOwlkUZR0NeI+8UPsYxA3gVYEZKLoGVMIEhYdQ\/Mq1+VxZLFacAVQ6hqVO9mySxSfLEnd+bAcTpz4gP68MSpwvfFoOLnnTP4Ahow+Hz1TP4sz2XB5t2W+GhmFsd9vwie+F1MOoq5l9eI8\/aFQYM8LtR\/eJbFWBn3goBGQPcAxftj8WLvjvs4pQnMIiZNYxHSM5p4ssDpF3t+Ep6\/5zj5HCf9ezn8jgdoeLrT\/kPvOdcd4FdYZI04xDKh6uTCmyWhrCCaCifAwse7h7Pg9Tab+e+L6Mra3372QiQiur\/9PdXM588siny51k8axcRyrnvQCQgu1H94llGsirAgAAKJE5ARcX0jYPMhEMsI2F2AFSG5CAoCIGBJgFfccb8lxNISWFgwiGUYIfwOAuVLIGxE3IX6j2Z4+ZYvpBwEckMgbEQcYnnW1LNnz1ZnBldX957z3d3dTUuXLqVt27YVCoMLsHJTMpEREHCMAI+I865gfsfEuFD\/nfAsFy1aRFOmTKH6+nrau3evpxldgOVY+UJyQCA3BHiQh3cI81sj7kL9d0IsGxsbacSIETR\/\/nxf47sAKzclExkBAccIhI2Iu1D\/nRDL1atX07hx4wrmW7t2LTU3N\/cxp8Di5npbW+9xs7hAAATyQ8BvkKempob4X0tLC82cOZM6OjoyyXTmYjlmzBhqamqiPXv20JIlS4ib5LNmzVJg1q9f36\/Pkr9YuXIlrVq1KhNgeCgIgEA6BPSjcfUn3HPPPbRgwQL1VUWJJYvhvHnzVMa9BnJM8RRo4lk2NDQozzKrt0s6xQSxggAI+C17ZK9y2rRpSjArSizDioSI5ebNm\/s0xV3oswhLO34HARCITyBoI2AX6n\/mzfBJkybRsmXLaM2aNarZzZ7njBkzMHUofpnDnSBQlgSCRsQhlmdNqs+z7Onp6ddfycFcgFWWJRCJBoEyIRC0EbAL9T9zz9LWji7Ask0rwoEACMQj4Dci7kL9h1jGsynuAgEQSIGA37JHiGUE2C7AipBcBAUBEIhBwG9E3IX6D88yhkFxCwiAQDoE\/E57hFhG4O0CrAjJRVAQAIEYBPyWPbpQ\/+FZxjAobgEBEEiPgNcgD8QyAm8XYEVILoKCAAjEJMCDPOt2\/y29fHBdIQYX6j88y5gGxW0gAALpEOBBns7399O63U0QyziIXXizxEk37gEBEIhGgMWS+y710x5dqP\/wLKPZEaFBAARSJuA1yAOxjADdBVgRkougIAACMQnIaY\/Nbz5Y6Ld0of7Ds4xpUNwGAiCQHgFzRBxiGYG1C7AiJBdBQQAEiiBgbgTsQv2HZ1mEQXErCIBAOgTMvS0hlhE4uwArQnIRFARAoAgC5rJHF+o\/PMsiDIpbQQAE0iEggzx8NG7n+wec2M8WYpmOrRErCIBAkQT0QR54lhFgugArQnIRFARAoEgC+t6WLtR\/eJZFGhS3gwAIpENAH+SBWEZg7AKsCMlFUBAAgSIJ6IM8LtT\/TDzL1atX0+HDh2nJkiUFnPzduHHjCAeWFVnCcDsI5ISAPsgz6trL1UGGFXVuuIjili1bCmLZ2NhIY8eOpfr6epo+fTqOws1JYUc2QKBYAjLIc3z47soRyzFjxlBTUxN1dXUpfrpnqXuafI5IW08dAAAImklEQVR4Q0MDtba2UnNzc4G1C254sYbH\/SAAAtEIyCDP1nNXV45Y6oh0cRQR3bx5sxJH+bxnz54+zXSIZbRChtAgkAcCsl1bw\/9+GmJpepJhYllXV0dtbW15KAfIAwiAQAgBGeT59vH76JGmr+avz1IEr7a2VqHQ+yf5czGeJd+\/cuVKWrVqFQoaCIBABRDgfstLZhyhC2\/oyp9YhtnPHA2P0mfJ\/ZnsWXZ0dIQ9Br+DAAjkgAD3W150QxddM+cCiCVGw3NQopEFEEiJAPdb3jj+Rrr2S+dCLKVpjnmWKZU2RAsCZUxgzjX1NOeav6Kav\/7\/lSeWceyG0fA41HAPCJQ\/ARnkGb5oP33+izMy64LLZAVPHPNBLONQwz0gkA8CMsjzJ49OhViGmRRiGUYIv4NAfgnwIM\/omy+hL67+JMQyzMwQyzBC+B0E8kuA+y3nz7ofYmljYoilDSWEAYF8EvjD6z5HfzH6H6nu9T+AZxlmYohlGCH8DgL5JXDd0Fvo6zdvJDlmIoucYoAnC+p4JgiAQCQC7Cwt\/Z0fUvObD9LLB9dFujepwBDLpEgiHhAAgdQIsFg+fsd6WvG9r0EswyijGR5GCL+DQH4JuFD\/4Vnmt3whZyCQGwIQywimdAFWhOQiKAiAQIIEXKj\/8CwTNCiiAgEQSIcAxDICVxdgRUgugoIACCRIwIX6D88yQYMiKhAAgXQIQCwjcHUBVoTkIigIgECCBFyo\/\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\/tCaM0Zp6AAAAAElFTkSuQmCC","height":199,"width":331}}
%---
%[output:1cc47b25]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUsAAADHCAYAAACQsnGJAAAAAXNSR0IArs4c6QAAFTdJREFUeF7tnU9oHPcVx19aEBRSQk2IWB8CAhEdQ2KIbib0uAiVLGKhzsECrXpZ9iYkMMuamsWgsDexkFYSSBcfRMBlY0IuDUKBwkKtQ081CHwo8bLxKaik7ULS8pt0zO5af+aN5v3Z8VdgEsdv3p\/P+72v3syOnNeuXbv2X8IXCIAACIDAhQReg1jihIAACIDA5QQglpczggUIgAAIEMQShwAEQAAEEhCAWCaABBMQAAEQgFjiDIAACIBAAgIQywSQYAICIAACEEucARAAARBIQABimQASTEAABEAAYokzAAIgAAIJCLgSy3K5TLdv36Z79+5Rt9tNkD5MQAAEQECHgBuxDEJZrVZpMBhQvV6HWOr0H1FAAAQSEnAhlrVajZaWluirr76iDz74AJtlwubBDARAQI+AC7GMy8VtuF7jEQkEQIBHYKLEslAo8KqD9StJ4PTG717Jul+Fov\/56PdmZU6MWAahDM8y33\/\/fTNYCOyfwJ2\/\/Jz6\/yKa\/oX\/XJEhn8DfW7+hXq\/HvzCDKyZGLINIttttajabZrAy4B25eO+996hSqaCWrIAO+fn+13fph+969MvHf0zlHb1JhU38orgvpVLJbP4nTiwtYWV1IsKWXCwWaXd3NyuXZn681fLGx59GYpn2ds1bPVdpbJ5qiZcly\/mHWF7lNOJadwSuKpbuCkJCEQGIJeMgeIDFSBemRgQglkbghcN6mH9Xm+VFvD3AEj4PcJ8BAYhlBhAduvAw\/xBLhwcDKaUnALFMz87zlRBLRnc8wGKkC1MjAhBLI\/DCYT3MPzZL4SbDvS4BiKUub61oEEsGaQ+wGOnC1IgAxNIIvHBYD\/OPzVK4yXCvSwBiqctbKxrEkkHaAyxGujA1IgCxNAIvHNbD\/GOzFG4y3OsSgFjq8taKBrFkkPYAi5EuTI0IQCyNwAuH9TD\/2CyFmwz3ugQglrq8taJBLBmkPcBipAtTIwIQSyPwwmE9zD82S+Emw70uAYilLm+taBBLBmkPsBjpwtSIAMTSCLxwWA\/zj81SuMlwr0sAYqnLWysaxJJB2gMsRrowNSIAsTQCLxzWw\/xjsxRuMtzrEoBY6vLWigaxZJD2AIuRLkyNCEAsjcALh\/Uw\/9gshZsM97oEIJa6vLWiQSwZpD3AYqQLUyMCEEsj8MJhPcw\/NkvhJsO9LgGIpS5vrWgQSwZpD7AY6cLUiADE0gi8cFgP84\/NUrjJcK9LAGKpy1sr2ishlnt7ezQ3N0eDwYDa7TYdHBy8xHdmZoZarRZdv379XDsPsLQOBuKkJwCxTM\/O85Ue5l90s9zc3KTZ2VlaW1ujhYUFWlxcpHq9Tt1ud6QvQVDD1\/LyMtVqtTPtPMDyfJiQ208EIJb5PAke5l9ULIMI9vt92tjYoPn5eWo2m9TpdGhra+tFR+Ot8uTkJLIrl8u0urpK29vbI1uoB1j5PIb5qgpima9+xtV4mH8xsYxF8PDwMBLHcVEcbmmSDdQDrHwew3xVBbHMVz9fCbEc3yQvEssAJAjmzZs36dmzZ9Ft+9OnT0e6HotltVql4+PjfJ4IVHVlAhDLKyN056BQKFD4FT7zKJVK1Ov1THI03yy5t+GB0s7ODu3u7poAQ1DfBCCWvvuTJruVlRWqVCrRpbkUy1BYkmeW488oz9tA480yPPcMm6XVd5c0zcY1egQglnqstSKFrbJYLEaCmVuxTPIs8qzNMtxqf\/bZZyMfBOGZpdbRnOw4EMvJ7t952XuYf7Hb8Ljos96zDM8zG40G7e\/vR594x883X3\/99eiyo6Oj6JPx4S8PsPJ5DPNVFcQyX\/2Mq\/Ew\/+JimVXrPMDKqhb4kSMAsZRja+nZw\/xDLC1PAGJnTgBimTlSFw4hlow2eIDFSBemRgQglkbghcN6mH9slsJNhntdAhBLXd5a0SCWDNIeYDHShakRAYilEXjhsB7mH5ulcJPhXpcAxFKXt1Y0iCWDtAdYjHRhakQAYmkEXjish\/nHZincZLjXJQCx1OWtFQ1iySDtARYjXZgaEYBYGoEXDuth\/rFZCjcZ7nUJQCx1eWtFg1gySHuAxUgXpkYEIJZG4IXDeph\/bJbCTYZ7XQIQS13eWtEglgzSHmAx0oWpEQGIpRF44bAe5h+bpXCT4V6XAMRSl7dWNIglg7QHWIx0YWpEAGJpBF44rIf5x2Yp3GS41yUAsdTlrRUNYskg7QEWI12YGhGAWBqBFw7rYf6xWQo3Ge51CUAsdXlrRYNYMkh7gMVIF6ZGBCCWRuCFw3qYf2yWwk2Ge10CEEtd3lrRIJYM0h5gMdKFqREBiKUReOGwHuYfm6Vwk+FelwDEUpe3VjSIJYO0B1iMdGFqRABiaQReOKyH+cdmKdxkuNclALHU5a0V7ZUQy729PZqbm6PBYEDtdpsODg7O5BvbhT88OjqijY2NETsPsLQOBuKkJwCxTM\/O85Ue5l90s9zc3KTZ2VlaW1ujhYUFWlxcpHq9Tt1ud6Qvw3ZvvfUWNRoN2t\/fHxFWD7A8Hybk9hMBiGU+T4KH+RcVy7At9vv9aEucn5+nZrNJnU6Htra2XnR0ZmaG7t+\/Tw8fPjx36wzGHmDl8xjmqyqIZb76GVfjYf7FxDKIYKvVosPDw0gc49+fnJyM3GIHEV1fX6fnz5\/Tu+++G7G56Da8Wq3S8fFxPk8EqroyAYjllRG6c1AoFCj8Co\/xSqUS9Xo9kxzFxHJ8k7xILMPG+c0339Dy8jKVy2VaXV2l7e3tM2\/DA6WdnR3a3d01AYagvglALH33J012KysrVKlUoktzKZaczXL4GeV5ohqv4UFYw2Zp9d0lTbNxjR4BiKUea61IYassFouRYOZSLAPIpM8sk9yue3hmoXU4ECc9AYhlenaer\/Qw\/2K34QE859Pw6enp6Da8VqvR0tLSS68ZeYDl+TAht58IQCzzeRI8zL+oWMbb5fh7luF55vjrQcPvWT548GDkE\/PgxwOsfB7DfFUFscxXP+NqPMy\/uFhm1ToPsLKqBX7kCEAs5dhaevYw\/xBLyxOA2JkTgFhmjtSFQ4glow0eYDHShakRAYilEXjhsB7mH5ulcJPhXpcAxFKXt1Y0iCWDtAdYjHRhakQAYmkEXjish\/nHZincZLjXJQCx1OWtFQ1iySDtARYjXZgaEYBYGoEXDuth\/rFZCjcZ7nUJQCx1eWtFg1gySHuAxUgXpkYEIJZG4IXDeph\/bJbCTYZ7XQIQS13eWtEglgzSHmAx0oWpEQGIpRF44bAe5h+bpXCT4V6XAMRSl7dWNIglg7QHWIx0YWpE4FfVDv3nb5\/T919vG2WAsBIEPMw\/NkuJzsKnGQGIpRl60cAQSwZeD7AY6cLUiADE0gi8cFgP84\/NUrjJcK9LAGKpy1srGsSSQdoDLEa6MDUiALE0Ai8c1sP8Y7MUbjLc6xKAWOry1ooGsWSQ9gCLkS5MjQhALI3AC4f1MP\/YLIWbDPe6BCCWury1okEsGaQ9wGKkC1MjAhBLI\/DCYT3MPzZL4SbDvS4BiKUub61oEEsGaQ+wGOnC1IgAxNIIvHBYD\/MvvlnG\/z\/wwWBA7XabDg4OLsQa7MPX8vLyiJ0HWMLnAe4zIACxzACiQxce5l9ULDc3N2l2dpbW1tZoYWGBFhcXqV6vU7fbPbMdtVqNbt26RU+ePIFYOjywk5ASxHISusTPMfdiGbbEfr9PGxsbND8\/T81mkzqdDm1tbb1EK\/z5+vp69N9PT08hlvzzhCuICGKZz2OQa7GcmZmhVqtFh4eHkTjGvz85OYnEc\/wrFtbp6WnchufzvKtUBbFUwaweJNdiOb5JXiSW5XKZPvroI7pz5w7dvXv3QrGsVqt0fHys3iwEnAwCEMvJ6BMny0KhQOFX+MyjVCpRr9fjXJ6Zrdgzy6SbZbC7f\/8+PXz4MPrw57IPeELlOzs7tLu7mxkEOMoPAYhlfnoZV7KyskKVSiX6bS7FMhSW5Jll2CrDtjg1NTXS5fEPeeI1PDz3DJul1XeX\/B3FfFUEscxXP0M1YassFouRYOZWLLmfhscCG\/6JV4fyd+g1KoJYalDWj5HrZ5YxzrPeswzPMxuNBu3v77\/03uVlt+GW31n0jwgicglALLnEJsP+lRDLrFrhAVZWtcCPHAGIpRxbS88e5l\/sA56swXqAlXVN8Jc9AYhl9kw9ePQw\/xBLDycBOWRGAGKZGUpXjiCWjHZ4gMVIF6ZGBCCWRuCFw3qYf2yWwk2Ge10CEEtd3lrRIJYM0h5gMdKFqREBiKUReOGwHuYfm6Vwk+FelwDEUpe3VjSIJYO0B1iMdGFqRABiaQReOKyH+cdmKdxkuNclALHU5a0VDWLJIO0BFiNdmBoRgFgagRcO62H+sVkKNxnudQlALHV5a0WDWDJIe4DFSBemRgQglkbghcN6mH9slsJNhntdAhBLXd5a0SCWDNIeYDHShakRAYilEXjhsB7mH5ulcJPhXpcAxFKXt1Y0iCWDtAdYjHRhakQAYmkEXjish\/nHZincZLjXJQCx1OWtFQ1iySDtARYjXZgaEYBYGoEXDuth\/rFZCjcZ7nUJQCx1eWtFg1gySHuAxUgXpkYEIJZG4IXDeph\/bJbCTYZ7XQIQS13eWtEglgzSHmAx0oWpEQGIpRF44bAe5h+bpXCT4V6XAMRSl7dWNIglg7QHWIx0YWpEAGJpBF44rIf5F98s9\/b2aG5ujgaDAbXbbTo4OHgJa61Wo1u3br347w8ePKCtra0ROw+whM8D3GdAAGKZAUSHLjzMv6hYbm5u0uzsLK2trdHCwgItLi5SvV6nbrf7oh3z8\/O0vr5On3zySfTfg3CeZecBlsMzhJTGCEAs83kkPMy\/qFiGrbLf79PGxgYFUWw2m9TpdF7aGofbe56dB1j5PIb5qgpima9+xtV4mH8xsZyZmaFWq0WHh4eROMa\/Pzk5icTzvK9yuUyrq6u0vb09cssew6pWq3R8fJzPE4GqrkwAYnllhO4cFAoFCr\/CY7xSqUS9Xs8kRzGxHN8Qk4jlRTaxWAZKOzs7tLu7awIMQX0TgFj67k+a7FZWVqhSqUSX5lIsuZtlbH96ekrLy8svMY3FMtzKh83S6rtLmmbjGj0CEEs91lqRwlZZLBYjwcylWAaQSZ9ZxltoEMHzbtE9PLPQOhyIk54AxDI9O89Xeph\/sdvwAD7Jp+FJbs+DLw+wPB8m5PYTAYhlPk+Ch\/kXFct4uxx\/zzJsko1Gg\/b396POhg9tpqamRro8\/q6lB1j5PIb5qgpima9+xtV4mH9xscyqdR5gZVUL\/MgRgFjKsbX07GH+IZaWJwCxMycAscwcqQuHEEtGGzzAYqQLUyMCEEsj8MJhPcw\/NkvhJsO9LgGIpS5vrWgQSwZpD7AY6cLUiADE0gi8cFgP84\/NUrjJcK9LAGKpy1srGsSSQdoDLEa6MDUiALE0Ai8c1sP8Y7MUbjLc6xKAWOry1ooGsWSQ9gCLkS5MjQhALI3AC4f1MP\/YLIWbDPe6BCCWury1okEsGaQ9wGKkC1MjAhBLI\/DCYT3MPzZL4SbDvS4BiKUub61oEEsGaQ+wGOnC1IgAxNIIvHBYD\/OPzVK4yXCvSwBiqctbKxrEkkHaAyxGujA1IgCxNAIvHNbD\/GOzFG4y3OsSgFjq8taKBrFkkPYAi5EuTI0IQCyNwAuH9TD\/2CyFmwz3ugQglrq8taJBLBmkPcBipAtTIwIQSyPwwmE9zD82S+Emw70uAYilLm+taBBLBmkPsBjpwtSIAMTSCLxwWA\/zj81SuMlwr0sAYqnLWysaxJJB2gMsRrowNSIAsTQCLxzWw\/y72Sz39vZo\/P8vPszfAyzh8wD3GRCAWGYA0aELD\/PvQiw3NzdpdnaW1tbWaGFhgRYXF6ler1O3233RNg+wHJ4hpDRGAGKZzyPhYf5diGXYKvv9Pm1sbND8\/Dw1m03qdDq0tbWVS7EsFApULBbpiy++oF6vN9Gn21stVxVLb\/Vc5XDkqRaIJRHNzMxQq9Wiw8PDSBzj35+cnETiGX\/FsKrVai4Ept1uE2q5ihScfe2\/S3+ga8\/\/St9\/vZ3KeRAY9CYVOtGL4r6USiWz+TffLMc3yfPEMsC62fiTaEPgfPIJ\/PkfP6PfvvMj3Zr7cfKLQQUjBI6Pj6MFw+rLXCyTbpYB0Bsff2rFCXEnhMAP3\/XonWefT0i2SJNDIDyysnxsZS6WAVaSZ5YcqLAFARAAgawJuBDLJJ+GZ104\/IEACIAAh4ALsYy3y4ves+QUBVsQAAEQyJqAG7HMujD4AwEQAIEsCUAss6QJXyAAArklMBFiWS6Xo1cGpqam6MmTJ7S8vDwRDeHmPfxBl8cCk9QTv91w\/fr1qIRnz55FP5n19OlTVyUlqWX48VD496Ojo5F3fz0VlLSeOOdgv7q6Stvb23RwcOCpFEpaS\/wj0nHy0v1xL5bD72E+evQoeoF9\/IV1V53+fzLcvOPGSzc8Lauk9YQP68JX+IGC896ZTZtDVtclraVWq9GNGzeib87n\/WRZVjldxU\/SeuIYcV\/efPPN6AV8T2KZtJbxVw6vwi\/pte7FMnyXuX37Nt27dy\/6WfHhT869bSvD0JPmHTf99PQ0ujz+sc+kDdSyS1rPeD4e+5W2Fq+bP7ee0JO3336bglh62yyT1hJEtdFo0P7+vprYuxfL8N39ww8\/fHErF35\/1l+0oSUaSeOkydvrMIaa09QT38aGf3p6dJKmFs+bJaeeWIy+\/PLLaI68iWXSWoZv1bUe97gXy\/HNZFLEMk3ensUyTT1ee8WtJdjfvHnT7fNXTj3hjD1+\/Di6g\/H4zDJpLcNn69tvv40ez4W7M8lvyu7FMul3mqQbn5Zdmrw9iyW3nmC\/tLTk7pnYVbZkj48UOPUMP4P1+gEP95zF86zxjdm9WCZ9hqElgknjpMnbs1hy6gmiEv6WqPG\/kzQpO2k7Ti3DuWgMZJrak9Yz\/ulxiDUYDFx9Q0tayzincZFNw\/Gya9yLZdJPxy4rVPvP0+TtWSyT1uNVUIb7n7SWIPrT09Mvbu1Cf7w9fw35JK1nmIHXzTJpLcO90Xrrwr1YhgYnfe9KWxAvi3de3ueJomexvKgPw3mftb14fNcyaW+G6\/FYR3wGk9YzbO\/xmWXScxZ\/eBh+RDp8abx\/PRFieZko4c9BAARAQJoAxFKaMPyDAAjkggDEMhdtRBEgAALSBCCW0oThHwRAIBcEIJa5aCOKAAEQkCYAsZQmDP8gAAK5IACxzEUbUQQIgIA0gf8B5h2WGJGz8RkAAAAASUVORK5CYII=","height":199,"width":331}}
%---
%[output:3719e395]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 11.864247 seconds.\n","truncated":false}}
%---
%[output:5f7dd01a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7t3U9oZNedL\/AzwX9o0mqCMRFOmDF6iNfMchScDiSILLLKogeLIGYMD2ujRVB68xDWYJowGBGQ0SLQEQQkgcJAzyA8mJdF1kLgRcfYA8EQAs0IZp5bKDAQpjsYO8bvcYq5PberJVVdqX5V98+nIHRk3frVuZ\/zK9VX554q\/dkLL7zw\/5IbAQIECBAgQKBBAn8mwDRotgyVAAECBAgQ6AkIMBqBAAECBAgQaJyAANO4KTNgAgQIECBAQIDRAwQIECBAgEDjBASYxk2ZARMgQIAAAQICjB4gQIAAAQIEGicgwDRuygyYAAECBAgQEGD0AAECBAgQINA4AQGmcVNmwAQIECBAgIAAowcIECBAgACBxgkIMI2bMgMmQIAAAQIEBBg9QIAAAQIECDROQIBp3JQZMAECBAgQICDA6AECBAgQIECgcQICTOOmzIAJECBAgAABAUYPECBAgAABAo0TEGAaN2UGTIAAAQIECAgweoAAAQIECBBonIAA07gpM2ACBAgQIEBAgNEDBAgQIECAQOMEBJjGTZkBEyBAgAABAgKMHiBAgAABAgQaJyDANG7KDJgAAQIECBAQYPQAAQIECBAg0DgBAaZxU2bABAgQIECAgACjBwgQIECAAIHGCQgwjZsyAyZAgAABAgQEGD1AgAABAgQINE5AgGnclBkwAQIECBAgIMDoAQIECBAgQKBxAgJM46bMgAkQIECAAAEBRg8QIECAAAECjRMQYBo3ZQZMgAABAgQICDB6gAABAgQIEGicgADTuCkzYAIECBAgQECA0QMECBAgQIBA4wQEmMZNmQETIECAAAECnQ8we3t76eTkJK2trekGAgQIECBAoCECnQ4wObxcv349HR4eCjANaVjDJECAAAECWaCTAebGjRtpfX09ffbZZ70u+OijjwQYzwcCBAgQINAggc4GmJdffjm9\/\/77aXNzM92\/f1+AaVDTGioBAgQIEOhkgCmmfWZmZqgA89JLLz3RKcfHxzqHAAECBAgQmKCAADNgBSaHl9u3b6e5ubnH07Szs5N2d3cnOG0emgABAgQIdFtAgBkQYHJw2dra6u2ZKVZe8r9WYbr9xHH2BAgQIDBZAQFmyACzsLAgtEy2Vz06AQIECBB4LCDACDCeDgQIECBAoHECAowA07imNWACBAgQINDpADPM9Bd7YFxCGkbLMQQIECBAYDwCAswAZwFmPI3oUQgQIECAQBUBAUaAqdIvjiVAgAABArUQEGAEmFo0okEQIECAAIEqAgKMAFOlXxxLgAABAgRqISDACDC1aESDIECAAAECVQQEGAGmSr84lgABAgQI1EJAgBFgatGIBkGAAAECBKoICDACTJV+cSwBAgQIEKiFgAAjwNSiEQ2CAAECBAhUERBgBJgq\/eJYAgQIECBQCwEBRoCpRSMaBAECBAgQqCIgwAgwVfrFsQQIECBAoBYCAowAU4tGNAgCBAgQIFBFQIARYKr0i2MJECBAgEAtBAQYAaYWjWgQBAgQIECgioAAI8BU6RfHEiBAgACBWggIMAJMLRrRIAgQIECAQBUBAUaAqdIvjiVAgAABArUQEGAEmFo0okEQIECAAIEqAgKMAFOlXxxLgAABAgRqISDACDC1aESDIECAAAECVQQEGAGmSr84lgABAgQI1EJAgBFgatGIBkGAAAECBKoICDACTJV+cSwBAgQIEKiFgAAjwNSiEQ2CAAECBAhUERBgBJgq\/eJYAgQIECBQCwEBRoCpRSMaBAECBAgQqCIgwAgwVfrFsQQIECBAoBYCAowAU4tGNAgCBAgQIFBFQIARYKr0i2MJECBAgEAtBAQYAaYWjWgQBAgQIECgioAAI8BU6RfHEiBAgACBWggIMAJMLRrRIAgQIECAQBWBVgWYjY2NND8\/3zv\/w8PDtLa2dqZF+dgHDx6k1dXVdHR09NTxc3NzaWtrKy0sLKTj4+Mqto4lQIAAAQIEggRaE2AWFxfT8vJy2t7e7lEV\/39\/f\/8pulu3bqWbN2+m27dvp9\/\/\/vdpc3MzPXz4MC0tLQkwQY2mLAECBAgQGKVAawJMXlGZnZ19vJKyt7eXTk5OTl2F6T+2\/+sysBWYUbabWgQIECBAYDQCrQkwObDkW7GK0v91meu0FZj79++fGnaKALOysvL4EpJLSaNpPlUIECBAgMBFBVoVYMorLnlVZXp6+tTLQhkrX3LKoeS5555Ld+\/eTXfu3DnVsAgw5W\/u7Oyk3d3di5q7HwECBAgQIHBJgU4GmBxucjDJe2Du3buXzlutKQLM+vr6EyswVmEu2XnuToAAAQIELiHQqgAzzCWkmZmZ3qbd8iWj8gbg\/k2\/9sBcorvclQABAgQIBAm0JsD0XzI6axOvABPUScoSIECAAIExCrQmwFR5G\/Vpl5CmpqZO\/SwYKzBj7EYPRYAAAQIEhhRoTYDJ53vWB9kVqy4HBwePN+vmFZrr16\/3mHyQ3ZDd4jACBAgQIFATgVYFmAhTKzARqmoSIECAAIHLCQgwA\/wEmMs1mHsTIECAAIEIAQFGgInoKzUJECBAgECogAAjwIQ2mOIECBAgQCBCQIARYCL6Sk0CBAgQIBAqIMAIMKENpjgBAgQIEIgQEGAEmIi+UpMAAQIECIQKCDACTGiDKU6AAAECBCIEBBgBJqKv1CRAgAABAqECAowAE9pgihMgQIAAgQgBAUaAiegrNQkQIECAQKiAACPAhDaY4gQIECBAIEJAgBFgIvpKTQIECBAgECogwAgwoQ2mOAECBAgQiBAQYASYiL5SkwABAgQIhAoIMAJMaIMpToAAAQIEIgQEGAEmoq\/UJECAAAECoQICjAAT2mCKEyBAgACBCAEBRoCJ6Cs1CRAgQIBAqIAAI8CENpjiBAgQIEAgQkCAEWAi+kpNAgQIECAQKiDACDChDaY4AQIECBCIEBBgBJiIvlKTAAECBAiECggwAkxogylOgAABAgQiBAQYASair9QkQIAAAQKhAgKMABPaYIoTIECAAIEIAQFGgInoKzUJECBAgECogAAjwIQ2mOIECBAgQCBCQIARYCL6Sk0CBAgQIBAqIMAIMKENpjgBAgQIEIgQEGAEmIi+UpMAAQIECIQKCDACTGiDKU6AAAECBCIEBBgBJqKv1CRAgAABAqECrQowGxsbaX5+vgd2eHiY1tbWzsS7detWeu2113rff\/ToUbp9+3a6d+\/eU8fPzc2lra2ttLCwkI6Pj0MnQ3ECBAgQIEBgOIHWBJjFxcW0vLyctre3e2de\/P\/9\/f2nJPKxKysr6Z133kl37txJOfjMzs6m1dXVdHR09MTxAsxwjeQoAgQIECAwToHWBJj+ELK3t5dOTk5OXYXJx05PT6elpaWB1gLMQCIHECBAgACBsQu0JsDkwJJvRSjp\/7qQnZmZSZubm+n+\/fvnXmIqji8CTF6xKS4huZQ09j71gAQIECBA4AmBVgWY8orLWassRYD5zW9+k77zne+kq1evDrUHpqy2s7OTdnd3tRIBAgQIECAwIYHOBphr16493ribV2umpqbO3QOzvr7+xAqMVZgJdayHJUCAAAECKaVWBZiLXkIqbwDu3\/RrD4znCQECBAgQqJ9AawJM\/yWj8zbx9n8vB5jXX389vfXWW0+9lVqAqV\/TGhEBAgQIEGhNgKnyNur8GTA3b9584hJSefWm3BYCjCcJAQIECBCon0BrAkymPeuD7IqNuwcHB73Pfcm38gfZPXjw4NT9L\/k4AaZ+TWtEBAgQIECgVQEmYjoFmAhVNQkQIECAwOUEBJgBfgLM5RrMvQkQIECAQISAACPARPSVmgQIECBAIFRAgBFgQhtMcQIECBAgECEgwAgwEX2lJgECBAgQCBUQYASY0AZTnAABAgQIRAgIMAJMRF+pSYAAAQIEQgUEGAEmtMEUJ0CAAAECEQICjAAT0VdqEiBAgACBUAEBRoAJbTDFCRAgQIBAhIAAI8BE9JWaBAgQIEAgVECAEWBCG0xxAgQIECAQISDACDARfaUmAQIECBAIFRBgBJjQBlOcAAECBAhECAgwAkxEX6lJgAABAgRCBQQYASa0wRQnQIAAAQIRAgKMABPRV2oSIECAAIFQAQFGgAltMMUJECBAgECEgAAjwET0lZoECBAgQCBUQIARYEIbTHECBAgQIBAhIMAIMBF9pSYBAgQIEAgVEGAEmNAGU5wAAQIECEQICDACTERfqUmAAAECBEIFBBgBJrTBFCdAgAABAhECAowAE9FXahIgQIAAgVABAUaACW0wxQkQIECAQISAACPARPSVmgQIECBAIFRAgBFgQhtMcQIECBAgECEgwAgwEX2lJgECBAgQCBUQYASY0AZTnAABAgQIRAgIMAJMRF+pSYAAAQIEQgUEGAEmtMEUJ0CAAAECEQICjAAT0VdqEiBAgACBUIFWBZiNjY00Pz\/fAzs8PExra2sD8RYXF9Py8nLa3t5O+\/v7Tx0\/NzeXtra20sLCQjo+Ph5YzwEECBAgQIBAvEBrAkw5iGS280JJwTozM5M2NzfTiy++2AspAkx8w3kEAgQIECAwCoHWBJi8+jI7O5tWV1fT0dFR2tvbSycnJ+euwty6dSvdvHmz52gFZhTtpAYBAgQIEBiPQGsCTA4s+ba0tNT7t\/\/rfs4bN26kN954Ix0cHPRCjAAznobzKAQIECBAYBQCrQow5RWXvCIzPT39OND0Y+Xv59sHH3ww1B6YlZWVx3tg7IUZReupQYAAAQIELi7QyQCT98u8+uqr6c0330yvvPLKUAGmTLyzs5N2d3cvru6eBAgQIECAwKUEWhVgssQwl5Dy5aW88nLnzp007LuQ1tfXn1iBsQpzqb5zZwIECBAgcCmB1gSY\/ktGZ23izXtfchi5evXqU3B3797thZryzduoL9Vf7kyAAAECBEIEWhNgLvI26iw67AqMz4EJ6T9FCRAgQIDAhQRaE2Dy2Z\/1QXbF573kdxz1r7AIMBfqG3ciQIAAAQITFWhVgImQdAkpQlVNAgQIECBwOQEBZoCfAHO5BnNvAgQIECAQISDACDARfaUmAQIECBAIFRBgBJjQBlOcAAECBAhECAgwAkxEX6lJgAABAgRCBQQYASa0wRQnQIAAAQIRAgKMABPRV2oSIECAAIFQAQFGgAltMMUJECBAgECEgAAjwET0lZoECBAgQCBUQIARYEIbTHECBAgQIBAhIMAIMBF9pSYBAgQIEAgVEGAEmNAGU5wAAQIECEQICDACTERfqUmAAAECBEIFBBgBJrTBFCdAgAABAhECAowAE9FXahIgQIAAgVABAUaACW0wxQkQIECAQISAACPARPSVmgQIECBAIFRAgBFgQhtMcQIECBAgECEgwAgwEX2lJgECBAgQCBUQYASY0AZTnAABAgQIRAgIMAJMRF+pSYAAAQIEQgUEGAEmtMEUJ0CAAAECEQICjAAT0VdqEiBAgACBUAEBRoAJbTDFCRAgQIBAhIAAI8BE9JWaBAgQIEAgVECAEWBCG0xxAgQIECAQISDACDARfaUmAQIECBAIFRBgBJjQBlOcAAECBAhECAgwAkxEX6lJgAABAgRCBQQYASa0wRQnQIAAAQIRAgKMABPRV2oSIECAAIFQAQFGgAltMMUJECBAgECEgAAjwET0lZoECBAgQCBUQIARYEIbTHECBAgQIBAh0KoAs7Gxkebn53tOh4eHaW1t7VSzGzdupPX19XT16tXe9x88eJBWV1fT0dHRU8fPzc2lra2ttLCwkI6PjyPmQE0CBAgQIECgokBrAszi4mJaXl5O29vbPYLi\/+\/v7z9BMjMzkzY3N9P9+\/d7Aaf4+uHDh2lpaUmAqdhADidAgAABApMQaE2Ayasvs7Ozj1dS9vb20snJyZmrMGXs\/vuWv2cFZhJt6TEJECBAgMD5Aq0JMDmw5FuxitL\/9XkMwwSYlZWVx5eQXErytCJAgAABApMVaFWAKa+45FAyPT196mWhMnmxH+bDDz88dbWmWIEp32dnZyft7u5OduY8OgECBAgQ6LBApwNMsf8lz\/+gTbx502+x8pL\/tQrT4WeNUydAgACBiQu0KsBUuYQ0THjJ9eyBmXiPGgABAgQIEHhKoDUBpv+S0XmbeAe986isJMB41hAgQIAAgfoJtCbADPs26jwFOdxMTU2dedlIgKlfoxoRAQIECBAoC7QmwOSTOuuD7IoVl4ODg\/TrX\/\/6iQ+xKzAePXqUbt++ne7du\/dEh1iB8YQhQIAAAQL1E2hVgIngFWAiVNUkQIAAAQKXExBgBvgJMJdrMPcmQIAAAQIRAgKMABPRV2oSIECAAIFQAQFGgAltMMUJECBAgECEgAAjwET0lZoECBAgQCBUQIARYEIbTHECBAgQIBAhIMAIMBF9pSYBAgQIEAgVEGAEmNAGU5wAAQIECEQICDACTERfqUmAAAECBEIFBBgBJrTBFCdAgAABAhECAowAE9FXahIgQIAAgVABAUaACW0wxQkQIECAQISAACPARPSVmgQIECBAIFRAgBFgQhtMcQIECBAgECEgwAgwEX2lJgECBAgQCBUQYASY0AZTnAABAgQIRAgIMAJMRF+pSYAAAQIEQgUEGAEmtMEUJ0CAAAECEQICjAAT0VdqEiBAgACBUAEBRoAJbTDFCRAgQIBAhIAAI8BE9JWaBAgQIEAgVECAEWBCG0xxAgQIECAQISDACDARfaUmAQIECBAIFRBgBJjQBlOcwHkCX3zxF+nTT\/8mPf\/8P6UvfenfYBEgQGBoAQFGgBm6WRxIYFQCObj86U\/fTs8++176wx\/+JX3lK3\/VCzLPPPNe77+5ESBAYJCAACPADOoR3ycwcoFPP\/3b9Mc\/\/ixdubKRPvlkLT3\/\/D+m\/N\/y11euvD3yx1OQAIH2CQgwAkz7utoZ1Vogr77kW15xyeGluOUQk8OLS0m1nj6DI1AbAQFGgKlNMxpIswXe+OSTtPbJJ+nm1FR679lnzzyZTz5544ng0n\/gMKswf\/HFF+lf\/vCHtHHlSnr7ypVmwxk9AQIXEhBgBJgLNY47ESgLFOFlmEBRrMDkIJMvGxW3HFyqbOYtHvO9Z55JN69dMyEECHRMQIARYDrW8k531AJVwkvx2MUqTHnvS76c9OUv\/6i3H2bY27f\/9Kf0y4cP07996UvpR1\/+8rkrP8PWdBwBAs0QEGAEmGZ0qlHWTiBfxvmbTz\/tXTYaZuWlfAL5HUiff\/7t3opL8S6k\/N\/y\/peq70LK4\/jZo0fp259\/PvDyVe0QDYgAgQsLCDACzIWbxx27K1CEl7\/99NP0j88\/f6l9KHk1ZhTvPPrZH\/+Y8niqhqnuzqIzJ9BsAQFGgGl2Bxv92AVyeMmXjfLlm8uGl1EPPgeYHGTsixm1rHoE6ifQ2QCzsbGR5ufnezNyeHiY1tb+++2c5Wmam5tLW1tbaWFhIR0fH9dvBo2IwBgFiss1f\/7FF7Xdc2JfzBgbwkMRmKBAJwPM4uJiWl5eTtvb2z364v\/v7+8\/NRUCzAS700PXSiCHl\/\/zn\/\/ZG9NfX7vW2zhb15t9MXWdGeMiMDqBTgaYvPoyOzubVldX09HRUdrb20snJyenrsIIMKNrNpWaK9Ck8FJWti+muT1n5AQGCXQywOTAkm9LS0u9f\/u\/LqMJMINayPfbLtDU8FLMi30xbe9Q59dVgc4GmPKKS16RmZ6efhxoTgswKysrj\/fA2AvT1adL9867+MTbfLnor77ylcYC2BfT2KkzcAJnCggwKaVhAkxZcGdnJ+3u7morAq0WaNvKRXlfTP7Qu\/wOKjcCBJor0NkAU\/US0vr6+hMrMFZhmtv0Rj5YoAgv+UU+v9i36Vbsi2njubVpnpwLgUECnQww\/SsuNvEOahPf75LARf40QNN82ra61DR\/4yUwCoFOBhhvox5F66jRRoEuhJdi3sr7e\/wdpTZ2s3Nqu0AnA0yeVB9k1\/bWdn5VBC7zd42qPE7djrUvpm4zYjwEhhfobIAZlsjbqIeVclxTBUb5d42aalCsPNkX09QZNO4uCggwA2ZdgOni06I75yy8\/Pdc2xfTnb53pu0QEGAEmHZ0srOoLNCEv2tU+aQueYfyh\/bZF3NJTHcnECwgwAgwwS2mfB0Fmv7pupGm5X0xG1eupLevXIl8OLUJELiggAAjwFywddytqQLCy3AzZ1\/McE6OIjApAQFGgJlU73ncCQgIL9XQy3+CoMl\/SqHaWTuaQDMEBBgBphmdapSXFih\/7slfX7uW8t83chssYF\/MYCNHEJiEgAAjwEyi7zzmmAWsJFwOvLzh2UrM5Szdm8CoBAQYAWZUvaROTQXa\/HeNxkUuwIxL2uMQGF5AgBFghu8WRzZOoLhs5N00l5u6vKE3B0Fvrb6co3sTGKWAACPAjLKf1KqZQBFg3nvmmXTz2rWaja4ZwynejZTDS\/6kXjcCBOohIMAIMPXoRKMIEyj2v1iFqU5cXH5jV93OPQhECwgwFQPMSy+9lL7\/\/e+nX\/3qV+n4+Dh6fi5Vv0ljzSfapPE2aazZtlhFuDk1ld579tlL9VX0netiW6xenff3keoy1mHnpEnjbdJYm\/bza9h+qftxAkzFANOkv43UpLHmaWjSeJs01sL2n6em0rV33011DzF1sC2\/dfq8dx3VYaxVXmSaNN4mjbVpP7+q9EydjxVgBJja9GeTfmA1aazlH67PfO976X\/8+7\/XOsRM2rb8jqNBn5cz6bFWffI2abxNGqsAU7UTR3O8ADNkgFlZWeldMsrLmltbW6n4ejTTEFOlSWMtlmDZxvfC3x8cpD\/\/4ov0\/b\/8y5gHu2TVSfft\/\/rXf03\/+4UXhnrH0aTHWpW6SeNt0ljr\/vOr7tsdqvZxcbwAM0AuP4lu377du7zhRqANAs9+\/HGa\/ru\/S\/\/3H\/6hDacz8nP4n9evp\/\/40Y\/Sf9y6NfLaChKYhMDOzk7a3d2dxEOHPqYAMwRvDjH5f24E2iLwzMcfp8+\/\/vW2nI7zIEDgHIG8AtPGVRgBRtsTIECAAAECjRMQYBo3ZQZMgAABAgQICDB6gAABAgQIEGicgADTuCkzYAIECBAgQECA0QMECBAgQIBA4wQEmMZNmQETIECAAAECAswlemBvby+dnJyktbW1S1QZ3V1v3LiR1tfX09WrV9OjR496n19z7969Ux+gfGw+4O7du+nOnTujG8yASlXGOjMzkzY3N9PXvva1XtXDw8Oxm1cZb\/nUc4\/k29LSUi1tNzY20vz8\/OOxjbsP+lGy1\/Xr13v\/edJjKY9tcXGx9+GVzz33XHrw4EFaXV1NR0dHp85p+Rw+++yz3gdf7u\/vj23+q4y1GFTxHLt\/\/\/7Yn1tVxjvpn1tVxlo+dhJ9MLaGm+ADCTAXxC9+SE3ixfSsIZdfLM974ez\/YXXr1q30gx\/8YKw\/aIcdaz7XfOzU1FTvReOrX\/1qL6T98pe\/HGvgqjLeYn6y62uvvZZ+97vfjTXADDvWPL6bN28+DrqT6INyL5fH881vfvOJsV3waTqSu5WfLz\/\/+c97YfqsF\/ocCGdnZx8HnPx1\/hDM836ZGMkg\/6tIlbGWH7cIsuP+eVZlvMWxDx8+7D2fckBYXl5O29vbYwmIVcZaBK3i59S4+2CUPVXnWgJMxdkpGjMn6nz76KOPxv4by2lD7n\/C5Cf366+\/nt56662nVmH6j+3\/uiJJ5cOrjvXHP\/5x+sUvfjGWH1KXtS3\/RvuTn\/wkvfjii+njjz8eW4CpYtt\/ruPug9NWX\/J\/yy9OxYvFwcHBWIPqafPf\/0KZg9Z3v\/vdc1dhijrjfpG9yFjzvOfnWF5d+vDDD8f686zKeM\/7mVb5h9AF7lB1rOVwNe4+uMDpNfIuAkzFactP9pdffjm9\/\/775\/4mVrHspQ\/vf4Kc94Q5bQWm\/Jv4pQczoECVsfavEkSPbZgXsGF+GOXfuPJtenq69++4LiFVsa1TgOnvyUle0uh3OW2latjnyzC9MsqevshY84rdb3\/725RXvcZ9CanKePtXt0bpNkytKmM9bQWmvDI3zOM5ZrCAADPY6NQj6vQDNg+w\/7eT4req81Yuistgg67pX5DozLtVGWvx225exXjllVd6Nce9zF1lvHl82f6NN95Ib7\/9dvrhD3849gBTXnkbpg+KiZrkMvdpKy512WPWv+JSZSWgfPnzrD0zo3x+VR1rPpdXX301\/fSnP+317CQCTHk16zzb3J\/FLwST2CdV1ba8d2\/cl5FH2VN1riXAXHB2mhxgit8OiuXicf+WWCUQFPtIitBSbIx75513xnZpocp4czvlF60PPvigN75xb+KtOtai\/QvnSW2cbWOAyS+43\/rWt8a6t6zKi2w2z5c533333YmtKFcZb7FPp+jRce\/ZqjLW\/p9TdVhJvuBLXa3vJsCcMz3ld2j0v6tnkgGmfyd+fnHPL5jDXnM97ckU9ULbpLEWqyfFO7mK1Z4qtsVvtG+++WbvXSpRrqMYa13CSx5H2y4hTSK8ZMcqlznysd\/4xjee2HM0iRWY\/o3kZ12e67+ENO6fwVVsJz3WWqeOEQ5OgLkg5rifPIOG2X+p4Lyl2HEGmNPGXWWsp53HuC8tVBlv\/9uSi\/Mf12W6KmMtXvDG\/Q60s3q5PK9128Rbviw3aBPvJC\/F9T9fzhtr+e3e5TkZ5+WOKuPtP5dx90iVsQowg16xRvN9AeaCjnULMMWli\/xv3jB63m\/+p11Cyp9xMc7LMsO+1bffedyXu4r2GHa8\/e0UuQJzXhAYpg8mcTnuvKdbG95GPe7LGv2eVd7qW77vpH6eVRlv\/8bYcV+WqTLW0y4h1eUXhQu+5NXybgLMBadlUk\/484Z73oetFRvginfDFE+w\/NbJfBv33ocqY+3\/ILtxj7X\/ck3\/5cR+2\/IcTSLADGt71mrRuDdJ93tNYoPmoB8D5edL\/2paef7PWtUYZ88OO9Y6BJg8hirjLff2JD4crspYi31l+RxOFLd+AAACLElEQVQnMdZBPd2G7wswbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIOAANOGWXQOBAgQIECgYwICTMcm3OkSIECAAIE2CAgwbZhF50CAAAECBDomIMB0bMKdLgECBAgQaIPA\/wcviV5D4DCyewAAAABJRU5ErkJggg==","height":199,"width":331}}
%---

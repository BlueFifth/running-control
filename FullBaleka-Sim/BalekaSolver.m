%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88, 4.10, -0.88];
dq0 = [2,0,0,0,0,0];
X0 = [q0, dq0];
Sim.time = 2.1;
tspan = [0 Sim.time];
options = odeset('RelTol',1e-8, 'AbsTol',1e-8, 'Events',@(t, X) collision_event(t, X),'MaxStep', 5e-4);
tic

Animate = 1; % 1 = drawnow

t_total = [];
x_total = [];
te_total = [];
ie_total = [];
count = 0;
% Using ode113
while tspan(1) <Sim.time %[output:group:788ba238]

    % solve (with event detection)
    [t, x, te, xe, ie] = ode113(@(t, X) baleka_dynamics(t, X), tspan, X0, options); %[output:9b9e07c3]

    % Store results
    t_total = [t_total(1:end-1);t];
    x_total = [x_total(1:end-1, :);x];
    te_total = [te_total(1:end);te];
    ie_total = [ie_total(1:end);ie];
    if ~isempty(te) % Impact
        count = count + 1; % impact/break counts - for diagnosing hard contacts

        if ie == 1 % Front foot landed
            dq_post =dq_f_impact_func(x(end, 1:6), x(end, 7:12));
        elseif ie == 2 % back foot landed
            dq_post =dq_b_impact_func(x(end, 1:6), x(end, 7:12));
        elseif ie == [1;2] %both feet landed
            dq_post =dq_fb_impact_func(x(end, 1:6), x(end, 7:12));
        end
        
        % update initial conditions to restart solver
        X0 = [x(end, 1:6), dq_post'];
        tspan = [te(end) Sim.time];
    else
        break
    end
end %[output:group:788ba238]
toc

% Import important coords
BalekaParams

l1 = UpperLink.l;
l2 = FootLink.l2;
l3 = LowerLink.l;
l4 = FootLink.l4;
l5 = FullBody.l;
foot_offset = FootLink.offset;
th3 = FootLink.th3;

clear FootLink LowerLink HalfBody UpperLink FullBody% declutter workspace

%%
%[text] ## Display generalized Coordinates
% display state of all variables
plot(t_total,x_total(:,1));
hold on
plot(t_total,x_total(:,2));
plot(t_total,x_total(:,3));
plot(t_total,x_total(:,4));
plot(t_total,x_total(:,5));
plot(t_total,x_total(:,6));
legend('x','y','th1','th2', 'th3', 'th4');
title("Generalised Coordinates");
xlabel('time (s)');
hold off

% display derivative state of all variables
plot(t_total,x_total(:,7));
hold on
plot(t_total,x_total(:,8));
plot(t_total,x_total(:,9));
plot(t_total,x_total(:,10));
plot(t_total,x_total(:,11));
plot(t_total,x_total(:,12));
legend('dx','dy','dth1','dth2', 'dth3', 'dth4');
title("Derivative of Generalised Coordinates");
xlabel('time (s)');
hold off
%%
%[text] ## Forces
coords_total = zeros(12, length(t_total));
foot_total = zeros(8,length(t_total));
accel_total = zeros(10, length(t_total));

contact_total = zeros(2, length(t_total));

controller_total = zeros(17, length(t_total));

% controller_total(17, :) = repmat(States.Compression,1, length(t_total));
constraint_total = zeros(8,length(t_total));


for i = 1:length(t_total)
    % States
    coords_total(:, i) = x_total(i, 1:12);

    % ph1 = ph1_calc(x_total(i,3),x_total(i,4));
    % dph1 = dph1_calc(x_total(i,3),x_total(i,4),x_total(i,9),x_total(i,10));
    % ph2 = ph2_calc(x_total(i,3),x_total(i,4));
    % dph2 = dph2_calc(x_total(i,3),x_total(i,4),x_total(i,9),x_total(i,10));
    % ph3 = ph1_calc(x_total(i,5),x_total(i,6));
    % dph3 = dph1_calc(x_total(i,5),x_total(i,6),x_total(i,11),x_total(i,12));
    % ph4 = ph2_calc(x_total(i,5),x_total(i,6));
    % dph4 = dph2_calc(x_total(i,5),x_total(i,6),x_total(i,11),x_total(i,12));

    % th1 = x_total(i, 3);
    % th2 = x_total(i, 4);
    % th3 = x_total(i, 5);
    % th4 = x_total(i, 6);
    % 
    % dth1 = x_total(i, 9);
    % dth2 = x_total(i, 10);
    % dth3 = x_total(i, 11);
    % dth4 = x_total(i, 12);
    % 
    % q = [x_total(i, 1); x_total(i, 2); th1; th2; th3; th4; ph1; ph2; ph3; ph4];
    % dq = [x_total(i, 5); x_total(i, 6); x_total(i, 7); x_total(i, 8); dph1; dph2];

    out = SolverFuncRaibert(t_total(i), x_total(i, 1:12));

    accel_total(:, i) = out.ddq;
    foot_total(:, i) = out.foot;
    controller_total(:, i) = out.controller;
    constraint_total(:, i) = out.lambda;
    contact_total(:, i) = out.contact;
end
%%
plot(t_total, contact_total(1, :))
hold on
plot(t_total,contact_total(2, :));
legend('Front contact', 'Back contact');
title("Contact bool");
xlabel('time (s)');
hold off

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

contact_sols.time = transpose(time);
contact_sols.signals.values = transpose(contact_total);

sim("data_inspector_Baleka.slx");
%%
%[text] ## Animate (drawnow)
if Animate ==1
    fps = 60; % Desired frame rate
    dt = 1 / fps; % Time interval between frames
    t_interp = t_total(1):dt:t_total(end); % New time vector
    
    % Interpolate all states in x_total
    solsy_interp = interp1(t_total, x_total, t_interp, 'linear')';
    
    
    x_sim = solsy_interp(1,:);
    y_sim = solsy_interp(2,:);
    th1_sim = solsy_interp(3,:);
    th2_sim = solsy_interp(4,:);
    th3_sim = solsy_interp(5,:);
    th4_sim = solsy_interp(6,:);


    ph1_sim =ph1_calc(th1_sim, th2_sim);
    ph2_sim = ph2_calc(th1_sim, th2_sim);
    ph3_sim =ph1_calc(th3_sim, th4_sim);
    ph4_sim = ph2_calc(th3_sim, th4_sim);

    %Front Leg
    x_f_hip_left =x_sim - l5/2; % y same as body
    x_f_hip_right = x_sim + l5/2; % y same as body
    
    
    x_f_knee_left = x_f_hip_left + l1*cos(th1_sim);
    y_f_knee_left = y_sim + l1*sin(th1_sim);
    
    x_f_knee_right = x_f_hip_right + l1*cos(th2_sim);
    y_f_knee_right = y_sim + l1*sin(th2_sim);
    
    x_f_joint_left = x_f_knee_left + l2*cos(ph1_sim + th1_sim); % calculated from left leg segment
    y_f_joint_left = y_f_knee_left + l2*sin(ph1_sim + th1_sim); % calculated from left leg segment
    
    x_f_joint_right = x_f_knee_right + l3*cos(ph2_sim + th2_sim); 
    y_f_joint_right = y_f_knee_right + l3*sin(ph2_sim + th2_sim); 
    
    x_f_foot = x_f_joint_left + l4*cos(ph1_sim + th1_sim + th3);
    y_f_foot = y_f_joint_left + l4*sin(ph1_sim + th1_sim +th3) + foot_offset;

    %Back Leg
    x_b_hip_left =x_sim - l5/2; % y same as body
    x_b_hip_right = x_sim + l5/2; % y same as body
    
    
    x_b_knee_left = x_b_hip_left + l1*cos(th3_sim);
    y_b_knee_left = y_sim + l1*sin(th3_sim);
    
    x_b_knee_right = x_b_hip_right + l1*cos(th4_sim);
    y_b_knee_right = y_sim + l1*sin(th4_sim);
    
    x_b_joint_left = x_b_knee_left + l2*cos(ph3_sim + th3_sim); % calculated from left leg segment
    y_b_joint_left = y_b_knee_left + l2*sin(ph3_sim + th3_sim); % calculated from left leg segment
    
    x_b_joint_right = x_b_knee_right + l3*cos(ph4_sim + th4_sim); 
    y_b_joint_right = y_b_knee_right + l3*sin(ph4_sim + th4_sim); 
    
    x_b_foot = x_b_joint_left + l4*cos(ph3_sim + th3_sim + th3);
    y_b_foot = y_b_joint_left + l4*sin(ph3_sim + th3_sim +th3) + foot_offset;


    fig = figure;
    
    ax = axes('Parent',fig);
    
    body = animatedline(ax,'Color','g','LineWidth',1, "Marker", "*");
    f_u_l = animatedline(ax,'Color','r','LineWidth',0.5);
    f_u_r = animatedline(ax,'Color','r','LineWidth',0.5);
    f_l_l = animatedline(ax,'Color','r','LineWidth',0.5);
    f_l_r = animatedline(ax,'Color','r','LineWidth',0.5);
    f_foot = animatedline(ax,'Color','r','LineWidth',0.5);

    b_u_l = animatedline(ax,'Color','b','LineWidth',0.5);
    b_u_r = animatedline(ax,'Color','b','LineWidth',0.5);
    b_l_l = animatedline(ax,'Color','b','LineWidth',0.5);
    b_l_r = animatedline(ax,'Color','b','LineWidth',0.5);
    b_foot = animatedline(ax,'Color','b','LineWidth',0.5);
    
    axes(ax);
    
    axis equal;
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) 0 1.5]);
    hold on;
    tic
    drawnow
    
    for i = 1:length(solsy_interp)
    
        clearpoints(body);
        clearpoints(f_u_l);
        clearpoints(f_u_r);
        clearpoints(f_l_l);
        clearpoints(f_l_r);
        clearpoints(f_foot);
        clearpoints(b_u_l);
        clearpoints(b_u_r);
        clearpoints(b_l_l);
        clearpoints(b_l_r);
        clearpoints(b_foot);
    
    
        % axis equal;
        % axis([-1 11 -0.5 1]);
        % body
        addpoints(body,x_sim(i),y_sim(i));

        % FRONT FOOT
        % upper left
        addpoints(f_u_l,[x_f_hip_left(i), x_f_knee_left(i)],[y_sim(i), y_f_knee_left(i)]);
        % upper right
        addpoints(f_u_r,[x_f_hip_right(i), x_f_knee_right(i)],[y_sim(i), y_f_knee_right(i)]);
        % lower left
        addpoints(f_l_l,[x_f_knee_left(i), x_f_joint_left(i)],[y_f_knee_left(i), y_f_joint_left(i)]);
        % lower right
        addpoints(f_l_r,[x_f_knee_right(i), x_f_joint_right(i)],[y_f_knee_right(i), y_f_joint_right(i)]);
        % foot
        addpoints(f_foot,[x_f_joint_left(i), x_f_foot(i)],[y_f_joint_left(i), y_f_foot(i)]);
    

        % BACK FOOT
        % upper left
        addpoints(b_u_l,[x_b_hip_left(i), x_b_knee_left(i)],[y_sim(i), y_b_knee_left(i)]);
        % upper right
        addpoints(b_u_r,[x_b_hip_right(i), x_b_knee_right(i)],[y_sim(i), y_b_knee_right(i)]);
        % lower left
        addpoints(b_l_l,[x_b_knee_left(i), x_b_joint_left(i)],[y_b_knee_left(i), y_b_joint_left(i)]);
        % lower right
        addpoints(b_l_r,[x_b_knee_right(i), x_b_joint_right(i)],[y_b_knee_right(i), y_b_joint_right(i)]);
        % foot
        addpoints(b_foot,[x_b_joint_left(i), x_b_foot(i)],[y_b_joint_left(i), y_b_foot(i)]);
    
        axis(ax,[(x_sim(i)-1) (x_sim(i)+1) -0.1 1]);
    
        drawnow
        pause(dt);
    
    end
    drawnow
    toc
    hold off;
    clear fps body f_foot f_l_l f_l_r f_u_l f_u_r ax fig l1 l2 l3 l4 l5 b_l_l b_l_r b_u_l b_u_r
end

%%
%[text] ## Functions
function out = baleka_dynamics(time, q)
    solv = SolverFuncRaibert(time, q);
    out = [q(7:12); solv.ddq(1:6)];
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
    persistent contact_f contact_b % contact flag
    persistent groundheight_f groundheight_b % log ground heigh for when contact is found
    if t==0
        contact_f = false; % start in the air
        contact_b = false; % start in the air
        groundheight_f = 0;
        groundheight_b = 0;
    end
    isterminal = [1; 1];
    direction = [-1; -1];
    

    ph1 = ph1_calc(q(3), q(4));
    dph1 = dph1_calc(q(3),q(4), q(9), q(10));
    ph3 = ph1_calc(q(5), q(6));
    dph3 = dph1_calc(q(5),q(6), q(11), q(12));

    foot_f = foot_Func(q(1), q(2), q(3), q(4), ph1, q(7), q(8), q(9), q(10), dph1);
    foot_f = reshape(foot_f, [4,1]);

    foot_b = foot_Func(q(1), q(2), q(5), q(6), ph3, q(7), q(8), q(11), q(12), dph3);
    foot_b = reshape(foot_b, [4,1]);
    
    % Check for both feet on ground
    if (~contact_f)&&(foot_f(2)<=0)&&(~contact_b)&&(foot_b(2)<=0)
        contact_f = true;
        groundheight_f = foot_f(2) +1e-5;
        contact_b = true;
        groundheight_b = foot_b(2) +1e-5;
        value = [foot_f(4); foot_b(4)];
        return

    % Check for front foot
    elseif (~contact_f)&&(foot_f(2)<=0)
        contact_f = true;
        groundheight_f = foot_f(2) +1e-5;
        value = [foot_f(4); foot_b(4)];
        return
    % Check for back foot
    elseif (~contact_b)&&(foot_b(2)<=0)
       contact_b = true;
       groundheight_b = foot_b(2) +1e-5;
       value = [foot_f(4); foot_b(4)];
       return
    end
    
    % Both in contact
    if contact_f && contact_b
        % Feet both slipping 
        if (foot_f(4)<-1e-2)&&(foot_b(4)<-1e-2)%check which way round??
            value = [0;0];
            return
        end
    end
        
    if contact_f
       % Edit ground heights
        % if (foot_f(2) + 1e-5)<groundheight_f
        %     groundheight_f = foot_f(2) +1e-5;
        % end
        if (foot_f(4)>1e-3 && foot_f(2)>groundheight_f)
            contact_f = false;
        end
        % Front foot slipping
        % if (foot_f(4)<-1e-2) %check which way round??
        %     value = [0;1];
        %     return
        % end
   end

   if contact_b
       % Edit ground heights
       % if (foot_b(2) + 1e-5)<groundheight_b
       %      groundheight_b = foot_b(2) +1e-5;
       % end
       if (foot_b(4)>1e-3 && foot_b(2)>groundheight_b)
            contact_b = false;
       end
       % Back foot slipping
       % if (foot_b(4)<-1e-2) %check which way round??
       %      value = [1;0];
       %      return
       % end


   end
   value = [1;1];

end
%%
clear Damping dph2 X0 dph1 dq dq0 dq_post dx dy dt dth1 dth2 ph1 ph2 q q0 th1 th2 th3

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":38.9}
%---
%[output:9b9e07c3]
%   data: {"dataType":"text","outputData":{"text":"Time = 1.5681","truncated":false}}
%---

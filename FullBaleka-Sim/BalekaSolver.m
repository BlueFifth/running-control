%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88, 4.10, -0.88];
dq0 = [2,0,0,0,0,0];
X0 = [q0, dq0];
Sim.time = 2.8660;
tspan = [0 Sim.time];
options = odeset('RelTol',1e-10, 'AbsTol',1e-10, 'Events',@(t, X) collision_event(t, X),'MaxStep', 1e-5);
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
    [t, x, te, xe, ie] = ode113(@(t, X) baleka_dynamics(t, X), tspan, X0, options); %[output:7f72cff7]

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
toc %[output:0adc2265]

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
plot(t_total,x_total(:,1)); %[output:3a76391a]
hold on %[output:3a76391a]
plot(t_total,x_total(:,2)); %[output:3a76391a]
plot(t_total,x_total(:,3)); %[output:3a76391a]
plot(t_total,x_total(:,4)); %[output:3a76391a]
plot(t_total,x_total(:,5)); %[output:3a76391a]
plot(t_total,x_total(:,6)); %[output:3a76391a]
legend('x','y','th1','th2', 'th3', 'th4'); %[output:3a76391a]
title("Generalised Coordinates"); %[output:3a76391a]
xlabel('time (s)'); %[output:3a76391a]
hold off %[output:3a76391a]

% display derivative state of all variables
plot(t_total,x_total(:,7)); %[output:25c9ac31]
hold on %[output:25c9ac31]
plot(t_total,x_total(:,8)); %[output:25c9ac31]
plot(t_total,x_total(:,9)); %[output:25c9ac31]
plot(t_total,x_total(:,10)); %[output:25c9ac31]
plot(t_total,x_total(:,11)); %[output:25c9ac31]
plot(t_total,x_total(:,12)); %[output:25c9ac31]
legend('dx','dy','dth1','dth2', 'dth3', 'dth4'); %[output:25c9ac31]
title("Derivative of Generalised Coordinates"); %[output:25c9ac31]
xlabel('time (s)'); %[output:25c9ac31]
hold off %[output:25c9ac31]
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
plot(t_total, contact_total(1, :)) %[output:3627f8b6]
hold on %[output:3627f8b6]
plot(t_total,contact_total(2, :)); %[output:3627f8b6]
legend('Front contact', 'Back contact'); %[output:3627f8b6]
title("Contact bool"); %[output:3627f8b6]
xlabel('time (s)'); %[output:3627f8b6]
hold off %[output:3627f8b6]

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
if Animate ==1 %[output:group:8b486b04]
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


    fig = figure; %[output:8ecc8a8b]
    
    ax = axes('Parent',fig); %[output:8ecc8a8b]
    
    body = animatedline(ax,'Color','g','LineWidth',1, "Marker", "*"); %[output:8ecc8a8b]
    f_u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:8ecc8a8b]
    f_u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:8ecc8a8b]
    f_l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:8ecc8a8b]
    f_l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:8ecc8a8b]
    f_foot = animatedline(ax,'Color','r','LineWidth',0.5); %[output:8ecc8a8b]

    b_u_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:8ecc8a8b]
    b_u_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:8ecc8a8b]
    b_l_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:8ecc8a8b]
    b_l_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:8ecc8a8b]
    b_foot = animatedline(ax,'Color','b','LineWidth',0.5); %[output:8ecc8a8b]
    
    axes(ax); %[output:8ecc8a8b]
    
    axis equal; %[output:8ecc8a8b]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) 0 1.5]); %[output:8ecc8a8b]
    hold on; %[output:8ecc8a8b]
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
        addpoints(body,x_sim(i),y_sim(i)); %[output:8ecc8a8b]

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
    toc %[output:6b248595]
    hold off; %[output:8ecc8a8b]
    clear fps body f_foot f_l_l f_l_r f_u_l f_u_r ax fig l1 l2 l3 l4 l5 b_l_l b_l_r b_u_l b_u_r
end %[output:group:8b486b04]

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
        if (foot_f(4)<-1e-2) %check which way round??
            value = [0;1];
            return
        end
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
       if (foot_b(4)<-1e-2) %check which way round??
            value = [1;0];
            return
       end


   end
   value = [1;1];

end
%%
clear Damping dph2 X0 dph1 dq dq0 dq_post dx dy dt dth1 dth2 ph1 ph2 q q0 th1 th2 th3

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":38.3}
%---
%[output:7f72cff7]
%   data: {"dataType":"text","outputData":{"text":"Time = 2.8660","truncated":false}}
%---
%[output:0adc2265]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 116.745746 seconds.\n","truncated":false}}
%---
%[output:3a76391a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWUAAADXCAYAAADC8EBxAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfX2QVsWZbysyRBkhfjGZATQQDDEfJGGCqBiiG63KIjvKrGLk7l2ogtkbCsneuARyCcGSnWUXltWtxVm5QTawfxDFjZgJMbmr2RCyKiNh3GgSV0URgZkaQI0wAoKYW79+eV767Tkffc7pc06fd55TRTEzpz+e\/nX3r59++unnnHXhhRf+QfDDCDACjAAj4AQCZzEpO9EPLAQjwAgwAhIBJmUeCIwAI8AIOIQAk7JDnWEqysSJE0Vra6uora0tZ+nq6hILFiwQu3fvNi0ms3Tr168Xo0aNEm1tbWLPnj1S9sOHD1uXd\/78+WLGjBli27ZtYtGiRZ7tmz59upg3b56oqakpv9+4caNYvXp1ZngAi1WrVokhQ4aIJUuWyHrTwiSzRnFF1hBgUrYGZTYFEfG89NJLYtasWbJSImn8jEne0dGRjTCGtbhCyl6kvWLFCjF58uRAIjdspnGytEkZ42Hu3LninnvucXKRNgaqnyZkUi5QxxP5pqFlpgmDSsqbNm1KraogTZmw279\/f3kxS02QkIJ1Ura5iBZ1jOTVFy7Wy6TsYq\/4yGSyPaesNPEbGhrkn1TNmrTDHTt2iM9+9rNyK6+bP0CkY8eOlXnVdzTpT5w4Id\/BhAKzRF1dnTQd0KPWF6Ypq3Uhv2pO0M0N6ju1jZDn17\/+tZgwYYKn1hsFO71OtS2QL+g9vTt06JA0T9Du5corryzj8\/rrr0vM\/cwXaANMLMAd6dCHaB9wpkWN+pDwhslmzZo10ixCfa7m8etP5A\/Cv0DTo2pEZVIuUFfSRAyzgeqa2GWXXSYn+fbt26WtlcohstHL9SJR0jCJlEEWRBJERHr5JGcQKU+dOlWSFdJu2bKlwtZKcsNODlMN5LzqqqvK9apyU14QkpdN2RQ7agvVqWueRJhh70GIZErSy6A29\/b2etqUqQ4qQ8eBFhgTfCFnUH+qZen429TgCzTNcheVSTn3LjAXQCcWXRsmLbOnp0eSMBEHpcN7HAZ+7Wtfk3ZUmtSqFvnoo4\/2OXTCpB4+fHjooRSVQy0yIQ0iKF2bx+96e1XyJ62QtE0QSJA2bErKOvGTHLQYNDY2ViwM+nv8rmKP33W5wmzKYcRPh7mqtkxasX6QOmzYsMD+VDV4fUdgPjI5pU0EmJRtoplyWaakQ6SsehhANNLMmpubQ0lZ9exAXr9JD4IgskQ6aM8gLpX0gzRl5FG33PidzCW0eOiwgjweeOCBPmQThI+p+cLL\/q0SOtpGniS6KQGLkL4gei0uSUlZ17RxqOfn3UKk7NefMGH54e+iJ0\/KU8yJ4pmUnegGMyF0jVedNCpxeBGDWoOuNYZpymper4MkvTwTc4jXYaVqq4UJAo9K7qocXodlNg76iqAp33333RULQ9Ci56Up+402HX8\/t0Kz0cqp4iLApBwXuZzyebnEqWYDU9usn\/kCEzHqJFfdysj8Ac3MxHzhZ0rx0jqDyD\/MpuxlRlC1WLJD27Ipk+kIdcS1KfvZrUHKOIRVMaJdCmm++J381oP6Mwj\/LH23c5pOTlbLpOxktwQL5XV5BDn8PBNUkwAmepCmTNqReiJPZg\/Ybb00Zd22De8CHLjRwV+Y94WXJwHJoXs66Id4JGeY9wUhanJ5xIb3hUrKqFst09T7wo+UdZMEzDlE0iBSFU91YSRvGrU\/1YWJMAq6fFPA6VI4kZ0kZZUQeIAUbkyxwIwAI5AAAedIGav8mDFj5NYLGsHSpUvFhg0byv6ZCdrKWRkBRoARcB4Bp0gZ2+Dly5eLzZs3Mwk7P3RYQEaAEUgDAadIGfbKhQsXioMHD8qbZnjYfJFGt3OZjAAj4CoCzpEyomXR7TEcjrS0tIi1a9f20Zzr6+tdxZTlYgQYgYIh0N3d7YzEzpGyakOmU\/1du3ZVhGIEIeMK6\/jx450BkgVhBBiB4iLQ2dkpLyO5QM5OkTKR8NatW2V8Wz9SBhnj5pgrIJoOxc9\/\/vNizpw5LLcpYAnTMd4JAYyYveh446Yrk7JHp8P7AhHHEIAGlyJuvfXWiuhYyEKk7AqIpmMXGv6UKVPEunXrTLM4kY7lzrYbGO9s8XaNT5zSlKkrVD9lr4horoGY7RDi2hgBRsAmAq7xiZOkHAa4ayCGycvvGQFGwF0EXOMTJmV3xwpLxggwAhkgwKRsAWTXQLTQJC6CEWAEckLANT5hTTmngcDVMgJpIMD++5WomnhTMClbGImugWihSVwEI5AYAfbf7wuhif+xa3zCmnLiqcAFMAJuIFBU\/\/200CO\/6TDXWSZlCz3gGogWmsRFMAKJEeB5UQmhKR6m6RJ3kGEBrCkbAsXJGAHXEXCNXPLGyxQP03RZtYdJOSukuR5GIGUEXCOXlJsbWrwpHqbpQiu0lIBJ2RKQXAwjkDcCrpFLUfBwDTcm5bxHDtfPCFhCwDVyidoshFfAExT3JkqZpniYpotSd5K0TMpJ0OO8jIBDCOjkcvZQt2OOf\/BOZQxj+vLQE088IW688cbEXyAyJVvTdFl1NZNyVkhzPYxAygjo5HLeF1vEeV\/8i5RrjV\/80V9+Vxz95dqKAuhr6fAvpi+ax63BlGxN08WVI2o+JuWoiHF6RsBRBLw05QFDGxyVVohT73QJXVvG14bmzZsndu\/eLc0YSR5TsjVNl0SWKHmZlKOgxWkZAYcRcI1cokKlfjgZccd37twpP3YR9zHFwzRdXDmi5mNSjooYp2cEHEXANXKJCtMPfvADQZ9+C\/o+p2m5pniYpjOtN2k6JuWkCHJ+RsARBFwjl7xhMcXDNF1W7WFSzgpprocRSBkB18gl5eaGFm+Kh2m60AotJWBStgQkF8MI5I2Aa+RSFDxcw41JOe+Rw\/UzApYQcI1cLDUrdjGmeJimiy1IxIxMyhEB4+SMgKsIuEYueeNkiodpuqzaw6ScFdIh9QyqrxFDxg8WNfU15ZQnuk+I97pPiMOd7zoipZkYlw4\/r5zwjf1HzTIVNBX12\/mNtWJQ\/UCB3\/Gg397rPine6zohqB9Lfz9Zfm+7ya6Ri+32RS3PFA\/TdFHrj5ueSVkIoZLIyIbzxKXDB5fxfGrHQZEmsWASX3zTBWJES52cyPTQ5FY7tjTRS5NdTvCuM+kx8UHeahlxB0XcfMBx4bwrxB23XFZRBOFHWO7df1S8sf9d8dSOQ3GrciIfFtHRS0eWMScShnCDGmrKJO3Vl34NSNJ\/Y\/7wcfFXH\/k\/IiyouxPgZSCEKdmapstAZFlFvyBlIt1JEy4RI4eDdEuaHH5XCdkL9KeePSSaZm1LpT8wWa94YLQs++CWt8X+B3v61IM00MDOH18r39Fklz+f1spU7ey1ZXtz0axBxovmXSEXsO8\/tkc89ezBclsmXXmJ\/PnaCZeISVde3KeN6qIHwqbnjS7vHYLpIvnQY3tSW1BByFc88DFxuLNXvLZsX+hieKavBpbb50XW6k4p6qD7VP2nxT1TWsukPOL8AVGLyDT9viOnKupDQKKenh55vRp+yjNnzhTLli0THR0dseQyJVvTdLGEiJHJaVJWo0apbQsDEUQLwgUBXNowWP4PcsX\/mNBSUzs94fF3PNDc8OztghZXIgaU89wTXxFNM7dZ1+owIUcvHSGJ9b9u+e8YXdc3CwjeZnmmQpF2DDJe2fZiaDbgCoypn7BQ0qMukug7r0dN71cZlXPRJx8NlSdqAiLkfWt7PBfSqOXZSq\/Pi2+MHyzuaiwt5i4+9+7sFfcpprn58+eLxsZGeb16xYoVoq6uLtFV6zCeIExM02WFobOkjA6aMWOGeOmll\/p0jA4iJiDMDtDIVG0MEx9bZhBv3O0ySBll3Ll4p7U+SYOQIRyRBbTlgz9+25q8QQVFJeRMhDpdyZu\/a5Zau+2+w+IHUxFwdunR5wU05ZG17mrLe3tPCVVbRjCihQsXipUrV4q5c+fyNWuXBhd1DmQ6cuSIJynfvfbL4qwXStsa0oDxM5EwJqON5\/7ljVLr\/vyNP7VRnCxj+Jw6ccnUC8SLc18L3fZGrZTMISg77Qe2Y+Czou1FIw05bXn08kk+mzudvHYjJti5pvGZyKynwe4YwYjGjRsnyTmu6QLlmuJhmi5Oe+LkcVJTJtsSti949GhRAHHgA++L77w9Rmz70Yslc0RKB0c0sUHKprbMoI5Ie+tL5b8499VUbMtk4x45fLDYtuJa65ponEEclKd9\/WRpqrKhLV9y0wXyYC8tbJO23TVyidMe2JJvv\/12T2UsankmeNTX1wv8a2trc+aA1DlSRqdMmzZNLF68WNx9992+pDzpnyeIXWe9LBq+O1qsW7cuan9FSo9tMCZ1Uu2bzBaoPE1NFtocPAGCttdk8iEgTD0hiJiQb97bF4g\/\/8ovImGZdeJJEy4W7RsmJz4XoENZF80WhKkJCWWNf9T6EClu1apVYuvWrYkixJlqyrNnzxZz5syRYrriteIUKauh+zZt2iSCDvr+um2ZWHb2Ykk8adtPbWlbZLZI20MCxDm8pa6PeQRE\/NVbLiu7rOmeJ+Q5QR4Q5CmBAzccroGYvnVsn\/j4gXPEC+8fE6\/s\/L1zdlUvEkD\/4UniRQMNGbuQNExOUYnLL301kLINr4soixS0ZIQJBTEzKXuMLApwXVNz5gIFkumHfTT4vt2zQHR3d6eqdaJ+cvcyPclXLxQgPyYz+Z8e3vluKif2uub7\/v0N4uS6N8WhH79d9kSBKYYOP7+\/eY\/0NKGHDkrh1oaHTDXkqYLD0oGzLxTPnzomBv\/NIfHysFOZb+VVFzK4CeIJcis70tkr09x06cXS9h1mW6Z+u3jqBeVy6eIOFrosFIAkBF10UqbD\/W3btiX+6oipphwlXZK+iZLXKU1ZFzzMJe5\/rZstzm05R7qUJXG6DwPM1K6sXgRBmeptPLrZZUurJ3cyyObl+3vv8R7x5Mkj4vHzx5QJ1tRlzQsPtO1zj31CqG5g+N3Wdp7IlW41wh+7RLqlW3KmFzD8LuDUnX2OxEF9r\/oX0y4GdcpLOKcv5pAcR3b2pr4jCxuH+s4GC6n6gJRnzLrHGY0vrD1pvzddpEzTpS0vlV9oUsZ2o+6BodaIIQj0MLsyETK8KnARBBqq7YWCzA\/k9kc+1\/8p3f4OVmi+xz43SGqyF37vsPjJmtcTjyevA0SyL8dZFOnA8OKpF0riHTK+VuKFv+s3F0GI9BCmpteVSfuF+WXW3MvFP25+tUy4uEWJyx+kcftd4EkMXkgBtMvBTVK63EQ+2uplpzA5jr4\/Uux796tMyqeBMiVb03Rh+Nt67zQp+zVSBfH98ccl+cQhhiggBtmVMfFhwwVx7V\/bY12jIjKmG3Nw+4P5IexwzqYm6+dqF7UOwgqEru4mQLxpx\/nQF1aSAeYK9FsWMUa8LjbRONRNRvg7Lbxn0py55aian\/D+4rpPOOVFEGV+IS1cYZcuXSo2bNggL440NTWJJUuWeLrFmRwImpKtabqo7YmbvvCkDJvyxI5xFdvquGAE5SO78qg\/+399Jm9aB3g6GUc1P2CxAvF0THw+ESRkuvByBTN1wYMcID\/SiNPaTQQ1NM6BLcXzgK963EtEXqYm\/WKTepM0bme5Ri5R26F+AiqIlOmL17W1tWLjxo2+XhqmeJimi9qeuOmrgpRBitiKpqktw3Z7yz2Xi3uPH5BYU10mfsdwy8LWlK57m7jWIc\/9y78g64pKxjQYiExNDqjInOClLYZdSKFFyeu6uErGsKcf2vJWJhqp14SgswHTA1uQ6Q\/XT5Z+8DARYacSxTWSyBiHjCoJm\/R\/nAntGrlEbQO+0dfQ0CC6urrEc889J770pS+Jw4cPl\/+2YMECWSRcZR9\/\/HHR0tIi2tvbmZSjAp1Geq\/Bh200tr9p+f8S+YJYcAiERQCaI0UJ86tXJVdoWuQB4XdDUNWO4fFw57d\/lejSionPMpE3+go2VtVDxGQXoF4bhwYMrwcEUBrSOFhqxigzK\/NA2HjDtfk7F\/8q1PSDcnQfZ1OfZ68dTprBkajNXuEHwvDI871+GUvXlBFmAZrws88+K1pbWysImLRlJuU8e1Cp24uUTbfRcZpApHXDwPNF\/U+PS20JZAXSweNHyKRpqdte+hvy6eQAwoaZJIl2rLcv7DBOvdACbwSyjaMcLHIgVVNNW81bIvh3c9WMvfo6SiwTL\/9m\/A07Hi\/3Ot0PPO4OJ84YRR59XpDJLW55aefTr+f7mS8OHDjQ50IJk3LavROxfL9tGjn42zxso5tcEHHO7wZLTdd0++sXNwOT9\/6\/+YKc3LSVpWh2NJFtXOkmWINs7l5Xh8mTBO5gUc0N5Lpm2\/Mk4hDxTY4+MelDihDoFdeDiJnekZmCFlQswoiWZ7MPTdrvF6jLJG8eaXQ7OpNyqReqwqasDijaboMUDm15O9ATwsT3lcJrQhtunlgvLyGYxMGgSR1kgwQ5qC5PJh4VcSYP2dz1A78iXB2O096gPKY+52R28jN10K6GfIfVSzlhXjG220TlFd2mzKRcpaSMZtEXIby0NhMiVicNyF29Fh3mr0x5adLajC6XZDJDW9bNEGlGq0sia9p5TfrQ5Gq2eosyLyJWsSo6KZNJAod7dNAHlzg2X6Q9IyyUbzr46OKA\/t07nXSDRJKf+FE+02Rqk4zjfmUBGt8iyD2O3Nq8builWb9LZYf1TZofN0gTB9N5kaYMLpVtiodpuqzaVnXmi7SBg\/kCT1AoSNNT+rRl1cunWMA4gIMtGZ4krgVqzwKTsFgmFLjflV2OKSaukYup3GmlM8XDNF1acurlMilHRNrE1xWaGGzFrk1q9QAPbn1e3wSMCEchkwfZlUlLtv3FkiyAco1csmhzUB2meJimy6o9TMoxkA6ySZoc8MWokrNYRgBmKHhI6Bc56CzA1JfZsliJinONXBI1xkJmUzxM01kQyagIJmUjmCoTBdmV0\/h8VAwROUsIAugnuCGqMZa9\/MqLBKRr5JI3dqZ4mKbLqj1MyjGQ9vN1LfLWNwYMhc7iZcIgLfnmWdsy9zG2AaZr5BK1TSYBiSgQEa5j4\/H6sDLVa4qHabqo7Ymbnkk5BnJ+NknWkmOAmWMWdcdTDQuqa+QStWtNAhKtWLFCRpDDdzvJha6zs9MzKL4pHqbporYnbnom5ZjI6SYM2vp62SljVsHZUkaAvDDgSUMXeVw7nI0CgU4uw84dGSV75mkPHNtbUadJQCJ86Vp96CPLixYt6iO\/KdmapssKICblmEjThKZJTNHEknwHLqYonC0BAnRlGsGfit53OrncfvkCcfvl30yATrpZH37l78XDr6wqVxIlIBEyqeYOfNNTf0zJ1jRdumicKZ1JOQHS0JbVp6i2yAQQVEVW+JW7cCMvKZhemvIlDmvLB4\/tFaq2HOWaNQe5TzpaLOd3ZWWjqGBoXhahGS3DyMVVGQKuzIu4sJqScpiGTPWb4mGaLm67ouZjTTkqYpyeEXAUAdfIJSpMJqSM2MoLFy4UK1eu9PxMlFqnKR6m6aK2J256JuW4yHE+RsAxBFwjl6jwmAQkamxsFGPHjq0oetu2bex9ERVs2+mLPvhs48HlMQJnD60XE66\/Sfzjotn8NevTw8GUJ0zTZTXKWFPOCmmuhxGwjACIeOBljWLgpY3iQ+Omis9c9Aex\/JpTTMpMypZHmkFxrq1sBiJzEkbACgIg4gFDG8SgcVMlEeM59U63eO\/5H4lPntMlVi9fwqTMpGxlrJULmT9\/vsAHE+nx+oQ4k7JdzLk09xEAGZ\/3xb\/oQ8Qn93SKk2\/slA3geVHZj6Z4mKbLapQ4Zb6AoV89WQVBNzU1CXx9oKOjo4yJayBm1VlcT\/9CAEQMbXjQuD8RA4bWlzVilYhVRIo+L0xiX6C9uMVHh31+h3xRFinXcHOKlPUp5\/fFWtdA7F9Uwa1NEwEiYtiJYS+GaeLknp3ivee3lDViv\/qLPi9MYl9AUbvuuuvEggULxIQJE0RLS4tYu3at4Bt9aY5KpWy1k1TQafDNmzdPIBgJP4xAkRFIQsTVpClHjX0Bfpg5c6ZYtmyZp8+yySJVX18v8K+trc0ZW7yzmjJdo9y1a1cfH0QCGwPywQcfFOvWrSvynGTZ+yEC5Dnxoc9MjawRm2rKUT8SnHU3qN++RN1RYl+QCSOp+WL27Nlizpw5sunNzc2iu7s7axj61OckKRMhHzlyRIbo0x8i5dbWVqkpuwBk7j3JAjiPgB8R46AO5omkj64Z4mvlI1rqkhabWv59a3sqPklmcqNv9erVZXmCFDckMtWUp0yZIomZSdmnq8NipJqCndpI4oIZgQgIpE3EQeYLaMqD6gdGkDbbpPqX4qOSMqSFxownSHkLI1sT8s4SGac05bCVj4BxDcQsO4zrch8B\/VIHDus++H2XOP7CFisasR8CRZ8XJqSMtuOqtRrkvr29XagadFSecA03p0gZnYLDu5qamopxp\/squwai+zTBEqaNgE7EqI8udRx\/fov44J30bZVFnxcmsS9AvuwSl\/ZojlF+0QdfjCZzFgcRcIGIg8wXDkKWqUimPGGaLivhndKUTRvtGoimcnO64iMQRMR+lzqyajXPi0qkTfEwTZdVPzIpZ4W0w\/UQ0eDWGIgFD13ddVjszETT\/YhV00TeRMyasv8wMCVb03RZDTgm5ayQdrAeGdRG8ZOFiCBmlXRKdtHk7loONj9QJK8rzrhZZ8t9LQ08XCOXNNoYpUxTPEzTRak7SVom5SToFTCv6qJ19ocbPK\/w6oREJA0PApBSyZugdHBVbRo1rjdjscIVZzyIvuaSNhw05Fwjl7ynhykepumyag+TclZI51yPTrTwCDCJp0D5QMTQqvEQYVGT8A6PStql30t\/P\/VOVybeB3Eh9gv8c\/SXa+MWmUs+18glKgimAYmo3BUrVogxY8bIOBi7d+\/uU50pHqbporYnbnom5bjIFSQfEQ7CPkYJbhPWPJSrEjTMHtAyvUjbi7yltv1O12nSzl7zVu3E2DHgMQ38E4ZNXu9dI5eoOJgEJKIyyX320KFDTMpRgU4jfdEHXxqY6GXqZIxteFb+siQLETeCsp\/9YQRnLxH52UMb5M8gQ\/qbLr+qfXsReFztW41LDBImm3k1mGH0eTF4xKVZDLXYdby7742KvKYBiXDJbPny5eLgwYNi+PDhTMqxe8BiRiblSjBVrZU0Vgr7CDIuwjY8LQInpKDFS2xOh8MsCi5Rpo0+Lz7zjYXiM99YFKWITNO+cN8K8cJ9K8t1mgYkgtkCz759+8phPNl8kWnX9a2sP5Kybi4gktG1TTJRuOwlkHT40CeRpNZtoIHTNedq0oq9MPTSlAePGJkU7tTyv7tvr1C1ZZNr1j09PWLatGli8eLFYurUqUzKqfVOxIKrlZR1jZe2+V4HazhUoy0+CBiHatWwBY84FDi5gkDR54UJKY8YMUJMnjy5ot+7uro8TRimeJimy2qw8UFfBkjrmh2qlDZVH9sqaXZIJzXf099gY+LNoLMKXIVr5BIVShNSVgMPqV8hYfNFVLQtp3d98BEJn\/fFFtnyIBcyXdtFetZ4LQ+YflKc6\/MirBtMAxJROUzKYYhm+N7VweflfkamhbjeAhnCylUVHAFX50VesJriYZouq3YU1nzxvf\/9p2LdugfFo50lt5p9R05lhVlFPV7fV6vGk\/1cwOVKIyHgGrlEEj6FxKZ4mKZLQUTPIgtLyj9s3N+nQSDmvb2nxDNdJ+S77d0nxTPdpZ9tPml8X82mfFxW\/0SAyAUxyfkTacL4g6hMyhbmiwrigN4DYmTtADHi\/AFiRO3ZYiT+P3+AuLr+TKB8kPWml4\/Jmu\/rfDe2BGndjostUMEyol+or\/xEp4U1r52PbUjRZjwm7Ua6JEoEvsq8ZMkS+W06fkoI4BueWKSCHiZlC6PFFEQiZ5D11Q01ZaLGwIc2\/cgrx43MHmpsBIjP5gn\/TlSJ9+r6gX0WyCjdT8SMBXVf7weyr5KQVpS6o6a12W7Ure76ouz4QMz4l+czonaAuLphoJxv+Fm2p\/eU2HvklPz\/ma6T8v8kD\/lf3zBhjCwG9anPk8++Kp7c8arcMYTtGkz5JIm8UfIW1nzR1tYW+euzNHGuqh8o7mqsLQ2W01q0F0FTxLAPjZta\/rRPEW7HoV3Dzj1zaeDAsb1RxkSktLTw6QSsarzPdJ8sYw3zkp8WTFolEkOzRD+BjG\/7+If67HzuPb3jeeTlY0K\/TkwTVr+cEKlhIYnVsaQu+NQ2ufAbthtVqRo12l0imjOKhDpW8XOSHZ9NHKgsGgfUV+qCGmVRUWVDv6Iva0deKoZdNUn2My6b1F1d+jnoee2R74vtf3WnUVOZlI1gCk5kC0QMpNsu\/1AFQWOy\/7juz8SgcX8ifYnpO2tpkrFORnrrg4gM5Hv9iNvFpy6cJIadN7KCjKkckPKBo3vFw6\/8vfjtW08n7gHCjUhDJyIQUlLzA006TEC5yFw1SZqmxl7xUfH20OHyb29\/uPS\/yYPJ3Lt3rziw\/T9l8tceeUhObnowicMedQG67ePnyuSkvYOA09Lk1QUAGKh1YxfhR9BpL8w6EVO\/AwssllEf9Dn6hAi4duTIMhGjrJ5nnpJFHthe+r93b+mQH4uv\/uhxNYJkscUnUdvrl75facp+IMA88T9uvFZqZk3Hfya6BtSJ9kFfFn+7\/iFrISeJeLGlI60yzM7oJS8m\/qu\/rxPPHZgqLh96myRhkO5v33xa\/Oat0mClB0SM59MXXSOuH\/FVmRbE\/PArqzyhIIIfdu6l4lMXXVMi8TefluXrhKSSUZwJqApAGtHo2+6Qk1Aly9KkKxEq\/Uz\/YzJiezzlvIMSU3q+9x8vi0dePi7zUdmkXanl4z0mOt6h\/J\/d3uSJC\/oMGiDIkMwnMH\/F1QBNJy+R6iXnjpQLLj3jh31U3Dz6Y+XfYRY4cWq4NA\/Qgz7Xn9+8+XTFwkxjh9JRfbdf\/k250OOhXdbP9z0kf0cZ75xsdsMyAAAYIklEQVTskMqMuihDmYkzDtAfo2\/7qhh21bWyH4hM0S8gXxAvkbApblHTMSlHRcwjvQ0Q\/TwoLvqvdaJ52NtSeybTRtytImmUn7hilPjEFR+VLan5zXb5f3lyK1tc\/B1asf6AvDExv\/LRL4rLh94qjrw3QU6Wzw3bIoYM2iHOH\/SrsqzI6yfv7ZcvEJhwyLv611+XWjOR7aW1XxeTPnKXGDSgS5w\/aIeoGdAl3jx6s3jvVINMP7Hhu+L4qUel3CC8JLZdlYTVrSiRJE1GSQqntaKwYUNYT\/\/4ubJNQX1H9ZN5A5rZDZvaJUETMRMud40fHFpemGxB74kIsQh++sJJsp\/xeJGqSpLy59OLLn4edM7+ChPPg7\/ZKscSpQGpXz\/8q7Jc9KeqRXuR92\/fekqmwwIg5blwkhhaM7GcFOMEYwNj5cbNpTjbpg8tjKNuvaNMxMAeuxXT\/jatyySdDT4xqcc0Tb\/SlP2I2Ct4j27auHdnr9HBoLqlAxn\/atwt4skvnTn9fXJ6U+SBB60FZIoHmut\/7HuobIbwO8z0cg9Efkz6aaPvkxMME6vmnJJrIYj+lXceEaMu+E557GAx+PfXPyvqzl1R1sgxyX++\/yE52aOaQjAZEbkMGjEeImHbk1HvO1pIgjQ5yHbz089JmU78dJNY+nIpwP3zAz4ifnFRo\/jhoz+vCJ5jOsH0dCBDlYCJfEkjRf\/ioV0PcD4I85PBuYDabtWmqy7Sn7rwmjLhQ+sFWUv7\/Wn79aAB++Ui7\/WAhFHuj179Y\/k\/ZJo\/7p+k9ry0Y5oRJFgAEbkOpok8iVgVlknZoOvWr18vxo4dK06cOCFwoLdp06aKXFFA1L8+THEljr9Q+vJG2ONld4Yd7yfHLhEnPn21JBk6gMBk\/lbNC+KCd7rEa5dNEE9Mnif++8XXpR0TIQq\/\/HC7HIw\/vObzYdXK95jAtJX8+b6H5dYzbHLq9kfdPRDlgqRQNibXifdLdlmV6L2EU00btJ3+2s+\/YNQOlYxBeiBh2HSj2P2MKtISeWnPaDu2+arpgTC6YcLHxMCvTBc7P3tLhb2azCBxFlTqRxAxaar4m2pyirPAheGhmlyQVnULxeEpHq\/DWfJ0ofLVswGv3RFI\/q+v2iy+s31a4CJNZgoQMsgYh3Bp938YRvQ+Cp+YlpkknXOasvqJF4Tma2pqkr6XHR0d5XaGgUixJ+SHQceVtlZ0YJck0LtK0CBcTN7GXz8mPrbnWfHY6CbRc+2flmUkLRBkTIOPtDE9jqwfCS67arN8RaaGJB2tHiYmPYQDQa+5\/lcCC8Xq578eKBYWLdgLsRg9c9edkXcJSdqs5tUPJ73KBS6wjb4woE68NXS4PPUnmyYWVJhaNl56kbFItJCRySjJLsO4Uo+FqeSaVukWqh\/OxrEHU1XLJpbGqZ+2TISMHRIWZDWGctx22cwXxic26zIpyzlShpaMmKmLFi0SFKCkvb1dqNGh\/EAkrRifPlI9J2x\/+BKD66p\/uF+8NHtahY2YPAbgxuNnG6PA49CW\/TQFmCuibgtNOttmGpIR2rKf9k6mCtcmorpA0SUjE8+JGW+8KUklzNVKJ2MckmEBC9vl2OyfLMsibRkLNNqpPzQO8lyUg\/BgUg5AB595WbVqldi6daskYfp9165dkqTpUUHsOSqkNqy7sNkmYtRNBxQgZJPJ6dfUoMkdRQvNcuJ51RWkIblKyEkwo8U4yIyB\/qMdDsjYz9MliRwu5sVYgH1aN2nROIB2bOJ2mEfbmJQDUNc1Yz9SHv6J8aL5Ww+If33iufLnfdL+6CUdUGALq57SxxlEfpNbndCm9to49dvK42dPpPaZmGlsyWJSju4CZpJHTxN0LkALKjTipdunVa1m7Ifbo1N6pDmLtOUo5ro4fWEjD92AjHMZzUb9XmU4Zb6Ioikfb14jPnPRH+Rh3WMP\/HVa+EjtGH6UdEABorHhtgNtG2WrvrGkbRRpQsPMAlNG8+N1sg9oIibZSaTRmao7IDwcwmzhfjKgfTds+mH5sIrSqYRchAU1DYxpLJBJK+rBdhoyhZU5e\/ZsMWfOHJmsubk59Ep2WHk23jtFymhQFJtya2urDDjidbddvzSgguVly\/W6FYQ80JChHdu2i6Lcq+8tmUGwtSMbbdgpto1Ot10GNCRyi8JiA7xMPUxsy+JVHhEyvFfw4ODNz\/5pIg\/5NZMZo2g7HJM2xk2DA2AcaN5\/1irp+x3XYyVu\/VHzQVOeMmWKJGYmZR\/0knpfqJotqlBvg6lVwhuAnqB79Mj\/5PSbU3HfoUO\/52fNE3f+YYH0QY6rwUUdjDbTk5b4L2+tEkM2fUsehLliPyTZ1JuMRNJBh5Rh+GDxgZkGpHP9m1fLG5NF2uGEtS\/uezJp\/eKuAXKh9rslGbf8NPKxTdkA1bh+yuQPS5qtLV9YcvgvXVP2vqJs0CzPJDS5P7nlA\/H9JTdHvpARt17b+UB0o1sXid+N3OeMloz+unPcP3keQEG7N3HpC8JJ7bufLb\/T0\/PANs6m5dGhNM4\/svYH\/qObvik+8sC3xAd3bRQP\/dt8U5FzS8ekbAF6LxAxCDFJoAHb1GzJrEDuTNiamd5eMmkqiOPr\/9Apfjf1bDl5kppJ9CvEJjLYSEO25C9sOCV2P\/KQExp\/kEmItGWyhcfFAH136LbLyn2HckpEuDdzMlR3fridSE\/WOxeyJc9sbYh02y9uHyTNx6ScFEEhZBBv\/bTUli+kGgQGouK2EmlUpreXojSRiGPF\/r8UZ\/35tRVXkBEkB0Stazqq\/ZsiZaFO7BBwIInHBsFHaQfhf3j630mbLQU\/QhnYxka9kh2lbq+0dBsStw\/9FlHdWyBqnTQe0HdH\/wjhJUtBdcIeP82Vgi6F5af3fuWQhxDImExkWdl26SAU5yTn\/myPnD9nxsBTuYyFMDyZlMMQMnivgxjF9YZMEajGJACMvsUNu71kIH5FEr082nbiAkpJ6y3FjSUbeJD9uxRZq3Sl2yvITlTZoqSHdgSSABGo8R0o2hiVpV6gUAPqIL4DngPHSuEY1Sfo0oVahpoHMSWwOAQdnOqeI1Hai7ReY0G9QIQbgfpDsZ696grq2yjpd\/8brrGfCUWKvsGThX0XdnYsBOpBL3YlCC2rB1lSd5+Qz2sM+I0XFQ\/KZzpmdCyZlKOOfI\/0Ooh+J\/4gB2hKGAz6oKAYw9ShagAYqtIrEIxNbZkOoZJ4XKgTWdecdC8BC9B7FhFUjx5+Uo1OhhCheCgyGhWuhqkMimYW1p6gMKXIG3YTLah8G30XJr+t91mNA8gbtgDQnFRjf+vjgPo\/Sd9HOS9gUrYw0lQQT102usL1xktLsx38BRoWAswk9UeFBoFT+6TlBEEaJ2ZD1C4i00UebnBBE9fkWjO0XSy+Ub1esui7qP0QNg7S1pZpx5qWqcSvr\/VFnbRuk\/5HWiZlCyNNBfGyBd+WdrxnvtwkfX0pmDs6BNdc07BnJtGw1ObHJYSoEOJad5q360D8ZDaJKlve6U1ieOgyFukqPMluckU8aV\/kuTgnkZ1JOQl6p\/MSiP\/z638prvzBTwT5+eJ1VsFfktojbZpBwiA1CYIUVobf+yy3xnFlDMuHCw9RfMSz7Lsw2aO8h0cGhc2Mks80bZjpwrScrNMxKVtAnEBcubtLXHTFl8SUb5cCbmftvB91MqtNz3r7m9aELKp2pPZF1AXWlvnKwlSIVARpy0ERCiMVqCQm00XW7ndx5VXzMSlbQBEg\/u2a74oNHxkl4Bfb8a8rrV\/qMBEzydXorEwX+vbV9oQsqnak9m8UzdfrhqDJWHElTVqLc5F3TEzKFkYnQJy28Qdiz7HzxZHpf5cLIVMz4rjI5XVybxoP2LSL0j7YMZXDRjrTRTIKgduQy3YZacUmKfKOiUnZwigDiOPX\/bvoWfN98aP\/m+81zjjactamC4KcrgVH+XpGUHcVWTvS22V64JdX31mYNrKItMwMqp+6LVmzKodJ2QLSAHHxg\/8i5t\/0x06E2jPVspJo1xZgk0VAW7Zl9yuyduSFp8kNvzg7I1t9Z6sc2wRa9B0Tk7KFkeUaiKZaFpqet03S5oSsBnuyOhxxgBd0LTsvs5OFKVNRhO0DP9ox2T6vsN1uv\/Jc4xPn4imbdIRrIEJmU0+MvG2StiZkWttgk\/5PK03Y4kp9lyTkZ1qyRy2Xd0xnEHONT5iUo45mn\/SmblUu2CRtnMBXkz1Z7dKgxdWFvrM0XGVERf3LN3HLLvqOiUk5bs8r+VwDkUQrik3SxoFftdmTqQ+DFteoZwcWhnpqRdjeMaV5YzQ1EE4X7BqfsKZsscfDJm3e9mRqqo0rt0XXjvy63c+EUS32ZLXdvGMqocGkbIEEXQORmmRqk0wSFc4CfLKIpBMy7XgattoZpxwvE0Y12ZMJE94xMSnHmR+eeVwlZQgbZMJwySaZZEIW3QUqbCB6eWG41Hdh8pu+t2HCqIYdk2t8wuYL0xFsmC7IrcolH1fTCUnfPUTQ9APbn5IoVKs9OWjHE2aaMhweziWLsmPCmBl16x2yDfB1px1Xke3JbL6wNCRdW9nUZoXZJMMCr1uCyKgYE7copEHwfJAzIozh6xXVoB2FAaTueKrRnqyaMBD6NiwWNi3iIGD67BXiJl997\/3imbvuLC\/YYbi6+N41PmFNOYVRggmt243z9k\/2aiZMGKrWo6ehD2BiwpILHDQk5EsrkHkK3RGrSNWujIUWn5ZK82MEsYS0kMlkx0Tf3VPDfkLDxt+xYIcRugUxUy2CSdkCvK6BqDfJa6vrkumC5KUJ6RULw8sPmbav9C0+C13pbBFqeE4X+84mcGE7JvrunvqVeBoL0JzJrGVTpizLco1PWFNOoffJhEGfr8f2d5n8KvZDuUa082qq34TsDyaKoK5XA03hi8wueMykMFRlkUE7JmjDeF\/NCzGTcsjImj9\/vpgxY0Y51caNG8Xq1asrcrkGoleTVBOGi6YLktnroKfavStMyQ19iA9wgqCr4Wq1X7uj7phM8StKOtf4xClNeeLEiWLhwoVi5cqVoqOjQ4Cgm5qaxJIlS+Tv9LgGotfgU22SLrtTeWlJqi25KBMrDTnRh9jl4Ks21WhPVjEL2jEV9fuLpmPCNT5xipR1EEHSra2tor29vUJbdg1Er86HTRIPvpLsMimTvZAOa6ox0JDp5NTTod9wwOeSx0zctoTl6887Jtf4xGlSnj59umhpaRFr164VmzZtKpSmrNqVXT8oUrWkavdBDiMn9T3MTp++6BppwjD9XH2U8l1K63WZKK2vlLjUbsjCpGzYI6NGjRKrVq0Su3btEosWLfK0Kc+bN090dnYalphtMtW3FQdFLmtb0JJwOeS1Rx6q+kOdbEdBcWqDt43qc9xfdkz19fUC\/9ra2kRzc7MTH83IVVNWD\/V6e3vLtmMi5CNHjohZs2b1Gdm0suHFgw8+KNatW+fk6IdNEh4X2AK7fHpPdmUQ8w2b2qveB9nJweKAUKoJg0i66D7IYbDOnj1bzJkzRyZjUvZBi+zI0IB1DZmyECnD3ox03d3dYdjn8p58XVG5ywdFdPqOywG1I0cW\/jJALp1dBZWqh779xSUSWvKUKVMkMTMpewziIJOFmtw1G5DffCRXuN+8+bRY2jHN2WlLW1UIWPQ4Bs6CXADB1Nt90Jqr\/damruQxKXsMUhzswU5cU1NT8Vb3VS4KKaMROPA7cHSv+O1bTzs9LbFdRQyEF+5b6bScLFx6CNAtTpixTOJhpCdJtiW7xie52pTjQu8aiHHbwfkYAdcQgCdOf9sxucYnTMquzQqWhxHIEQG6OFT0yG9RIGRSjoKWT1rXQLTQJC6CEXACAZwv4EGsi\/7yuMYnrCn3l5HH7WQEGAFPBJiULQwM10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApW+gH10C00CQughFgBHJCwDU+YU05p4HA1TICjIAbCDApR+iH9evXy9SzZs2qyOUaiBGaxEkZAUbAMQRc4xNnNeX58+eLGTNmiJdeeolJ2bFBzOIwAtWEAJOyQW9OnDhRLFy4UKY8cuRI1ZByfX29mDJlinj88cdFd3e3ARJuJGG5s+0HxjtbvJmUDfCG2aKnp0fU1dUFmi\/mzZtXOHJra2sTLLfBILCQBOTGeFsA0rCIouPd3NzsBJ84Z76YPn26mDZtmli8eLG4++67PUkZnb9kyRKBFY4fRoARYASSItDZ2SmVJReeXEmZ7MYAore3V6xevVrccccdYvPmzWLTpk3C76AP6UHM+McPI8AIMAJJEYA50RWTYq6krAMJLRmrVU1NTcUrr8O+pJ3A+RkBRoARcBEBp0hZByhIU3YRTJaJEWAEGIGkCDApJ0WQ8zMCjAAjYBEBp0nZYju5KEaAEWAECoEAk3IhuomFZAQYgf6CQOFIWT0MdPUA0FRG2MzHjh1bHmvbtm0TixYtcnLske+4q\/IRaGFyuo75qFGjxKpVq0RDQ4NsUldXl1iwYIHYvXu3M+Miioyu4w1QVRldmIOFImXc9GttbRXt7e1iy5YtcvDu2rXLKSIzlZEG9tatW6UroMsPDVoXBmwQTmFyFgHzFStWyCZi8SN5XRvjpjIWAW+45TY2Nspbw+rczXNOFoqUoYHOnDlTLFu2THR0dAgMjjFjxjilSZjKiAGwdOlSsWHDBumT7eJDkwpX3fHglqWLmrKpnEXAXB8HLo5xUxmLiHfYTiuLeVooUsaqdt1115VJGL83NTXJ230gaRceUxl1n2wXt6kqni4MVpP+DZKzaJjT1hr\/65ESTbDIKo2f62rR8GZNOcaI0bUGF0nZVEZV9gMHDkhTjFfwpRgwpZKlGki5aJi7OL71wRUkY5HwxrydPHmyEzZ81pQtU5ipphxlcFsWMVZx1UDKRcIc4+jWW2+VAZVcNW9FlbEIi4wL5qJCkbKpvTYW61jKFFdGncwtiWOtmGolZdUcZg2shAWBGBBsyyWznJcdOaqMro9xtNGFhaNQpGzq2ZBwTiTKbiojJh5Ck8JW6Oope7XZlIuAuQukEDYBTGUsAt6qjK7Y8AtFygDN1Ac4bGCl+d5PRl3bVP0jXfW5JpyKqikXDXPdrxf4u3YIHCQjwu2qXjpFGOOqjC5gXThSTpNMuWxGgBFgBPJGgEk57x7g+hkBRoARUBBgUubhwAgwAoyAQwgwKTvUGSwKI8AIMAJMyjwGGAFGgBFwCAEmZYc6g0WpRMDU9coGbjiB37lzZ2BwKJM0NmThMvo3AkzK\/bv\/nWq9HnsgK1JWI4UFAQL5Fi5cKFauXOlMrBWnOpCFsYIAk7IVGLkQGwjkERAGF3eWL19e\/oJ6WDvUsJVhafk9IxAHASblOKhxHusI6IHTN27cKOugKIBz584VgwYNkl86RwD43t5esXbtWtHS0iJqa2vl7+q1ZAowgzKCLgTgos+0adPE4sWLy4Hkgy4TFOGqsPXO4QIzRYBJOVO4ubIw8wB9xABBxlXzBUgZxI0APTt27JBR9YYMGSKJGA\/ydXZ2ynjPutkj6Kvo+jVbnXT1G4FFjBHMo65YCDApF6u\/qlraIJsySBkPxRVWiVaPHaITqR4kSgVRTwtSnjFjhoCm7vX1iTxMLFXd6dy4PggwKfOgcAYBG6S8Zs2aim\/cUeNOnDjhGQbTK6YHETPlVQm6CJ84cqZDWZBYCDApx4KNM6WBgA1ShvkiSvCksLR6GE3WlNPoeS5TRYBJmceDMwjYImXdphwUn1i3KXt9OUaNucw2ZWeGS9UKwqRctV1bzIapX6Tet29fhfeFqU0Z6VTvCz\/TBdKFeV\/oedn7opjjqkhSMykXqbdYVusIRPVTDjN3WBeQC+x3CDAp97su5wbrCPCNPh4TLiHApOxSb7AsuSFgEtfCJE1uDeCKqwaB\/w+gKzQrup+i\/AAAAABJRU5ErkJggg==","height":215,"width":357}}
%---
%[output:25c9ac31]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWUAAADXCAYAAADC8EBxAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQucTVX7xx9kkHFpMMyYcY8QcheFlC5yySRKFwq9vJMuEl1EJEXSW0i5lN4uouJPkl6KlNxHbkluYcyYYcTMuI3b\/\/Nb03Oss2fvc\/Y+Z59z9plZ+\/PpE87aa6\/1rLW+61nPetazCkVFRV0m9SgJKAkoCSgJOEIChRSUHdEOqhBKAkoCSgJCAgrKqiMoCSgJKAk4SAIKyv80xvjx46lt27Z5mubzzz+nyZMn+9xknK+\/+egVYPbs2VS9enWaOnUqzZs3z+cyWn2xZcuWNHbsWIqMjKTs7GwaMWIErVu3Lk82KF+dOnVc\/56TkxP0spqtW8+ePSkxMZH2799Pffv2pUC1G8suMzOThg4dKr5n1Lay7FJSUjymN1tPK+m0MgiUTKyUqSCkVVDWQFmGJ3fCVatW0fDhw33qD3Z2ZICjQ4cONHDgQFGWUEHZW50YPCijDGyGdCAmKJ8aR3rJKVDGJDtx4kQqXbq0S3ZG8vS3zt7eDzSUkX9ycrJfSo+3OoTj7wrKHqCsN0BC1chaaISqHPiuNyiHarLwRybBkq83TRmybdWqlSNWFN7a2R95BzJvf8rlhHcVlD1AmQEkDxIevBEREeJNWetjGGGpWa1aNYKGjQdmEaRr2rQpVa5cOY8GxEtZpIWWFBsbK97j5f6BAwdc5gL8O5sMBg0aJMwXc+bMoY4dO4p3eEmshQxPMJz3rl27xDLd6JFND7LZQWvm0ebjDTra78nfkZfocj74PuSplbfVtnj\/\/fd15QvTjzdNWfstrTnBqB7yJIY\/b9myhWrWrEl65gsrSoBsQkK+euYNozZEer2+On\/+fDez1N69e6lRo0auPi6DdP369SIt6mHUPtoycr9t0aIF9e7d29UVuA95ak9v8ncCTO0qg4KyCSgzVNPS0vLYHWVg8yCQzR1yR8an0Bn598GDB4u\/M9jxPkMbabnTA7TNmzd3+7Y8sGBTBvC5nLCByxrXhg0b3JbEVatWFXmtXbtW1yyj1XStLGOtaJzyd3jiOXz4sJgseEBj8kP9+HeGmVYeWg1Try2syFeu8+LFi4X8eNLr3LmzWzt6qocR7PUganZC06bjyVw2eXhrQyP58B6FrAhw\/9SDslH7cJlYZnrtJfdXPTnx2OL+ayR\/u2DolHwUlC1AOS4uzg183JEYbnrLdiPtAqAdNWqUm+aMomg1Wh683qCMd3mjavTo0W4Q0b7L35A1a+6QPOAZjvh37b95WnrqQVlPu542bZrbpIMNL0\/Q1IInISHBcltYka8elLHK0G5W6oFUroe2nJ7AaxbK2skc9ZL\/jbVYT22o7at63\/Y0GcuaMvqy3sQg9x1sCuNhrdjbRC+PLXmF4+TNYrugrqBsAso8Y7M2qhU+dzRvUJY12K+++oq6du1K8sBh7QX5aaHlDcpYgjMMFi1aRD169HBpwtqlH5dfz3NCD6raAYuNRlnLkeXhCSxy3lw\/Hqych9ZkozXtsDbIsLPSFlbkq4UGQ0\/+HjRIhpNRPTp16uTmIeNJPmbNF3qTIpcPq7BNmzblWVVpvwuFQPbc8TSZetKUjdonPT3dZSrSK5MRlI3a00j+\/nhG2QVRu\/NRUPYAZe0g0Wo92sYwA2Xu\/ABiVFSUy3ShHTTR0dGWzBeAMnfc48ePC3c1dpWzYlLwV1OGTIw2q\/SgbOQa5m2JbrUtrMrX02qA4Y42\/PTTT+mhhx7StRGzLOQJzJs2bGajLxw0ZTaRsYuhN5u9lY0\/Wf5G7ph2gzKY+Skoe4CyrH3AJc5bxzIDZdk8obexheKgozF0OA1DWqtZy9qOvLEib8BpJxftgPE2uXhbamrf13Ph0iubni2WIa2dlLTLY20dtGU0Wp4bydeTzdNoqY682AyltcV6sn0D0kZ+x2Zc4vBdeb9Bz3Rgxqas9XHXaw9M7r5oytw+x44dc8kIftfcL7UTi6ex5U3+Rr7ewQSpnd9SUNZAWStcrU+t1hQgb+qZgbKsPWn9n+UlGrQwDGzt5g06Ni\/xtUtj5M1ahLbcRrZqM4cXtHY8s1qNvPvPctV6a8hpZHOKN00Zh1WstoUn+XqDvJEnAR+aMaoH6i1\/15P3hdz3tLLTQtwu7wv54JHcR9AWZr0vjGzK8l4CVm94zp49KyDNky6gr903Yc8meXx4k7+dUAx1XiGHMgbWgAEDaMaMGa5TadwhC4JRP9QdQH1fSUBJwFkSCCmUeWYuX768y\/6J2bVWrVpiNoXrETbD8qPdyFndQJVGSUBJwCkSCCmUAeAqVaoQoMyaMrRk+APDhstLFngT5MddVqd0AlUOJQElAedIIGRQhtmiT58+tHTpUqENA8rsJL5y5UoBYdak9+zZ43PsCeeIWpVESUBJQEnAuwRCBmVoxPCnhFbMNmU+RcSasScox8TEeK+dSqEkoCSgJGBCAqmpqSZSBSdJSKCM3WgcxMBxWnmjz6ymDCDDztykSZPgSEl9RUlASSBfSyApKUm4GToBziGBsp6rlOzm5c2mDBjDlccpQjTbWxs3bkz9+\/dX5TYrMD\/TKXn7KUCLr4e7vHE2oMBCWW5rrUucGe8LhrJThGi270LDh2\/xrFmzzL7iiHSq3MFtBiXv4MrbaTwJiabsCcr4zZufstOEGNwupL6mJKAkYKcEnMaTkEPZF+E6TYi+1EG9oySgJOAMCTiNJwrKzugXqhRKAkoCIZKAgrINgneaEG2okspCSUBJIEQScBpPlKYcoo6gPqsk4K8ElK++dwma8aZQUPYuR68pnCZErwVWCZQEbJaA8tU3J1Az\/sdO44nSlM21rUqlJOAoCYSrr34whch+095cZxWUbWgVpwnRhiqpLJQELElAjQHv4jIrI7PpvH\/RnhRKU7ZHjioXJYGgSsBpIAlq5U1+zKyMzKYz+Vm\/kyko+y1ClYGSQPAl4DSQ+CMBDjzG0SH9yUt+16yMzKazq1ze8lFQ9iYh9buSgAMl4DSQ+CMiBWV36Sko+9Ob1LtKAiGSQH6AModT4Pv7EFsd0SPxIIIkokn26NHDdSuRVVGblZHZdFa\/72t6BWVfJafeUxIIoQS0IClcxtnxxS+ddI9XDODyVW98ae1XX31FixcvpnHjxtGyZcuoY8eOtGDBAtfdnVbFbRa2ZtNZ\/b6v6RWUfZWcek9JIIQS0ILk6psH0NU3Px7CEnn+9Omfp9Ppn2e4EkFLZo1Ya77ga+DgY4xr4Xx9zMLWbDpfy2H1PQVlqxJT6ZUEHCABPU25SJlYB5RMvwgXT6aQrC17gjLC+SYmJtL+\/fuFGcPXxyxszabztRxW31NQtioxlV5JwAEScBpIrIrEm\/kCZgvEHseVcb5emmxWRmbTWa2jr+kVlH2VnIX3bonrRbdUvp9Grutu4S2VVEnAWAJOA4kvbaXd6FuzZg3hFB5flKy9AMPqN8zKyGw6q9\/3Nb2Csq+Ss\/DemJYL6PpyrSlhSUULb6mkSgL5G8qBbl+zsDWbLtDl5fwVlIMg6fdv2UjRJeIVlIMg64LyCaeBxIlyNysjs+mCVUcF5SBIen6nNPEVpSkHQdgF5BNOA4kTxW5WRmbTBauOCsoBljQ0ZGjKeAauaEbpZw4F+Isq+4IgAaeBxIkyNysjs+mCVUcF5QBLWkE5wAIuoNk7DSRObAazMjKbLlh1VFAOsKTrR7WmV1stUJpygOVc0LJ3Gkj8kb+KfeEuPQVlf3qTiXcVlE0ISSWxLAEFZe8iMysjs+m8f9GeFArK9sjRMBcF5QALuIBmrwVJXKkijpZEctbFPOUzCkiUlpYmjlfDT7lPnz40ZswYWrduneX6mYWt2XSWC+DjCwrKPgrO7GsylF9e2512HP\/V7KsqnZKAoQS0IHmmSUka0jTSsRKbtCmb3k465Sqf0Yk+JECkOByvHj9+PFWsWNHno9ZmYWs2XbCEq6AcYEkrKAdYwAHOHqcxr49qQ5O3PhngL1nLXk9Tjo90rrZ8KPsiydqyUeyL9evX07Bhw2jChAk0aNAgdczaWrcIXWqnzWyeJIFBPbjhuyKJ0pRD12d8\/XKva4dSr2ufc5yPeTiNAT3ZewpIhN8QjKhhw4YCzr6YLvBNszIym87XPmT1PaUpW5WYxfT5FcrRrdpQxRvb0La3J1iUSHglZyg7zcfcaSCx2qpG5gsEH4ItuVevXpSVleWz6UJB2WqL+Jk+nDpkfoXyrXMXCSivffYJ2vflHD9b1LmvKygHrm30NvoAZbtc5Mxywmy6wEnCPWelKQdY0hjUt8TdL2JfwC65InlugL8YnOx7H8wQHwKQAeb8+igoB79l\/fW64BKbha3ZdMGShIJygCUtQ3nu7jdpe8avdPTMobA+bl0yrgp1+3UznUo+SNmHDtEPvboGWIqhy56hzBMqJlcnHJV3GkjsaiGYNXr37k2rVq3y69YRZb6wq0VM5hNOHVKGMrRkmDPwhHNwItiTb5u3SGjJNe57gD6vUs5ky4VfMmzSos0woWLDD23oBE+McBoDoWp1szIymy5Y9QiJpsw2o9jY3OtrUlJSaOjQoWLHFQ\/bmnJycnRvsnWaED01lgxlaFjQtPILlLe9PZ4aPDOcFrZuLLTm\/PgwlJ3WduE0BkLVL8zKyGy6YNUjJFCGUzgenNphQPNtA\/itVq1aAtKdO3d23Xgru8U4TYhmoYx0MF8g4H0425ehHbd6a4rQkGFbzs+bfQxluY2d4IkRTmPAG8zkjT34KY8cOZI+\/vhjcXCEb7zWc4vztiFoVkZm03mrh12\/hwTK2sLLIB41ahTxMUu+1XbRokVu93Q5TYhWoIzlb\/1yrWlHxq+OWAb70pEA5QbPDBMackGCMpswnDChhtMY8NbHZLhi7A8YMIBmzJjhEcrMhsjISPr888917\/EzKyOz6bzVw67fHQFldiQfPXo0TZw4kVauXCmErNWiudJOE6IZKCMNTBcY2NElqlCFEvFhe2cfgAwwA8rY8EtbszrfemDwVV5oP7QdPGmcMKGG0xgwGh9Gd\/TBrAmT5ubNm6ldu3aUmZlJ\/G9YQeOB8rZkyRIBcK3SZpUTTpNlyKEsO5FDmGPHjnUJ2RuUcQ15UlKSXRNUQPJhm7IMZdgnsSwO180+GcowY+DJr25xMpShIePINZ5Qb\/ZpQVKl8tUB6b92ZXrw8Gm3rIwOj2g1ZXhiQBOGWUNmAzIzWklbgXJMTAzhv6lTp1JCQgKlpqbaVWWf8wkplNEwPXr0cG3maW1E3qCMWs+cOZNmzZrlswAC\/SLgC604+up4l68yvol\/d4Jt0pf6A8Rwi4MrHACNzb786oEhQxnH5LEf4IRj11ooD0usS8MT6\/rSnEF5Z\/zUnTRh6k7Xt4yOWRuZL9LT091W0XZBuV+\/ftS\/f39RrgIPZdiR0bFGjBjhdrYdjWXWpoyZE5qyE2Y3o56tB2XYlXFvnxNsk76MSBnK8qafL3k5\/R0tlDG5OmFC1dOU42Odqy0fSjlNsrbsFChDS+7UqZMAc4GGsrx00e6q5jfvCy2UOSgR7u1bkfwFzd090elcylM+Gcrss5xf3eK0UMbBH7RdqINLOc0OarUTmzVfsPdFoDRllNtpsgyJ+YIN\/HJDyr7K+clPmaGMumLpy4OZI8eF2jZpdTAhPeJewC+Z7cj52QNDhjKbm5ywynEaSHzpR3qxL9h2jM093ujDalpB2RcJB\/GdcOqQRlB2akhIM82ohTI8MHC6Lz9GjNODMjTlUHtghNMYMNOnApHGrIzMpgtEGfXyDImm7G\/lnCZET\/XRQpm1LY4eF46bfVooY7MPG3\/50QNDhjJ7yzhhlRNOY8Df8e7r+2ZlZDadr+Ww+p6CslWJWUxvBGVk44RlsJXq1BiZe0S8ev2plL72F5dmDBszwnjCrpzfHj0o84QaSpdGp4HEie1uVkZm0wWrjgrKAZa0JyhjGYwDCU4K5wlvChwG0YtlccP\/XUfnUnOodqOv3cwV+dEDA36\/C2e3pdTZk+js37kBlxjCTljlOA0kAR5GPmVvVkZm0\/lUCB9eUlD2QWhWXoGmhR17PBjMsnblhGWwti6e7MMt1zWkzKTsPFDOjx4Y7Pe77rWxAso48ANTEz+hXuU4DSRWxkSw0pqVkdl0wSq3gnKAJe0NylpQB7g4HrOHGQJar17g+mIxEQRNWQ\/KyDS\/eWAsmt2W2rQoT0ZQDrVLo9NA4k+\/tRqQSBtlcteuXbrXRpmVkdl0\/tTRyrsKylak5UNaT1AGkHE6bOTa7o4InO7pNpHSTUpS3Wk1BZQb370hT2S4\/BYDI+P3BJrzfweoxo5PdDXlUK9ynAYSH4aG6xWrAYlwlgER5Pr27es6ao1DZIg6KT9mZWQ2nT91tPKugrIVaVlM26Z5ebqj8AyX+QLR4eQlMLIL9TJYrpIZKCN9zZgVeaCcn2JgwJ68edmd9MSLm+jhkt8ZQjmUqxwtSDhOt8UuGrTkere1+BqQiOOuc+HlU8AKykFrUvcPOW1m04rhgXuqEmySGNzfjPgXbdqzSyTRg7ITfF65\/AxlbPRpr3iq3L8ixQ2oKJLqQTk\/xcDAZLro47bUtc8qeqn2KgFlxMEeua67q6lDvdmnHQPs9x6iIen1s9jQlk+vmj3R5ykgET6KoEQcf3nevHlKU\/baEgFK4HQowx55MOUUAc5bpj1DC37aTOlnDoqwj1pNOdTLYG4ivncPQGb3NtkDg6F8VZFKVDU697JU+RZr9sDID8etMaGi7Rp3XEpLHvxdF8qhXuXoacoIfOXUR3svpR2xL1SQewe1NnfIh598ijKLFKX0tasdUzpe+kLLmjKuGf296CUB5R+TvxDHrLWxLpzg8wrhMZSX9+xKN06akudC1LrTalDpJpHEUEY6rdydtNnHtnxfjrHzpArzBaD83bblov12HP\/VrZ+FcrPP6YqJtwHpL5Q9acj8bbMyMpvOW53s+j2sbcoT9qfQpWo1xCEGWWuzSzi+5MNLX2hZU15rRpGbXhNQNoJD\/ajW9GqrBSEPcCND+VTyIbpt3kI3X2QzUHbSZh8f+rB6wIMnVQ41CSh\/vP5D3cBRoVzlOA0kVseKWfOFXkAixMcYNmwYTZgwwS3CpLYMZmVkNp3VOvqaPmyh\/Pr70+njStUdd829vPSdMq4p1Uv9j0coo+GccIiEfY1ZA9YeCGEf5ajmtYT5Qk9TdtJmH0PZanhU2Z68esMxjxuxoVzlOA0kvgDI14BETZs2pTp16rh9ctWqVcr7wpdGsOsddMgXZ35I88vHk9NuVJaXvmahHEqNi9tEewBEthFfuHhE+Cgf\/fZviula1xDKTjrZx1DGaUkrJgx5UmW78cJjLwhtWfuEcrMvP0DZLh4Y5WNWRmbTBbq8nH\/YaspPfPQJLTyV46jLO2V7MrQsDPA7C0\/3qimzCcPqUtvOTqJ3Ko9txMf2LhI+yskz0ujaxDsottzbQu7ao9hOOtmnF7PCjLzkSdUblPl3q9q4mXJ4S+M0kHgrbyh+Nysjs+mCVYewhfIj\/\/2CVmSeCsnlnXD\/gpeCdqMLS19s7sGejMcslEM5uOFVUSw2gkpm4Ybq4W6wZSifK\/MjVeh8DR2ekUb1x3YxhDLq4ZTNPu3dembii2gnVTNQ9mdD0Z9B7jSQ+FOXQL1rVkZm0wWqnNp8wxbK3T\/\/mlZu2Sb8afl4sN33xEGDxfU\/8oDmDTGYTbTxg6Fl4enad5X4PyCNTTxPG33cIKHwV+aj0ygDtN8SETe4QZnjYBzP+tg0lJ0SxlOGslkThnZSNQPlUMXFdhpIggUsK98xKyOz6ax825+0YQvlW+cupKSkzQLKgfKR1RtwvETXiw+BU2A4mssXRFqBMuzKwT4hVuHua6jygIoi8tt1HWfoQhkrgpNXTxJ97Njiv71qyk6xK8tQ1gYTMhow2knVDJRDZVd2Gkj8gZDV2Bf4lnx7kd4mH9KYlZHZdP7U0cq7YQ3lnxctdLuSSM8jwIowtGkZynIgeoYy0srf01v64t9mJ3zvpilDOy0WU5Qyk065fS4UgxtubnjOpZ6nZo8uFT7Isq2YvSnO1pxCRxf\/TWe2RFKbT8dQqRJ36tqUkVegJkir7chmBcgVjxl7PaD8y4ajbrcu4xh8UrFX6c2108UEhrbbOWifW3FCcVTeaSCx2j5yequxL+BO1759exo6dCg1b96cBgwYQDNmzCB1os+fVvDzXXTIFl8vodXvTnKZEALhI8tQli\/JlKEsn2rjo9VsT0YVtVAGkBmEgNzhmWlukgi2CQNlydyUOzm0eelHXShHtahJGRHDaeegvVQ84ga6bd4ikd6TqQhtEWrfcYYyjrYjLoS3zTi9SZU1ZUD53QMfEYL8ow3hhbJvTG44Vjyh8J7JD1C2I\/ZFz549qU+fPjRmzJg8PstmZWQ2nZ\/YMv162GrK1\/3fMuEOx3Zd+YZl07X3kpChLJ\/bl6Esf19v6YvscfhgxeG59ObK0YRNNd4wwwD\/7Z4\/hOmAH3xP7yi2XfWR82F7MsPllim\/5IEy7MN1n+xDB9IfsARlJ\/grM5RxvB2R+LSxF7QylQ\/9HDx82vUza8pf375E\/BtMONCYoS1z24VilaMFCdrTyY\/cz8VENngw8cGQqlWrUmJiIn311VeUlpbm0nwRCc5T7AuGujJfOKDl0SEBZVlTDcSyGZCsd18j2hS9mL4fv0AMQhnKEMXie6qIQdqmeQWK+ijTbenLUK7YfA01fbrfP5pW7vIXhzEARGhd\/PDgDsb19QxlaMD4sx6UWaZ7U2+hdS23utUdmjJAdijlNMkQQ10C0RZWux1Defvx1UKT1QYU0uan9U\/m3xnKy55fJVwCsbrRa7tgmzC0UJYDRlmVVTDSs+z4W\/4es+Z82PSxZ88edXgkGA1p9I1azVtQi6+\/y3OqzG53LED5r0\/\/J4qBOMKAqRbKf265l4pEHxdpssceoR3fpLsVGyaJ5t22CCizuQADO3cpnNc+GSwTBsdHhraOP3uCMuq4sctKt7pffrKJKxIe4nzAL1t+7G4Lq\/1NdlUDMPF4sisbrXTw7o5q42lxn+VitYC9ALQdNkkxUfETbNc4PU0Z\/cmpD\/YtZG3ZLiijvnJecv3NmiXMpguWbMPSfGEEZbvtync2f5j+nrKF2qx8mFa3\/yTPEh6NBC2y2paLdKZxMdr+zVE3WyN+Zyi3Hj9InIpj7RiDWs+EEaylMJtSGMp3fr0+j\/mCJ6DtS7vQ1sd\/dYPy\/T9eCWOJiHgI3iM\/2huvg9Wh9SCJNoBd2dPN4XqbfMgLUM7p8QG9Xf8rF5T12i5Y7cb1cxpIrLavWfOFXuwLfAtHreUg94sWLaLJkye7FcOsjMyms1pHX9OHNZTLjfiZ0tf84goWY7e\/8o0db6dLY4\/QAx9PpA8aDBTL\/Iz\/xtDN\/\/nKJe+UjGeozjcbqXzna2j5+Sw37QkgeP7uBdTupi3U69NnqdzUKq6BjQwAaRzIkE0YDAJvNlBfG5zfkzV1aMoMZXkDj32yt\/y3D+0YsdgFZRy7nnbmZRFvGOYLDggPd0BX\/vc9IPzHQxXKU9Zc2d3Qk1kIN41wECKuA9oPQC\/8+Dh6PWaFW9vmXo11ym0SBsAD3W75Bcqs4SKGxfHjuSvNpUuXEoINjR07ljIzM2nz5s3Url07GjFiBKWnp9PEiRNp5cqVAr7KJc5fAtj8PmtwzYbsp2qn410R1uy2ZTKUB77zqdCyYH4oX6MrRZe9cu0MoHzzt1vo+i4VaNLZdLfNOy7PoJ8W0sB5z1PkiEruvxuYMHzxWQZAS8bFmw5jirpgSckbfT337xCastarAmaIzROH0M53P84DZXiawJ5stPTHu3qHbGzuDrrZyVDW27DFhIFj4tgoNvK8YChvf+xlmn\/VKtF28qSmNWH40m6+ysJp2p2v9Qjke2ZlZDZdIMsq5x2WmjJDObPnG9Ticu6pOw4eb6cts+2dj1ClacOp5vpoWjSom4inC2+ExkNzD1PgAZSHbPlL\/Hlk2XQ3zZehfPfnc+jTvybS8UdLu2lbRiYM1pbNnkRDem2UN28dSLZvI60elLEygEkIQAa8+BvQlPvtfpLuuP0H8Rm4AyL4EkOav233ysVbneTfZShzbBFZnjCvIJg\/b1jyTSOybdwTlPXajtN7c7+zUg+jtE4DiR11sjsPszIym87u8hnlF9ZQ3nJDF3H\/HZaYPBDstGV2uPs5qjTteSG7vV0Sad2WL\/JA+cSB9TRw7+viphFoynhY+2QoP7J9Ib2\/4Q36q1FhN20LafV28vHvVj0xGJhmNFPZ84IPsQC+0LaX17nBdYkrQ\/ng\/Nw7+bg+gHLHlf1pwKNrXP0KJozVG4662ZbZ\/KG9pSQYnVu78QbTguyBwVCGeeWGmNPi+qdy9ea7FY0h+8Ojz9OPx37Kc2hEz4QBbVnv2i+76+w0kNhdPzvyMysjs+nsKJOZPMISygwHQBnaq7xstPOYrwzltPfn0A\/jnsgDZQj5uV0PiuPVH\/11xG3zjsvS4chq+uPoWFp+PjMPlLVeGFhKw70Oz+CG79DWn+PyXCGl17AMZb379bTp2fOCvQmgSd74wyI6Xa4Qpbb+tyvWB9K1mzNdnObzBmVoytCYtWALlbasB2XIgT0wZCgnditJt3aLpa59fnbzEPAGZT0vDLwzptUCWpH8hW5gfDOD0kwap4HETJmDncasjMymC1b5wxrKslYHTYiXp3aZMJo++AjVef1tqrf4Ev3euTCtH\/Oa2OxrNOA5t\/aZnD1AbHph6Strvgzl+KPHaNvpfZR+YrxwLZMfeRlcsfBVYtMMdlrA+ezxcrRr7iP05\/YSXsEsu+p5C8zEUGaXLhnK5Tp\/5oo\/DA+NJs9NotO\/lXKLMQJN+eoJ3Wj2tD2uqmiDw\/MPrC3rxQoJZCfXQll7EwlDGUflI28\/KPYEtN4zDOU5fYbSph0r83jW6Jkw0G63VL6fupV\/PaC3yTgNJIFsS1\/zNisjs+kKkpCbAAAgAElEQVR8LYfV98ISyjhp1vKRYfRtpwaUmpoq6iwv92FywCYOtDt\/HoZy2z776dCdlWn\/AxGUfugvio6v5pbt6FPPU99HFggo8wZa2oeFhD1WfspkPEbTGi\/MUyQsg0tsPkcvVI+n+MpXu0J\/YoBPHHo7ld\/3rADz5C1P5rknjjPjCQB\/55gchcvEUNGqTen8gU106WSunPCwO9yfb91M5w8kUe3sYi5NucfACy5tEukg66hSfYXtVTZfZI2\/m757P9eWzo82FjH+HeCKT7hFBDyyOzaJp7bVQlnrgcFQxiZmsQ7rqOyRy3SiUiGxJ1BsVSbdGBNB5y7EUt\/rfiUjKOP7sgkDB1CGJ9YVxcKFuSf21qbvLz1OHyzN2+b+9Eu86zSQ+FMfXwIS8ffGjx9PtWrVEnEw9u\/f71YMszIym86fOlp5N2yh3ObJIcLdiqGMSrM29FH8PCr1Um+\/3bEA\/ppjh1P9AbuFn+u0dr\/Qdffelke+gPKtt31GydllqNKD9Sm+TzrVWNiECg18i67OuCzMAnjKnXiRJjf8IM\/7vAx+PzOWuvVdleeEXNNadejNu2aKQQ676JStT7rsvnpQhl15x4efUJmHPqAiZWJEkuiMH+mNG7+ljeda0ldR2XTV1ccoZfUw8Vvcd\/+lZh+OEuUElNmfFxNM5TY9hLcJoNz7yT5EQycRNOWl97bIE1RJ3vBLu3SByt99DcX9E4Uuvu7Lwrvj5F8baOOwV\/O8a6XTmkmrhTJ7YLzyRyIduPE26vBidypR6ira8dlwKt5hPd3w\/QVadiFLHKSZ9sUxiitVhPaeqEjHTy2j9596iMps209fDr2yMnDJ\/Z+DJDVePykO08CM9cU\/roFPNHyXahXqQoXrzxPH7LUHbMzUwyiN00DiT12sBiTibyHuBY5nHzt2TEHZnwYw8y77IObk5NDUqVPzRH+C9lar\/7O0bPh7ebK7L\/pBan28JK0bVYx27d1NWxdsMfNJ3TTVbylH9W7vQMVeyaAuR8oIDZI1LPmFJ89MpDvGVKV7K\/5N\/zo9h4b0ryRAVm1lPbp5fmEqdcda+qj7TVQvcyRNv+OjPNCt3yVauMsBDB+M\/F23LNCa72rQkbqVe11ceY9wlLLdUtaUD63\/k3YmXSU05KzFo6lK5ZK0aPbNdPFkKh2CC9v158SmXNawA5TVdSolFOtARQZHuaDMm6aActmqLaj69VOp5uCbqGnHtjSv6zgB5TPjOtDmL45RctZFt\/LC3xcHSWA\/x\/Fz2Q8b7YZA+ng\/9c8ZtL7ve242XJ8bSudFIyh\/U+kkzT\/3I7VPbCk2NtPTF9PVl9+hThsaUMTR7fTBnefp7i8L04\/balCheo\/T\/Zeb0IIBn9Bdx76iFtN207w\/z9Da1PO0sWgDKlw2Rkx6DSauoyHFoynrYBt6WjPnoj+i7xS\/JoP2l9lIb+45QMciLvhd1QblLtOkgXdRQkKCUExQFyc\/2ltqUFZ\/AhIB5OPGjaOjR49S5cqVFZQD2fjycqRz586uoCXr1q1zfRaDu91T\/emTl350\/dszXU5TXMmT1KLiQSqysx59F9GFktpUoBajz9GWy9m08+J+oWVaeaAVV64TScX7fCaC2gDKvHEl5\/Ng+vv06yfHaODpz+nLP8\/Qxkbd6cIzwyj6p83UYV5HF5RvyX6VXu04PQ+UoWGWHFFJgFIbFlJbXjZpxP3dl45svFH8jHoduLEQnRlys\/j7mawL9P3AEXT65xnCNo1btdksgr9jyV1zxXEasvuMgGpq5g5a8loRAWWYanZk\/Crsykh3ZkspavroUvrXf\/vQkjMV6NC\/Jgqodv7ybrrln\/fXpObQoayLlJx9iR58ogHV7lCJHj31lytWhFwHwAPth0kE+WQnH6RD81fo3uRipa20abVQblijB71y3VT6Mv0z+vbSy67DMsezZlPE6c9o+tbCtH\/ZX\/RYu9Iu\/+3K191DPabMpKO1C1Gr1JGU9nkEdT33A8VezD22nVKkIqUUjqYNPU9QemQR+vLtR3WLXC7nKupAZ6nTqfvE7xkRF+jXqNzofJggfHka1Yyh8XdFu6DME54veQXjHa1XkNkTfUYBicAJPMnJya4wnsp8EaCWxOyJSFHDhw+nli1bitM92iOUAGOHe26hVj8Poide2kj331NV7PwDaqvX58ZgAIROP7eQiuyJonaTrmhzrGEyzOC9YfQwPOAPDZskToTBLAKgaKEcNfFtoUW9nXRKaNPFYouKkJc4ov1gsSh6K+Ee6nhmBtWbOYO++OGI0LYAMzywxZ5pHCFc6nY9f4ZO7S3tVbqAa\/d2TeiBSg1F2p+yutDGPkXoeMoZioot4ToEwiFFn3hxo1g+Y1leeXl9odmtf3oz7dhXjx6v\/5ULytjog0vXU3+0FlC+e+lZ+gOubxOH0LYisDHnarornrhJ2F7vu7Y43RgbIWyweAqVLkbzH6pB\/8s4Sh1ePSRgLdeVKwY417jvfopudZPwF+YHGhU8SK78\/1CeuwC9CucfUxbcJTG5FGvYmZq2H0dD91QUm8Hf3vOmK3501pmlVDHrXXr8uyxKmLCLzrUt7fKgqXzTvWISxtPr1OvUs1fu4ZFKx3+jFldnUFxkYerVrCydGdyQnj99mBJ\/yqRrk7LpUPZFWpOSo1tvmKI6xN1PdzW4TZij8KBPpp8+RDuOr5b+7FmB0Jov+PCQGdmEIs2pZPd29Cf2BfjQvXt3evHFFwmKG8dWVlAOQMvKtiUcpTSKAMVQfvbcy6IU8FbAbR\/yMV\/8O3skxBxYTi03zqGahbpQ+qULYjDABFDmRMU8g0IeJJUG9qbIzm1oT+dEca2TEZQrbVtMa\/oPEJCVr4u6mLyU4vpXpHtPXUs\/NXuVHjr7EV1clkQ3\/JZ76ALP4UJX0bVPt6Qvv6lI\/1fvd6F5sa3XjIgrlzxJlSNPUmydSDpdrxlFbjpP2U2L0ta+iZS5a5Hw5oBcYFIAkB9qV5pWvBxPb1xdWQRPOr5whDh8891rRcTneAKq9sJwGhSRJNLNKPMptd03lX5LKUGZNz1GZ3J+o1+e6pHneDjyFzGj36tBjaLKUqORv7tgjbyhlWPiwoPJix+8E9OtLlVucy+ValKSShS7QdyCIj+AdPahQ5S+9hfXP2OgZx86KP6OP+f+P\/fvrClPp\/9Rqc6j6PTP0+nTUmNpd7ENNP\/5F6lmzAqRDppy03PzaeW\/drlsvpiMLqVHCbNNux8qi8kOJqpWdV7P0yR8hRSgjI3C1oP2uk1S2nrzBCUm1bZNCJDGJHhyb20XpPkjDGtMLghBKvfNuLoV6JWJz7s0ZTN9xUlp\/IFyXFwctW2be\/UaPykpKXlMGGbt7mbTBUt+jtro02rGnqB8S4cb6feXn6JjaX\/oho9kATKYMVhxAejZnCs2Zmw8AcxXXVWJiv7zZ36v9ImKYtmKAcFQhtZ15pmb82jKF35dSPPuf0y8Ci0aGvaaIU9Q\/eQ4ajy4mQhmBAgAyl9\/c5bm72tIFfYvoWY524S5pUbH6tR49S7aERNBU9qWpqj0ayj6nwD40Li2FalEJeONbYY4Xl2+bg3xbXbfK\/\/lAfHnSs3W0D1vDhZAntSuNJVuEkmj7r6GMhIP0k\/\/uY\/WjRtLMw49SBEfvElxkUVoasOGbkF43pmVQa\/3XUw9on8T9fsuoquA8vo3eucJ0o\/f2d0OMM\/adErcV4hvQ6PG061mTfF3PHtPVBKgxmSGevKDi1xrtalLF9OjBKxzoivRybJpdLocic1CtBX+7+0p8s4f9F2xCAFkmHLgNvlLu\/\/Sj3VmUb0mq8Xr4iDMmRk0u8+nLijDY4QviO34\/GFa9kZl0XYDbxmZx\/QkHzGHDR4TDI5jY3K+rm41uq1FTapzXXW3oiZn567cYOriB3b\/+NirqVmtOpR2OVdxiCxflM7+HaVbzbhzZ6jJtYfDFspmzRd6AYnkwEPyLSS+aMoxMTGE\/7B3xfZ5b\/0q0L87CspmNWV4K7RvUpJe\/qko\/ZJVlt77NtfezOYAWWiARIPpbahkVm8qX7NLHnnKGxDQxPjBYK1RpQZFXc6gt27uJQY0gs0cGFgtD5T5wAYGIsdUgDseu+mNH9WRqkbPob7nv6OV78xyi7ksD2rAql7b0nRyZDw1So+iZhn16GDNLrSvagtT\/QBaX+\/9pWhpw3upVtIy6vbXKTrwv840fUcPGnXjTpHHv4ko4plYcdx7+hPDCLbpCx0H0qza7wigPPteV7qq8ov02bnjAl7HHt5NdWYuoLNZ5yk7+RDVf+xhF5RTvyws8ixSJta16RV9+2Eqd9NZOvPFHfTzU+to3orzNH5hC7HZxbeAmKqMJhHssPwA0PwcjMkFHIDNj5hs256mP4uXJWx6ntv6DZ04sIFqdL1MZ+seF\/7GHSUf67tyFtFL7RLdgNvmkzFU9\/p\/E61\/g07f9YIulKHtLpzd1nUvI+TXYHprKl3iTqpUe4BbX\/JU5wqazVKkZXDLfVLOo050eXq4eCHHgMSXNtVu9FkJSMTf8xfK\/fr1o\/79+4vsFJQNWtGMTRlQvrFTS+r8n210U6kTeXJy2WqzLtLuCdWo+E+Z9NOwXJ9aeZdab0dYzgyn1KrElhTanhGUS5\/fQZlF67v58bI\/LkN5Sbc3KfP2R6nLxfV0ct4Ut6PIetHJYrrWpZvfyY1EB630zLktQqO75uRhwgCOzr5I9VNzqELWJVqbmkPL1++hJCKqO62mgGjL5ckUsX0tNRz\/ANUp\/ROVKraBil71It337d90uVd5cfsJtDks8fEkfvQJDY6cIb4xKvUZot0d6ZXiWwkB4rHxiImmbJt7qPBfF6hs61KiTNtXr6C\/d3Vzkz28O2Laf0cXT6bQkRV3iVXAG1VqiokB3gdla\/5JZWrupuJRGeK9Q4dP0eajmXT4VFnhTng58yzFnDgoVhDt4nP3BrCZhmdRsVtpU0QDN43aEwi6P7qPTtfsI5KwVo36YfN1+fgR1OT7TaK++K3l+V\/prba9XFDmybX6mTg6GDOcytb+j2i7POBuXl4cz+aDQ3iv7pOPUJmqLajo9bvFqgyyQhzh3P\/OU9ambNef8W+YiOMji1CrmKIezR6yuQd1ctqS2xcoB\/odMzKCltypUycBZgVlgxYx433BUP6x+2qXn7LcuZF1+6aRtPDua8Su+LS5V1y32KVJT6vWFkkLZWwSbehT2E1TZijDZ5o3hXDDNh6G8sdXvUj0\/BPCBCBDWS86GdukoX3\/8nQPkQ+Cl3OQcKNBDNMHzBLvfXGM3npmm3iv2cPVqXiRBKpUfPyVSHr\/RKbLGHJexAzhcJY4BQlIvXLqBXrn55r0W5MfKOezCNq1uTG1GPkSlSn5t7BXX+5WMxfKC1+gv97J1VLPH8yNpayNqYH643RbVonfqFD9ef+AGDeVnDLlsyubPYY0jXQ1D5s8jDYRkXDGRzfSzHqnXHbvE99F0EP3f0mnO1ShuskLaVOje0Q9YLsGlJ+SVlFs8joy6A2hXfOE+v4Dz7mVW76thEEeGR9Py3t282lzkiso11veREW9eRNxb2Q9emH8u44BSaAB60v+ZqDsxAnOUeYLFrw3P2VAuc7dtYS2Jx8ekRsOdkH4yea8nSIOKrBN08rglqEMrTK+VBHa+tgRSrvpXtenYI9cdbSZ0LLgRSCfWmMowxZd+K0HqWuzQlT95zeF5o1HewURBvZt8xaKDS0Gu5nOiEF8x93X0PZnYunaYX9R6r8\/dUVAw\/usEY9c191140nDSQlu9wEylG86kSigXL7lBhFHenShs1SpYhY17nuny14NmG35b988x47l49tcdzvjC1vRKvl04Ruv\/yY00ftqF6fWlYbQlph\/05Keh8WKiaFc7OBm+uimK4eC4D2DB1DuOXAQ7RjQhNpc2idOiMqbyfIJRvbUwV5C+tpce7Vdj169z1W6no52GisOTxiNAbu+H675mLUVm4V3sOTgSCh7qzyO\/zYcUM8QytDYaoyMc4sXLOdpdnDX7HEdta5Tij55bQN1qjqXci5UpuUPbHCDMnyPN\/1WWXglaOM76EEZXgx847UMfZQPmnYu2K1rWvL1TrXvf1q4rslmFPhZj1zbnWLnlKRCP0bSy3u+dcUKwbcZyrgkFQ+uv3rtmzXCrxfeHHAFvPTxL1S4z00CZnpQ5uPbKQ+cElq4ldCj3trc6HcjrbLYfdeLgzLRv+4Tr0LLPJw1iApffsLl\/sdQxu8cL4RXKpBdhT8vC6+bLwelUKt6yW5Qllc5v6VeLSZTtD9f5Otrfcy+1\/zaOOr79AvCjKEeYwkkJSWJicvTo6BsQw\/yBmUACto0NnQ4NKW3z+oN7og7rhV+t3s\/3ELZ52ZT1rnmNOHWL93MF\/BfnfDIbMLA1Nqo+Wgve23AZ\/ihcx+JSGo8qPm2C3\/DXOpBWXbY54s9T3+9k6p+2IY6\/NlPQBpuV3pQxkWXtZe2F\/7Z8EL5aUgRAR14eBh5X8D7AM9t054OSvhKvTblCXfqlNZ0OfMcQVNmLweeJDfckCw2bPWgzBMjzFEch3l257V06x2F6fCkYa5NWvn269ghE8RkineC+bDngPzNeu1Ki32Dfa8eotr3PyPcQn\/olWv7H9NyvrhZPeLBHHHgCHdFap\/cW+HjXe\/g96JVcsF\/\/mCSyA+rgn1ffiH8zBFPHN+CeY0fHK9Hf9z3ajJhDFQoEU9Ttj4VTNG4voVVhLeVhIKyDU3DN\/cevOeorsAZDt5Ox3kqCgb3lNeaUuHSxYW5gQGrB+XBt72Xx1UKeWuhfEdCE+HvCk0Z7k\/yJhGWzLBH+jqwZSiXqdqcbpu3yO3WD5gwMsseoZ\/HTBNmib2\/\/OGKBscTAmzK0JSxAQXZidu7S8TTqXK5wZVg5wZ8ALM\/lg1wO33I9mTYoaGFByPQu6f20wuOxJCFSeX04sZ0\/sIREWwJD8sd9eT4zzKUG99VgSrMGezapGXT002P\/i5kE4qY0Xr115uceRXAQZkQYIk3cbV5yMH\/9fKXo\/5hgsYdlRwCltPz+ON9i1D3BW\/IUVD2JiETv3PIRD0oMxz4glIT2Rkmkc0LnqDcre5E3Tw8QZlPIQLQ\/mrJ+Lg8GAFTmCPk2MrQErt17ScinsGevHDRLJeWLN8oAijjzkAO1M8VQ35YCbAt1gjKyLtpemevoUb9aRcz7+pBmUNxAsq4pbxUiTtdV3sBqjiIgsmMzT4M5ak3fEl1H7+drvv8AReU2ZVxXEbPkGjJRjKQ+wGfSJT3OdAnL41N0w1Fijy9QRlpuG9hgtaDMsLXYqV1008Pu+1bmGm3UKRRULZB6p6grIWTP5+TocwDdNYNr9CJhL3iAENsubfF8VszUD52X1Wx9P+w7KtiYMOUgQdaOG8S+aolG0EZ\/y7HVh7Q6xXaOmQ+Zd8V6RYCVAtlvQlNC+X92xPdAvZD7q1eb024zzAYtmRv7aoHZbzDN5B8++4oF5SxQtg4bCxV75FrT+dNVm7z8eXfEtECcYM32otNTw+M+IvK\/2dZnr0Eb2UL5O96Kybt5nPRWcdpx5dbdc0XvkBZ7i+sFB0bcp7eLLPSEX3Bm7wVlL1JyMTvnqBsh+mCi6AHZdxqDSjjwYEQbNw9cWveaHX4XdaUAWVsvuGWEmxAITYHh+m04wor7WTE1zvJUOaNOPkCUJTTDJQ5P6RHvIhDO191gzLyrn9fQwFlT7dGm2heW5IYQRkbkIgzIUO5wr5PaXtadaHxymYILZTRdljZsD15wPuXqPjQWUGNE+1NOHI\/gA+21rSCFRP2FaDJ6tmUzUCZ44Rj1QRNWYYyf\/98vygaUWWuI\/qCN5kpKHuTkInfPWnDvHTS63AmsnZLovWOgJa1oMVr9OddV24P6ZERSc910g8eI0P5wI2FhXfF5SebiEtG4UKHAW71wlOjOmhlIl93xBuQ3qCMdAg0xD7R8re0UMYtKnxzCdJhMrw+qg01mJRAcL0L9WMEZdjWEevjozcec2nKcSufpV21cjei5NUKQ3nk2aeo4eyphLjZ19ad4XJlHHoc5yOvaNahrjO+r2fGkjd8GcpG5j0zextyXzi2837a\/O5Glz8497GE13PjhDihL3hrFwVlbxIy8bsRlPUuBDWRnWESLZShZf155woRy4KfFtsK0+T+uXEhtI8elDHocSfc6vVHxUEEebffn7JqZcKhHGXNz2gVwbGYAWUjE4ocRxqashbKmAwREW\/z5FxXuFA\/RlDmzS4cfWebcsTrCAiVG3tD9qBhKD+7O0HczMKXGSAUasn4eJp17buO0pL1oAyAyq56Ta5vTzBfZI89IgJSaR8zUJb7AqAsm0LQx6JLVKGn5s0VYQnm7tbfbwl1\/5C\/r6BsQ2sYQdlIE\/T1k1ooQ8s6kbDHJyjviEsWm0iAHg98OaKcv\/6tWpkwaGUtCQMmc9OpPMtWM1CW40hrocyT4a3TnqJxs\/v5Km5b3zOCMk+UMpSPDG5APy5MyfN9dqFDHG3Y1BGU6LUH3xBeM5M216bt1Xv57C1ja2WlzPRWTPiZ7eQMZa3HBGdhBsrcF3CrzsH0B2jP6j9cG8PoY1X+akTd178U8s1eszJWUDYrKQ\/pqjWpShWnlTF0xfHHFU7+rBbK0LKKPnjODcoxS8\/S\/43606umvJ7WCPuevOlil+lCT0PSu7AUISlxM7XWtOMrlGGbhqcHg+DmkYNo4veDbGhh\/7MwgrIcJIo1ZStQ3vfVHLFJ+26JofTL+mOWTl76XyvvOWihnOt3XMVVzppt6lL5SUVFlMA9G\/PGjbECZSgXEYcmuTw5eHLutPA5KvRDZFiYLiBRBWXv\/cprCiMo22lPRiHMQPn8rAxKmn7YK5SxpNfesm2H1wV\/WG\/1gEmAj2x7Mu2YgbJ8s0Xh1N9oNz0jNvoAZd7kO\/zAKcOLXb02qs0JjKDs2rwb1VHEvYAHzfqEmrqAkjVlyLJDxWRx+GfRxsu0rP1Mx5ku9CZnrXmMN8mveiLFLY4HXzrAFzh48gTivgAoV9j2Gf14eK7QlLmPwQ9+zbL\/hYXpQkHZpoGHma3otAtuu75225P1oIylLwAk25StQBkDG2YK2PjwQCtB0HZ\/TRd6gxH\/xstMeGCYgbLs16xtKvkOQC2UcXoyvkZN+qnvlQ1Qm5ra52zkkKhyJuyrDPMFP0ZaowxlgKhuy+uoyBejqUzPJyi12m2OM13o9QNuN\/bCQf+9a2A1scqUL3LllRXy8LS3gN\/lSfzy+jfExQFYnbJiMHz0srDwuuD2V5qyz8Psyot6UOaj1Vp3L38+p9WUMUg5aD3naxXKvOlip+nCCMo8eKD1FInOEKE9WbuV5cLpzEL5mp2ptLFsb5f5CHbEMicq0doXrN2B6E\/beHsXbQczA4fVlNNjwxbeF\/zoyQS\/aaEMl0bIkjfPsInqtEe7YuJ+xnsZgHLTf1WmjH8ftAXKZxeOobQbd4p+xSumhDdeDxt7stKUberBelCGtoYQl3bZk\/U0ZT0oezo5KHtfwHwh+yND84J2YtfANtr8ZO086\/RSETVPb9KyCmVMLJdvmi5WKtFr6lLkd9mGfq82NbnlbBjKeBGxRjxBOTVhj+4xeRnKLCPUHf7MgYgGZ7mSOi9o+4EcYAnR6zBOOnSNpf912+ZWZ1815YNfDKDLHbJFv2K3yMIjKoaN6UJB2Y5e949hHht9W2f87tq0MvIs8OeTWk0Z9sg7hnd3M19Ueq25OLKs92ihzJsuADGiikEzDTSUeSLAYQ8Obq8tqxkoy9dqAUjV38miElPjqVlaZxFPw2g33x\/5+\/OuDGWcoJRDbmLDdv2Mma7sZX9r+ZsylPHv7J+rjQboTzntfldvcpb3MjBO2jSv4BeU5b6w++0+VGooCZ91bCR32NWPvh+\/wDF7C2bkq8wXZqTkJQ2ECCjv+naP2waD3WDQQhnFGjL+TTcoI9bDjLmvmIIyww8eGFr3OH\/FYqQpu\/yVZ95guJIwA2U5+D7cqzC478ocRFUONBLxNIxMAP7Wy9f3ZSjjhnOOYY38fIWyr2UJ5ntGG75sNkO7JbSMoU+6bPZZU5ah\/OtrHYT2jbEH85iT3CLNyl1B2aykPKSDEHFj8pol6wSUuSOGAsrYaR73UT9XcB+52FpNGb9Ba8HjyX7ri4h4I08bTIgHEA57wJ1LG2gI3\/IFytCKiu+MomvPNaNtN\/xP1yziSz3segdHoYcn1qM2LcqLLGUTBtoFAYn4Masp21W2QOZjBGVelaHdHqtaiSZ2Xu8zlHmC5hOgiHmC6IBnEg8RjlcnbXfOhq8ZWSsom5GSCU25zXvN6dslSwRk7D40wp\/X3gyCf3\/4lSfdjlkDyjBf6J1i04MyIAkI2mW24LIylPF3rdaKb7afM114eex89+M80jUDZZ5QeDKBxoXj2LDj47HTlm9DFxFZcIwK\/Fk2YfBR44ICZb6mDH0ObqNDikeL0AA46s+PFZuyDGVsICJP3AqP2Nx2brTb1Q+85aOg7E1CJn6HEAHl5ZuXCRjYGYRI\/rwelBs\/2UwECedHG5tYfp+hHIwAPTKUtcFm8BvsoQfnr9CdDHyBMm+sijgZKTm6wW1MNGXAk2xedqeI6gabMsCMB3sD2JwsKFBmE9bXLWOE3feNqytTnzt\/8hnKPEGz6xzyRB\/DbTVOnJy9dTIFZW8SMvE7hNjlvTtp4ZH5YmY2OqlmIiuPSfSgzAH2+UXY0JqldyYcxdU+wYQyvo3JqXSTSHGgQ9ZYMGBuX\/mRiPUgR43j8pqFsnwYRZaD3WYjf9tNO7EOT6zrCgDFv7F2d7JsmltgJfld7UafneUKVF565gtu36X3tqBrp5TUhTKbuVAub37KWiijL5RuWpL2jUkWfS\/cHgVlG1oMQuzz3kP0eaHZLigHAgxmoMxxY\/VuV+DgN8HQlCFWHpD4s2zCwL+3mJ0oArrLsTd8gTLbJvlkmHYCsKF5bc2CYx8jU56j3gQAABG2SURBVNmubAbKmFRvibs\/rHxujTZ8sZdx+JevqVDdX+i9oifozjuWu2nKVqHM4Tv55B8m\/nAEMvqFgrINQ06GsrApG\/jf+vspb1BGJ8T3+1zIDVMIMMtPsKGMb\/NSUjZhsM29XpNcFzw+UegPlPEuTBh49DYP\/ZW9ne+zCUM+SFLQoMzHrWEPnpw9QISNlW3K\/kLZzvYKdl4KyjZInIX4dOF\/idz0ri+y4TOuuLl8+zTylJft1bZcpLmP73Cd\/NJqxKGAMpdP1mDZ5l7m9BAhFu0mo1nzhR3B+O1oF6t5sHscX1KL9wHlioWvorRLFwzNF\/lJU0ad2bbMIUj9hTLHVbHaHk5Lr6BsQ4uwEMcUfpGOU4ZbDAwbsndl4U1TLrwki9aM3i\/S4+jujoxf3bTlUEBZ64XB2jM052InO4h4GFoThlkoc7AaraZtp8wDkReOWwPM8mYfoFw7M46K7atHe189ZOg9k1\/MF6xQtBv5uwhB2qn2UDdRW9WUMUHj4ZCggWi3YOWpoGyDpFmIfcc+TNkxWQHb+fcGZdlEwJtCA1c0c\/kshwLKDGHAGWaFzKRTwqTBNndtpDqkl48Q2+2qZ0Nz+50F25X5thfWlB+MiKJmC56mrT\/H6dqN85umDHNTs0eXUl06SaOrt1ZQ\/kcCCsp+D7HgGeatQBnVwnVR8oZfqKDMJgy4KCGovXy8WvZZ5abI71BGPTN+TxDV5c0+aMqA8t4JF6jcvmfFb1O2Pul2CCi\/QRlmrMpteogNX+1RcVlTNnOwSWnKNoDMIItCUVFRlwOXfWByDtbMZhXKgDAe3vALFZTZM4KlL9vc5chxfAMK2xqdHNPB356EUJ443ccbXAzluY9vpxN7a9PgRv+03ZYnXXEb8iOU27dqQF1K3kyfFn9U98IFyNkMlLXB8\/1tn1C+HyyemK2jgrIHSVmFstavNVRQ5uU5\/q\/nsqYXbB9hKfMzlNGW8FdmDww+2fZu\/99cISz5UlWYoPDkNyjDjHVb0dL0QvU46rmtr\/BHZnOVVU0ZE3l0q5uUTdksaS2kU1D2EcrYud\/0weE89mzZhBFKKOu5xnFVtSaMgqApM5TZA0PvuDHfSsImqPwGZdSZQ81qV0zyJQZmNGULjHF8UqUp29BEwRKiJ025QZESecIfomrwwliR\/IWIJxtKKGOjD3EpsNGnfTAAAeLlPbsJbakgQJnjYLAHBkO5V6PlbuKBtnz0zCFhgspPUNa7eUa+6VpBeSolJCRQamqqDYTyLwulKfuoKQPKM5uty\/O2bFcOJZS9dQvZhFEQoMweGBzGE1DukREpAvPIj2yCyu9Qlq8LU1BWUPbGDI+\/O0VTNoIyBjZiYTgZytCS+Mh0QYAyOpTsgWEEZb7DDweBOsTdTxVKxIfNrcyoo9Exa08xMeC3jttU2KylzBd+4cnvl0OiKVevXp0mTpxIsbGxogIpKSk0dOhQ2r8\/9yDG7NmzqU6dOpSTk0NTp06lefPmuVXUCVAue+Qyfd9tW54GkDUtJ0NZtisXFCizBwbc4u5Y2IBuPlaMRvTbmKcNYcJYcfgLuj6qTb6BMnvkaGNH83VhEIKCcgE2X4wfP14MhOHDhxMDes+ePeLv+K1WrVoC0p07d6auXbvSiBEjaN26K6YCJ0OZN4uwg9\/r2ufEEexgBSSyMkXLtxwXNCjDAwMB8PHIN5Kw\/NiujL+Hq6asDT1gFHOcQYz7+xSUCzCUtfCQQTxq1ChKS0sTgG7ZsiWNHTuWFi1aRJMnT3a95gQoe4odyx4Y0LScDmUsXWvcdz\/ld5c4dB72wEBcZRy9PphyyhVjWe6TWOEAxtjwC1cooz5u\/ukGFwvz5Lzt7fGiD+BR5gsr6o39aUNivtBWA+YKPKNHjxZmjZUrVwoIa7Vofi+YUJZ9W\/F9+bScUUDvcIAy6sKbfSXj4gsUlOEWByjDE2PC1J2GJijcJhPOUEbF2Fzh6SIInM6DTZkfBWX7QWslx5BDefDgwS4TBQoua8beoJyYmEhJSUlW6mspLWtWeIlPggHKTf9VmXZvPGF4ywIHJ8J7TtWUUTbe7Mt1ixtO0JZwZVR+fdgtDh4YON2nveWa6837Atszcj0zRq7rHlYigftbjZFx4sIDjnni7bZ3vqm7oGnKMTExhP+wd1VgXOK0m3qrVq0Spgk8AHKPHj1cm3mc1qymjDxmzpxJs2bNCsigkaHM\/q0M5etmn9HVshjEsCennz5E15dr7UibMsrJNkSGMm7Zhm0xPz\/sgYE6yvGV5TrzvkC4Qhl14Y09BM069u3fIigVHxzRa192jytoUO7Xrx\/1799fiKTAQNlogMOODDOEdhMPpgyzNmVo1dCUA+XwLd+IzBHGGMolX8sQy1+jByYMfpy40YeyYYMPNkUcr4amXBCgjDadMi73GLUcJ1vbjtx+AHO4acpcF7j94Zg9zGxypEC9PitfnFqQzBfQkjt16iTAXKChLJssZK8KdBYneV9w5+Ug6RjEF+8qJcwXGf8+6IqZoNfJYcKAzysep0IZZYNdGYMQNsWCAGXUGWBeveGYxwVBfoAyH7WHCaPutJp5bjnXCgB9oaBpyqhvsPaozK5AQ2JTZj9kuZCyr7JT\/JS5fBwkHcvdvxoVMQVluFXBdOF0KMu2xIICZTODgyfVcNaUET8ZZgy+O0++TFdPBgrKyiXOzNjQTRPsmY03iLBr\/9m54wLK6ODydTragvLBEadDWbYlKihfacX8AGU5hKsnezLXmifo\/BwtUA8oweaJN\/CFRFP2Vihvv4dCiNgggg359X3JAsqLmnv2+pChLN9G4q1uwf6dD47guwrKV6TPKx24xWkvxA12G\/n7PRyx1gtMpc2Xoax3ua6\/ZXDy+6HgiSd5KCib7C24ERlP57mb3W7yMHodwWzggYHHyVCWA9EoKF9pTZ5U8wOUTXZxYn9lBWWzEgtMOgVlk3Llzb7561Ip7dJ53ZgJclbs66qgbFLADkvGk2pBgjKbshSUQ9sZFZQtyB8mDNiRV284qns8NxyhLN84ob3l2oJo8l1SnlQVlPNd0+apkDJf2NDGoRIiNvwe6F6V5iw44NWlig8gOF1Tlv1TFZSvdE5uPwVlGwasw7MIFU+MxKI05QB1GAXlAAk2SNkWRCjzpq8yXwSpkxl8RkE5QPLnYOlO15RRPvZPVZqye2fAAZKCpCkrKCs\/ZZ9x6LTlhl5FFJR9bl7HvFjQoMyeOEpTDm0XVJpyAOXPR3Wd7BKnNGXjDoD2m7v7TXEJbkF4FJSVpuxzPw8HTRmVCxco86EBZb5w75LyzeQ+d9YwelFBWUHZ5+6qoOyz6HRfVFDWlyd8ldPPHBJ25YLwKCgrKPvczxWUfRadgrK9ostXubHPurIph7ZZlU05gPLnoDZOtynz8VplvghgZwiDrBWUlabsczcNF01ZQdnnJlYvhkACCsoKyj53u3CBMkcaU5qyz02tXgyiBBSUFZR97m7hAmWOn6Cg7HNTqxeDKAE+cq9sykEUus6nlE05gPIPFyizhqRsygHsDGGQtYKy0pR97qbhoikjfgKuhHL64QMFZZ+7Yr56UUFZQdnnDh0uUPa5giF4EQPyVPLBEHxZfdIpElBQVlD2uS8qKPssOvWikoChBBSUFZR9Hh4Kyj6LTr2oJOBRAjBlnUo+VKBWTU7jidroU4NUSUBJoEBLQEHZhuZ3mhBtqJLKQklASSBEEnAaT5SmHKKOoD6rJKAk4AwJKCjb0A5OE6INVVJZKAkoCYRIAk7jidKUQ9QR1GeVBJQEnCEBBWUb2sFpQrShSioLJQElgRBJwGk8UZpyiDqC+qySgJKAMySgoGxDOzhNiDZUSWWhJKAkECIJOI0nSlMOUUdQn1USUBJwhgQUlG1oB6cJ0YYqqSyUBJQEQiQBp\/Ek5Jpyz549acCAATRjxgyaN2+eaJbZs2dTnTp1KCcnh6ZOner6d24zpwkxRH1JfVZJQEnABgk4jSchhXL16tVp4sSJVL58eRd8x48fT7Vq1aKhQ4dS586dqWvXrjRixAhat26dS\/xOE6IN\/UJloSSgJBAiCTiNJyGFMgBcpUoVAWXWlKElp6Wl0fDhw6lly5Y0duxYWrRoEU2ePDnsoRwTE0OdOnWiJUuWUGpqaoi6oPXPqnJbl5k\/byh5+yM96+8qKP8jM5gt+vTpQ0uXLhXaMKC8YcMGoTmvXLlSQJg16T179ghIa80XiYmJYQc3mGNUua0PHF\/eANyUvH2RnG\/vhLu8ExIKeOhOaMSbNm0SWjHblA8cOOCmGRtBGY0PkwZmOPUoCSgJKAn4K4GkpCShLDnhCbj5gsEaGxsr6rtq1SpKTk6mpk2bUt++fUne6DOrKSMfgBn\/qUdJQElAScBfCcCc6BSTYsChrCcs9q6Qf2NPC9hcvdmU\/W0A9b6SgJKAkoBTJRASKMvC0LrEmfG+cKowVbmUBJQElAT8lYDjoIwKefNT9rfS6n0lASUBJQGnSiDkUHaqYFS5lASUBJQEQiEBBeVQSF19U0lASUBJwEACYQdl2KDhuhIREUG7du0SHhxOe8yWUbvhCc8U2R\/bSfWSD\/U4qVzasngrp9NlrvVWSklJEadb9+\/f7xixWymj0+Utm0vxZyeMwbCCsnzCb\/HixeKgifZgSah7rtkycsfmgzKhLren7\/PAckKH9aec4SBzbHTjweRs5Kcf6r5itozhIO\/Bgwe73HONThAHW95hBWU+BThmzBgRC0P21HCKJmG2jOgAI0eOpI8\/\/jhPwKVgdwKj7\/GgysrKEknYVdEp5eNymC1nOMhcK1sn9nGzZQxHeXtbaQWj74cVlDGrtW\/f3rWcw9\/1AhYFQ3BG3zBbRtnEgbycuEyV6+iEzmqmXT2VM9xkzktr\/N+JZjpuD8hcr4zhJm+lKZsZYZo0Wq3BiVA2W0a57Onp6cIUA43UqYMvP0A53GTuxP6tHbaeyhhO8sa4bdu2rSOUI6Up+zA5eHrFrKZspXPbXESfsssPUA4nmaMf9ejRQzeeuE8NGICXrJYxHCYZJ5iLwgrKZu21Aeh\/prP0tYxamJv+YJAS5lcoy+awIInS62cABgTb0sYR9\/piEBP4Ukan93GIzwkTR1hB2axnQxD7Zp5PmS0jOnXFihWFucKpu+z5zaYcDjJ3AhS8jR+zZQwHectldIoNP6ygDKGZ9QH21rEC+btRGbXapuzD6VSfa3kzx6neF54mj3CTuV6wLqdtAnsq46hRo9y8dMKhj8tldIKsww7KgYSpyltJQElASSDUElBQDnULqO8rCSgJKAlIElBQVt1BSUBJQEnAQRJQUHZQY6iiKAkoCSgJKCirPqAkoCSgJOAgCSgoO6gxVFHcJWDW9coOufFFvrhF3egxk8aOsqg8CrYEFJQLdvs7qvba2APBgrIcKcyTQFC+YcOG0YQJE0RALPUoCQRCAgrKgZCqytMnCYQiIAwO7owbN44WLFhgKlqfHLbSp0qql5QEvEhAQVl1EUdIQBs4\/fPPPxfl4iiAgwYNomLFionLDWJjYyk7O5tmzJhBAwYMoMjISPF3+VgyB5hBHp4OBOCgT\/fu3enFF190BZL3dJggHI4KO6JBVSF8loCCss+iUy\/aLQFP5gtAGeCeOnUqbdiwQUTVK126tAAxnrFjx1JSUpIIDq81exiFlsR72mO2WuhqTwSGY4xgu9tJ5RdYCSgoB1a+KncLEvAGZWTFoU1l0Gpjh2hBqg0SJRdJmxZQ7t27N0FT19v0C4WJxYIIVdJ8IAEF5XzQiPmlCnZA+f333xdaNEwc8pOTk6MbBlMv+h2Dmd+XAR0OVxzll\/5QUOuhoFxQW96B9bYDyjBfWAkz6i2tNkSl0pQd2HHyWZEUlPNZg4ZzdeyCstam7Cn2r9amrHdzjBxzWdmUw7mHhUfZFZTDo50KTCnlm7OTk5PdvC\/M2pR5Aw\/X++AxMl3gN2\/eF9p3lfdFgemKIauognLIRK8+7AQJWPVT9mbucEKdVBnCWwIKyuHdfqr0NkhAneizQYgqC9skoKBsmyhVRuEsATNxLcykCWcZqLI7QwL\/D8Oce3U18w0hAAAAAElFTkSuQmCC","height":215,"width":357}}
%---
%[output:3627f8b6]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWUAAADXCAYAAADC8EBxAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX9MHdeVx0\/iGicOtmUnNX1uftGwQdptm63ZhLg\/nKg\/1F3EojVyUMpGCqqhsYVoZS81qoWgtpC1pKy9WYTiFtMFybUjXNUqQVm1UhLidpsSL1Ru+49dVk4dDIvjJrEhTkLqZHUmvS\/z5s17c897M3NneN+RLAzv3Dt3vvfcz5x35s69161bt+59wgEFoAAUgAKRUOA6QDkS\/YBGQAEoAAUsBQBlOAIUgAJQIEIKAMoR6oy4N6WlpYXq6+uTl7G4uEh9fX00PDwcu0vr7u6mzZs309GjR6m3t9f39tfV1VFzczOdO3eOGhoafK8fFcZXAUA5vn0XqZa7QWxwcJDKy8t9BduhQ4foueee8wX03Obp6WlX6ALKkXKvgmoMoFxQ3R3Mxaqo79e\/\/jW1tbUFcxIiYsiXlpb6En17Qdfr83wvEpFyvgou3fKA8tLt29CuTAIwFT1z45zpDQXd06dP07333mu1\/8yZM9bXe3UOdVEnT560bgCZ\/s52lZWV1NXVRcXFxVaxmZkZam1tperq6pQ0izqHXTBV76lTp+iee+6hoqKiZHlOOfDhPLcz1ZHtc0A5NPeM3YkA5dh1WfQarBvBOu2cMFfAVsB12jt\/VzlsBUP753\/84x8tIF+5csUCMUPensP1upGozxWw1bmcNwN1bunngHL0\/DgqLQKUo9ITMW6HDpRV1HrhwoXkgy3n33Sg7Za+sEekKvouKSmxomEFUae8ulBW0FVtZcgfPHiQdu7caVXJwOfImdvV09Nj\/c3rc7ebRIy7H033WQFA2WdBC7E6L8CxJm6RoR10DKrOzs6UnLFbJG2HsopOFxYWqL29nXbs2JEsX1FRkXX2hFebnZ8r6K5evZqOHDlCjzzySDIKt0NZ53Nu6x133IHZF4U4WDSuGVDWEAkm2RXQedAXRKScLb2BSBleG1cFAOW49lzE2q0zJU6annDWyb\/ff\/\/9ydkX9il3c3NzVuTJB8+NduaU169fb+WYVfrEmY\/OlN5wPhxETjlijrcEmwMoL8FONXVJOi+P6My+UC+cOKGsInKeCcEP4J588smU2RX8N\/u86EyzLzjdYP9MgVfNqmD97A\/6uE4+nLM0MPvClKct7fNGFso8AB999FHat28fjY+PL+1ewNVBASgABf6iQCShrCIifpLOD0UAZfgrFIAChaJA5KDMX4G3bt1qvUp73333IVIuFE\/EdUIBKGApEDkoq35B+gIeCgWgQCEqEFsoJxIJY\/312kcraPntFVrnX\/fqBL17fkLLNiijO+lP9NDdN2hV\/8v5tTQ7O0u3rlpGmxLL08p87+xNWvVE3ejS8g204tPVns2MQv95NlLD4POr3nDtT3vR42ffTvrJsZnVNL1wjZYtXKRdG937fKn4AmvAPh+VI5ZQZiBzrnnjxo2h6\/i7P11He361jD51s\/eGLXNvEV28eh3t\/+w1Lft8LoYHT2L4G\/TqP3TRO4lPplR16w\/\/if5cvJ6uFa\/Pegqu4yMLF5M273wstR7+nOt4taorn6aGUpb7iA\/W3u34x6c\/QutXvk8lN2ZuTpj955coK2Z\/Tx\/9r3aarftBsr+Vbzj7035O1ffsJ3ywH7z2hW\/Sul\/8h6vvxMkXdLSdnJy0ZvJEAc6xhDLDmKdNmRBx+e0b6X\/\/+jG688Xv0nuXs99dr1+ToJc3fZdWPruXbnl3hj7zmc9QY2NjIO2+tXgZ9ZWfp29N3EAv080pfvjTigv0vbMriaPgbIeqQ9k0n7ndipZUu98Z\/g5d\/MNp+teZUh0\/N2pz9YudxDdvhrKbn5z\/++9T8cQPiCPhTIez\/8K6oHz85P7EcvrOhpdJ9R23OZtvqGv69t1vEkfT7Cd8fPvuq9b\/+eexmVX01OzqlMtnewaz3RfyaXdY2rqdR7W7trYWUM7WEdlyygrKJkTktMWaRw7Ra301WlBe1zxCl49st1IYDImqqioaGBjw3Qc53fDiw7dQ3ejr9OLsYkr9rzSV0K4XrtDxs29lh\/Jf6lBGm566RNPz15Lt\/pup41Zag88R9WPNPx+y2v2510dc9b5lzymaH91L7\/x2NCuU7f0X1jXn4yebEkU0XL2WVN9ZUM7iG+qaDjywmh66+0bLT\/jg3\/n\/\/PPAxAIdnHwz5fL5705fyKfdYWnrdh6TPHFrT6wj5bhBOUjHCxLK9oEbJyhzuy\/\/aLur7FGGcj5+YhLK+bTbZFlA2Qf1TYqYT6Tsw6VnrAJQTpWGI2VA+YN8eliRcpD+HWTdJnkSq0g5WyeYFBFQjk\/6AlAGlHVgbpIngLJOD3nYAMqAsg9uFFgVSF\/IpQWU5ZqllTApIqC89KFsnwO\/tvmntPD0PuNzzXWHDc+04Ad9\/DCWZ86o41cP30L\/8sKVtIfA6vOdG2+yHvQdnFyg6fn36N8eWG3Z809+QOx80Mf2mzYURf6hr84UN5M8QaSs69lZ7ADlpQ1lk3PgfXBPVOFQQGf+MaDsg9uYFBFQXtpQNjkH3oehgSpsCujOPzbJE0TKPrgsoFwYUDYx3dIH90QVNgV0YatrF5a4kZ2njNkXMhfAlLhUvXKdEhe1ASrzAljbFdDtS127sNQFlIVKI1JGpCx0mVDN7buz2E+sduXOpzG8awxvu9XW1pZPNa5leRcX3lexoaFBVDfvINPR0UFDQ0M0PDycUlYXtrp2ooblYQwoC8UDlAFlocuEas5Qbmpqov7+\/jRI5dMQtZv31NRUIFBm4PMhhXK269WFra5dPvpJygLKErWIrCU7c137QngqkTnSF0hfsAKZIKWgOj8\/T\/x\/++ayxcXFtLCwYK28yAdHnleuXKE777yTePcftq2oqKDNmzcnf7dHpfa9GdU+hup8GzZsSJY5deoU9fT0ELdB7XvIETwf9fX11k\/+XW2Cy3sx8qE2q3XWeezYMfrKV75CfA63fRZ1YatrJxqQeRgDykLxAOXCipR5pbgoH86VCt3SFwzKvXv3JoHI0agz8uX0QVlZmZUGYMjyVDJOU6gIVpV3RspqA9qRkREaHR21zjE2NmZBXEW+XF9NTQ319vZa+24ylLkN6pytra3U2dmZtP\/6179Ozz\/\/PPFGtlz2wQcfJLbZvn271Ub+f3V1tfX3n\/\/859ZORW7fDHRhq2sXlh8AykKlAeXCgvLKLzTRyi98Q+gl4Zlf\/cUP6Oov+pMn9IqUFVSduVi1KiNHwI888ggxZBmiKtebCcpuqzkq4DOcuQ51rqefftqKbFUb7MC1Q1lF\/M3NzcTRsoqC2caZ00b6Ijxfy3omk3c2QLmwoMyR8rI1GyLi+enNuHZ5JmUJ2aUAZY7OP\/7xj1vpFN6nU0XKgHJk3ZCsHUc4z2ViLimgXFhQjvAwcG2aLpS90he6kbJb+oIjYZ5JkSl94RUp22d58P9XrVrlmr7glMjPfvYz+upXv4r0hWlHBZTTewAP+lI1KdR5yrpQZrUUUJ0P+ninFieUOQfMgFQPCe0P+ux5bJVq4Po5v+z2oM8Nypwv5geJzgd\/\/HDwrrvuSu5qz22wPyR86aWXrJ1l+MEk55o5D60OXU7o2oXFPeSUhUojUkakLHQZmBtSQBe2unZhXQagLFQaUAaUhS4Dc0MK6MJW1y6sywCUhUoDyoCy0GVgbkgBXdjq2oV1GYCyUGlAGVAWugzMDSmgC1tdu7AuA1AWKg0oA8pCl4G5IQV0YatrF9ZlAMpCpQFlQFnoMqGau73R5\/YKslejgl7rwn7+XBcjcntxxV6vLmx17bw08+tzQFmoJKAMKAtdJlRz55Q459t1uo0JE8q5LkakXt\/ml0zGx8fTLk0Xtrp2utrlawcoCxUElAsLyjz\/O8rH9PyH+\/BxO72gbF88iO3Vkp7ORYXsr1X\/5Cc\/seYCX7hwIW0VN+e8YX6tOojFiJxzodUaHTzHWi2C5OwnXdjq2oXlB6FCWXWgWnnKuf4pX7Rbh+a6TmoQIgLKhQVl3iB0V0VxEK7kS50HJhZSNjV1S1+oVdb4hAxfBicfKm3w5JNPWtC1Lyr0m9\/8hng7JQYxv\/Ks1rGwN9oeqd5xxx3WYkP79u2jHTt2WGb8wokfixEdPHiQdu7cmVwzQ73xNz09bS10hEg5R1eyrwjFKzxlEtP+VSbT1xOTdzZAubCgzJHybcXRjZZfWbhG9mg5U6RsX93NGRU\/88wzSaCqNIA9OMoUibrlgoNYjIij4sceeyxtIXukL3KEsSpmf5\/d\/r68umvbo2TlQJleGQWU0zsDr1mnaoLXrD9c5N650hsrpZbC5DUqskE5W6QcBJTdFiMClPOEr1tx590z20MEnYhaQZmX9uN1X8M8ECkXVqQcpm\/5ca5skfKhQ4eS6x2rtY95bWO39AXP2OB1KzhAmpiYSIukVSpEfePl9IXa8aSqqipj+iKXxYjc0hd8Am5XvumLRCJB\/M\/UAmdufR5KTtkZGXs92WUw8+IkmabyKCjzBR0+fJgGBgb88GetOgBlQFnLUQwZeU2JU2OLn+ucPn3ayhdz1HzvvfeSff3iTCB0btek6uPLzbRDCANP7TqS62JEXD\/nve2LJ\/GNgNvMixC5bSOl841627Zt1NjYaPWWiVUnjUFZN1J2wtorfcGdxJHy7OxsaEMAUAaUQ3M2nCgvBXSgzFEyR\/YM5oKCMiurk1PWeUjBdemInVdvZikMKAPKQfkW6vVXAV1O6Nr527rMtYWSvuDT6+SK3SJl\/nry4x\/\/ODmNB1B270w86EvVJd8Hfex3YX4DC2vAF9J5dHPFBQtlFS3zAtX2ecrOvcLsC2\/b81R2ZzIpYiFFyrf1z6WM4QMPrCaGf93o65Ef27lCmQcyz3tlH8MRfwU4vck32GyHSZ4Yyyn73bUmRQSUlzaU2VfVE3nen29VdSfNj+5N2QfPb3\/2s777E8tp18Zi2vXCFZpe+OBtv1uLlxHfUO1\/c56TX5LZlCiiA5ML1kdcB\/+ffx7\/w1t0\/OzbKUXYno+Dk2\/62Xzf6+JvO17feEzyBFD2ocsB5aUPZeUmDOV1zSN0+ch2evf8hA\/eE3wVDNbh6rW06alLyZdKsqW2VIsY2g\/dfaMFbj4UxPmn861B9XlcvjV5qQ4oeymk8blJEQFlQFnDRY2Z5AtlTk0xbAFlY11IoT3o8\/MSAeV0NYN40FeIOWW7soUYKQPK4U2vzcREQFl4t0CkjEhZ6DKhmiNSlsttMshDTlneX2klAGVA2Qc3CqwKQFkuLaAs1yythEkRAWVA2QcXDqwKQFkurUmeIFKW9xciZZsChTBPGTllPOjzARN5VYGcslA+RMqIlIUuE6o5ImW53IiU5ZohfaGhGWZfpIqU6xt9iJQRKWsMt0BNECkL5UWkjEhZ6DKhmiNSlsuNSFmuGSJlDc0QKSNSZgX4rTzO\/ef6Rh\/mKWOesgZu0k1M3tkQKSNSzslpQyoEKMuFNskTzL6Q9xdmX2D2RazWvgCU5YMcUJZrhvSFhmZIXyB9gfSFxkBxMQGUc9MtpZRJEZG+QPrCBxcOrApEynJpTfIE6Qt5fyF9gfQF0hcTC2nrJsfpRSKvYQ8oeymk8blJEREpI1LWcFFjJoiU5dKb5AkiZXl\/IVJGpIxIGZGyD+TQrwIvj+hrZVkiUkakLHSZUM0RKcvlRqQs1yythEkRAWVA2QcXDqwKQFkurUmeIH0h7y+kL5C+QPoC6QsfyKFfBdIX+lohffHAamv\/Nn4VN+pHoS5IhEhZ7pmIlOWaIX2hoRleHkkVCVDObTdrrH1RYGtfDA4OUnl5OS0uLlJfXx8NDw+74kbZ8YcnT56ktra2FDuTd7a45ZR1tpdX4ipb9Ts2Tk3QuuYRpC+QvtAIi\/wzCS190d3dTWVlZdTa2krV1dVUU1ND7e3tND4+nnI1drv169dTR0cHDQ0NpQAcUE53gEzwBZS3u46WW\/acovnRvfTOb0czjqY47maN9IUcjiZ5YvRBH0e\/c3NzVtRbWVlJXV1dNDIyQr29vcl2lZaW0v79++nEiRMZo2g2NikiImXklOXDPrwSgLJca5M8MQZlhm1PTw+NjY1ZEFa\/T01NpaQmGNa7d++mV199le655x6rvdnSF83NzTQ5OSnvhTxKAMqAch7uE3hRQFkmcSKRIP7H6dTa2lqanS2QnLIzMs4GZY6gL1y4QA0NDVRXV0dNTU3U39\/vmr5g+Q8fPkwDAwOynsjDGlAGlPNwn8CLAsoyibdt20aNjY1WoYKCsiRStueQM8Fbfd1ggHOkHObdDVAGlGXDPlxrQFmmN0fJVVVVFpgLCsosk25OWSfNYTIHBCgDyrJhH641oCzX2yRPjOWU+cSS2RclJSVW+qKlpYW2bt2aNn3OpIiAMqAsH\/bhlQCU5Vqb5IlRKKto2TlPmfPNzmlv9nnKR48eTZmhwfWYFBFQBpTlwz68EoCyXGuTPDEOZblc7iVMiggoA8p++XEQ9QDKclVN8gRQlvdXWglAOf5QVi+F4OWRD92bdxJhoOM16wKZEucDC1OqMHlnA5SXDpQvH9lO756fyOieeKPvCjGsD+A1a78RlrW+0F6z9vOqAOV0NfGadaom2RYk0oWtrp2fvp1vXUhfyBU0yROkL+T9hfSFTYE4bZYJKGOVON3hDijrKpXFzqSIUU9f7HrhCh0\/+1ZSPSxIlL4gkW4ErGvng0v7VgUiZbmUJnmCSFneX7GLlAHlD7oMkTIiZd3hDijrKoVIWaSUiogB5Q9kW9s8Qu+9MUOXf4RIWefbEmZfFNiCRCK6aBibvLMhfRGP2ReAMiJlDZRYJiZ5gvSFbi9lsQOUAWUf3CiwKpBTlksLKMs1SythUkRAGVD2wYUDqwJQlktrkieIlOX9lVZixaeraVV1J13af69nbWE+vUdOObU7kL5A+sJzgP7FAFDWVSqiD\/oAZUTKPrhwYFUgUpZLCyjLNYtU+gJQBpR9cOHAqgCU5dICynLNAGUNzZC+QPqCFQCUNQaLwwRQlmsGKGtoBigDyoCyxkBxMQGUc9MtpZRJEZG+QPrCBxcOrApEynJpTfLErbVYJU7Yh4AyoCx0mVDNAWW53ICyXDOkLzQ0Q\/oC6QukLzQGCtIXuYnkVcrknQ2RMiJlL\/80+TkiZbn6JnmC9IW8v9JKAMqAsg9uFFgVgLJcWkBZrhnSFxqaIX2B9EU+6YudG2+iXRXF2KNvFnv0aeAm3cTknQ2RMiLlnJw2pEK5RsqAcoEu3Tk4OEjl5eW0uLhIfX19NDw8nNVV2Z6PhoaGFDtAOV02RMqIlBEp53bnM8kToznl7u5uKisro9bWVqqurqaamhpqb2+n8fFxVyVbWlqovr6ezpw5Ayhr+BqgDCgDyhoDxcWkYKHMUe\/c3By1tbVRZWUldXV10cjICPX29qbJxJ\/v3r3b+vv8\/DygrOFrgDKgDChrDBRA+QMFSktLqaenh8bGxiwIq9+npqYsSDsPBfCSkhKkLzT9DFAGlAFlzcHiMCvISNkZGWeDcl1dHW3ZsoX27NlDnZ2dWaHc3NxMk5OTufVEjqXwoA8P+nJ0nVCK4UGfTOZEIkH8j59x1dbW0myhzL7QjZTZbv\/+\/XTixAnrIaDXgz6W\/\/DhwzQwMCDriTysAWVAOQ\/3CbwooCyTeNu2bdTY2GgVKigo8wXr5JQ5Subot6ioKEVZ58M+9XWD89IcKYd5dwOUAWXZsA\/XGlCW6c1RclVVlQXmgoOydPaFAjn\/xJQ4b0dDThk5ZeSUvceJm0VB5pSVEG7zlDnf3NHRQUNDQ2nzlr3SFybubIiUESnnNvTDKYVIWa5zQUNZLpd7CZMiAsqAsl9+HEQ9gLJcVZM8cWst1lMW9iGgDCgLXSZUc0BZLjegLNcsrYRJEQFlQNkHFw6sCkBZLq1JniBSlvdXWglAGVD2wY0CqwJQlksLKMs1Q6SsoRlmX2D2BWZfaAwUFxNAOTfdUkqZFBGRMiJlH1w4sCoQKculNckTpC\/k\/YX0hU2BAw+sJo7I60Zf90HJYKtY2zxC770xQ5d\/tD3tRNevSdC65hG6fGQ7vXt+ImNDdO2CvRJZ7YCyTC+2BpTlmiF9oaEZ0hdIXyB9oTFQkL7ITSSvUibvbEhfIFL28k+TnyNSlqtvkidIX8j7C+kLpC880xw+uJVvVQDKcikBZblmSF9oaIb0BdIXSF9oDBSkL3ITyauUyTsb0hdIX3j5p8nPESnL1TfJE6Qv5P2F9AXSF0hfTCzQwck3U8ZCnGbieA17QNlLIY3PTYqISBmRsoaLGjNBpCyX3iRPECnL+wuRMiJlRMqIlH0gh34VWCVOXyvLEpEyImWhy4RqjkhZLjciZblmaSVMiggoA8o+uHBgVQDKcmlN8gTpC3l\/IX2B9AXSF0hf+EAO\/SqQvtDXCukLrH0h9JbwzREpyzVHpCzXDOkLDc3w8kiqSFiQ6BJNz1+zRFG+wQtJvTi76OpNOzfeRLsqiq3Fptiep7zteuGK9fMAImWNEeifCSJloZbIKSOnLHSZUM0RKcvlRqQs1wyRsoZmiJQRKbMCgLLGYHGYAMpyzQBlDc0AZUAZUNYYKC4mgHJuuqWUMiki0hdIX\/jgwoFVgUhZLq1Jnri1NtSc8uDgIJWXl9Pi4iL19fXR8PBwWptaWlqovr4++fejR49Sb28voOzha4iUESkjUpYDmUsULJS7u7uprKyMWltbqbq6mmpqaqi9vZ3Gx8eTSlZWVtLu3bvp8ccft\/7OgHazMykiImVEyrkN\/XBKIVKW62ySJ0YjZY6S5+bmqK2tjRi+XV1dNDIykhYF2xuZyc6kiIAyoCwf9uGVAJTlWpvkiTEol5aWUk9PD42NjVkQVr9PTU1ZkM501NXVUVNTE\/X396ekOkyKCCgDyvJhH14JQFmutUmeGIOyM+LVgXI2GyVic3MzTU5OynshjxKAMqCch\/sEXhRQlkmcSCSI\/\/EzrtraWpqdnZVVEIB1KA\/6pJGysp+fn6eGhoa0y1ZQ5g8OHz5MAwMDAUjjXiWgDCiH5mw5nAhQlom2bds2amxstAoVFJT5gnVzyiqq5gg4U2pDQZnz0mwX5t0NUAaUZcM+XGtAWaY3R8lVVVUWmAsOyjqzL3TSGiy5yRwQoAwoy4Z9uNaAslxvkzwxllNWJ3abp8yRcUdHBw0NDVlmnCcuKipKaatzrrJJEQFlQFk+7MMrASjLtTbJE+NQlsvlXsKkiIAyoOyXHwdRT1hQ5lXl6u6+kTY9dSmIywi1TpM8AZR96GpAGVD2wY0CqwJQlksLKMs1SythUkRAGVD2wYUDqwJQlktrkieIlOX9lVYCUAaUfXCjwKoAlOXSAspyzRApa2iGBYlSRcLOI8HuPIKcssagzNEklJdHcmxbxmIm72yIlBEp++3PftaHSFmupkmeIH0h76+CSl\/wxb7SVJK85tv651Kun\/dr44ic93GL+oFIGZGyro8CyrpKZbEzKeJSjpQB5VSnu35NgtY1j9DlI9vp3fMTPnhu8FUgUpZrbJIniJTl\/YVI2aYAImUfHCjgKgBlucCAslyztBImRUSkjPSFDy4cWBWAslxakzxBpCzvL0TKiJSRvphYoIOTb6aMBcy+8AEmGarA7AuhtoiUESkLXSZUc0TKcrkRKcs1Q\/pCQzM\/5injQR8e9PHMGvYlfn6w64Ur1s8DiJQ1RqB\/JoiUhVoiUkakLHSZUM0RKcvlRqQs1wyRsoZmiJRTRcI8ZcxT1hg2lgmgrKtUFjuTIiJSRqTsgwsHVgUiZbm0Jnni1lqkL4R9CCgDykKXCdUcUJbLDSjLNUP6QkMzpC+QvmAFAGWNweIwAZTlmgHKGpoByoAyoKwxUFxMAOXcdEspZVJEpC+QvvDBhQOrApGyXFqTPEFOWd5faSUAZUDZBzcKrApAWS4toCzXDOkLDc2QvkD6AukLjYGC9EVuInmVMnlnQ6SMSNnLP01+jkhZrr5JniB9Ie8vpC9sCmDpTh8cKOAqAGW5wICyhmaDg4NUXl5Oi4uL1NfXR8PDw3jQ56Eb0hdIXyB9oQEXpC\/kInV3d1NZWRm1trZSdXU11dTUUHt7O42PjycrM3lnQ\/oC6Qu5V4dXApGyXGuTPIlF+oKj5Lm5OWpra6PKykrq6uqikZER6u3tjT2UE4kEVVVV0TPPPEOzs7Ny78lSIshIWbX7y2\/+kpYtXIzNHn0lN75Pn3t9JE1v3W2edO187UgiysdPTEI5n3b7raGkPkA5i1qlpaXU09NDY2NjFoTV71NTUxak1aFEbG5u9h1uXp352kcraFV1Jy3\/zxovU5q7StYebyuf3Wvt8cZOy+mYINrNsHzx4Vvoe2dX0vGzbyfbpv7+rYkb6MXZRc82v1T9XtLmvtHrrf+rdr\/8RENsNk59u\/b79Lef+Bjt\/+y1NL2d\/ZJJFF07T1GFBvn4yedXvWEtt1k7tpymF65ZZ9bxgYcTV2hXRTGxn7C9fenOYzOr0ha5Z\/tvPvAJ+uxTl5JXl0+7hRL5aq7aXVtbGzpPIh8pOyPjTFBmEZ\/f+aCvHaNb2cWrRL\/703X0pdve1yry7CvX0adufp\/Wr9Qyz8vopj88R+987JP051XrU+rhvz9x1356Zc0nPevvmfzwZtO6cSTF\/qFzT1Dl68\/Sm3\/1Rc96TBuw7nxk6ifdftG1M3296vwfmb9IK\/7v92l95OUDWy4do8+dP2b5yYZrc\/TQy09YvsR1nbjla\/Tft38t5RKVfVR9gdt+VeCnk5OT1s07CkekFiTSjZRZuOHqtVHQL1Jt+J\/ln6K\/e\/d3aW16+oYv08iKL2m1lctveO8izVy\/nrg+57F3\/t9pw3tzWnXByIwCM9eXpPWRjg+ofudWP3b1qOVL7APfX1nveiFR9oWX37+Znppdrd0BnE70O6WofXKHYaSgzG3TySnnerEoBwWgABSIugKRg7LO7Iuoi4r2QQEoAAVyVSByUFbRcrZ5yrleLMpBASgABaKuQCSp5wSVAAAElUlEQVShHHXR0D4oAAWgQFAKAMpBKYt6oQAUgAI5KBA7KNfV1VlTV4qKiujMmTPU0NCQw2UHW0S3jep1ctWakydPpszHDraVstrtD2BlJcO19mpn1DVXM5A2bNhgCTczM2O93Xru3LlwhcxyNkkbo663PV3K\/4\/CGIwVlO3zmEdHR60XTZwvlpj2XN02Oqf\/mW53tvOrgRUFh82nnXHQnB9088EvS2Wap2\/aV3TbGAe9W1paqKKiwgruMr1BHLbesYIyR6CPPvoo7du3z1oLwz5TIyqRhG4b2QE6OjpoaGgobcGlsJ0g0\/nUoJqfn7dM1OvvUWmfaoduO+OguVPbKPq4bhvjqLfXN60wfD9WUOa72oMPPpj8Ose\/uy1YFIZwmc6h20Z7iiOqX1Pt1xgFZ9Xp12ztjJvm6qs1\/4ximk71B2vu1sa46Y1IWWeEOWycUUMUoazbRnvbL168aKViOCKN6uBbClCOm+ZR9G\/nsM3WxjjpzeN28+bNkcjhI1LO4eaQrYhupCxxbp+bmFN1SwHKcdKc\/Wjr1q2u64nn1IEBFJK2MQ43mSiki2IFZd18bQD+p11lrm10wlz7hCEZLlUo29NhIUnpeRoGA6+E6FxH3LNgiAa5tDHqPs7yReHGESso685sCNE3006l20Z26pKSEitdEdWn7EstpxwHzaMABa\/xo9vGOOhtb2NUcvixgjKLpjsH2Muxgvw8Uxud0aZ9DmdU51zbH+ZEdfZFtptH3DR3zuvla4vaXOVsbezs7EyZpRMHH7e3MQpaxw7KQcIUdUMBKAAFTCsAKJvuAZwfCkABKGBTAFCGO0ABKAAFIqQAoByhzkBToAAUgAKAMnwACkABKBAhBQDlCHUGmpKqgO7UKz904yfwExMT1i7qmQ4dGz\/agjoKWwFAubD7P1JX71x7ICwo21cKyyYIt2\/37t30+OOPWwti4YACQSgAKAehKurMSQETC8Lwizv79++nEydOaK3WZ1+2MqeLRCEo4KEAoAwXiYQCzoXTjx49arVLrQK4Y8cOWrFihbW5AS8Av7CwQP39\/dTU1ETFxcXW7\/bXktUCM1xHthcC+EWfLVu20J49e5ILyWd7mSAOrwpHokPRiJwVAJRzlg4F\/VYgW\/qCoczg7uvro1OnTlmr6q1evdoCMR9dXV00OTlpLQ7vTHtkWlqSyzlfs3VC1\/lGYBzXCPa7n1BfsAoAysHqi9oFCnhBmatSS5vaQetcO8QJUuciUfYmOW0ZyvX19cSRuttDPxMpFoGEMF0CCgDKS6ATl8ol+AHlQ4cOWVG02uNOabO4uOi6DKbb6ncKzKqsHdBx2OJoqfhDoV4HoFyoPR\/B6\/YDypy+kCwz6mXrXKISkXIEHWeJNQlQXmIdGufL8QvKzpxytrV\/nTllt51j7GsuI6ccZw+LR9sB5Xj0U8G00r5z9vT0dMrsC92csnqAx9v78JEpdcGfec2+cJbF7IuCcUVjFwooG5MeJ46CAtJ5yl7pjihcE9oQbwUA5Xj3H1rvgwJ4o88HEVGFbwoAyr5JiYrirIDOuhY6NnHWAG2PhgL\/D5J2kFeOJMelAAAAAElFTkSuQmCC","height":215,"width":357}}
%---
%[output:6b248595]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 9.730042 seconds.\n","truncated":false}}
%---
%[output:8ecc8a8b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWUAAADXCAYAAADC8EBxAAAAAXNSR0IArs4c6QAAFf5JREFUeF7tnV9oHFeWh8\/I+oOwI0weImxwGDFhs7CwIcmQMHgwmdcQFCJEk\/glYkFPit6MBEErQ7Yx2PSbrCe1lgh2nEEEzPRDHjMas2bQS+d1DQIvXmLRCTsYy0bI8mh2T81cp7rU3aruul11q+5XMEwkVd065zunf3187q26P3v55Zf\/KhwQgAAEIOAEgZ8hyk7EASMgAAEIBAQQZRIBAhCAgEMEEGWHgoEpEIAABBBlcgACEICAQwQQZYeCgSkQgAAEEGVyAAIQgIBDBBBlh4KBKRCAAAQQZXIAAhCAgEMEEGWHgoEpEIAABBBlcgACEICAQwQQZYeCgSkQgAAEEGVyAAIQgIBDBBBlh4KBKRCAAAQQZXIAAhCAgEMEEGWHgoEpEIAABBBlcgACEICAQwQQZYeCgSkQgAAEEGVyAAIQgIBDBBBlh4KBKRCAAAQQZXIAAhCAgEMEEGWHgoEpEIAABJwV5VKpJJ9++ql88cUXsr29TaQgAAEIeEHASVFWQZ6bm5Nnz57J0tISouxFKuIkBCCgBJwT5fn5eZmenpZvv\/1W3nnnHSpl8hQCEPCKgHOibOjTvvAqD3EWAhD4O4HcivK5c+cCF3Z3dwkmBCAAgcIQyKUoqyBrr\/mtt96SarUq6+vrhQkIjkAAAn4TyKUoqxivrq5KuVyWer1Otex3DuM9BApFINeiPDU1hSAXKh1xBgIQQJTJAQhAAAIOEXBWlDsxMu0LKmWHMglTIAABKwQQZSsYGQQCEICAHQKIsh2OjAIBCEDACgFE2QpGBoEABCBghwCibIcjo0AAAhCwQgBRtoKRQSAAAQjYIYAo2+HIKBCAAASsEECUrWBkEAhAAAJ2CCDKdjgyCgQgAAErBBBlKxgZBAIQgIAdAoiyHY6MAgEIQMAKAUTZCkYGgQAEIGCHAKJshyOjQAACELBCAFG2gpFBIAABCNghgCjb4cgoEIAABKwQQJStYGQQCEAAAnYIIMp2ODIKBCAAASsEEGUrGBkEAhCAgB0CiLIdjowCAQhAwAqBVEX5yy+\/lNdff12ePXsW7Ea9ubl5zImJiQmpVCpy\/vz5tuexHZSV2DMIBCDgIIHURPn69evy2muvyZUrV+SDDz6QyclJWVpaku3t7SYsKtx6zMzMyPz8fMvzEGUHMwmTIAABKwRSE2UV20ajIYuLi\/Luu+9KuVyWWq0mKysrLxwxVfLOzk5wXqlUktnZWVlbW2uqqhFlK7FnEAhAwEECqYiyEdutra1AhKPiG+YSp6JGlB3MJEyCAASsEEhFlKOVcSdRVq9UmC9duiQPHz4M2h33799vctaI8tzcnNTrdSsgGAQCEICACwRSEeW4lXK37QsFWK1WZX193QWW2AABCEAgMYFURFmtjNNTjvaQ21XUplLWvrRWyru7u4lBMAAEIAABFwikJspxesWtKmVtUXz99ddNE4L0lF1IHWyAAAT6QSA1UTbVcnSdsvabl5eXZWNjI1hhYfrPZ86cCfy9c+dOsBIjfCDK\/UgFxoQABFwgkKoo23IYUbZFknEgAAHXCCDKrkUEeyAAAa8JIMpehx\/nIQAB1wggyq5FBHtySeDo6FU5OPhYDg9\/LWfOfCYDAw9y6QdGZ08AUc4+BliQcwKHhxfl6dObosKsYnz27Js59wjzsySAKGdJn3vnmoCpjvf3F2Vw8K4cHV2QoaG7cvr0Z7n2C+OzJYAoZ8ufu+eUgAry48e\/D6wfGflKRkZ+J48efScvvTQZCDMHBHolgCj3So7rvCQQro61VTE29mHQstjfX5CDg09oXXiZFXadRpTt8mS0AhNQQX7yRHvHF4LqeHT0xgtvHz+uyalTD2hdFDj+abmGKKdFmvvkmoBWwto7DlfHxiGd6Nvbq9G6yHWE3TEeUXYnFljiIIFwu2J09HpTdWzM1ZUXKsysunAwgDk0CVHOYdAwOR0CZqmbmcwLtyuMBSraOsHXTrDTsZS7FIkAolykaOKLFQLRpW6dHgahdWEFOYOECCDKpENhCRy9eiQHHx\/IyO9GZODBQCw\/o0vdWlXH4YF0gk8n\/mhdxMLLSTEIIMoxIHFKvgioGB9ePJShu0Py6LtHcvbNs4E4D94dDH7X6mi31K2T57Qu8pUXebEWUc5LpLAzNoGDTw7k6c2nMnp9VPYX92XkqxHR3+nPozdGj43Taalbp5vSuogdEk7sggCi3AUsTnWfgFbJemhlrIJsDhVmFeRoG0Mf+NDVE62Wup3krbYu9BgbmzzpVP4OgdgEEOXYqDjRJQKm96s9X32Qwxz7C\/tNYhy1OVot\/+rgnPzy6bT8y8C\/yWenT8vdodbtjeg4pnWh77kI398lRtiSTwKIcj7j5rXVRpBfPTqSfzr9gfxp5KeNc02lrOKsLQtzqBi3m\/C7eHgoN58+FR3vwcCA3Bgdla9GRjoy1gpbHyhhgs\/rVOyL86mKsu5oHd2jr5VX5jz9G3v09SXuuR00LMh\/HPiFvHn27DFfTLUc7iVrK+P0Z6eD\/nK7Q0V5YX9fPjk4CMRZhVkFutVB6yK3KeS84amJcpzdrJVW+LxXXnmlaVNVQ5M9+pzPq74YaFoGP5f\/lvsyEbQbWlW0uvLi+cXnQWVsVl\/o77Sf3G71RdhgFeePDw5kcX+\/pTgbO3gjXF\/C7P2gqYmyVr+NRiPYmdrsWF2r1WRlZeVFECYmJuTatWty+\/btYGfrdgei7F\/eGiHUCbn\/GPoH0ZZDqyo5Skar5lYrLuIQbCfOtC7i0OOcXgmkIsoqtpVKRba2tgIRNj\/v7OwEIm0OFeuFhQX58ccf5Y033gh+3al9MTc3J\/V6vVffuS4nBMKC\/M9jb8h3jx7J9dHRtq0F226FxVnH\/sXAH+XBwClWXdgGzXgBgVREOVoZdxLlcrks33\/\/vczMzEipVJLZ2VlZW1trqpxNpawOVKtVWV9fJ5wFJRAWZJ1U036v9n3jVMm2kag4l\/bH5fOD\/5I\/yG9kd+RPwReD9p85IGCLQCqi3E2lvLy8LBsbG4EItxNvI8oq4Fop7+7+NPtuCwzjZE\/ACLIuOdOlZyqKWiVrH1n7yVkc5mX210b+MfiCMCs2ullOl4Xd3DM\/BFIRZcURt6ccp81BTzk\/CdarpeahDiPIOo6pkj8cG8usOo2+zL6X5XS9MuE6PwikJsrdrL4YHx8P2hfz8\/MyPT0tq6urLdsXU1NTVMkFzNNWgqwV6e8fPw4e7siqSu606qKb5XQFDBkuWSSQmiibajm6Tln7zeGWRfg8\/e9bt241rdDQ31EpW8wAx4ZqJcjhKjnLNkGcffiMOMddHeIYfsxxgECqomzLX0TZFkm3xjGCHH1hvArdzSdP5MGpU5lVyUoq7j585gGULCYj3Yoo1vRCAFHuhRrXWCdg9sBrtYOHVp21vT2ZfOml2O+msG1g3DfCuTAZadt3xkuXAKKcLm\/u1oJAJ0HW02uPHwdXTY6NZcYv7j58LkxGZgaJG1shgChbwcggvRI4SZBdqJLjvszelTZLr7HgOjcIIMpuxMFLK04SZFeq5LitC62S9W1zWbZZvEykgjmNKBcsoHlxx0zqDQ7ebfu4sgtVsvKMuw+fC22WvMQfO9sTQJTJjswI\/PnP\/\/vi3vqiIfOyeBXqoaG7QS\/5wv+vT85yFUPc1oUrXyCZBZMbWyOAKFtDyUC9EFDR0\/aA7gh9ePhref78YjCMirT2aF8b\/G3wEvusdveI27qgSu4l+lzTigCiTF44R0CFeurxVGDXvx\/9a1M1PTDwPzI09J9iqul+Gx\/nZfZmGRy95H5Hw4\/xEWU\/4pwrL43ImZfYq0j\/5S8Xgio6XE2bilpbHVpZ2xbquK0LndzjCb5cpZjTxiLKTofHT+PiiFy7tocR6mh\/uheS5mX2Y2MfBqLf6oh+gfRyH66BQJgAokw+OEfALC3r9kX2KtQHBx8H\/kT70720PeK0LuJ8gTgHGIOcJoAoOx0ef43T90foHnlJ3p2sIq2HCvXfBPuTpv50p7aHaV3oe5zbTTLySLW\/+dlPzxHlftJl7EQEzDKzu4OD1h6xPqk\/HRZgFXHd7aTdkeUuKInAcrHTBBBlp8ODcaYa1S2X+vXazk796XY7Vrvwfmeyo5gEEOVixrVQXpl3Slx8\/jy1R5hVqJ88uRlwHBubPMbTVMn9+qIoVABxpisCiHJXuDg5SwL6gIYKc7cTgP2wmYdF+kGVMZUAokwe5IqAmQC02WfuFgCPVHdLjPO7IZCqKOvmqdHtoDoZq+frofv1hQ92HukmxMU714ii9pmzeC8GVXLxcsolj1IT5bgbpxo4umnq5cuX5d69e4iySxnjiC1mok3NSbOvS5XsSAIU2IzURFmr3kajIYuLi6KbpZbLZanVasc2RVXW+veFhYUA+97eHqJc4ARM4loWE4AuvLkuCTOudZ9AKqI8MTEhlUpFtra2AhE2P+\/s7AQiHT2MgI+Pj9O+cD+HMrdQn6rT1RD9ngDkkerMQ+2FAamIcrQy7iTKpVJJPvroI\/n888\/l6tWrHUV5bm5O6vW6F4HCyc4E0pgA5JFqsjANAqmIctxKWc+7du2a3L59WzY3N+WkiT4FVK1WZX19PQ1W3MNxAv2cAKRKdjz4BTIvFVFWXnF6ylola\/U7PDzchDg62WdWX2hfWivl3d3dAoUEV5IQ6NcEoFbi2iLJYrVHEh5cmz8CqYlyt6svjJDr\/7MkLn+JlaXFticAefFQltH0796pibIR2eg6Ze03Ly8vy8bGRtCyCB8ntS+mpqaokv3L2dge25oANI9Ufzg2Jro2mgMC\/SSQqijbcoSHR2yRLP445t3MvT4BaKruB6dOBeuhOSDQbwKIcr8JM37mBMITgN1Wu0bU2X8v8zB6YwCi7E2o\/XbUVLwXjo66egKQR6r9zpssvEeUs6DOPTMhEJ4ANJuydjKER6ozCZP3N0WUvU8B\/wDE3WqKKtm\/3HDBY0TZhShgQ+oETpoANMvg6CWnHhrvb4goe58C\/gLoNAHII9X+5kXWniPKWUeA+2dKoNUEII9UZxoS72+OKHufAgBQAmarKZ0A1C2ntIrmkWpyIwsCiHIW1LmnkwTMBKAa99XICA+LOBml4huFKBc\/xnjYBQEjzFol80h1F+A41RoBRNkaSgYqAgHtJ6sw3xgdRZSLENAc+oAo5zBomAwBCBSXAKJc3NjiGQQgkEMCiHIOg4bJEIBAcQkgysWNLZ5BAAI5JIAo5zBomAwBCBSXAKJc3NjiGQQgkEMCiHIOg4bJEIBAcQmkKsq65150j74o2vn5ebl8+fKLX9+6dUtWVlaaTmM7qOImJJ5BwHcCqYlynN2sdRPVhYUFuXHjhmxvb4sK9OTkpCwtLQU\/mwNR9j1t8R8CxSWQmihrldxoNGRxcVFUfMvlstRqtWNVcBh1u\/MQ5eImJJ5BwHcCqYjyxMSEVCoV2draCkTY\/LyzsxOIdLujVCrJ7OysrK2tyebmJpWy79mK\/xDwgEAqohyteOOIcqdzTKU8Nzcn9XrdgzDhIgQg4AuBVES520rZnL+3tyczMzPHYmFEWf9QrVZlfX3dl3jhJwQgUHACqYiyMozbUzZVtVbA7VobRpS1L63n7e7uFjxMuAcBCPhCIDVRjrP6Ik5bQwPDRJ8v6YmfEPCPQGqibKrl6DplrYyXl5dlY2MjoK994uHh4aZIRNcqI8r+JSoeQ8AXAqmKsi2oiLItkowDAQi4RgBRdi0i2AMBCHhNAFH2Ovw4DwEIuEYAUXYtItgDAQh4TQBR9jr8OA8BCLhGAFF2LSLYAwEIeE0AUfY6\/DgPAQi4RgBRdi0i2AMBCHhNAFH2Ovw4DwEIuEYAUXYtItgDAQh4TQBR9jr8OA8BCLhGAFF2LSLYAwEIeE0AUfY6\/DgPAQi4RgBRdi0i2AMBCHhNAFH2Ovw4DwEIuEYAUXYtItgDAQh4TQBR9jr8OA8BCLhGAFF2LSLYAwEIeE0AUfY6\/DgPAQi4RsBJUdadr6N7+YXBsR2Ua2mEPRCAgC0CzolynF2vEWVb4WccCEDANQLOibJWyY1GQxYXF0V3ui6Xy1Kr1WRlZeUFO0TZtTTCHghAwBYBp0R5YmJCKpWKbG1tBSJsft7Z2QlE2hxGlOfm5qRer9tiwTgQgAAEMifglChHK+OTRFnpVatVWV9fzxwkBkAAAhCwQcApUe62UtbWhlbKu7u7NlgwBgQgAIHMCTglykqDnnLmOYEBEIBAhgScE2VWX2SYDdwaAhDInIBzomyqZdYpZ54bGAABCGRAwElRPokDS+JOIsTfIQCBvBJAlPMaOeyGAAQKSQBRLmRYcQoCEMgrAUQ5r5HDbghAoJAEEOVChhWnIACBvBJAlPMaOeyGAAQKSQBRjoT13Llz8v7778s333zDk4J\/ZwOT1p99uBznApPk3xOIcoQhy+2OJxVMWn\/Q4EKuJJfg4yPkWpT1LXG233uh3\/Srq6vSj7H7EcA0xoRJ+0qZXGlmk9dcsa0jST6XuRRlDfzS0pJopcIBAQhAICkBl942mUtR1gCoMOv\/OCAAAQgkJaCVsivVcm5FOWkQuB4CEICAiwQQZRejgk0QgIC3BBBlb0OP4xCAgIsEEGUXo4JNEICAtwQQ5chuJ60yQXdD0fc7m+POnTtNG7kWOXvMFl3nz58P3Hz48KFcuXJF7t+\/X2S3X\/jWjf8+54kCC\/vv02fE9gfBe1E2idQuiaL7BtoOgOvj6U4weuhu4u02snXdhyT2xfXf9zyZn5+Xt99+W2ZmZiS6AXIS\/j5e660omw\/R3t5eEPdGo9Gy+tUEW15elo2NDdnc3PQxR5p8Dm\/X5Uu1HAbQzn\/ypPmjEd5r0\/sPTZcAvBXlMKdOCVQqlYKn+4aHh73853s0n5SVHloR+Xi08588+SkbqJSTfTIQ5RN6yvrPssnJyeAJwh9++EEqlYpode2jKIVZbG9vJ8u8HF7dyX\/y5G8B1X9JXLp0ybu5B5vpjCjHmOgLA\/dVmNTv6enp4L0gPrZxuvXf1zwxnxXf21xJRBpR7kGU33vvPa9WIOgHTN8zov9a8LFC7sV\/FWXf8oTiJYkU\/3QtonyCKOsHcnx8PGhX+Lj6wPeKL67\/vudJ2H+VF9\/nHpLIM6LcQpSjE3\/h9Zf37t3zqp8cXXuryebTWuVO\/l+9erVp1Y7PeWKE2Kzn9ylHkghwq2sRZdtEGQ8CEIBAAgKIcgJ4XAoBCEDANgFE2TZRxoMABCCQgACinAAel0IAAhCwTQBRtk2U8SAAAQgkIIAoJ4DHpRCAAARsE0CUbRNlPAhAAAIJCCDKCeBxKQQgAAHbBBBl20QZDwIQgEACAohyAnhcCgEIQMA2AUTZNlHGgwAEIJCAAKKcAB6XQgACELBNAFG2TZTxIAABCCQggCgngMelEIAABGwTQJRtE2U8CEAAAgkIIMoJ4HEpBCAAAdsEEGXbRBkPAhCAQAICiHICeFwKAQhAwDYBRNk2UcaDAAQgkIAAopwAHpdCAAIQsE0AUbZNlPEgAAEIJCDwfz\/fNC1FZNVzAAAAAElFTkSuQmCC","height":215,"width":357}}
%---

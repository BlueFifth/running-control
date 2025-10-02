%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88, 4.10, -0.88];
dq0 = [2,0,0,0,0,0];
X0 = [q0, dq0];
Sim.time = 1.2;
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
toc %[output:631217fd]

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
plot(t_total,x_total(:,1)); %[output:1ec0d671]
hold on %[output:1ec0d671]
plot(t_total,x_total(:,2)); %[output:1ec0d671]
plot(t_total,x_total(:,3)); %[output:1ec0d671]
plot(t_total,x_total(:,4)); %[output:1ec0d671]
plot(t_total,x_total(:,5)); %[output:1ec0d671]
plot(t_total,x_total(:,6)); %[output:1ec0d671]
legend('x','y','th1','th2', 'th3', 'th4'); %[output:1ec0d671]
title("Generalised Coordinates"); %[output:1ec0d671]
xlabel('time (s)'); %[output:1ec0d671]
hold off %[output:1ec0d671]

% display derivative state of all variables
plot(t_total,x_total(:,7)); %[output:50b72b4b]
hold on %[output:50b72b4b]
plot(t_total,x_total(:,8)); %[output:50b72b4b]
plot(t_total,x_total(:,9)); %[output:50b72b4b]
plot(t_total,x_total(:,10)); %[output:50b72b4b]
plot(t_total,x_total(:,11)); %[output:50b72b4b]
plot(t_total,x_total(:,12)); %[output:50b72b4b]
legend('dx','dy','dth1','dth2', 'dth3', 'dth4'); %[output:50b72b4b]
title("Derivative of Generalised Coordinates"); %[output:50b72b4b]
xlabel('time (s)'); %[output:50b72b4b]
hold off %[output:50b72b4b]
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
plot(t_total, contact_total(1, :)) %[output:19703cda]
hold on %[output:19703cda]
plot(t_total,contact_total(2, :)); %[output:19703cda]
legend('Front contact', 'Back contact'); %[output:19703cda]
title("Contact bool"); %[output:19703cda]
xlabel('time (s)'); %[output:19703cda]
hold off %[output:19703cda]

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
if Animate ==1 %[output:group:320e9e5a]
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


    fig = figure; %[output:98025b9f]
    
    ax = axes('Parent',fig); %[output:98025b9f]
    
    body = animatedline(ax,'Color','g','LineWidth',1, "Marker", "*"); %[output:98025b9f]
    f_u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:98025b9f]
    f_u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:98025b9f]
    f_l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:98025b9f]
    f_l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:98025b9f]
    f_foot = animatedline(ax,'Color','r','LineWidth',0.5); %[output:98025b9f]

    b_u_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:98025b9f]
    b_u_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:98025b9f]
    b_l_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:98025b9f]
    b_l_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:98025b9f]
    b_foot = animatedline(ax,'Color','b','LineWidth',0.5); %[output:98025b9f]
    
    axes(ax); %[output:98025b9f]
    
    axis equal; %[output:98025b9f]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) 0 1.5]); %[output:98025b9f]
    hold on; %[output:98025b9f]
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
        addpoints(body,x_sim(i),y_sim(i)); %[output:98025b9f]

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
    toc %[output:834af8b5]
    hold off; %[output:98025b9f]
    clear fps body f_foot f_l_l f_l_r f_u_l f_u_r ax fig l1 l2 l3 l4 l5 b_l_l b_l_r b_u_l b_u_r
end %[output:group:320e9e5a]

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
%   data: {"layout":"onright","rightPanelPercent":63.6}
%---
%[output:7f72cff7]
%   data: {"dataType":"text","outputData":{"text":"Time = 1.2000","truncated":false}}
%---
%[output:631217fd]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 309.259470 seconds.\n","truncated":false}}
%---
%[output:1ec0d671]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPYAAACVCAYAAABrcgc8AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2UFcWVvkocRH7lZ4YZGEbIInGjkhkEBGLQk3j0RDKCQVTOSZiNsBsX9KxZFj0E2SPLonFJSIIs4sCubM6BSDQkBF03ya4SIfxlhvi3iAbYAWaGgUFQfpQhyJ6v39xHvZruru7XXe91v64+h8PM1E9XfXW\/vrdu3aq6pHfv3hfIPAYBg0BBIXCJIXZBjafpjEHAQsAQ2wiCQaAAETDELsBBNV0yCMSO2FOmTKGZM2dSUVFRevROnTpF8+bNo+3bt0d+RLn9ra2tNHv2bBo5cqTVH\/59\/\/79Wvrw5JNP0vjx42nTpk306KOPOr7jwQcfpKlTp2akr1mzhpYuXaqlXV4q5Tbt2bOHampqyGtfvNRdqHliRWweUKfByLcAehGSKBP7ueeeo2HDhtl2g0nlpY9h58kVsTE2kyZNorlz55KuD2zY2DjVFxtii5pEJPDo0aNp4cKF1K1bN2pqarK0YJQHRSZ2rtqq0nKMb1tbGy1btozWrVtnyQyTXf57rgQU75GJrePd\/I44yJCX\/seG2CxgdqYkBqW6ujrDHBcJDyDEARPJtWPHDpo4caKFlSy8bnVwGsrt3buXhg8fTjwleOCBBzI0nzhV8KKx7aYb4sds8ODBtHjxYiorK7PabTcVEa0b9L2xsdEy+51MccZXtnr4Xa+99lqGOS5rd7t63fKIOAD3q666Kj1GEyZMSE8H0LfNmzfT7bffTk6muPjR6t+\/fxp7maRyexi3UaNGOU4\/5KmJ3E+5zqh8GGJBbBauvn37ZmgTpy+XTEjOx6DzvFacp8t5iouL05aA+B6uwy4dafh3ww03dGgaC6WK2E5t44\/Ozp07M0jNLxLJ7TZlsSOgH3zlj4rYUe6jlzx2Hy+Ur6ur60AyfoeK2HbywP11wsTpnfjA4ZH9Dfgb1+k0dcnntIUxiAWxmaggop2ZKAs35klwFIkAixqppaUl7YBjDcVfZiaIqg5oep4CuDmkZBNPdpapfpdNdTuTUdRYK1asSBOf2yV+6Oza6oSvHVHsTHYmKfJjfEpKSixCiBaQnAd52QkqWgk8TnYfCRWxxY+bXI\/cFxlHthL4w438bBVx+0QrDc5atsyi6NuJBbGdNIqTaSWbwuKgQrDr6+stoYLgsTfdadCctMD69estYssfG84vm3CyteDkFRcFSrYiQHI3bQzBX758udUuPOJKgdsc2w+xneoRp0p4t50HXsxjNwZO46xynnGbxA+507zcaVxkYjtZbOKUjT9gooxEheSxIDaAc5tj+yUlC5W4xOS3Dia2ikB+NTZraDtzFkIzYMAAizR2T7bEdsNXnmOHTWxxDIISW7RGVB8Dlcb2Qmx2MDp9LHLlGLWThdgQWwTPyYHhZkaLnbfzTMvEtjPFxTrk\/FhDt9N8TASvGttOGESNxHNQJyeN+EHwaoqjX05ecX43m9VezGwvedgUl9fvg5jiTsRmK0a0rlTjYmeK235NhT\/6sXxUdQVNjw2x0VHVOjabYk7OMxZOO6GSiYo8PIcWQeY6GhoaOpi8bk4jr8QWPcLy4EJjb9y40dZ5hrxsBvp1nvF7vKxje3GMecnjtOxnFyDD7VPNsZ2I\/fjjjzti5uRQdXOeocySJUvo4YcfTq9MiGMVBc94rIgN8OxIa7fcI+ezc+S4meKiBsYauTi3gglmp7GRR\/T24p2rV6+mO+64g9ijL39U7CLP7DzGotA6melidJjf5S4WTK+RZ2Eud8mxB2Ib\/C53cVSdbIqrxkVebWC8VWa2k58n31GQsSN2UBPFlDcIJAEBQ+wkjLLpY+IQMMRO3JCbDicBAUPsJIyy6WPiEMg7sUtLSwn\/zGMQCAuBzqWXUcuIJuq1sV9YVWqpp7m5mfBPx5NXYoPQiI6qqqrS0TdTZwIR+BO9R09f+n3qTX3oLy5cTVMv1EQWBQRKYUlVB7nzSmwQGrHFujoX2RENoWH4KGIn065du0KorXCq6DW\/C31Ax+jj2j9Tr\/lX0OW1Pelk\/ekOHcw3fpWVlTR9+nQrtBkED\/uJBLF1dS5ssEx90UagR1VXumb5Z2n3A3vpo\/rTdM3yIXS2+RztW3Awcg1npaZL9g2xIzfkpkHZINC5tIiGzB9oFd39wD7r\/353XElD5pfTHye+S2eb27KpVlsZQ2xt0JqKCwkBkHjAjBJLO0Nb8zN6+\/V09KXjkdPahtiFJH2mL1oQgLb+wi8+Z0tgaGyQfvvoN7W8O9tKDbGzRc6UyxqBuC1BDpheQphf7\/7blAkuP9f86xA6tLLF1omWNUgeCrotZxUEsRGAP2PGDKqtrU0fkgdcdHdOxH7QgCusX8vLUv8PGtA1Y2jK29M5HycOas\/Pv5cP6Eprf9FATy3b7WFo45fFLEGGN2Zuy1m6ZV+788ztPK0gnROJyiQdN6pvirTtZAQJZaLaDduBxjMZfz7YeHGOdqApMw0Z75tYQd9btrsgyW2WIMMhtmo5K4jse2mhdmLzCaJojB+NLRMXGhV\/A2ntCMvkBCmZjPjbQYG0B9oJe7CdrDKhvQCGPHNmXkOPzLyGqqf9jrbsbPVaLBb5dAtcLEAIoZEqHFXpQZugldjYszxnzhzC0bU4HtiO2Jj\/3HbiKF3XqYurdmUSbtl51Orzlh2txETNB7l2\/eZ2wkekuub1oGMQqfK6BS5SndXYGBWOqvSgTdNKbGz2x4O5htMcu2R5TytS6GtvF9E7v0qRlp98ElcF7LiRfWnD6i8VnNbWLXAqXAslXYWjKj0oDtqILV6XglNC3Jxnz+xaRu+OeDOSgQRuAG947iZrWlB56ytBxyH08lgCwgNvMZ6i9t\/xc1t7sAaCNhCZJQZv6Ba40Dsa0QpVON5\/\/\/3xDCnFkTE4eA\/H9XjxirdUNdLAGSXpcMCIjldGszDnh0kepiNNJmT3Ealjmfg529RGJ+tPZQRhcBrK9r3jSuoxoiv1qJLKtZOZ65fxTZG8jcqbBtPjX12oLYY5auMKOT18+LB1UaHTrSfZtFlFbByXhQ1QsQopdTpMEACJ5y7LnUdsLwSSY32zATTXZZ5eNMLykkNrZ+uMA9mgWftOuDKDkKImFX8GRvj96Mbj1LkspZkRhIGHCfpR3WnrAyBrZPEjwBqdtTk+CFWVVTTrwt9rE7hcj4\/qfSDzokWLCMdJ43RVHO\/sdhupqj5OVxFble71PU75tJni4gu9aGze4YJIIQg54nvj8hz737uste1Zc+t8NRmERhgkSAlCIhTyZB3I2NFEFitmzdxvQorMHELZuvEDW03up1FOAndpz1Lq1DN1V1icn\/MfNtGnH2bugYZ8Tps2zerWggULQrmOWUVcVXpQjCNHbHQImhvCGxdy8\/KXV63NxMTUgzVv48qWoGMZSnkngbviphl0xU1\/Hco78lnJmdefpTOv13ZogmiSh9E+FXFV6UHbkBNiOzXSrXNMbjmoP2iHdZWH1v759mZ6Yv8hSwN\/VH+KGmtbOmhQ3qyAdsCU9kpoXtcfNzJ1KgiCcbCmv3lnK\/30Fw1ZTwNQF+pGRN59kyosU\/ySXt\/qYIoXssZGrMXYsWMtXGGS8w0fQWRFRVxVepB3o2xkic2aG\/PJQ7UtngkQFJBsykMD3\/Ps52lP8fmMuS+O6MGDeS5MbJ5Doz+tLx33vJUQxPvlc1+y6mKCp4JvTtO4Uf3SpEaYK6YEXh7Ug48EPhDwEeBBnXsO9KGKv5yVmDk2x1o89dRTVFFREdrF9yriqtK9jKFbnkgTGw1HgD+brNCA2IIXlUc2qb9V0Z8u\/c+T6bk20kFu+A1SBG+z1eJu\/QHp4KAD6WbN\/YNtpBumAsgHsiIfyL1lx9GMvKyVEX5738RB1geByYygn7XrG6z8ugUuKmPH7XjxxRetACq+bAGxFziZpqYm2JFKKhxV6UFxijyx0UHeRC96g52WfIIC4qW80xzZ71xb9S4OgvG6nAbyMsmd6mbPPchvZ8LrFjhVnwslXYWjKj0oDrEgNndSJBT\/jZeB2JPsBgibxkFB47Vguzlyth5yuU1B18jZ1EaMPcfLI5JPFX6rW+CCYh+X8iocVelB+xkrYoudZTO3e1U3ay3XK2kx3w3jwbTA7ridMLQ2z6nzEYuuW+DCwD4OdahwVKUH7WNsiR204zrLIxoN81a\/69poE0j99D+PsJqXjw0mugVOJ+5RqluFoyo9aF8MsYMiaFM+W60NUt87scJyhN1Z87tAS1jZdku3wGXbrriVU+GoSg\/aX0PsoAg6lPe7rTMKpEZXdAucJrgjVy3j+P3DT9A7h99OhfY2pU5KxSac\/qWlNOf+R2jJzB+Zc8UjN3ouDfKzrTMqpDbEDk\/CmNj\/+PI8+tMl71k+oJRfKBXbz8+5Bz5jiB0e7Lmpycu2TjH4JF\/mt4iG0djhyIYKxzF3jKZ58x6jx2bON8QOB\/Lc1gJy45EdYaylccQSgklmfbcuL3NqGQ2VQOYWPf1vk2PE+XCQoDu8VDiq0oP23MyxgyKoKM9ebj7dFCRG1BeHcUbtxFPdAqcZbt\/VI0785ptvptmzZ1tleQtn0HhxFY6qdN8dkQoYYgdF0GN5OexTDOP0WEVOsjkJ3MDunai8W6ectEHnSw6eOk+HTp5Pv8LEimtAW\/dXS0OTA1fJ8dyBK9JUgdOYPFzVlb4jneaiqQlaq\/1B3SlaIt2+yaf94JAFPEHNcNTBOF448W\/U2mJ\/tkBlVRXdWfO6mWNrHXFTuYVA0jQ2+qxz2+bm33yPunym422fuJgBW2S\/\/q2NhtiGe\/oRSKIVxUd5ffTRR9Zce\/\/+\/YGBVuGoSg\/aAG1zbPncs6ampg6g6e5cUHCSWD6pY2JOUPEg7Xza4\/vvv59x+uPJkycz9rkmVYg8QJi3LEkcEyih+fPn0+rVq0M5PcVtSsMDqxtnbRpblkysDw4dOjRDa+vuXN7YEeMXJ21McJAhjgDeunVrKE4zr8TVjbMhdoxJqKPpugVOR5ujWKcKR1V60D7lhNg838YFAuJSAndu4cKF9NJLLwXtiykfAgK6BS6EJsaiChWOqvSgndRObJ5vo6Gyx5E7t3LlSlq1alXQvpjyISCgW+BCaGIsqlDhGNsrfoC+G6m9OBhiMYIF1kiVQBZYd7V1R4VjLK\/4EUkte8JFJFWd14a6qdgRgaSNiegR37lzJy1evDjj1FI7oLycZKrCUZUeVES1meJYF+zevbvrgr\/uzgUFJ4nlkzYmfokNUo8fP5727NnjekSxCkdVelDZ00Jsp0v5Tp06Zd0wuH37dqvdujsXFJwklk\/amOBc8bKyMmpra6O1a9fSrbfeSo2NjTR8+HAqKiqiTZs2pR2+rKyQ3qNHj+QR2yshkiZEXnHJZz6nMeELB\/LZtjDefbDpTMa+dzuNzY7eCRMm0OTJk2nZsmUZgSuJNsW9DIIhtheUcpvHaUz4gMbctib8t8mXL9gRmyMmnSLSDLEV42KIHb7gBq3RaOyLzjND7CylyRA7S+A0FkvamLg5zwyxsxS0pAlRljDltFjSxsQQW4N4JU2INEAYepVJGxMOourbt2\/aK863bxqNnaV4JU2IsoQpZ8Vwuf0XPltqeYGx46m+vj5n7y60F6lkW5UeFA8t69heG6W7c17bUSj5QEx+OvUss368tFfqb516ltKl7X\/j31PpZVYaP9f1uUCLxp43xA4oFCrZVqUHfD0ZYgdFUEN5JuhlFanL+S4blPpfJCSTUv6bU3POf9hsJX16osn6P\/37h5m\/f2FIf\/rhI\/cbYgccVxVxVekBX2+IHRTAoOWZxNCwna+fYGlPJnQGASVCWiSVSJkiborAqbJN9Gk7ob22U7fAeW1H3POpcFSlB+1\/3jX2T6bfZO3Fbm5upkOnPrX6I577LJ8DHbTD+SovamE2i+1IfK6hziLsuYb6rIgZtH+6BS5o++JSXoWjKj1oP\/NO7BfHn6PDzc2EA+lVjx3hD7Yf\/o6PAtKj8CEAiaF1YULL5GUtfPbNX1lz3nMH6iwti\/+j8OgWuCj0UWyDn00giBUfNmyYVVze9yD3S4WjKj0oTnkntp0HlknON0\/w7wO7XWr1t7x7p\/SHAHnsPgr8EQDRf\/beJ4Sy25rPWeW3NqeuMw3rEYl8+fUT0tViHgsNzKSNEoGd+q5b4MLCPKx6vBIbZ4+PGDEivfEDJMdTU1Nj2xTG8YlHHqL6XR1XF4qHDte6+hBJYmczaOLHAD\/zR2BMWZF1NY1MftbuW5varCkAflcRXjanoZHl+TAT+eybG7PpRt7LJI3YfnZ3iYMDoldXV2fsVhTTGcd+L8+jzoffdhzXO+sGaFlWLBhie2EE3z91Y+llVnaQfox0XzETHCb+jjN96Q9F19ma1NDG8DBbWvlAHUWNyOKHTLxzSzXl6Tf0evrGvB8mxiueze4uyI7dqbt2xH56cysdO3I0QzzLzrdYvxdfQfTrl39miO2FvH7ysAauGFhOFQMH0rjPVVjFbzj3lvUPT1OnYuv\/DZ2\/YnmYt7z7f7R5d4Nvb7Ofdjnl5Q8Tk3NM+wdKtlaCvOts\/2vp6FcXdiB2cZdy6telPEjVkSh79OODdOTji1fuZLO7C0cWz5gxg2prax3PIRctnz\/ubSaselxWUWVhAEsPV\/wgxuDyn3\/bEDtbybBbF3ZaVmJTGvPhkuN\/pLuHXm69dsrVXdLmvGjGY96uMuH9tPsiaWFNXGa9Exq3a\/kg2tS7itquHUM3vLGeit7eZlUrOgthZfDKAtJEP4PYBtEJKbfNyRS\/Z+hsumfoP\/jpSiTzPv\/+v9Dz7y9Ot81vrDifQ\/7CCy\/Q0qVLHfuomtKo0oOCp9UU52Nk0EjxJApudNidE9eE8TWUvdIclIH3iwT2ut4rmvKyGS+SHfWD8CLpug4cREPuvpfw\/+lDBywIWrZuofEfpBwrTGKeGhzvNYCO9yyjvRWjaN9VI2lfxSgrH8qijreWfI\/eWvJU0PHvUN5pTJKksZ1ixTGvtjt4wW4QVLKtSg86sNqILZoraKSd6ZJt57LVwF4J7AdUkB1khLPObs6+r2IkrateRCDqlSca6coPG+l4zwHW7\/zg7+LDaSDxqYMHLTIf2baF9v1srZWt+MZx9JV1G0IjN0j7+T5j6ZYB91JVVSX1+WZzoufYdsR2kmEnWVHJtirdjwza5dVGbNm5YHfpmapzIoE5rNIK7miPbxbDJNmJBRNaB4H9AA2ydxtYTpOfXU6HTn5K7+7eT5976ccWsfGwufxWpxLLtO468OLclTX66UMH6dTBFKHtnusenkPXPfwI\/XZKtWMeVZtB6FsG3kO3DLyX8DPmnh8Wv0df+rvBiSG2191dkFUcYig+bmvZKtlWpavGTpWujdjyOp\/duh93btHSVVb0mRjUIW9OkL3QUSCwCC4IWTJmnEXS4hu\/aP0MTbv1O7OyJp5q8L78\/AYry3\/fU63K2iEdhOY58zvHfk\/\/c+in9M4HvzcHTPpG0r6Airiq9KDN0Ersw4cPp094tDsnCp375K5n6MiZS9L9sCNwVKKyZLCZzIMn32cRGQ+bz5gDO2nboIPG5bMxydnsBqlBaDiTRC+xboELq+9Rr0eFY2xvApFNbydif\/Gh5bRh7Upq3F3vO6wSxMIDLdmtPPUz\/54i2cVlDVkQYOZm++BdTGY4wPAc2bbZcobpJrPcZjbJfzm2Mu2Uc+oXSD3r+h9T8RXl9Oqhn2Z4h7mMSiCzxSxp5VQ4xvYmED+muNdN\/awhoan453wIDAjcrbzccmbp8Ez77ZMXkxym94PX\/5jePvZ7mr99kuMrVALpt21Jza\/CUZUeFDdtprisobNxnl3UwKmloiF335deLmLtCM0LzcxLSF4AYU3vJa9THryPnU\/FXQbR2x9soSNnDlrz1Fw\/KpOcSS2v4dq1U7fA5RqbfL1PhaMqPWi7tRE7rOUuCO2YHzydXvaBlsy1uSuDzPNUaEDMT0Hoa\/uMTf\/sphGDDphTeScvuR9So27dAqer\/1GrV4WjKj1of7QRGw0LEqDCAR1Y0mGT149WdgMGxMTDIZKYc\/LDafgdmlh85PzyPBVlF9y43iI6tGOutTdMcjjxeAns873H0j\/duN5qixht5YaNboELKrBhl\/e6u0uWZ693dy1+dTPtat\/dxX4dWJjDivsmb3cXkxqmt9d5rEhWEBW\/g5hifDO0quoRPcTIC5LygzhjPDC7Xz30vGNVC0avtzT4Y9sm5ZzcN37\/aYvcW79cTc\/c8gernUvffEjV7XS6Ibb9bZuwQCdNmkRz5861sMKtnHxjiB24jONPPrlAJz6b2sNt97w78dZkxIqD1BBOOKd+O+XOjLkzkxdRUtf2Ti0vgbiscUVtCycR\/g5ipgL\/U15wJi4TlskqE9ozExwywkyHGQxSuX0Egr7Hrjw09+Crx1LFMw302IqL56V5eVfSiJ3ttk3VNT8yjuIKDsbhpuqJdFP1nfQf37y38Iktk7rrsQuWLGLNFQQWNS6IiHXYFFkPpOe38u4dL8KsKw9vnPBjCofRFrx3yMJHqGHMpb7DTp2I3bm0iDq37yYLo435quNs8zk6Kxy0kc22TY5W86KxnVZ8dH9Atc6xVYMndm7Pkda05\/v8d9bQ4DOpcEfWsiBxPj3Pqr44pfM8Fx+ipW88pN00Z2cZLIXWuyussFM\/EXBOAjdgegkNnFGSLQyRKXeotoUaV6b2Q+Pxu20TG0GmTp1qjkZyG1EWojlPPGnNQ0Z\/cw6N\/8F56nosZTLDOQWTOtdOqLClkANDdM+72XmHjyDPq9kK4hBXlc\/CaGz1pXyQD68nqCRWYz\/xzLP0y9Nt1NbraovUO954Ph23HDbB8l0fm+a6nGqY18P\/MH\/bpIwwUfQbBMeSGBySeKDFebcYYgI4FkC3iZjvMZDf73c\/NpdXHbagwlGVHhSnvJviVat+bTm3+nz3ddq4LTNuOWjnolieTeWwyQ1tDS+4ylnHKw7AhjeriDj12ruHvnH5JYnZ3eWV2CUlJXTzzTfT7Nmzaf\/+\/Z6PRkqsxp717z+hH036K3pj344o8lBLm3jeHSa5scSGVYBvv3qD7zaD7BxvX1lZRbNv+WJiiO112+a6desy4jLM8cMuYqbbHPEt4TksEKbmDvNDkeQxCXP4VTiq0oO2Je+meJJvdgxjzs2OOQhCGKGsugUuqMDGpbwKR1V60H4aYgdFMGD5oFFqYWp+dEW3wAWEKzbFVTiq0oN21BA7KIIhlM92fhy2tjbEDmEw26tQEVeVHrQlhthBEQyhPHu0VXul5Vfx8UZhBr7oFrgQ4IpFFSocVelBO2mIHRTBkMr7dYDZBaOE0RTdAhdGG+NQhwpHVXrQPhpiB0UwxPK8cQRLVqpNKXC84XRRu2CUIE3SLXBB2qajrJ9tm\/x+lFm4cCFt2LDB8dIAFY6q9KB9NcQOimDI5RFkgp1nbh5uNt11bC7RLXAhwxW4umyIzdfprlmzRkns3Sv+TId2t9q2E2e4z99+V+Hv7go8SgVQgcok12WCM3RJI7bfbZuIEUcEWo8ePTxp7BcW\/Y7aGrp0kMzS0v5UWVVF3\/2vKfEiNpsr3bp1szrV1NSUDsdLqhB5\/e64eck5HjybCDMv73ciNkeneakjynnk8\/H8bNvkvK+88op1hW7iTHF5vyr\/fvLkyYyLwpOmHbwKvJOpzX\/3eyqK1\/cin9OY8JlqfuqKYl75zjM\/2zZxuAKe9evXmzk2D67dfcKG2M6iz460u16+uAc67GAUu7cbjW2\/bRNY8dFIxcXFhtheiA0PI674MU8mAj\/\/akvGQYQw0fGEETrqhHXSPrZed3cBF\/nuLmDo5EBT4ahKD8qFnHjFeb5dV1eXvvJHNPtWrlxJq1atCtqXgisvam02w8PcEeZHYxccuO0d8kps7O7iJ4zlrthe8cMg8Pwav\/NeVk7jrxaIvWvXrkKVnaz7hXPe\/qbns5bWxomrX7n26\/T4u19L11dZWUm4H625uTnrd8gFS0tLad68eWbb5tKlGccmZUtsJ9nG2E2fPl0bzqFobJwmgQ3lRUVFlpyweeJGauRjIQLBzWOPwMdvdKcTG\/pZib2qj1KX4SdzApXXa5dy0pgYvoSVllvT6+vrrbl6mB9mfl8oxLZrvJMn3E5DgODmcUag99nB1uB3vuoT7TDp1iTaOxCRFzCx3YiLMdVBakCgjdiIzunevXsH8zsiuJtmOCCg26mTFODzjaMWYsvBKTyYquNkkjLoUe6n8XuEMzr59lVoIXY40Jha8oGA8XuEh7rOObSqlYbYKoQSmA5yG79H8IHXOYdWtS5vxOYdMmig2y4ZVQcKPV1ccbCLtxf7L95uir+bqY+7dLCD97XXXnPcpRVX+coLscVbFEaNGmUF1GPtdPv27XHFUUu7xZj7FStWKG94xMcS69qPPvqolvYUUqWMbVlZWUEqlrwQGwKIp6amhgr5qxmUCPJtE7xlUA70wXsMjt7RZiuotbVVuf3Se63RyplzYjvt\/HK7uTBakOWuNfL9UG73RYmhkWKUVO5aG5833XbbbVZjT5w4odzMEZ9eZbY0b8QW5zXGhLQXH1lDQ9NMmzaNFixY0GHaIkf\/Gd+FmpJeYr7VtUQzhyF2NMfFapUfYiPv5MmTCRcwQGPLv0e4m3lrmiF2iNAbU9w7mH5McblWL5eze29JYeY0xA55XEXT2zh9nMGVTW8355kTsQtxKScscTTEDgvJ9nrMcpc3QP0sd8kn1OD3ESNGmGVEF6gNsb3Joa9cJkDFG1xuASogb\/\/+\/dPnyIkBKm1tben5trc3JS+XIXbyxtz02CAQawRy7hWPNVqm8QaBmCBgiB2TgTLNNAj4QcAQ2w9aJq9BICYIGGLHZKBMMw0CfhAwxPaDVp7ywjPe0NBghZHKG0Ny1STZA+\/0Xq\/5ctXupL7HEDviI++28SNXTXeLUZfbYAKOcjUq7u8xxI7GODi2IgrE9rtJx0+EXMQ6RAmqAAABnElEQVThj23zDLEjPHQgyNSpU9MtxEkzLS0tNGPGDKqtrbXMcxxvu3v3bho+fLh1rjsHpohX0sgnr4jBQapAFjvTXz6sUj6pxWjt\/AuVIXb+x8C1BbLGFonGxEYFfAINk3bTpk3WSSri9Up8Cot466lqF5gcqmq3uUQ8OIM7Y\/e3iENdUM0zxI74cHohtngnmpxfJCJOzbTbz+1GQjnN6R42GUa721UjDnVBNc8QO+LD6YXY4gXsbsRubGzMMO3Fru\/Zsyfj7nI3zSvGpDsdmBgF30DEh1Zr8wyxtcIbvPIwiY3W+N3x5abNxbm2THBD7OBjH6QGQ+wg6OWgbJjEhsYWT1nx0nwvc2W7XVLGFPeCrr48htj6sA2lZtkrbec882qKs\/MMDeOTTlVzZnnpyi6\/nXb28kEIBSBTiS0ChtgRFwzx\/Gt4uuEAk5e7vBIbXnKxPu46e9DtoLA7\/VQ+OFFeMivkfc4RF5d08wyx4zJSeWynCVDJI\/hZvtoQO0vgklTMhJTGb7QNseM3ZnlpsdfNHV7z5aUTCXrp\/wNJe0ZAzBUHjwAAAABJRU5ErkJggg==","height":236,"width":393}}
%---
%[output:50b72b4b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPYAAACUCAYAAACgLtSZAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQuYTUe2Xoj2auLRHq11PBJBxIg3kQSJJOItMYRrwh0MBpmReH1iJDEmE65EviRGBBNmjIhISCdMmOQmkoh4MxjBFaHp1lp7Nh3tdb9Vso461bV37X1q73NOn1P7+\/JFn9pVu+qv+mutWrVqVZGKFSteB\/MYBAwCMYVAEUPsmOpP0xiDAEPAENsMBINADCJgiB2DnWqaZBAosm\/fPrbGzs\/Ph9mzZ8OyZcu0UOnTpw+MHDkSTp48CWPHjoVDhw5plSfL3KpVK5g2bRpLmjx5MmzcuNHzbzgtcOHChVCvXj32+r59+2DQoEHSrIRLQkJCID03Nzfi9XfaTrFfW7Ro4Xs\/Y91efvllaNeuHaxbtw4mTpxoWd3Ro0dD\/\/79g9KXLFkCb7zxhtMmev4e1YnGhdO2eFGRALGpMF0w\/CI2Emjr1q2so6KF2OJgsiI2dahVh+li7sVAUJURzcTmJ1exHXaTrarNuunhIjb2Ta9evWDSpEkBQRpQxQmcjIwM3yRtKEDVrl0bZs6cCdWrV4doI4DYcbL28eTn60+TU2JiIkQb5rJ2+DVhq8aESsoRvqLGSePZK01UVU+7vvdzcqH2i2MoQGwiUFJSUkAl5wcfVpzPzEvNgwcPQuPGjQFVyzVr1kC3bt2YKj5r1iwYM2YM8GXy6hU12EqN+uSTTwKkJuCwDosWLQLMQ6r4iBEjmDrME0fWYFFyqiYKUX3mVWexLKsBRANMpkpiHbt37x6kjtthzpNr06ZN0LNnT4aB+O1Q+g2XNC1btrRUZ51IbNlyg8eYn6Sx3rKlCI8r9vWxY8cA1X4rVZzwFfuSvvXll18GqeOidJeVa\/cOjwPiXqtWrQAvunbtGsAP2\/bNN99Ap06dAks0cZLi\/65WrVpgSSeSVKwP4WbXX0HGMx4kHDi4jkWJwj\/00SpVqhRIx7S1a9eyxtEae9iwYUFrJHECwbJxTc6vPanTUe0eOHAgk9Z2xKYG8jOj2OFW6rAVuWWTDU+ipk2bsnbRIyO2bLK0m\/lFQvJtRnsFrWtFrPhJV9YvqnS+32T9gKSvWbNm0JpaXGNb1Y1w2bx5c4FJWiS33ZJFRkA3+IqTCt8PNG6cvCObvDA\/LhPFNT59w2qN7aS9Vu9YfRPHMz5SYiOI+ODAtSILT3wedNXMrjK60ODGAYbGPH5AEAnFNTbWlTemWf1NZaKB0E615DtY1n5R07BStcS2kGHSagbGdZId5llZWYFJkLCgCYhmcVUZVv0mTjhi3WkCpgnbithWRlM7DQrHz9y5cwPEp\/HET3QyYlvhK5s8ZSo7jQF8H8da1apVGTn5SVp8hxdEvFCgPpVNEipi85qLWI7YFhFH0hJI4OL7uHS1JDavGoiFI8grVqxgZOLJgu\/JCIOVRbIgeCTpxI4SpZVsprciNlrFaWajGQs7iACVzbLUJpk6aGWco3KwbrzqakVsK4liRWxaUsgGJuK1bds2Rmz6PrZbrKuqDKt+o29a9YOK2DSgRO2KdkbspBPiN2fOHOlOh90a2w2xrcrhl0okzMSxyb8j6wOrflYZz6hO\/PixstuIGiQRWSQ2aWwF1thkpGrWrFlA57cjNqbxW04yYlOlUPqmpKQErbnF96libiQ2DnAqh7bX+DV3pIiN2Nitsd2SkgYVLxXdlkHEVvWb2A8qYhPuMnUWJ1vsd37pIqrCoRDbDl9xje01sfk+0CU2P5GoJgOVxC5AbGo4zQS0NlapmaoBgh0uSgGZekuNE1UfVF\/F9bJMoooDSmboE7ULmWTE37xSxbEsfqa1kgR2ajRfR9mkKWIhU8X5Mqy0EXECslJBrVRxmb8CL5FoDWq1A8Bj7lQV5\/EVbRz0bfrdiZrt5B1xgqN266jiVsSmyY4ftyJPxSVRQBUnBxXqfFHdFY1nBNThw4elqpPV2pVXP\/m1iZWKxncUn1dmFScHFb4skUR2RgiZU4nKeIYTjpPtLsRVtY9NE52V8YywkA0qK3uD235T9YNKYvMWYXHCxP6W7XCIY86JMUk2GTvZx3ZiGHPyjtX4thovWF\/VGtuK2C+++KLU4Ihl0gQpGi0DxjMitt1akwYJTzbVOlQ0oojrUyKjCCZvuKMG86BhPRcvXgwDBgxgfcwvA1RrLnHgqPYX7ba7eGmhKgfflZHWLeZOJDa\/7nbTb6p+EJcBMiOobMnDD1orNZ33DnO73UVEd+p55uV2l+hZKY5TN9td5FUnCgseU+QfbvV26dIlsJwVdxsIb898xWV7snYS1EoNNr8bBAwC+gh4QmyZ9MaZZujQoTBv3jxWS\/q3ri+6fpNNCQaB2EfAE2KjZG7UqBHb+iLVGH+rW7duwD0VVaDjx4\/bOvLHPtymhQaB8CCgTWxyQN+xYwd07NgxQGwkMj5kmBL\/puYlJycD\/mceg0C8IZCZmQn4nx+PNrHp1BVWjvd7FiU0SnB0euEt0EholPDotGIeg0C8IYAGSXTy8oPcWsRGgxk6siBZReOZE2IjodEbza\/GxfJAadKkCQwZMsRgF2InRxo\/+j56EiLBvX5CJjZuXbz00kvMtZT2dEWJrVLFidh+Nc5rsKKpPNR2OnfuDKtXr\/Zlxo+mtvpRl0jj5\/fYD5nYVm6atNeNFedVb5nxzO\/G+TEgTJkGAS8Q8Hvsh0xssXGiKu5ku8vvxnnRAaYMg4AfCPg99n0jNoKhclDxu3F+dIgp0yDgBQJ+j33PiB1KY\/1uXCh1irc8ZrvRvx63287ye+wbYvvXr1Ffstlu9LeL7LazDLH9xT6uSzfbjf51v2o7yxDbP+zjvmS\/B1c8A6zCVpWui51RxXURLMT5\/R5chRga7aqrsFWl61bAEFsXwTDmr1IqlX3tRF66J1\/1e3B5UslCWogKW1W6brMNsXURDGP+DjX6wuhfvA5v\/Ptp+OLoe9pf9ntwaVfQ5wJkMQS8+qQKW1W6bj0MsXURDFN+lNZvddjCvvbegf+B9w7M1P6y34NLu4I+F2CI7RPA8T6w3MDKExulNUpt3Sce8eddoTGkFUZPpYsp6NYQDBwyZcoUFoYo1MAgKmxV6bp9ayS2LoJhyt+w4r3wx9Yr2Pp6T863htgh4E6RfjBiKsYYw\/MLSGw8OozP+PHjYcaMGeyCO7xaSOemThVxVekhNC8oiyG2LoJhyt+37ljoUONJRurKpVJhysZe2l+2GlxFb02GYrfevFZJ+0MRKuDq2Qy4dvZmIAOU1nhl1NSpU9nVy+LfpJpjIE7+5spQqq8irio9lG\/yeQyxdREMU\/5wErv0\/UOh9P2\/CVPL\/PvMxa\/fhotf34i5h4\/soBJPdKuL\/EKpoYq4qvRQvmmIrYtaBPKjNRwl9Z5T65nkHv5Fc+1aGIkdLMHx0FL58uWhcuXKTCWnENmhAK0irio9lG96RmzxTLYYX9uc7tLtnpv5p7Zawf5AYvetOw4eX11Vu3C\/B5d2BT0ugCTygQMHCqyx8TZRujweLz+gyEChVkGFrSo91O9SvpBVcTJEpKWlMSODaJgw57F1uyY4PxI7Oy8ddp9az\/aykdhoKW9Y6V44cTEd9pz61vUH\/R5crisUhgy8MNq5cyeTzjiG8XfeCq4bVVeFrSpdF4qQiS37MB+J1En4Yb8bpwtONOXHPWw0nBGxURVHUqP0xgfJ7dagZvD3r4dV2KrSdWvmG7GdhB\/2u3G64ERTfiT2F0eXsu0ulNh\/+K4X2\/5CZxXc18Z0t44rBn\/\/eliFrSpdt2aeEZtUnOXLlzPV3G2U0lWrVum2JabzE7F353zLCI0OKkRwVMPRao7SGyW5U19yvwdXTHeIonEqbFXputh5QmxaX+OmPn9BAH\/zhyyuODVu\/vz5sGDBAt22xHT+DztnMTKjOo4kR4JXKZ0aZB0ndd2pV5rfgyumO0ST2IMHD2bho\/2K0KtNbBmpsc1GFfd2WBOxUe3Gf7N1dV56ELHpkAiq6U6MaYbY3vYRX5oKW7wxEz3eopLYoiWcb5gooU344dAHEfmJ06kuIjZKbdFgRttiTgxpqsEXeo1NThW2qnRdBEOW2LQneP78+aBre6hCZrtLt2tu5idikyRGlRt\/kxnLyKfcidT2e3B5h4A\/JZnTXRJcrS4MyMjICNywaRxUvBmQVsS2OpeNUvvuSvcqnVgMsUcH3TfnTW\/dKEWFrSpdty4hS2zdDztpvBffiIUyRClMxLWSyjQRqLa\/\/B5c0Yi9k2ObZCPCU2ChnvBSYatK18XOEFsXQR\/y35ZSGo4cuwj0f5HYuM2FhjI7dduJIc1qcNUoWwxSE4v50LLwFpmeexWOnr8a+KjdsU10J6UrqfA9OsIZqr+4iriqdF2kDLF1EfQ4P5J5\/MgG0K9nTUbu9ZuzYc7MEmzvmvaoidiqPWuVIc1qcI1pWgaeaZboccvCX9yrW3Nh1rYLgQ\/bHds0vuIe9o84sOr8sh8r\/Yf33\/XwK4WnKCT1RwsfYGSeMXsvtG1RmZF80jNXYGjqPwLEJmcU1UEQlUoebxLb7tjmiRMnArfH4i2mOmq4k2Vm3Ejsy+06AhK7TI3bIGvDevi8b\/fCw0iPavrmS80YmXsM+opJa3xy\/vM4zHmlBNyV+VqA2G6ildqp5H4PLo9g8awYVaAFc2zTI6hpYL349yVQceIL8Fmf7nDhaDp0XPYRI\/d3z47y6EvRXwyq3iidR03aAus3nwxUePu\/OkFCxkOw772nXLmL8i1GlVz0UnMiVaIfNXc1tDu2SRFV0GHk0KFD0i1cN19TTZqqdDffkr0bFWvsD5NqwP7sk\/DRvU1YHau0bgsdl6XBrlnTYdesGbptjEh+bMOJ79Y7+jaq4Ejg6bP3MhWcf1CKd0h5khFbpXrbfUy23vZ7cDlqfJhfkh3bpKAKJoKKR52BA+vPb70Ni6rVLkBiVMtbv\/Imk+JOCeJRtWyLQQv1gzWeZM4hVoctcDmBWgc+NFlZFcqvq0dN2lrgtbYtkuDtURO0iU3rbd5bLR6Jbde5oqquM55U2KrSdb6NeSMusSfN\/yt8mJQqJfBD76VBYmqqkhy6ILjJT2tWuz1ilNZtXn2T2QvQEIhax4WjRwp8Bkn95p+aQWpKGWjy8KeW1fj0zy\/Axa9HaElsLJy2zSh8sd+Dyw2ukX4XDWv9+\/eHJUuWhLx3zbdBha0qXRePiBP7qb8thU3lKsGS2ypJ29L\/SE5UGdOI2Edq7YQJK7rCpcz8AvWmpQQl8EuKEskJkNSlAkt64GRJ6bpaLHDluDfg2p4+8Ep2w6D1dyidz5N7\/S0LYfbs2b4dRAilfrGSR0VcVbouDhEndq8lH8CWU2cspTKRJFpUciR2j+6DYXfjtbDrnrVwblsunNt6AY7Nv3HiCh+R2Pgbknv\/0tegwZw6bDJAgjevcStUfOdcgXW12Knj2j8PHVL6wg8NfwUydd3tICByn6myHxoMu8UQ2y2ADt5XEVeV7uATtq9EnNgPvfcRHP3+JHzep\/vPUTgLxu5qNGY8NBozISrW20jspFcSGKjNl\/aF1ysNhhpDqzKyZn9yGvIz86H6fU9Aw\/+aHgQ8quQnzk6Hck3LwI6e38O8d9rA\/LsusIlh74gfLDsJ18YYQOGxRh2h1XOTodJdH+r2+Y3Jp1Qq\/HXQ11DpqUxDbE8QDS5ERVxVum6VIk7s+iv\/BUnvH4b2n6ewtqBBCkMAiXdTRct6G4l9\/eMSUOfoI9Bh2cPw7PLmcLZ8FqQMrQqVf1axy5bqBFXKTwjqmzOHN0FOwgT4YWo6lNpxiVnBn33j37B3UGnIXnWa\/S57UPp3n\/MRPPpONtzRbYztWtztYHi0bQ94YebEuCU277CCnmcUY5z\/t51LqV3AQxVxVelu+1J8P6LEvqNFS2j5wT+h5ls\/Qtpbo1kUTiQOSigk+Bs7nw4KGIDkxieSzitN724PNVf+Aa6dqMg0jOuLvoEFr\/UMwrXt36dCzXYjC\/TNyYMfw9oOgwIuo2gwQwneYM7tluQmtf6+tGzo++AkRmxyXtHtfL8Hl279\/M6vQ2wkdb169WDdunUslLH4qLBVpeu23Vdiq45t4qB9\/N0P4My492Hp8tGBtqCaOOoXr7Ojh0hwCtiHVmbcAoskuVs1fhJu\/3g2c55B187sX9ZkExKq2umb9kPe+SvQfFxLKbGx3t+0SWXS+t2VhwPrZStyY3tr33kv\/GLhbKi8\/zr0S30F5mY\/BMdeHQ+VruWwMuhBbaHsz\/7dlzLyg9b8\/CBBS3xq9dIwYWQDaNK0KRy98GRcSWzZ6a7FixfDgAEDIDExEXJzc2HNmjXQoUMHSE9Ph8aNG0N+fj4zMuIFfXwcAsSVD\/\/F46wirio9aontJNACSaMND3WHQwcKOnOgkWd049fZepBU9B9LH4XESf3ZNtiGZ0aFfY+7frv20PTvH8C6cS9C74z7oXmtB+Drx69DbrPirC+Q2FeuHoey5WtI++anmYNh3vCi0H3gV8zCTfdk3dWuHJQclQOnvikJrTfUhKyzx6HUi6+zLTN6Kl7LgVNFK0Gry9\/CgEvvBKR3ypCqULlrhYCFvlzTRPZvXLvzVnv0bENC0+GSvCupcN\/DE+KG2Hanu0RVHD3QNmzYELhYoGzZsoE4A9QfcamKO4krTkaxzX\/8Dk4mXIVityYXIENKmbPQucIxaHMqEa58e18gfd0zxSD7ziJwcMtpuOuTa5CTcAVOJlxh6TkJV9nffjx1kxKhdu8qsDktAyptzoDx2ffAuvNfwV8OvwwVqpcM7F\/Lvo2ETz26GwaW+BT6zqoNbZKLQ7PLu6D61RPQ\/PIu2JOcAM93qQAVEgdCxbKDoGzOVuj64QY48+NwqHgmA+D51bCpUgdAgiOxX\/7zDpi7JwvqTEmFY\/OymDqPD1rc60ypAUjwo\/OymPQmUvPebVZSg6S6H\/iFs8z0jItByxbV6S5+jT106FCYN28ek9Kosrdv394QGzvPSTBDJHa73w2BRw7+Az7aUZyplrellAH0tmpz52XodfuuoHGAkuan05UgIeNB9vuGBvXh2wb1oXTOdai14TpTV\/E\/\/uEJjsRH0uPDe41lX7zhPCJ6kvF\/07\/vatoOGkz9NazuNxjOrF\/JIoaiRsG7e\/b4dnuQpMWykdR5+TsADWuvfd8HLr2\/GzKKVYFrZzJhQ2Y+bMi8zM4Olx\/6LCR06gPnL66BC2WXwK1nqkLbdU9B8113Q\/OBo2FpkwlMaj+dNxMyj+TA4Aqnof3+POi56jQrJ507f3y5e33Y\/NhlJrVfrVwdvkrvCjMXnw3A06jSdXh1+GMFJDZNAuEkoR\/fEl107U532RnPDLG53nESVxyNYffWLwu\/vfA2FC96DoqUKxEogdTF9ZtOsmOMMoMRSpacIklQtU3bwNobSV4mBxiJ8EGL9ZUrxwuMm1tuqcZ+Q+LQU7zYjd\/wKcf9zmc+Vz4LTjdIZvvu6E1GRyj5oAdWxG687S3Y0+IF6Lp4CGxevha+y7zMyIgPqtzUDnJowXUzGuuuP5jL2tHgUio0SRnOiP1E9jzYWyIdZpy6Bs0+bwqNz2YwyV\/t1A7AQAn0nH64FrxeB2DX1TxomJkPI9edgyt5N5yBLlW7G4o\/+nQBYhuJfdNCjlZxHWJnfLgdvtucxQytaIv56dRNR6zOXTrDc2v6wLZt2zyf13wznjkhNq4Nu\/RoAH9I28tIXaRcSShargQc3nsKrp+7BBgBg38oGgYvlbYDwIGmiVCuWRmo2vo+KJnQGGrmN4LjeZULSE2+LJmLJ6bnpsu3nSgvDvp7kvPgd7d3Yz+RJxq5aeJvVsTutKI3bHv8I9jywdqgk2u8bzm5oNL3cOLo1OIp2FrlE9jcbynbRkvKrgq4Vi8zuRqb9PZNzAu6B5ryjvtVHej\/1J2w5G\/74bPL5yBhzI07ryufvwodDuRBla0pcMfgGXGzxlZdyme13aVD7MyDOXA0JxFuKVaNTcgXfub1xUpFWF98Pf6RwkdsrDh\/gQD\/N\/4b14b1utwBxcflQvaBnayhrZNvGKHwSeUkDy+FKGzP2TtLwZsPlGPvojRqmHmZDVjpZMCFyaGJ4WjutUDoHDGMjtUUiscr8cQV7yiC4YBRVZ+2pRW0SU6AKss3wunyN\/bl6UFVfPCBp+Fqrcdgccn\/ZsTGo6l1fvkkc75BQsuOqZJGgIc3\/nSkL\/xq3WI4tL0kbJ7ej22T5U47Dns+PlGgulYnxnAyxUkQ1993XL8TRl1\/Nm6IjSBZne7CtGnTpjEcySo+depUdpWuDrGXVSgOWaVrs3JxDOCDBtbLV4\/DlYtJcGr00MJFbCdxxZHU5aeUhssjbnHVOPK3Ro8vNBgVWXoyYP1F0tMkUCOxaNAEQb\/z78gIzMfJ4gmPE8L9HWtAu0ENYdGIzwLfqVZyOpy82AOSyzVkxS2e9L8FiI1Lg+E\/vAS4FdVv8mEm1fFBzcHOus8TG2OFP\/LFQihVojFs+WsnaDYsBR75V5ECbqZ0Yiz92AXoPuhr6RzVfMZkuL97T+hz+nJcEdtqwvb6dzJM\/mHVbvj+4qkbuyUXb6rhycnJgP+d\/cdwV2PfaT19U8WdbHfhGhKldtaIs\/Djtpt7snaVJ1Lj9g66cPI+2k4bzb8nIzs\/IfDpmK9Wq2RIeLQu7J\/6FVsqbMjIh1LFHofmlWfCyh\/GwN++XwJoO8D1Mv8gsefnz2YkpP1nPJqqCgMlEhsJmXLfE7B\/5xMs2sr00ikF3EzTFt7PTozxkVioLuQLgNuF5Q\/ug+F31DTEDmXgKPKo9qlV6bpV8o3YWDGVgwo5ZjglNk9qcY9WFwin+WWqOOZFdZzW2S0X\/hYqtridFflT\/k6oUHYg+zcSm\/avnX5PRmycEA6f6McOkCx+umnQZEGBEMVILPg9fi2PWkKN\/DxzustpR7h8T0VcVbrLzxV43VdiqyqHjSs+5wrzk6Y9WLs85IgRKVJj3ayITZFDcdsLT3DhGlZ83jpX3bWvtx2x9444CMPb3MbqhNL5yZ412b9RIxAjsRCp+ZBTfg8uVf\/HcroKW1W6LjaFhtiotuNBC5wEznEhZXUBcJvfithkHcfbOY4\/t7kAsasWvQWG7Cljuea1qgcRm7QBcur5z7a27JQYracpBjkSmnc15SW1GEfO78HlFttYel+FrSpdF4uIE7vqnFth36r\/szzdhA1EFfyelfUDXlS6jdbJb0VsLJOusf1sxGtSYtdfmKc8ey3WjTQBInaDpwdCk7GvwvZVLYKOe6JTDx8EkcohSY3beOLhGb8Hlw7OhT2vCltVum77I07stn9pAatWr7YkNpKaghPYnVvWBcJpfjtiEwmf69ZGSuyc3x5xHQHFitjfzb\/HdjKk9uChGTTkfdanR4HwTH4PLqeYRuq9UE530V549eo3fAL4u+r4dqiwVaXrYhIVxN5UZANsbPVvaVuiYV3NV8yO2HRo5d2BY+GnBqeC2oMebt\/\/er3rI5cisSnI46dPtIRTmw\/a9j++i6q71Xaa34NLd3D6nT8UYqNBGB88qik6vBhi\/4wADqyBfxkAS4osZOtFMX4YWc2dGtf8HghYvh2xMR2t40hsjInGP0jseuO6Aa7B3TwiselEHLm0WpWFKjjulVs5vmC+eCS27rFNEW\/xsBOlq7BVpbsZI7J3Iy6xx8z+HbxZ9JUClnE6oYSVjgYVnMAjYlsFPEAiovFMJDaGQPrTxxtsL9KTdVCoxHYSccZqcOGhFvRtLuwP+mbzB3m8PraJ+IiHnQyxOYnx+6LDChjGok0Fd0psVMeTXk0ooIojsTvO+b3ylkyRTHRlLhnPSBKj+6mVc4vTAJBWxCZLfGEnthgi2utjm+JpMaOKc6o4RqZ4Lmss\/LjtSMAYFE1WcHFwqyQ2vt9uYXspsVHzUN1tbUVsfpBiSGYrYruJMhNvEtvLY5tYVu\/evQORVcR+U6naqnTdSTXiqjgSe+KC8fDT0LNsnY0PBglActPfuo30Mj9uK6UtesA29pjooEIRTMn9ldRrXG+jJLZ7aCLgiU1rZ9n1RyqDmRup4iVu0VCWG4lNJ71kh0BwXd2sWTOYPHkyOyQie1TEVaXr4hUVxMYwNOiBhuoqEprW1bJg\/LoN1s0fCrFr7bwK7\/1mT9CnyaGFv3JHVjcrYssuLXRiMItnYntxbLNr167QvXt3W1IjxiriqtJ1x2nUEHtP5m4mqfH5YepR6Q0buo31In8oxKbwROL30Ug1tfUK9jNeYu+U2BTQUTzmiVtbKLFle9ahSBUv8Iq2MnSPbT7\/\/PMsOin\/yPayVcRVpeviFjXE9iOKhC44svxeEhvLp4is+G88lik+MolNbqX8tUhupbUTqeIHfvFSpoq4qnRdnAyxXSLoltj1ThSDlc\/vt\/Vvp5sw+SgsVC2KqcavsclJhd\/LJg8z1e2e8ayKu+xqrddVxFWla31c97ZNXq3Biuzbty\/ownDVsU2\/G6cLjhcSu9vuBJg8eIuyKrTm5mOnYSYZsfF3NKCh8Qy3vJxsgRlVXNkFnr6gGtuqdN3KhCyxabM\/LS2NXTsqbv47CbTgd+N0wfGC2Hhm+p9v\/eioKrLL6a2Iza+znTijGGI76gLPXlKNbVW6bkVCJrbsw7wXjpO44n43ThccL4iNZ6adHjOVSW0itrg1RttaaB3HQx6hXJ5QGPH3o0\/9KFOFrSpdt06+EdtJXHFqHAaRW7VqlW5bwpKf1th2kVD4fWyZD7xdRenoJ\/mUWxEby0AjWpXW9zFnFauoq3bf8ntwhaVDND4SyiEQ\/Bzd24X\/jvm7u2i9vXz5cqaaOwk\/TANr\/vz5sGDBAo0uCl9WIjZ9kb+Di35DYmM8spGnK4QcMYXW2nioBB8nzixuUTDEHh3Yk3Z62yZOBuicgtF3xeUoj78K28GDB8OQIUN8izfnicSmBh47diwo3DB\/YZkYtRRBUDXe7UAN1\/sYfhgvMMB7sPARyU3EdnKpvazO5JmGe9sosQ2W1TPQAAAIIUlEQVSxvetZr093Wd3fpRrbXbp0YU4u6Jzlx1avY2Lz6ge\/IS8jNakr+H+7uOKqxnvXnf6URLG7sXT+tBcRG9fXsqgmTmpDKjmuuw2xnSCmfsfr0106Etvvse+Y2DLYRKD4d5zEFfe7cequ1n+DVHOU4EhufJDYt1UvA2t6BN895uZrJLUpTzhVcXTrLcFd3OCm3tH07qXMG\/eW0ePGV1x1KR9t5cZcBBX+nmCSynynxup2l2zg0iV26zdlw6jntsK0BS0A\/z13yn9CHudkIacCxP3tkAvmMlpNrHhkFi9jKOyP6Mrr5ekuwibmAi2IzinUUH4Gi0UHFavBjpIb19xtW1ZmpEaCyy4SdEMWMpxhnnAS20hsZ5fyYb9YnclWaaOqdDfjRPauliqu+3G\/G6dbP7f5cc1Njy6psRza6go3sd22u7C878XprmHDhkG1atVsbUeIh2psq9J1MTXE1kXQx\/z8OjucEtvHJkW8aN3TXYcOHQrax465NbYXPeT3rOVFHSNZhiF2JNHX+7ZqbKvS9b4OYCS2LoI+5ucNaEZi+wi0D0WriKtK162SIbYugj7mN8T2EVyfi1YRV5WuWz1DbF0EfcxPQRjurnQvi7DCh9L14rN+Dy4v6lhYy1Bhq0rXbbchti6CPufHcMZ\/bL3CENtnnL0uXkVcVbpufQyxdREMQ36U3F5La6y234MrDNBE7SdU2KrSdRtmiK2LYCHO7\/fginZoQj22Se3ivSuXLVsW1FwVtqp0XewMsXURLMT5\/R5c0Q6NDrHJ2SUpKUl6aYAKW1W6LnaG2LoIFuL8fg+uaITGq2Ob6C6NZ7iR2PPmzQMjsbnejseBFU2D3Qp\/DI5Ypkbhv5TvwtH0oMgyXh3bpFNin376KQvUYIgtjGpD7MjS3Ap\/ilse2drpf33XrOkskiu\/Juav7uGPcYoRVOyObWJsgq1bt0JWVhbw7\/E1Vo1tVbpu6z1TxWWnXOLpdJduR0Qif7xJbC+ObeIVPxQaKeaNZ6Ti4OCki8ri6Tx2JEjpxTf9lhpe1NHLMtwEWrC6lE92xU9+fn4BA5oKW1W6brs9kdgomRs1agQJCQkBYsdq+GFdwKMpv9+DK5rainXx4tgmnu7iVfuYVcVxFuzVqxfs2LEDOnbsGCC2m\/DDfgV0i7aB5WV9kpOToXPnzrB69WrIzMwMqeh4IzaC5MWxzbggNhkSsLH89aJuww9v3749pMEZr5mQ2LjswdDNoWJHZZiJ1ftRRJOmVf\/4jb2WKs7HWBYNE06ITY1DEMwTOQQMsb3HnohtVzKGHcbLMkLVuOzKdkxsMfzwrFmzWEzkFStWsM15GbHxw3bhhzEdyY3\/mSf8CDRp0sTXoPXhb1H0fJGIbUdcJLQfpEYUHBNbhMwqmCFZCLFhYmwo\/gKB6OmC+K1JPK6xw9XbkcY2ZGKLAMn2CMliiO9aWQ\/DBbT5TkEEVOtAg1noCPi9hlbVzDdi44dVDiqqypl0fxEwNg5\/8fVzDa2quWfEVn1ITOfX7EuWLGEX+ZmnIAL8kscqIibl4idS\/C03Nzew\/WiFbTzbOHCpOGLECLZVi7Yirx8\/19CqukaE2Lza3rJly6BtMlWF4ymdd6iYO3cuzJw5Ew4cOAATJ06UwmB1QVw8Yea0rYRt9erVIRYFS0SIzTuvEMBffvmlkdrCqBR9kXFCbN++PYwdOxZ4DyjMZnB0SumbTionT56EcuXKQVpaWsyNvbATW3TrE\/923j2x\/6ZokLS6TgaRQH\/9KVOmwKJFiwqcDY59pNy18NFHH2UZzpw5w\/aRDbHd4Sd9WyZZjAopB1aU0OIhBj6XbPsxFlVMD4ZgoAi7a3C9\/E4kyoqYxOZVb0NsfWLjJNC7d+\/AKSPx70gMrmj\/piG2hz1kVHHnYLpRxcVSzRJHjbMhthojV2\/wEtoYfayhE1VvO+OZFbGNUdIaX0NsV7RVv2y2u9QY8ZZu3OJSbXeJ59\/xb4z0QYEvnH0xvt4yxPahv42DijNQ7RxUkLy8Pz7voCKL6uHsi\/HzliF2\/PS1aalBICYQCLtVPCZQM40wCEQ5AobYUd5BpnoGgVAQMMQOBTWTxyAQ5QgYYkd5B5nqGQRCQcAQOxTUwpwHLeOHDx+GjRs3siibkQhaIVrgrSBw+l6YIYy7zxliR3mX2x38CFfV7XzUxToYh6Nw9Yr9dwyxo6MfLGsRDcR268vvxkMuyuEvtNUzxI7irkOC9O\/fP1BDPK3FXwSH6jkeO9y7dy80btyY3cTCB5Ns164dyytGXuGdg1SOLDLVnxw7EhMTWflipBYjtSM\/qAyxI98HtjWwCxJJxMYCyHWUSLtu3ToWaYW\/OpbcUs+fPx8IC606BSa6qsoOl4i3vmB9ZL9FOdQxVT1D7CjvTifExitdKVyS+D5PRAyux182R023I6GYJt4xbWdEq1u3rjTaS5RDHhPVM8SO8m50Qmw+AogdsY8dOxak2vNN37dvX0CK87\/LSM\/7pFsFTIwG20CUd62v1TPE9hVe\/cK9JDbWxu2JLztpzq+1RYIbYuv3vU4Jhtg66IUhr5fERonNR1lxUn0na2XZKSlxbe7kW+Yd7xAwxPYOS19KEq3S\/N9kPHOqipPxDCtKkU5Va2Zx60r2vkw6O5kQfAHMFMoQMMSO8oHAx79GSzcawMjzzC2x0cDGl0dNJwu6DApZ9FMxcKK4ZRbL55yjfLgEqmeIXVh6KoL1NA4qEQQ\/xE8bYocIXDxlMy6lha+3DbELX59FpMZOD3c4fS8ijYijj\/4\/PeBFgPM19IoAAAAASUVORK5CYII=","height":236,"width":393}}
%---
%[output:19703cda]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPYAAACVCAYAAABrcgc8AAAAAXNSR0IArs4c6QAAFdFJREFUeF7tXW+MVcUVPxSlSB9QKLLsqlXS4CaN1QAV6gezJI2RCiV+oBulTSCRrTEbYiTUICGghFhiNzZxQ+y6mEA\/YEM\/kOJqExMNG0gobpdtpKldl2TZ1v0jYvyzTyog2pyL85g3e++duXdm3r3v7e8mRPfdmTNnfnN+c87MnT9T5s6d+zXhAQJAoKYQmAJi11R7ojJAIEAAxIYhAIEaRADErsFGRZWAAIjtyQY2bdpE69atK5N+8OBBam9v91RiZcU2NzdTa2srnT9\/nrZs2UKDg4PeFKhkWd4qUWHBILYHwPfv30+NjY2hkvv7+2nDhg1OS124cCE9++yzdPjwYTp06JAz2VyP3t7e0M6okmSrZFnOwMtYEIjtuAGEp7506RLt3bu3RDRBdvV32+KXL19Ou3fvpmnTppWVZyOXO4q2tjZqaGigqCijkmSrZFk2uOUpL4jtuDUEgVVCCLIcPXq0zAOq3r27u5u2bt0aaCUb9Ntvv00PPvhg8LvoHIaGhgJSFwqFUi1ERKDKLRaLtH37djp58mSQViavLLOnp6dEaiF0ZGRkQrgt6zY8PEx33313kFwth3+Lq6MowxQH32G\/Y3PITByI7RB6QZZ58+ZpvadKLFkNQU5BHvbG6sNkO3DgAHGEoBJ7bGyMmpqaJuQRcqPK5g7jlVdeofvuuy\/w1ibEjtKNCciP8PyqMjpdOL2KQyXG8w7NIVNRILZD+JOExWEhuyAyq8RhPD88QcXkERGAyCc8I6fRheIij\/C8q1evDib2ZO+6Z8+eoDPgiKGjo8M4FJd1E\/Xnjob15YfLkYcfah3r6uq0aQQOILa5sYLY5lhpUyYhtkwkEXrLYSsT7NSpUwGxmRgijBZlcFr+LY7Y6sy8IPajjz5aIrFctqigzRhbhNSsPz+is4iqo0kagQOIrTXBUgIQ2xwro5SyYcvGrI6xkxBbNmgTYquyVY8NYhs1ZVUnArEdN1\/UrLggmwhL04agKrHPnTsXhM1iXC8m1ORZclG2SSiuTr7pZsURijs2IEfiQGxHQMpiTL5jJ5k8i\/PYgthisuvs2bPBmFye\/BK6CWLz31GTWoLIch3iZsUxeebBgByIBLEdgBgmwnTlWdLPPKrH5s9XwiOzHkzCN954I5iQYtJxhMCz56tWrSp5dV7EEtaxyN5Z1j\/sE1bU566wTgCfuzwZWYzYihCbG5Y\/wYRN1FS+yigRCNQ+At6JHTWZVPvQooZAIDsEvBFbhIwcCvJz+vRpeOzs2hklTzIEvBL71ltvJbFEcWBgAMSeZMaF6maHgDdiiyqJSZooYtfX1xP\/q6XngwtExaW\/dlaleZdHaPaZLmfyKinoVzPP0M0zpyYu8vj4d+n4+JzE+R6q\/4xum\/IR7RlZmDhvpTOMjo4S\/\/PxZEpsJjSvnlqyZImPumUm85E3rxpy3Q1uVDj90RRad\/tX9HDjV24EVkjKnGMv0LdH\/0lXCvMTlTi1eI6uK56jD3+2my7W32Gcl8u68a9XV+NdXHAHffjAbuO8WSTkFXW8HNgHuTMlNhOa10T7qlwWjcVlDt7zNM398O\/OvOx\/VnbQ7DOvlsnjTnHBggXU19eXVTW15f7m9s\/pNvqIWt\/7vjatnOAn9dfTUw1n6fHe6XSWvmecV+T7sjCfegbej\/XaWeO3ePFi2rhxY7BkmAnu+skFsX1VzjVYpvLmtB6hi++8SheOdZpmiU03b1sPXTj2kjN5TpQyEPJ80yy6p34a3fOn8wapryXhPIdWz6Hmro\/pxOjVyVeTR+R7f\/wK\/bd4Jcif10c4NV+2D2J7aHkQ+yqoTGweXyclGIhtb5Qgtj2GEySA2CC2zqyq3mPHVdB35XTg+noPYoPYOtvybfvePfZkILb6ya6weiddHuqli6fdfKKa\/cs\/0BfvdDmTpzM6V++fWPIdumXmVNrc\/VkikTcXpgZh\/POnivS30cvGeUW+94tXgjxJyzUuyDBh3OcsENsQxKyS1eonu6zwrKVy4z5ngdg5b+la\/WSXc9hzr57ucxaInfMm9N1AOa8+1ItAQGcXuve2wGKMbYmg7wayVA\/ZM0JAZxe697Zqg9iWCPpuIEv1kD0jBHR2oXtvqzaIbYmg7wayVM95dvmIYVl42MkpaQvn01nWr19Pu3btKl1wkFZWXD7bcuLy6+xC9962viC2JYK+G8hSPefZBbGPHDni7YJBW8KZVtq2HBA75QSDaQNlmQ7Evoa+2KI7ffp0mjt3bnBpQFdXV9nBieJcNUGKL774onTwIr\/jq4zEtUVhZ61xafIZakKeeoZbmnKWLVtWdkOqfAacfCuLegtLmJ46u9C9t7VpeGxLBKMa6Fuz62nq7GvX5FgWk1n2K5+O0FefXtszHOexBbnGx8dLN4oyCWfOnBnc\/cU3kKxdu7bslpMTJ04EB3DI6fgesKhQnA9u5F1tfGOp7DEfe+wxq3L4UJCWlhbq7OwMLlLkchYtWhToPX\/+fNqxY0dwKCS\/E2f48XfqKD11xNW9t21wENsSwagGmnFvC824191hC5Zqps6u7ioLG2MLjyWOQhaHaqiXJMiHbjApZCLxqagrVqwIiBRF7KiLDV2Xw2DJ+nCHJHST7wFHKD4JQ\/HJ7LHFjaJh3j3K25kQOypacFWOekyyfCWS8N4gtoGP8B2OGKhgnaQW6pAEBJNQXBBb50nlMNaE2Gk9tkk5PL5es2ZN6Y40eOwkVqGkrQVS1EIdkjRhEmKLia6oMbYJ4cR93kJH9qr8iDG2COcfeOCByDG2STkyscWQgssRQwN52CB0eP311zHGDjOeWiBFLdTBJ7F1s9XiW7XsIXmyimfG+RG3jAodo+TZltPe3h6QlK9G4iOz33rrLWKyy\/rx7Sr8iBA9Tk+dXejeJ2mTsLSYPLNE0HcDWaqH7BkhoLML3XtbtUFsSwR9N5ClesieEQI6u9C9t1UbxLZE0HcDWaqH7BkhoLML3XtbtUFsSwR9N5ClesieEQI6u9C9t1UbxLZE0HcDWaqH7BkhoLML3XtbtUFsSwR9N5Cles6zh60841lkvviBl1umeeQZcXkBSBpZpnlsr3bW5dfZhe69aT2i0oHYlgj6biBL9ZxnD\/uOLa+rTkNMELsKbwKJs6xaIEUt1CEJ+8OIra6ZZqKKb74sO26XlNgcItZic\/q2tjaSN5II\/eRoQd5RJe+8kn9nPVauXBlk591mIrLgNmtqagp+7+7uDjahcOckflMjEHU32U033TQhv4qhzi5075O0SVhaeGxLBH03kKV6zrNHeWx5x1XSXVLDw8PBJovnnnuOnnzySRJLUmXl1eWkYpfXiy++GCxm6e3tnbBLjDdvcAcjbx8VHYYcSnMHIC8nlVe3xe0mGxsbi7waWmcXuve2DQdiWyIY1UB8tc0theTXx1qq4zw734HFd2GFeU3xW9S+aX5vsuZapGFPOTQ0FEqWqJ1U6u\/8t+hY6urqyggrkzRujCzSPfPMM0H0ENbRYIwdY2q+ey3nVh4iMKoOfFj+5qWFSqjgtYzne4v0+1OfTyC2fIKKTCaxX7mxsbGUR7dLSoTuPT09dOONN9K2bdtIHaurXlUIV3\/niELsnWZiy9sto4itLkdl2f39\/SSigbDTYkDsSUrsWvfYsrHLYTI3d9JdUrJX37lzJ4WFuGk9tgmx1ck\/eGxLX1HLHtsSmtxmj5o8Cwt\/TXdJ8fhYd8iCWq4gI59qwh1D1Bg7KbHFxg4e9\/MOsrjdZBhjR5gpiJ1b\/kYqFnVKadjZY6a7pNQTSuSQWVYkavY7blY8ithiFpxnxQ8fPlx2ztrx48fpzjvvDLZs8sPjbN71xY+op5yfZ9XVR2fbuve2loHJM0sEfTeQpXrInhECOrvQvbdVG8S2RNB3A1mqh+wZIaCzC917W7VBbEsEfTeQpXrInhECOrvQvbdVG8S2RNB3A1mqh+wZIaCzC917W7VBbEsERQPt27eP+vr6LKUhe60gIO5Nb21tJT5qOenkmi0OILYlgrj43hLAGs6Oi+8jerVqaXMmN\/8TT2H1Tro81EsXT3c5qYJreU6UMhDCq+\/4kVeuGWSjmwtT6fmmWbS5+zN6v3htOasu70\/qr6fNSwp0YvRSqnJ18pO+Hx0dJf4X9uQuFJd3wYidMWGKRy3T44\/+4vFduaQN4Sr9nNYjdPGdV+nCsU4nIl3Lc6KUgRAmJ6\/Aa+762CD1tSSc58RD84J8gqQmAn5x+w1Bh\/Dn9\/6XqlyTMlyl8W37iUJxeU0wV1DexaNWWF6zG7UB33flXDVCUjmuiehaXtL6pE0PYkcj59v2ExFbXVMbtxDe5IpS35VLa5C2+VwT0bU82\/qZ5gexq4TY8rpZVln9W66GyakYILYZRUBsM5wQil\/DKZHHVj101JpeFi+PxfnvuDuEeaP8a6+9ZtZ6VZDKNRFdy6sUhPDYVeSx5R0tccSW7zvmvbXq31xl+Rvwyy+\/XCl7816OayK6lucdgG8KALGjkX7kkUdo48aNFPWd27aNEntsLlDMbMeF4qpi6mZ8mdi+KmcLTtr8ronoWl7aeiXNB2JHI7Zq1argXjJftp+I2KqH1p0iIVcrbDINY2wzqoDYZjhhjJ1yjG36uUt8wx4YGAjOrxJ\/qydPgthmBgtim+EEYqcktjopJi9QibrkXGxQF+deyWdZgdhmBgtim+EEYlsQ2wxis1QgthlOILYZTiA2iG1mKSlTuSaia3kpq5U4GybPoiHz7dQSTZ4lbllNBt+Vc62vqTzXRHQtz7QetulAbBA7dM+qrWFlld81EV3LqxQuIDaIDWLHsA3ENuuKMMbGGNvMUlKmck1E1\/JSVitxNnhseGx4bHjsEgLYj524D52QAZNn9hhOkODaw7qW56HKoSLhseGx4bHhseGxHfa48NgOwRSiXHtY1\/I8VBkeOyGovj\/1gtgJG8QkuWsiupZnUgcXaWxDcT7MkM8vM30wK45ZcVNbSZXONRFdy0tVqRSZQGyMsTHGxhh7whgbHjtFb\/pNFoTi6bGLzOnaw7qW56HKXsbYIHb6lgKx02MHYmuwQyiOUByhOEJxhOIOnQw8tkMwhSjXobNreR6qjFA8Iaj43JUQsDwkd01E1\/IqhRFCcYTiCMURiiMUd9jjIhR3CCZC8XIw4bHhseGx4bHhsR06GXhsh2DCY8Njm5oTJs9MkcpROteTXa7lVQoqhOIIxRGKIxQvQ+C\/LXWElWfpu2CE4umxi8zp2sO6luehyk6\/Y7MwENuulUBsO\/xCc7smomt5HqoMYicEFWPshIDlIblrIrqWVymM0o6x4bHtWwge2x7DCRLmbeuhC8deogvHOp1IB7HNYMRBC9dwArHNbCZRKhD7Klzw2JgVr6lZcR\/EvjzUS8WuZxJ1MFknBrFBbBA7hoUcioPY+m4KoThCcb2VWKSAx0YorjMfzIrrEMrhexAbxNaZJYitQyiH70Hsq41yaPWc4L\/NXR8nbiUsUEkMWVkGzIrb4ReaG8QGsXVmBY+tQyiH70HsbIj9xJLv0OalheCSAb7YL02kUClzArErhbTDckBsEFtnTiC2DqEcvgexQWydWVY1sffs2UNNTU1BHbu7u2nr1q1l9fVdOR24vt6D2CC2zrZ82763ybPm5mZqaWmhzs6r66XF\/x86dKhUZ9+V04Hr6z2IDWLrbMu37XsjNnvrRYsW0ZYtW2hwcJD2799PY2NjZV7bd+V04Pp6D2KD2Drb8m373ojNROZnw4YNwX\/Vv\/k3Ubl9+\/ZRX1+fDouqef+flR00+8yrNPtMlxOdB+95muZdGqGZvS+VyVu8eHHQWY6Ojjopx7WQrQ2DVF9fT4\/3Tk8s+i9Lh+l3782g4+NXv4WbPA\/Vf0YPN4zT8fHvBrPiunKzxI9x2b59O7W2tnpZTu2V2LKHZg++YMGCEtG5obhyxzb+0KTNqirN6Y+mUN2Mr2n+DW7Udi3PjVZ6KVOL5+gfF+vohR88q0+spPjjv39OXxbm05XCfOO8XN51xXN0cs5PafnHb9LFBXcY580q4b37\/uWlY86U2AymWJ2UFbAo1y8CLbN\/m6qAH18+TY9eOJg4b8eMdfT3639EnZ8+lThvFhl8fWv3SmwGKi4UzwJIlAkEJgMC3oitht5hk2eTAWDUEQhkgYA3Ypt87sqiwigTCEwGBLwRm8HTLVCZDACjjkAgCwS8EjuuQhyaNzY2BkkOHjxI7e3tWdQ\/92Vy5MOfRKZNm0YjIyOldQFhissdKb8vFovBJ5WTJ0\/mvp5ZKLhw4UJqa2ujo0eP1pz9ZULsTZs20Zo1awKjW7ZsWen\/YYDl5i0Mb2BggDo6OgIj5P9Xl+aKXJjHMO8eBLYNDQ016VgyIba8WKWWe01zMwtPKc9T8FJc7hBXrFgR6rWBoznaIgo6f\/48zZo1i44cOQKPbQ5feErZC7HnUf+2lV9L+eXIhqMZ9W+5rsuXL6cdO3bQgQMHSF6PX0t4uKrL\/fffH4j65JNPaPfu3SC2C2DDPAtCyHBkVQ\/Nnmb9+vW0a9euCeNmeSwupGHuIt5iuTMEsV2wmqjkoeUJCxDbntjcCaxdu5b27t0beGz1b0fNV1NiQGyHzYlQ3BzMJKG4KhVDHD3OILYeo0QpZA+NSZ9o6NTQO27yLIrYtfgpJ5GxxSQGsV0h+Y0cfO4yAzTJ5y51\/zv\/vXTpUnzHBrHNjM1VKixQMUMyboGKuh5fXqBy6dKl0njbrKTJlwoee\/K1OWoMBKoagUwWqFQ1YlAeCFQBAiB2FTQSVAQCSREAsZMihvRAoAoQALGroJGgIhBIigCInRSxDNLzzPjQ0FCwjFTdGFIpdcIOowwr2zRdpfSerOWA2Dlv+biNH5VSPW6NuqoDFhxVqlXiywGx89EOkVrkgdhJ1\/InWSGXc\/irVj0QO8dNxwRZt25dSUPerfXBBx+Urkvi8Jx3J7377rt01113BaesiIUpfBmDuDdNPXlFXhykW8gSFvqLhR2FQiHQTT2pBV47e6MCsbNvg1gNVI8tE00QmwWII5AEacUliIKEvb29pVNYxsfHS8dC63aBqUtVwzaXhN3yEvZbzqGuKfVA7Jw3pwmxmbTiuCQ1vUzEU6dOhe7njiOh+k7uKKKOaGJI1Q4h5zDXnHogds6b1ITY8tE+ccQeHh4uC+3lqvf395ddvyTehZFeXpMedWBiHuYGct60XtUDsb3Cay\/cJbFZm6Q7vuK8uTzWVgkOYtu3vY0EENsGvQrkdUls9tjyKSsm6puMlcN2SSEUN0HXXxoQ2x+2TiSrs9Jhk2emobg4wpgVE\/eW68bM6qersPRh3tmkQ3ACEISEIgBi59ww5POveaabJ8BaWlqos7MzWI2mHsYXN8aWT4Xl87TFI2bQw6AIO\/1UPThR\/WRWy\/ucc24uJfVA7GppqQz1xAKVDMFPWTSInRK4yZQNS0qrr7VB7Oprs0w0Nt3cYZouk0pMokL\/D2vQxfVjMAg3AAAAAElFTkSuQmCC","height":236,"width":393}}
%---
%[output:834af8b5]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 7.166743 seconds.\n","truncated":false}}
%---
%[output:98025b9f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPYAAACUCAYAAACgLtSZAAAAAXNSR0IArs4c6QAADdFJREFUeF7tnV9IXNkdx3870QyDcQiBggpJ1i3iS1uIgUhxqS4U8lBwYQmS5KHaB+mD8U0SKdZNrQUTfDNCg1rGhyaL5KVSX\/rQ6nalSMA8pA8N0hoSUAvZIGoYRs20\/E72DNfrnbln\/tx7zzl+L4Q4M3fm\/s7ndz73\/Lt35qNz5879j7CBAAhYReAjiG1VPlEYEBAEIDYqAghYSABiW5hUFAkEIDbqAAhYSABiW5hUFAkEIDbqAAhYSABiW5hUFAkEIDbqAAhYSABiW5hUFAkEIDbqAAhYSABiW5hUFAkEIDbqAAhYSABiW5hUFAkEQhE7lUrR1tYWDQ4OgjgIgEAIBAIXm6Vubm6mpaUliB1CQnEIEGACgYnd2tpKo6OjtL+\/L0g\/f\/4cYqPOgUBIBAIV++LFi\/T06VMaHx+ntbU1iB1SUnEYEAhMbIm2sbGxoNj19fVi183NTWQDBECgQgQiFZulHhoaotXVVZqZmalQkfAxIAACkYrd0tJCk5OT1NfXJ+TGBgIgUBkCELsyHPEpIKAVAYitVToQDAhUhgDErgxHfAoIaEUgcLELlRZjbK3qAoKxiADEtiiZKAoISAIQG3UBBCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALEtTCqKBAIQG3UABCwkALENS+qFbJZexWKGRY1wwyYAscMmXsbx2g4O6Mb+Pl14\/546k8kyPglvtZ0AxDYowyz2\/O6uiJhb7Vs1NbRcXW1QCRBqWAQgdlikK3Cc2+k03chk6PNkkuTfj+NxITg2EHASgNgG1YcH794Rt9qXzp4VUcsWHK23QUkMKVSIHRLoShxmfmdHfIxzfM2TaQ\/29qjt8JDuJRJ0P5FQOlT2QpYy1zNUtVxF1cvozitBM2gniG1QsljsV6dOeXa9uWt+J50WY2\/uquebOc\/cyFD8cZz4\/3cP3lHNrRoh9kHbgXgemx0EILYBecxmL9DOzp\/oJ9n\/UFXVMv0jvknx+ONjkfu13lJmbqVZYhY7ce9DC5++kxaSQ24DKoRCiBBbAVLUuxwctNHu7jx9TC\/pJX2cCycWe0Wx2Guqrv5GCF9dvSxek633clXVsWWxnfkdOmw7zH0GS86PWWgWG5sdBCC2IXnc2Zmnw8M2+mHtj+j1qVPEsmez5+ng4FPxvNyk7N8\/9TX9PvMX+iT2tei6L3YQ8bhatM6307m\/+XHsVYwS9z+03Nwt58fYzCYAsQ3JH8+Ac6v9TVU1JZOdx6Lm7no+2bml77j7GaW+fOlbWu6ax8eaKZO5TonEfd\/9sYOeBCC2nnk5FhV3r3+cqaf27L8pkbinJJ1T9tgnfyfqWKTs+ayYOJOtt7vF5tb6zwvfE8fh1p\/H8hDckEriCBNiG5IzuYb9g5qfiZa7trYzN6ZWLYJc4uKJMhbYKTePtZOdHy5Tfba9TX+sbqLfxn5B6fQd8RyfTOLxr4Ts2PQnALH1z5GI0LmGnU7fFsIVKzcvae3O74qJMhZZLndxC84TaHJWnMXmK9p4TZxbfe6WS8F5ku7MmVsQXPN6A7E1T5AMz72GzZNpvHmNtwsVyWsdW65ry6Uup9jOz+ITSiZzQ8jOLTd30b2W3QxBanWYENuQ9H779u2RK8tYru3tZ0KsmppbRZeCW2+xzPVV\/NgseD6x5UHcgmMcXjT+wN8AsQNHXP4B+MITlo2XrbiLLDe5vl1sl9wvIj+x5fv55CIlx0SbH9VwXy9a7LGxMWpvbxdRLi0t0eDgoGfEjY2NND4+Tg0NDbnXX7x4QT09PbnHLS0tNDk5SX19fbS6uhpuyQ06mrzZo7O29thtmqWOtwsVX1Vsp+ByHM6CJ5OfYwwecf0qSuyuri7q7e2lqakpEbb8e25u7lgxWltbaXh4mGZnZ8nrdX4DxFbLvhSb7+ryugacx9t8scrZs5fUPtBnL4hdEYyRfkhRYnNr3dTURAMDA7S+vk6pVIq2trY8W20+CXR3d9PIyAitrKx4FhJiq+Ve3nstb9d0v0uOt3nGutjJNK8IVMVGV1wtf1HsVZTYLDJvsjvtfuwsQH9\/P3V0dOROAl6Fk2KPjo7SwsJCFOU34pju+7C9gq7keNtPbJ4Z5398KSvG1npWoaLFdrbQ3ILX1dUdGTfLYjrH4vzc3t4eDQ0NHWm9pdjT09M0MzOjJyENovK6D9srrEqNt73EllexOYXGcpcGlSNPCIGJza15bW3tkW678zHG2OqVotB92O5PqcR42+sCFef6NS+vyTvJ1EuBPcMkULTYql1xdyGcE29yMg1jbLVUu9ewC72rEuNt9yWl3N1mkU+ffgyh1VIW+V5Fie3ueheaPPMS2z2ZBrH98y\/XsL2WuvK9u5zxNp8Yvtj5gv6Q\/bUYP7PQ3OXGNeL+udJpj6LEVl3ukmvYa2trYsZcPt7d3cU6dpHZL7SGXeijih1vO68J59s8+Wqy\/ybmIHSR+dJl96LE5qDzXaAi5V1cXKSJiYmczPIClY2NjWMz5Gix\/auB3xp2oU9QGW97LVn9K\/Or3E0g\/hFiDx0JFC12JQsBsf1p+q1he30Cd995O\/\/+PS28+6f4+qRfxq9+eC6bJX49Rd301\/c\/F0tW3EL30Cx9SXfFPnwRDN\/Z5bx81T9S7KETAYitUzYcsbB81zMZ+vTgQHy1sPxRACkm78o\/9SM35\/POIi1SB31Gf6O79BvqphTx41nqFv\/zuPmn1b8TUkuhX3\/3u2CQWtOKoRgWxFYEFcVuPBvu3JyXk0oB+euIpZT8f+757wTl70d7kRkU91OzyPKWSyxZRZHR8I4JscNjXdKRZLe63F\/YfPv2WzEhhiWrktJg3JsgtnEpKy1g2VKX9m68yzQCENu0jCFeEFAgALEVIGEXEDCNAMQ2LWOIFwQUCEBsBUjYBQRMIwCxTcsY4gUBBQIQWwESdgEB0whAbNMyhnhBQIEAxFaAhF1AwDQCENu0jCFeEFAgALEVIGEXEDCNAMQ2LWOIFwQUCEBsBUjYBQRMIwCxTcsY4gUBBQIQWwESdgEB0whAbNMyhnhBQIEAxFaAhF1AwDQCENu0jCFeEFAgALEVIGEXEDCNAMQ2LWOIFwQUCEBsBUjYBQRMIwCxTcsY4gUBBQIQWwESdgEB0whAbNMyhnhBQIEAxFaAhF1AwDQCENu0jCFeEFAgALEVIGEXEDCNAMQ2LWOIFwQUCEBsBUjYBQRMIwCxTcsY4gUBBQIQWwESdgEB0whAbNMyhnhBQIEAxFaAhF1AwDQCgYo9NjZG7e3tgsnS0hINDg4e4dPS0kKTk5PU19dHq6urprFDvCCgLYHAxO7q6qLe3l6ampoShZd\/z83N5WBAbG3rBQIznEBgYnNr3dTURAMDA7S+vk6pVIq2traOtNoQ2\/Dag\/C1JRCY2Cwybz09PeJ\/92N+DmJrWy8QmOEEAhXb2UJzC15XV5cT3Sn26OgoLSwsGI4S4YOAPgS0EHt6eppmZmb0oYJIQMBwAoGKja644bUD4RtLIDCx3V1vTJ4ZW0cQuIEEAhMby10G1gaEbA2BwMRmQrhAxZp6goIYRiBQsf1YYLnLjxBeB4HSCEDs0rjhXSCgNQGIrXV6EBwIlEZAC7F5HfvZs2ellSDid126dElcKru5uRlxJKUdHvGXxq0S7+I6E1S9iVTs+vp6GhoaEpeWYgOBk0YgyAuzIhWbE8ly8z9sIHDSCFjbYp+0RKK8IBAWgchb7LAKiuOAwEkiALFPUrZR1hNDAGKfmFSjoCeJQKhit7a2Et97febMGdrb2xMz4isrK568+\/v76ebNm7nX9vf3xfejOb9aKepE8Y0tzc3NIoxHjx7RxMRE1CF5Hp+v2+fvlTt9+jRtbGzkvtXGa2fnZcD8ul+eoi5wY2MjjY+P0+Liorb8o2AUqtjOb1Hx+kYVJwCvL2aIAlC+Y\/KJp7OzU5ycrly5kvs734kqqthlxV9bW6OHDx8KCfhv9xdLyvi87sKLKna\/48qyNTQ0aH1i9StHEK+HJrZsrefn58WZlVuR7u5uGhkZ8Wy1da9gzhOTzq2G8y477u3wCamjo8Oz1da5HO7KL3shb968oWQySbJeBSGJiZ8ZmtjuCuZ+7ISnewVztoLc8rkf61QRnD0L7k24Hztj5ZPv8PAwzc7OajXk8eJ59epV8fT29rYY3kHso5RCFdvZQheqRM6xuAzX63vJoxLI68Sjaw\/D3UIX6ik5x+KSrc5zBxyjuycYVZ3Q7bhaii0r2JMnT3Lddp78kY+jhmir2HwSuHbtWm6S0v04au5ex4fY3lkJRGx3i8utLf\/Sh\/NHAwp1xb1C9ZtsC7PS2doVdzPUeYghY4XYIYqd78zqHL\/5TZ65P0O3rq4zHp3nBNycC02e5RNb56UkiB2x2Hx41eUurwkfZxcxzNY537FsXO5y\/3oLP758+XLB6w2izgXE1kDsQheouNet3Reo6DiJY8MFKm7uzgtUdLwoyF2NIbYGYkd9dsfxQeCkEAhk8uykwEM5QUBXAhBb18wgLhAogwDELgMe3goCuhKA2LpmBnGBQBkEIHYZ8PBWENCVAMTWNTOICwTKIACxy4CHt4KArgQgtq6ZQVwgUAaB\/wP3IFjAgo47mwAAAABJRU5ErkJggg==","height":236,"width":393}}
%---

%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88, 4.10, -0.88];
dq0 = [0,0,0,0,0,0];
X0 = [q0, dq0];
Sim.time = 5;
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
while tspan(1) <Sim.time %[output:group:0a16c001]

    % solve (with event detection)
    [t, x, te, xe, ie] = ode113(@(t, X) baleka_dynamics(t, X), tspan, X0, options); %[output:43560919]

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
end %[output:group:0a16c001]
toc %[output:4f234634]

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
plot(t_total,x_total(:,1)); %[output:6017d1ab]
hold on %[output:6017d1ab]
plot(t_total,x_total(:,2)); %[output:6017d1ab]
plot(t_total,x_total(:,3)); %[output:6017d1ab]
plot(t_total,x_total(:,4)); %[output:6017d1ab]
plot(t_total,x_total(:,5)); %[output:6017d1ab]
plot(t_total,x_total(:,6)); %[output:6017d1ab]
legend('x','y','th1','th2', 'th3', 'th4'); %[output:6017d1ab]
title("Generalised Coordinates"); %[output:6017d1ab]
xlabel('time (s)'); %[output:6017d1ab]
hold off %[output:6017d1ab]

% display derivative state of all variables
plot(t_total,x_total(:,7)); %[output:9aeb5835]
hold on %[output:9aeb5835]
plot(t_total,x_total(:,8)); %[output:9aeb5835]
plot(t_total,x_total(:,9)); %[output:9aeb5835]
plot(t_total,x_total(:,10)); %[output:9aeb5835]
plot(t_total,x_total(:,11)); %[output:9aeb5835]
plot(t_total,x_total(:,12)); %[output:9aeb5835]
legend('dx','dy','dth1','dth2', 'dth3', 'dth4'); %[output:9aeb5835]
title("Derivative of Generalised Coordinates"); %[output:9aeb5835]
xlabel('time (s)'); %[output:9aeb5835]
hold off %[output:9aeb5835]
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

    out = SolverFuncHopTest(t_total(i), x_total(i, 1:12));

    accel_total(:, i) = out.ddq;
    foot_total(:, i) = out.foot;
    controller_total(:, i) = out.controller;
    constraint_total(:, i) = out.lambda;
    contact_total(:, i) = out.contact;
end
%%
plot(t_total, contact_total(1, :)) %[output:50a7ac19]
hold on %[output:50a7ac19]
plot(t_total,contact_total(2, :)); %[output:50a7ac19]
legend('Front contact', 'Back contact'); %[output:50a7ac19]
title("Contact bool"); %[output:50a7ac19]
xlabel('time (s)'); %[output:50a7ac19]
hold off %[output:50a7ac19]

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
if Animate ==1 %[output:group:2c151643]
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


    fig = figure; %[output:7176a8a7]
    
    ax = axes('Parent',fig); %[output:7176a8a7]
    
    body = animatedline(ax,'Color','g','LineWidth',1, "Marker", "*"); %[output:7176a8a7]
    f_u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:7176a8a7]
    f_u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:7176a8a7]
    f_l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:7176a8a7]
    f_l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:7176a8a7]
    f_foot = animatedline(ax,'Color','r','LineWidth',0.5); %[output:7176a8a7]

    b_u_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:7176a8a7]
    b_u_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:7176a8a7]
    b_l_l = animatedline(ax,'Color','b','LineWidth',0.5); %[output:7176a8a7]
    b_l_r = animatedline(ax,'Color','b','LineWidth',0.5); %[output:7176a8a7]
    b_foot = animatedline(ax,'Color','b','LineWidth',0.5); %[output:7176a8a7]
    
    axes(ax); %[output:7176a8a7]
    
    axis equal; %[output:7176a8a7]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) 0 1.5]); %[output:7176a8a7]
    hold on; %[output:7176a8a7]
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
        addpoints(body,x_sim(i),y_sim(i)); %[output:7176a8a7]

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
    toc %[output:33efac5b]
    hold off; %[output:7176a8a7]
    clear fps body f_foot f_l_l f_l_r f_u_l f_u_r ax fig l1 l2 l3 l4 l5 b_l_l b_l_r b_u_l b_u_r
end %[output:group:2c151643]

%%
%[text] ## Functions
function out = baleka_dynamics(time, q)
    solv = SolverFuncHopTest(time, q);
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
%   data: {"layout":"onright","rightPanelPercent":23.9}
%---
%[output:43560919]
%   data: {"dataType":"text","outputData":{"text":"Time = 4.9990","truncated":false}}
%---
%[output:4f234634]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 155.271996 seconds.\n","truncated":false}}
%---
%[output:6017d1ab]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASQAAACwCAYAAACmeRMNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQuUVcWV9hYQBRqI8grQgBiU38f4oEXEBz4S50+Q8FoKiEthBhhlGnVEAiNB\/MMgE5DBUSQyAyRgZiFilMggv+s3iQyR8BJ8JOqgjLyaxoZGIw\/RRuBfXzX7pm71edS591Sdc2\/XWSsr0rdO1a6v9v5q165ddc4499xzT5F7HAIOAYdAChA4wxFSCkbBieAQcAgIBBwhOUVwCDgEUoOAI6QIQ9GrVy+aPn06lZSUZN6qrKykCRMm0I4dOyLUZKfo4sWLqWvXrjRv3jzatWuXkP3QoUOxy3v\/\/ffT8OHDae3atTRp0iTPzg0ZMoTKy8upcePGmd+XLl1Kc+fOtQMGkcBi9uzZ1KJFC5oyZYpo1xQm1jpVZA05QtIcUDa6bdu20ciRI8VbTFD4byj4xo0bNWuzUywthORFWDNnzqQ+ffoEkljcKJkmJOjD2LFj6Sc\/+UkqJ6i48TRRnyMkDVSZeEx4FxrN51xEJqTly5fnXE\/Yi0EeEmO3d+\/eDJGH1Wfqd5WQ4pxAClVHTGGda72OkDSQ01mScDWs9B06dBB\/kj0q9go2b95Ml19+uVi+qEs+kEj37t3Fu\/JvrPA1NTXiNywbsRRr166dWC7xI7cX5iHJbeF9eQmlLrHk3+Q+Qp53332Xevbs6entRMFObVPuC+QL+p1\/q66uFksy9lqvvvrqDD47d+4UmPst2dAHLCuBO8phDNE\/4MyEzmPIeGOZOn\/+fLEU5DGX3\/EbT7wfhL+GWhZlEUdIGsPKShgW81Bn4C5duggF37Bhg4itcD1saGq9XgTCngUTEgyFDYSNUK2f5QwipH79+glDRdlVq1ZlxVZYbsTFsDyFnNdcc02mXVlufhfG6BVD0sWO+8Jtqh4Hk0XY7yADXj6rdXCfjxw54hlD4ja4DhUHJlcdfCFn0HjKdan4x+m5aah3qoo4QtIYDtWoVC+IvYuqqipBQGw0XA6\/I\/B93333ibgJK7TsPbz88st1AqxQ6I4dO4YGYLke7oqOwbBxql4c\/q32VyY+9gbYy4DxBHlBuoSkkh7LwURYVlaWRYrq7\/i3jD3+rcoVFkMKIz3euJC9JPaG1E2Dtm3bBo6n7LmpnqCGShZtEUdIGkOra3BMSPJOEqrnGXnw4MGhhCTv4OFdP4WHcTBRoBy8JhitTHhBHhLekZcZ+DcvEZk4VWhgOM8++2wdQwvCR3fJ5hXvkskMfeMdQ3X5BAJWJwMvYs2XkFQPCwFsv11MJiS\/8cSy3Q\/\/NO7YaphJLEUcIWnAqHo6ssLIRuNlFHL1qrcQ5iHJ73oFTdX6dJaAXoF5OTaDZRcemdhkObwCw3EEtQvBQ3rssceySDGI8L08JD9VU\/H3S53QUNWCL+IISXMIvbb95aWSbizGb8kGJYyq4PLWOS\/5MCPrLNn8lo9e3kYQ8YXFkLyWTrL3wnGnuGJIvFxGG7nGkPziVCAkbDjIGLF3yh4P\/s15aUHjGYS\/zdwsTfW3VswRUgSovRIj8brfDpS8DIKSB3lIPCvKOy+81EOcxstDUmNZ2EVCcJmD3GG7bF47RiyHuqOlBqxZzrBdNoZXJzEyjl02mZDQtlyn7i6bHyGpyzAsYZmgQCIynvKkwLum8njKpMwYBSWWRlDTgi6aCCHJRucGoaD1xwnvEIgVAeuEhFmkW7duwq3FjDN16lRasmRJJs8j1t65yhwCDoGCQsAqIWGJMWPGDFqxYoUjoIJSEyesQ8AOAlYJCXGQiRMn0oEDB0SmMh63ZLMz0K4Vh0AhIGCdkHC6mrOPEXAcM2YMLViwoI7H1L59+0LAz8noEChaBPbt22e9b9YJSY4Z8S7R9u3bs66tABkh\/b9Hjx7WAXENOgQcArUIbN26VSTB2iQmq4TEBLRmzRpxD44fIYGIkHlsG4x8FPHKK6+k0aNHO5nzAVHjXYezBkgxFGGccbqgaAkJOGGXDSfUcWgTiYW333571mlqlGFCsg1GPuMIr65v3760aNGifKqx+q6T2Q7chYhzUjZo1UPi4ZfzkLxO0CcFhh31dK04BNKPQFI2mAghhQ1HUmCEyeV+dwjUFwSSskFHSPVFw1w\/HQIREHCEJIGVFBgRxssVLTAEXBpJ9oCFBaqTskHnIRWYYTlxoyPg0kjqYha2pe8IyXlI0S3NvaGFQCGmkWh1LMdCOlv6jpAcIeWoXu61MASSMq4wuZL6XQcPnTIm5HdLNhOoujpThUBSxpUqECJO+Elh5ggprVrj5IoNgaSMK7YOxFyRDh46ZWIWS1TnCMkEqq7OVCGQlHGlCgTnIeU+HE6BcsfOvVkXgULXJ5xswBN03CrKuOvgoVMmSpu6ZZ2HpIuUK1ewCKjG1aBluq+2OflF9rUffLHh66+\/TrfeemveFxzqkI1OGRMK4QjJBKquzlQhoBpX0xvGUNMb\/i5VMsrCfPn7f6cvf78gSz7+yAPyh\/L9TJIO2eiUMQGgIyQTqLo6U4WAl4fUsGWHVMkoC3Pii0pSvST1M1H5CK9DNjpl8pHB711HSCZQdXWmCoGkjCsuEOS76HHFzZYtW8R9Yrk+OnjolMm1\/aD3ipaQWvRoRl\/vO05f76sxgVukOq\/r2VqUX7e5OtJ7Jgqf1b4xndX+TDq09aiJ6kPr5PZtjk1SxhUKhmaBl156ifhW1aBrnzWrC7xvDOPT7apv0Vl7S8U9ZbbvJCtaQrro2fOpRY8S2tjrPd1xMlZu5eI+dN3VranVxS8ba0O34o6j21HpmHb0zsD\/ToSsMVFc9Ox36JNpe+jAq5\/rip1XuUInpLw67\/FyEB7nT+1EbW47h46PbeQIibGLQ4HSSEhX3voa7d77Zdz6Fak+JiSbhCALmET7cehTJJBTXjgID7abBtOa0Jwp\/+o8JIxlHAp0xa\/\/F8H9TMoTkHXy7de\/T507NqX+I9YmvmzjGbBiQRXtXVhl3XSYkOAdgRRtPHHokw05bbWhQ0gtV7Whn\/Sd7ggpLkLqtfEyMb5JeQJehDRu8hZ6\/te7bOmdZzs8A9okBFkQJkSb7TtCylaFIDx4IneEJGGWrwLBMwKweI4vOkhb\/31voiRw8IPBov2Z8z6kWfM+zMgCOWuD7zXWgsxMSIe2HqEPx36SkQUe3HU924h\/myRNbr9dg0a0sudWK+OSrz5ZEdJiI0F48ETeqfI8evjbjzgPKQ4PSSYkmzOxl07B0LFkQ+xo3eYDBC8JD8sIMsJ\/21pCMVGjTTngD9KEjJBXJc44bYUJCXXuG7zdSkzNEZKehyTr5C0drqQ7TjzgCCkOQuKdHHgBg3u1p8d7rIvTpiLVhS3\/lUv6ZAwPgW08MEw88FKwq4GljA3yxAyIZSza+3Ds\/wjP7JkZZcI7gmwsrylSkgmx2Yxq+t0rlZHwzKWwI6TohISJ6ZFv6tm2v3xoUIZMVSDOXQlSRjmvhg0cXge2uNWZGGB36tCUJpVfTJ06Ns1apqzbdID2VGbvhKFs0IPyfrtnbODwjGD42PpnwmRCQN1BpKTTf5YvKL+HZ0C0C0ICZt8sOig8ODngPnJsN\/rBfefRIy9to68ra\/O4avbVZNIE2jY4U3hS\/Ly15wtfeL5VdSoLG5kQ27\/2Ff36sY+y3r1zYBeRItG5QzMxNvAqhXfpMS54MWxsUObYic5ZW9ilzRvmwmvW3qk4fCKrLdhJVVWVODKCPKQRI0bQtGnTaOPGjTnJxPY1cvrdtO3V7Zk6WC\/Zbv6twRT6Qf8Hi\/tDkdx7fCRy+PDhtG3bNnGK2YuQ7iwfRk3GNBT5RGGP7F1gJ6dNv3No74IqYXjn\/\/MXmbgIDOmZx6\/KUnbUfX3PNsIQcnlgMANGrvUkpYnlFxGMDLEjEBK8kEajWglSwg6g\/DApYRnHCZ06fZfrwHvwgLwSH1nh0G7HMe0ECd743HEhH+QCYZ0\/tVTgjXquKm0pqv7jiWO5wCLegZd6dPqnAhuZEJGLNP7stjT08t+IchiXYQO70KTyi8RYMcHnMy4s9M9\/dZKuv3VSZvnxUI9mNL4sXKdy7nSeL87ZcoSelBJXYStlZWXCTuQPrebaDBPStAaT6djblbTmvm2iKtY\/TFgYn6ca30aD+i4ufkLCQcGJEycKEA4fPlyHkM7r0YVG\/Owueu2MVcIwQCy1RnrccwxgaCAe3uLn5RDeA7CjP2hGY\/5mvVB6juf4EQh7T3JDqsekCoE6\/XbQkBSJp\/\/ItYQ4Dcp9OLIJHVj1uee2Owe6m582mMNbjmTIya\/\/sjzoO8jIa0udiRo4MWaPdC2lJm\/X0Ng3twlvUiY0JgmQwu7KowRP6M3N1bT\/ZPY4BMmFJRrvdMqeIcbrts6taM2924RHCuLGshFkJAf+uW\/skakeUdjYoN4B\/a6niqPDMoQED6lTSXq9pD1HTpDsJbG9zJo1i8aOHRvb0ZF7F42iJmMaZWKJsn7Ak8WE8Q\/ff7\/4CYldUHxSG4+Xh9Tu2Zb03oIPtHNleCkAT0kY5Zaj4l0GFkYHDwgzL8dxcp1hvAhJDljLv4OQ3tx8QBgZiOvV3dX0n5fWZOI3ccnA9cDQcTRE3kHz+4234L\/16Sn687fPMBJYByEx+WIGhmcGQkTbdw7sTLcfbJ6BYNzkt2LP0xLe6bSBWYQUN+Y26oPN7Nixgy677DICMeW6XIOs7CFhBQI7Uydy6A7G7e7O3enle9fFPiZBeFk\/OoI18KBBg2jy5Mn02GOP+RISztGUl5cTrlvQeQAgPAN4RfhvBhn\/fVnDJnR80WdimeY1++rUH1QGSzHEPOAFyQ97ZByfQbmTP2hOP9\/1aZ3lWr4y8PvsdnsdmZGJGuUR33pg4RUiVlS96jMjqQcgHjzwkuQZmOXEkhE4YVxMZLGjj8sW3VXwhAS7GTp0qOeKIqruyDFaEBJsRp7IT77aQHhOA\/tdT88PWFO8HpJ8ann58uUUFtQG0AsXLqRFixaFYs4Kru4gyQYRWkmOBTALc8BaroID2jIhvfPXjUSg1suDybH5rNe8AuYoIMdvOL7E8a24PUZZIJkgQYhY3mGM+O82Mun3vPdgwRMSbGf27Nm0Zs2avE76yx4SDs6CeOBRfzKtQkzkiB\/dceUwajqmEW06Yz3tHnigeAmJ73Rp3LhxlhGpgW1m8OnTpwsPKewrm1wZZ5nKSX9BHkMcBMB1cHxITipUDf6WAR3o6OTWRpZGcl\/kZRL\/3Yuo5OVknFiodWHZzDs3MgHJy2yT7cND6nLxOOs5NXH2KY7dNZZH9pCOtj8k4qzwkHijBR\/WhN2ArGxMGDJO1pdscuNhHlLUqw+8rtawNRMjPqTGkeSANvrNpHBk+qf0\/n\/uj1Nfs+ryiiPJyyUUVpeTxoQhyizV0Ia8s8jLbNNn2hDU\/scfP1GwhMQ70mvXrs37tkjVQ8JkD32BbsqbISCtM5\/9xvrRq6IiJD+jsjETq8s2GPwri\/tkpQOAFMru7Uh\/\/foZmYxtE0TgRcLycglt8nIyyRsIbCynvQzQBOaFVKdXoigmc\/nuMJRBfAl5SqYnjNR4SH6DGHdmrWqMppRHXrZ5xWcgB7a2r\/h\/3xglJPRP9j68lms24kdhONvyXuPWp7B+pf13HTxQ5qF5D9Kj5VNp51Z7B8IT9ZBsEZKtOBIC2\/CUOCtbPn7BQeUbqs8m5P6Y9kzkIH\/rfucIqOVAOpaTyC3is3VJGZEN71XHAJPqv067yEOaOnUqLVmyhJAq079\/f5oyZYrn1r9O8FsHD50yOrJHLVMvCMlv5ykqWDrl+XbIdZuqs9IAZBl2\/Mf\/9k2k1GlDtwzfPYQgP3ZR2CXn+JFpUtSR00YcKSnj0um\/Thn52togQuIvk5SUlNDSpUt9d+N08NApoyN71DL1gpDUJUxUkHItDy+FM65RB+9iJOGdIGZ03dVtxBEZeEZ8mDbXvsX1ng3vNSnjigsj3KndoUMHqqyspLfffptuvPFGOnToUOZvEyZMEE0hr2\/16tU0ZswYWrlypSOkuAbAhALJWcJxyelXD5ZnrW87RxzFgHeCB3\/jM2Z+eUum5EK8CGfEkHi45\/QVumlYrnF\/TS\/bVH2SDwabwjyfetUEUdVDwhlQeECbNm0ipMbI5MNekiOkfEZAedcEIaEJ04rPxAMywuFe+byauouBNAFkJ5u8DA3yMBmZuk4kjmGXr2KJoz61DlWfGBMTbcVRpzpWfku2\/fv310mWdIQUxwhYIiQ+u2XySyTyTQNBX9WQ7yAyAKGokj2xNJMRL2WRnCdfxxInJl4eks61JXHKEKUu9TobR0hR0DNQ1pSHxF6SqYvQ5Ptkwi7Q58CyqXu2uX54YEnvpOmoCLwk\/iiDTvkoZUzqUxQ5ci3rCClX5GJ6z6QCyXe+xPmxRL5LCBDonlPjNAETu10InOMwsclzajENt6iG0yLUu77jaMOkPsUhX1gdvAxDIJuD2tj2d0u2MORi+t20AvG9znEuETho7nc5mh80JohDvqXSdIwqpiEX1chXD+uSuk77pvVJR4Y0ldHBQ6eMiT7Vm21\/FTyOJ2FGxvULYd4SZvCgRxxozeFbY\/INln6Xk3G7urtDuBETj3odigkFirtOvjgO9fpdYie3GTYuKHvj6BsS+cZY3NjEVZ8O2eiUiUseuZ56S0g8I+PCML4mVj7Lo6PoDCRfOZvrrA6i4aMcqFPe9tUlIXlQkZQ57sdvGblfyIQSetXJEwZ+y3VcuN5upy6kcaceLtjDtXFjrkM2OmXilgv11WtCYkD52tjGkheES+3lRzYKr4EI87B0Bo+\/jcYfHuCcIfnd3XuPhla1bnN1aJlCKMC3NzSX7lRXx0UlLK9+daPuiXynPq0Y65CNThkT\/XOEZAJVV2eqEEjKuFIFgiSMDh46ZUz0zxGSCVRdnalCICnjigsEncO1fKgWR0zweH3Nh+XRwUOnTFz9k+txhGQCVVdnqhBIyrjiAkHncK38eSROE8Btq\/iWm\/ro4KFTJq7+OUIygaSrM7UIqMbVtknthwfS+uw\/tidLNJ3DtfgiifzIH5d0hJTnSCfFznmK7V5PKQKqPg29YAINveBHKZWW6IWPn6AXPp6dkS\/K4Vq8JC\/x8DENR0h5DrUjpDwBdK9nIeDlIbVJsZd04Ngekr2kKEdH3AVtBpTfEZIBUOtxlYWuT7qEFOYZsQro4KFTxoRKuaC2CVRdnalCICnjigsEHULC3Uj4PL3OV2118NApE1f\/5HocIZlA1dWZKgSSMq64QNA5XFtWVkbdu3fPatLvs0k6eOiUiat\/iRISf2OKhfC6+zcpMEwA7OpMHgGnT9ljoIOHThkTI2vVQwLTy24lyMnrCwpJgWECYFdn8gg4fXKEpKWFftdtOgXSgs8V0kTA6ZMjJC1VkYN1cr6EUyAt+FwhTQScPjlCClUVzpfYvn17nfR2VqDy8nJC+rt7HAL5IFDohKRzlg34IDubA9t+AW2UC8Ojffv2hP\/NmzfP+pUtVmNIrFRMRocPH6aRI0fW0TUGDD8sXLiQFi1alI8+unfrOQJhBph2eHTOsiEee9NNNxG+0dazZ0\/xbbYFCxZQLpnao0aNotGjRwtYBg8eTPv27bMGkXVCCjv4JzM4vjkFD8kmINaQdw1ZQ6DQCSnqWTYQ2IgRI2jatGmen9sOwwPeUd++fQUpFTUhBS3TZO0MA8yaJruGigIBVZ+i3AaaBADqZYBRzrLxsi2fJZvOss4ULlY9JACLuFDjxtn3U6u5SI6QTA13\/axX1Sd8Ow9fFU7rU7GgiuTPaOlkas+dOzfTnbCJX8e+dMqYwM8qIel2ICkwdOVz5QoLAS8P6az2Z6a2E1\/vO551j3hUQkLH4CnhCYrRBi3HkrJBR0ipVUsnWFwIJGVcccmvQ0hoC8dHQEBhn9PWwUOnTFz9k+txhGQCVVdnqhBIyrjiAkHnLBuWbHFt+9ebGJLuABW6Aun205Wzg4DTp2ycdfDQKWNi9JyHZAJVV2eqEEjKuFIFgiSMDh46ZUz0zxGSCVRdnalCICnjShUIjpByHw6nQLlj596si4DTJ7dky8sunALlBZ97WUHA6ZMjpLyMwilQXvC5l4uMkHQP13K38Y22bt26iXNt6ueRUEbHvnTKmFA0F0MygaqrM1UIJGVccYGgc7iW2+LTENXV1Y6Q4hqAQleguHBw9cSDgKpPzUo7x1OxoVqOVuzOqln3cC2OjMyYMYMOHDhAHTt2dIQU1\/g4QooLSVeP1xLlrx6aSH\/1UN1PTKcFrT8+OZP++OSsjDi6h2uxVMNTUVGRuYrELdliGFWbhFTavKGQuHf77AO\/FYdP0Pp9NTH0Rr8KyAI5SksaUKfTcu05fEJUsGHfcSvyQIZOJQ3pmvZnUu8OjX1xgVwvfvwVAae4H1UGyIO\/cVt7jpyg9ZU12ph4eUjNStP7Oe2jFXtI9pJ0jo5UVVXRoEGDaPLkydSvXz9HSHEqJRRo\/szHaMA99xm5C4kNv3f7M+mOC5sEiv7iR8cIxvfk1qNxdjFTF8sy58YWtbPbaQOH0eFhY+Tfln90zAgRQI47LjibhlzYJGP8IOT1+45n5EIZkKVMVJB3\/H8dioUs\/WSAAJADbYvJQyJKIWNlTeD42JzgTCiJDiGVlpZSnz59spqvrKz0XLbp4KFTxkRfUxvUfqVsb8YQeEbMlxRY4VmhocwvflQ7y8veEHtNMFAuizIgg3xl4EFEG+N7NBOEyHUHeUEs+\/iyElHFnC1HYiMmeGVMiIyJjnf4UI9mGQJDeRBTLh6T3DcdLGQMMUbAJGh8kjKuuAxWh5Dk60fk2yPdki2GUYACLZwylhY\/NYMaHt6fNSPmaoiy0UX1MmSDgeE9ueVozh6Banxzth4leGFRHpkIgEeuJKn2K1dCYWwFyf7XoUj9wTsv3naO6H6uhC+TO8ZnyKrPs+AsdELSPVzLnXaEFMWaNMp6KZDqzusaomx0MHwQQC6zOMRGXfAkhAHmQARxGJ8MH4gJ3oGXEYbBzH3BkjBXIlDbYHmAM4gp6FGJ+Y5XP895XLgdjMvyfueIeuRlZKETUthYRv1dB4+eF5TSwkfH0vcnPWMkbOInc2qXbEFfPGDFD4tfqF5Rrp6En+FFIQKZjOIwvjAjDCMD9krilEXEd06TQhA2JshQXsbxpNF7WbUgJx0DjGrUhVw+DA\/W1W+3b0\/Dl23LeTWQC0YFSUg63orsPeS6FAkCVJ6NofhBD+JEMJIoBBZlMGUDDwswmyJGWd4wbNYPay2Kx02GsgzAG7gDj\/8puUh80gfXJ7sPRlDmE0c7nxpJdy7blqVqrKs7qRWdNeSfjW0sFZWH5LVskZcJbBC5LKuiEgF7Gn6kxLLoLGOitO1VFssVtIcYildQ2gYZyZ4KiAceCmMjE6dJMmIZmJSe+KgpdR8yQXhK7qlFAF\/z6f\/OtKxJEuODMYOu\/sfhbvXnu2xhShHmUqrvi5mwRzPxZxgi\/m2DAGRPDbEY1TuJElMJw0T3dyYllYxtkpFMSnLAGikFpj0jFScmpecrm9OyfbWpFWLcMF5ltTozZ8tRqjidZqGLc67lIA\/anrP1iMij4ge6gsnkxY+PiZ1f0w88xfPoYCbmhjgijw8mkKg2GJe8BbtkUwHgICkSCk3mDXkBz7s8nFwJUszkF+UQ\/M53cOUYG3tKnF4QtrzMt22\/cYGyI33DxPI5TGYZDxgedITxsOGpyfLJGyOcbmJCVxq0bO8JS8OWHbL+3qW0VGAx8OwP6NdfXSyIEg++zTZ61Cia9+Agq8vcRAiJ7\/6tqakRbqH6dc2k2DlMscN+ByFxhnOUTOKwenP5Xc65wvthyYO5tFFI78h41BLBcd\/0BF1jRv8bfMvP8LP\/3kAhgge\/c4g6nKwSEFY2aEebvmydtcxu6EMotW1mk0pQ2XzGqG3TU\/T5vAHFTUjy1QhIce\/fvz9NmTIl6wubNgkJBy3b9b6O2l5zHXkdulQPOnoN8P4N66hq\/bqsdP+oitC2Se1RhqEX\/IjanP5vruPAsT3a1e0\/tpv+dPAP9P5nf9B+J4mCXkYvz95Nmjei1hefL8YFjzo26rioRikb7aefltDnlfktg1rVNKIfftpSyNKqpva4EZ6DjaMdnfnwxA76qOQr2v9l8Jie+ML\/89Unv6jMGjKvsif\/7P3+CeXdkz7t2LRBuTPWPSR4Rzh3M2nSJN\/PtdgAAwp+\/h3DxCFLKPeRPdnnhxgknZPhJZ06CYN55dorcyIlkNG0a1ZklFQlICaohj6zsWzcrWsaEYzn\/\/x3OX34zQ5Prsl1VufK\/GZk3ZkbBhQ0q5\/T4Wy69Oa2BFI6dvgb+rzyGKlG1\/jEAW0exYSz5alf0M7X12i\/wwbdtmknurnjUBp81i20\/9geev9gNtGrk0dYA6gPz31vXBVWNNHfbdigVwetEhJ\/UXPNmjWEVHe\/L2yaBgPkcc2\/PEMgkk9efJ7e\/\/kvs7CRZ2ov45WNiV3xW2f+PVV\/+Am9vfg1X0XyMkKQx+yjP6SDjb+hWW3eoerTM24+bvjD29sSiOmRi7Nn0iAN95uRT\/65bh2+ZTVmblkGdRbH7I0J4pK\/vVt4nDj1Du8z34dP90edMDBRjLvsaQKJvFGxjF74eHa+ohDqnH\/zW\/RGxQs0970H8q7PVAWmbdBPbquEpH7ALoiQLhjzLL366upIeKtGLM\/E8uzd6+4e1KT5mfSnN\/ZHduVVY2SD7dTrQrps4OW08ZdbqPqDTzzlVt+Fco4541aC2\/z4l4tic8MvatiV\/umaFfSzXT+l31UsqyOLn5seCWwDhc+\/404xUajXb8TR1IA\/vC1IbsPD47SrG3rBBLq5dBjNffeBWJfAqBdLc3hJ8LrS9iCgjf8FJSebktkqIUXxkFreNV\/0GduTuslsdcjii0piD4bX3V1uvYku\/N7F9NZTvxAeDR6vmZoBj2K8331hpXjtt0P7a43X\/Zc9TZe0upambhgUu2Ki7ptLh9Lg1en9hr3vJrUdAAAP1UlEQVQMErxWkAY81iikoQU0ETHZ\/WZIfy2viz2ZFz5+IhbPSJUTXhKWf2n0kkaNGkWjR48WIgd9blsX+yjlrBISBIsSQ5o+fbpI4NIlpLCOm1Z6BGC\/t3yl1gzPCg+FhPtu4nm5b5UIcE\/dOMhE9bHWCTLHEhrLKlNPlAljWq8VYqlmKtaDyQKTxqMbBsXqfcWBHbyjvn37ClIqekJKcpcNywEEOE0qPeIVmI3Xjx8XOBObVngopulZPg7lF3KeJnJd7yXXdrkdLN2CvFjGzTRZsBdrup1c8KoXMSQGJok8JPaOsBzAssDkA+IDKcmxEBgDB2hteEfcP56JOYiKttMWt7DhHTEeXqQkjw3K8VLalHck6x4mpktbXZvxlDj9I+kxqleEFEYGJsBgz8WkdyT3i3d2kFIAMsSsvONXzwsyBElwUDMMizh+Z1KCkiP\/BQZgcqkYRWabE4VKSpzugb\/Dc17auVUiXiV7y5ADcSXEFRG7MrWU1xkfEzao0671GJKOUCbAwCwMBTQRMJX7hBkOuSlITORcp9plyfWZGAkUEI+N2A5mf9zR3OzgKUGEbZt0JiRPpmWXx+ZEwWOD2NAmWi\/y0PAAH3i10I0mv90ldihtLKOgH7jbG54zZAMR4f8xRtAhG\/rhZ48mbFDH9usFIVmLUUh5K\/BEnnnvgczyiHd53htZTtPOfsq4wkPZ2dihCGp8BgHvNHhJtiYKYMCeCIxezQNiOfou72hlsmDPsJYQd2fFNdmjTTItwBGSRJ9xg2FrFmaFR97K\/Zc\/XWdbF9vabT46RVctOWls94ZhlIPrSDZUd7AQJ8GT5LYzG6XpYDb6Ke9qwUNSd7h4wrj9vm\/EcimOJEg\/j0BOzEUIwStHKukJI24b1PGOUKZeeEhRtnt1gVPLqYFqjhPJOUZYFuDvTea8adQdh8J\/b\/krmURArzgNG2iSeUogARCnjbgeJgscyWEC9soDGr77IN045wQ9\/dxAo1vxqsfOZIgYFj+qvLnqZa7vgZDG\/eKXNGP039D2zZtyrSbye0VPSLZmYa+dGXWWY8U7NOSntGrDE5EHS\/cFL0NXl0aXnHutiJXYWBawR8BBfe6HjYkCbXFf5biQV+Ioe7ALR5q9yM2r3yBDeQeYs7ltTBgc6\/zkxWWZs5ggpKtfWk3rnp6T9dFKXR3MtVxRERJmW4CLICFv7duIH\/nl+6jLIpT73rZ3shQv14ELes9L4b1mYXgJNnZzIA92sfDI58nw9\/0b3jSu8HwERN7G9yKp\/j97Rci48u8HmBgWUaffBKlOGLbiSHIsC\/Kxl9at59V09Uv\/t07s0RgwpysuGkIC8fSe84w4tS\/HS2zEj7yUm+MW8vY+yvX+7Ur60++WGdvtC\/IIvWZh7OiYjCPJ8mB8MFHgM9G2PFeMg9\/yR122PTJ5C+2677ycb23QMVbWU3WZ6jVh2IgjsX1gTBBS4AmDJ\/I3el0a20kJHXyKhpBkr0A2PPwdQcxPx\/5UBJlNJJx5zcAMvqxUKHf+9Em0q3eDzEykM0hRyrAieZ1s57Ni\/N14EOQ\/3raCHv54sNb5rihycFk5Ox7\/jQdLE5Zz\/Xf7U9ODp4zFbIKyrmUPlsv9an4jox6s3zLVa9xsbDzInhn0A7qBSQNEdd0D4wVBxXV0S0d\/UktIUdav6mwrGx4Av6SilC5edVLkeJi49iEor6j2qonaqytQrqTfdfTWiIbGZuEgj1AmBCgHGwEriolT9mgT29pQdNkLgJy97plIV43fKcYFE0Xcp+rl+JFXLEYO7LOXu\/rxhkY92KBlqurBfnfyM9T1jmH0ZaszBIa\/GTIgp\/u2\/IiANz\/4mJOsH5Dz8u9cTc\/\/0BGSuGD8nueW0aYWrcQAgLFrb2T8y1UNSCjjp6RT7f1G7BUwsDACkNNFj++kZS\/eLzwlzDp+B045cS6IybFTo17KheCw31axPMsxOX215C7fqzDkGyz5VkT0+8ie3Vn9ZxllHPA3bPHj8TqrJRMCB5qRtd1kzu\/p1e7rM5fV8SwJPPgGy51N9wgv5mgrCr3tUMYPh415W5\/bxzhhjND2yfFLBXZ875AXKfFxiqiXoUGOW0qHiYRDr2MgcmAf5ISrRma1\/hchm7zj9ResaxMZ0Q88iFXi36pu+o0N\/i7joeqZvP3Pkywm0g3PzaJjD90g4nCZJe\/BU6Jfl557ndBHjA9IfRPp3xTKdsN9lb1ZyNLznVJ6fsoAY96rl52l1kPCXSx3P\/AgtR52T0YBgohCvrZCvpALwLaa8nua+9xA8TorIf6bU\/MxoDCOXB8QHIzK69pY+e4bEBJ2epr\/eLjok6r08nETxMLwQAn5+ImOfCjrd7BXJgQYEowDhHDTwd7CYJmkOACNpRQezNC5Pqfe3U0bxo+jHR\/XXrTGXkDX2++kmw\/2zlJ4Ptcl38yY79igzaAEUF5Ss2EjO1r1VFCHnGjKR074ptAo2PCdTF5XI4MQ0A4mEyaHsod2ZtIVePcUZXhs8h2fI6vezATxZf2A3dzy86\/p0X8rc0s2r6QsTrPnwZe9JfxNHmD57hsY3btX\/DCLLDDjYka85NzanZ\/3P6s1FhALnqA7rGEg6u9BcSk5IY+PJBy48AxBBrzNy9uukJtnQFnJ8XutUfzFK\/TDQcVCNRYYGzwWOVlSjnMBm37X\/IhK+l1\/Oub2F89M9tLYa5Hr98JBNTw2dhjcySVv0rJHs3e0MGFgcuCxAdZ\/+myd8Mqi3C0eJhf\/DhLE+KM9zlNS4zzsrfgtm\/zGR9XRsLFRCQFL6NYv7spK4gTuiPtBh1YdeyHjNWNsauvfI5bAXk\/Y+HD70EuMz21vVdOE8pscIeWbJcqxERj3pbcMIwROk7r0nj0yeFBYEvDSgbfCMfjwFvg6XQ44R5l1o5SVCYFjRpyTA0\/i5o7DxNLWRDwHcvKyBMqPjYbfvWouH0sHF96Bw8TBy25Zf0BCIG9TF8fJMqqEwEtdjlG+sXeZWELjMXGpHxMv+gpZ7v3Dbhr2UF9HSPkSkpxbgSUDAnNJPvBAeHaSYxl8TYmJgKVff2VCkI9s8HIJXqJ8Bi9u3PiqEYwRJgpeysXdjm59nNAKr0JOnOTlEZbONnKlaj3g2lszsayTU1f4bm\/2EE2maWDCgj7ikrYRn+4o\/gvadBQlX0JCGwAWj9eyQEeGOMuAkPCk4fZGJgTIo+bC2LgriUkYMZCV115pJA0jythxjA\/v2MhaD5ON9TbsErmwenL9HYQIYvz2\/uN0x3FHSALHOAiJgTWxlR11sBHMhrGbPrSpI5fsldk4Q6bKlHT7qjzyXVE2LmQLG6Ok9Zbbv+KPR+mGc\/c6DyluQrJxkjxMydRbAcPKm\/ydd\/JsxES8+sFxkqQ8AD9CSoP3CtmYEJLSW54wWr78Ot3Tq4sjpLgIycaREV3iEDkuHYf5pgbo1hNHuahf34ijTbkOPjrhtZsYd1u69cnJq7rvmCqXtN7yhPX1jx6jCf9wlyOkuAjJlMK4evNHwEasKn8p63cNcYRNckEw1YmRtj\/BkguA7h2HQDEi4AhJGtWkwChGxXJ9cgjkgkBSNmjdQ7r\/\/vtp+PDhGYyWLl1Kc+fOzcIsKTByGTj3jkOgGBFIygatElKvXr1o4sSJNGvWLNq4cSOBnPr3709TpkwR\/+YnKTCKUbFcnxwCuSCQlA1aJSQVGBAUPpe9cuXKLC8pKTByGTh+hz8\/vHr1aqup9k7mfBCw824h6kZSNpgoIQ0ZMoTGjBlDCxYsoOXLl9fxkMrLywvKuHFDgZPZrJHDuB3OZjFG7Yyz7Y2lxAipa9euNHv2bNq+fTtNmlR7h4\/sbWAZB5Z2j0PAIZAMAlu3bhUTrM3HKCHJAewjR45kYkVMRocPH6aRI0d69hcMjf+5xyHgEEgGAVxda\/P6WvTSKCF5wchxI7Cv6hklA7tr1SHgEEgLAlYJKWiZlhZAnBwOAYdAcghYJSQEsbEmbdy4cVaPvXKRkoPEtewQcAgkhYBVQkqqk65dh4BDoDAQcIRUGOPkpHQI1AsEUkdI8rJu27ZtvrtwaRydxYsXU1VVVeqD9RzL69Chg4CxsrKSJkyYQDt27EgjrFkyAePu3buLv61duzb1WMvC++XdpRF0GWebWKeKkOTM7VWrVvnmKaV5AAvBSGbOnCkgxC5nIW00II2krKxMTFJ+Wf5p1A3IxDi3bt1aJHbKicBpk5llXbNmTZ1zpqZlTRUhYQYZMWIETZs2TZxtg+F069Yt1bO3nFOFwSoED0lVqkLA2csQCsUjhezAuHPnzgRCUk8mmDbyqPWD7KdOnUpLliyxTpypIiTMgDfddFOGgPwO30YF2Fb5QjIQGRPIjccvSdUWflHaKSQPiSfa1157TRwmTzshqbvhNpf0qSIkdaZ2hBTFRHMrW2gYs7fRp0+fgol9gfC3bNkivGevs5u5jZy5t2Sd2L9\/vwidBJ2qiFOSVBGS85DiHNrwuoD37bffnvqYhl9PCmGpKce9CimoLWNuc9JKFSEVYgxJXfoUSgwJxozDy+pdVOE0lp4SNg0l116ru1Wop6ampqAmAdVRyBULnfdSRUiFvMsGsAslhlQIhuylvCDRdu3aZWJdhRb7KhQPScbZ9i5sqggJSujykHTmkfzKeM3aNgOX+Ugvy14oMnN\/C4WQeHLlfC+b+YCpI6R8lNW96xBwCBQ2Ao6QCnv8nPQOgaJCwBFSUQ2n64xDoLARcIRU2OPnpHcIFBUCjpCKajhdZxwChY2AI6TCHj8nvUOgqBBwhFRUwxlPZ2zmKfGxCvXrxXJPdMrE03NXS9IIOEJKegRS0L56UNUWIcnHKoJgUL94nALInAiGEHCEZAjYQqo2iZPzyACeMWMGrVixQuuKC\/kOp0LC1skaDQFHSNHwKrrS6u2R+OACHlyTgXNuY8eOpbPOOkt8mAE3TOL7erg+A6fWS0pKxL\/l83AgDpzExxOUSY2s5UGDBtHkyZMzN1UGZWHbPE9VdINcQB1yhFRAg2VK1KAlGwgJpIVbDjdv3iyuomjRooUgITzTp08n\/saeutQLOmumnktTCUc9F5jkpWGmcHf11kXAEZLTijrXwcrEAkLCw5e3ySSjHrxUSUS9vUGGWi3LXzn2+yRWEstKpxr2EXCEZB\/z1LUY5iHpENL8+fOF98QfDuBO+l214XUzgvzpdbwvk1OS9zynbsCKWCBHSEU8uLpdi4OQ8MGAKNevhJVV72tyHpLuaBZ2OUdIhT1+sUgfFyGpMaSgS+DUGJLX9cXy\/eouhhTLUKe+EkdIqR8iOwLyDhc+41RRUZG1y6azZIOHhEfeZQu6GTFsl0191+2y2dGDpFtxhJT0CNTT9qPmIYUt8eopjEXXbUdIRTekhdMhl6ldOGNlS1JHSLaQdu14IqBzTk2njIO3OBD4\/xUZo3nDimYhAAAAAElFTkSuQmCC","height":176,"width":292}}
%---
%[output:9aeb5835]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASQAAACwCAYAAACmeRMNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQd8FcX2xw+i9BpqQpGOINI7SFMUAWkiIBYQiA8MRYqgiCD1CSKoiPB\/AQSVIihgRODJk96rFFF6SSCUhBISAqH9P7\/Bc50st+ze3b335jL7+fgRuFtmz8x897Q5ky4kJOQeqUNJQElASSAAJJBOASkAekE1QUlASUBIQAFJDQQlASWBgJGA34E0fvx4atCgwQMCmTdvHk2ZMsVrQfF9zd7HWQNmz55NxYsXp6lTp9LChQu9bqPRC2vVqkVjxoyhbNmyUWJiIg0bNoy2bdv2wG3QvrJlyzr+PSUlxedt1ftuHTp0oIiICDpx4gR17dqV7Oo3ll1CQgINGjRIPM9V38qyO3v2rNvz9b6nkfO0MrBLJkba5KtzAwZIMji4A9avX09DhgzxShZWdiImTZMmTahnz56iLf4Ckqd34kmHNsqwYkDZAWevOke6KFCAhA\/MxIkTKUeOHA7ZuZKn2Xf2dL3dQML9Y2JiTH3wPb2Dt78HJJCcDQ5vX9DsddoJY\/Z+Zq73BCR\/gdLMO\/lKvp40JMi2du3aAaFJeupnM\/K2895m2sXXBiSQ0DjtAOGBmyFDBtF2+WvPExHqdbFixQiaFQ6YgjivWrVqVKhQoQe+fKy+41x8HcPCwsR1bOKcOnXKYSLh39lM6tWrlzDZ5s+fT02bNhXXsBmgnWAMV773oUOHhGni6pDNLdnU0pq22vt4mnDa58nPkc0S+T54PuSplbfRvpg+fbpT+cLc9aQhaZ+lNaFcvQePIXYH7N27l0qWLEnOTDYjH0DZbMYznJl0rvoQ5zsbq4sXL05lih87dowqVarkGOMyRLZv3y7OxXu46h9tG3nc1qxZkzp37uwYCjyG3PWnJ\/lbASH5HgENJAbK+fPnH\/AzyF8zHgCyiSd3Il4YHcG\/9+nTR\/ydoYbrGVg4lzsckKlRo0aqZ8uDCj4kwI7bCZ+XDNIdO3akMgMef\/xxca+tW7c6NUW1Go4R1d2IpiE\/h6F75swZAUoezAA\/3o9\/54mslYf2w+GsL4zIV37nZcuWCfkx8Fu2bJmqH929hyvQOQOIXphrz+MPmWzmeepDV\/Jhn6T8EeTx6QxIrvqH28Qyc9Zf8nh1JieeWzx+XcnfahjhfmkCSIULF0416VmIPLGdmSquviqAzIgRI1JpTBCEVpPhgesJSLiWnbIjR45MNYG01\/IzZI2KO5UHO4MB\/679N3fqtjMgOdOqpk2blgq4cO66A4Z20rVr185wXxiRrzMgQbvUOuadQUR+D2073UFHL5C0HzK8l\/xvrL2460PtWHX2bHcfIllDwlh2BkV57CAAgoO1IU8fOXluyZqtrwIjAQ0kJjVrIVois5A9AUnWXH744Qdq1aoVyYOGv1q4n3bCegISzA6eCFFRUdS+fXuHBqRVd7n9ziJkzoCiHaxwqstfN1ke7iaVfG9+Px6ofA+tmao1Z1kL4IlupC+MyFc7YXjCy8+D5sAT09V7NG\/ePFUk1J189Jpszj4I3D5o37t27XpAm9Y+Fx9DOULr7kPiTkNy1T8XLlxwmMfO2uQKSK7605X8zUTA3WlWAQkk7QDRfu20L6QHSNzxgEFISIjDXNMOmPz58xsy2QAk7rRLly6JkDynAxgxo8xqSJCJK8esMyC5Cn97MkuM9oVR+brTAhls6MPvvvuOXnvtNac+IZaFDG9PWpAep3Za0JDYLcBpFJ58dEac3LL8XaWcmDXjAhJI8lcHYX9PQtUDJNkkc+bEhSAhZJ5wfA4DSqtRyV852YkoO5u1YNUOFk9g9aRea693FqZ21jZnvhcGlBbIWpNA+w7aNroySVzJ152Pw5V5wiavrG3o8XUBUK7yivSE\/fFc2b\/ozFzS40PS5rA56w982LzRkLh\/4uLiRKAFMkJeFY9LLVTdzS1P8neVy2UGSgEDJO1LaHNmtOaP7MDWAyT5q6nNb5LVUnx9MTm1jkp0Kps1WnMA9+avh7bdrnxTehLztHa73q+ZHOVhuWqjcvI5sgnpSUNCIqbRvnAnX0+AcxUx4oRQV+8h+3fwZ3dRNnnsaWWnBZhVUTY5qVYeI+gLvVE2Vz4k2XcIrR3HjRs3BKD4gwPgaf2kHMGW54cn+ZuBj7NrbQcSBm94eDhFRkY6spq5033lKLNaaOp+SgJKAvZIwFYgMfnz5s3r8KuA3qVKlRK0RhgXDma77FF7RKbuqiSgJGCXBGwFEuBTtGhRApBYQ4J2hLwi+IZYHUR0yi6vvV2CU\/dVElASsF4CtgEJplqXLl1o5cqVQgsCkDjRau3atQJArEEdPXrU6zVr1otE3VFJQEnAXxKwDUjQhJCXAW2IfUgcCWGNyB2QQkND\/SUT9VwlASUBIoqNjfW5HGwBEqIqSGbEUgTZqa1XQwKM4FeqWrWqzwWiHqgkoCRwXwK7d+8WaQ6+BJMtQHIWdpZD5p58SAARwqK+FoaZgVilShXq0aOHarMZIeq4VslZh5AsOIXljLy8NA8kWR7asL+eKBsDydfCMNOP0OqQnzRz5kwzt\/HptarNvhF3WpSzv+agLRqSOyDhN095SP4Shm+G58P9lIyhGajAI4\/S6TPXH25BBOjbc\/\/kLfCEsFJ8rRTYDiRv5G4FkMpNK0EJu5LozIzz3jTB0mu+HFdNTMAJU\/+09L7e3CxH1ayEQXfxl8veXG76mkI9ClDh8AK0rdY+0\/ey4gb1auSlIRHlqfcHOwMCkiWGF6FruxL91j+YNzdjb1GuZfkUkHiAWQGkWtsqUkzk+YAAUtTsBnT6bBL1HrrLijlk6h4Y8IDS723+MnUfby\/G8\/O1yB0wQHqlzeOED0aVpisDAkiVlz5BF5dd9tu4xfNvxqZQ5shcCkjBCqQ9q5rRph0XFZCIyBdAMpIyAg3py3HVqVXX9RQdAGakr4DkylGtgOTkUxtsGlIgAQkqOUy2YNWQVMqIPt3VVUgflkXC7kSlIcliVEDSN6i8OcvfQMLzc1TNZpvJlhZTRrzpRzPXuAvpKyDZoCFBA4DqGSg+pPiD7Wj+0lMBYbI9LEDydXTIDCB8fa27D74CkgKST8ejApJPxR2QD1NAMtgtZk02pSG5FrgCksHB6MfTea0nL0a3qikKSAYlGYxA2rQ9TkRy\/H0oIPm7B\/Q\/XwFJv6xsPVMByT7xKiDZJ1ur7swrGbj8LEr4YLE6DixYx+J17G4jl8E18mxX84stCxVl00jTLJCQ+FduWsmAcmorDel+J\/sqysZO7UdyBnYZm7tXU5f4AGy4iirXG8fWXdg0c9y4cbRq1SqxW\/KSJUscJaGNwAjnKiAZlJhVQMLyiOOjog0+3drTixbKQiIPKUBMNkQfcfgrDwnPx5fYrqUj2rGT5elwyvL0W9Z2qoV3u77hP3R9Q6TjjtCOWBPSmmxcYRU5RKi46u2hgGRQcgpIBgVm4PSHDUjQkNLnDDMgId+eeufqWZK1JHdAMrLPn7u3UEAy2McKSAYFZuD0hw1IBkQTEKd6MtlgqqHMDaqxeluHXgHJYFcrIBkUmIHTFZAMCMtPp2qd2lu2bCFkV3PteWdbixlpqgKSEWm5cbrpvQ07tZUP6UGJKSDpHUXBe54rIPG8UVE2Td8rDcm+yWC3U9lTy7E0AYevnNqe2vMw\/q6AZLDXzQIJ9XZQ5kJpSA8K3m4geOpqu59vdux4an8w\/K6AZLAXzQ4qBSTXArcbCJ662u7nmx07ntofDL8rIBnsRbODKpCAhAJgUXMaBEwekt1A8NTVdj\/f7Njx1P5g+F0ByWAvmh1UCkiBqSHx0gTlQ9I3IXy9lu3JF\/NTtmEFVYE2bfcoIOkbsEbP8gUQ3LXJF883O3aMytTO830NJP6QB12UjQUZFnY\/Q\/bs2bM0aNAgOnHihPi73dsg8c4WgeDUDiSTzRdACDQgFc6e3k5mmL53zLU7D9zD1eJa3mAVeUhdunShUaNG0bZt2wy3wRW0gxZI2AwSB9bbMJw4qcsXG0UGEpB4V4tAWMv2MAKpf9WsNKBaNsOT1lcXTNqVSJN3Jzke5ypTGyfw9vSYQwUKFBAr\/705HjogaYUkQ2jEiBGkdyttb8uQKiA5H6YPI5CgIRXJFrhaUnTiHZK1JFdr2bZv306DBw+mCRMmUK9evWxZOhK0GpJ2OrCQR44cSRMnTiSugKfVnvg6s34ABSQFJG8\/Zt5oHFZe425xLX6D26NixYoCTN6Ya2irq\/nF8ybofEhyB8kqKP59zJgxFBUVJRYGegJSREQEodSC0UMBybnEeGkAfrUrU9pdX\/ni+WY\/ZkbHmtXnuzLZMF\/gO+rYsSNdu3bNa3NND5CyxmanK6OSg2+jSG1lO23UwBOQILwZM2bQzJkzDfU7b0aYvDcbHRsVQ0kxpw1db+XJgyPK0ZCIcgGRh+QLIOgB0qPpC1LC3Mq0f\/IEK0Ut7pXWgYR3cObUlj\/gZmtse9KQCmV4hjrf60KFbiaTrzXNdCEhIfcsHxVEBL8RXnzYsGGpVEsIW68PCdoUNCRXu2y6ajcDKSzPZMqcoTLNK5rHjlfUdU8G0uEDmalHzxP0x6XNuq6z46RAAVLubF0oJHtX2jqwNx1fNN\/SVw0GILkSiNnomieXCFsWmDcF0j9JbS\/GUNfWLxqef2Y61BYgyWqn1s71RZSNgfR4\/vmEr\/FPdav4TUtiIO2d1p+uHCtDPddUpwvJ\/qliyU5Lf5ls\/HwG0v7J4y3XkoIVSJhTnTt3pvXr15uqFulOi9R+yNvFRVOfFi+kfSCxyimTUs5FsjsPies2lwxdI5rwvw6t6MLWTWbA7fW1X46rRgj9M5A+3NrWb1pSoAApf64hlD1zM6EdQUuy8ghWIPlCRtoP+bOXz9EHzZ5N+0AyKzyzg0oLJDtMA73vyEDaNnYM3bich6bs60trYr7Xe7ml57FK7i8Nqepbheix7nmIgXR+yyb6rWMrS9\/R7NixtDEBejNXMuJ5w5ZFzYR4Gt+0oQKS2UEFwYbUKEUQLA47TAO9Yy1qdgOqVzMvrRs0TVwCGAFK\/jh8DaSshYtSiZc70fFFC4TJLPso4NvDYbV\/z+zY8Ue\/uHqmHARCHtLw4cNpzpw5IimSdyZxFvr3tOSEZdRzwqd0t1gJh9ms\/ZCXu55AXzWuq4BkdlA9\/9NTlFgo1AEkO0wDHkT5MxcRf3TlF\/I1kJ75PkoA2JmJyiq5rzSk\/LXr0bMLoxw+PAUkY7iTwYJAUHh4OEVGRroFEu9Mki1bNpo3b57Tuts8v0Zs2EqZ6zcS\/YNDC6RCN6\/Tdw1qKiCZBRKqIuLrDA0pS\/w9OrB6geW+ChlI0xvvJFe+IWyBhK2QWEM6EL+Zhm9ra2xkGji78+l4lxqhv4DEPrw6I4rT3ebZCVGcuxdCRB9ZHXAwO3YMiNq2U13V1MbaUPhi9+zZQw0bNqSEhATif8NaURxYCbF8+XIBL8730zZUBlLeTm84tFSuJgrfKzTa0NBQ0T9Go9xmBGNLlM1Mg3Ct2UElAynf4Xt0Mku04ytgtm3a66EhAUiufEPxB9vRjUt5aNu4MQQYVchTl9otL2B1Mxz3A5BcaYT8BfS1hsQ+PDmKU\/RkJbpYJp3lAQft2MHHIJCP02eup2qeq8RIrYaEiBs0IJhycrIxbsZakhkgYd6gf9bUqqCAZBRIJV5+hWAecMSm1Y6qlPBoBfElLr\/sLh1s+YjlvgqthuQMSLxJJAMJ\/qPGhTvaGvr3BCSsZyteYapP8rPYZHMGpCd2VaRTdR6xPBdJO3Y47SJQoTR+6p80Yeqfjua5WjriymS7cOFCquVY3gCJtVT5Q\/74lruif6zWYD31Q1BoSE\/1H0xP9R\/igA6qEsJpKgPJLsGyhvT9kU\/o+yMTU8mbgYSkyNjZk4QW1afiFy7NO0+dped3AMlV9Ip3HGEg2SUTB6z\/9iExkKChAYhlKv1IFVfkp5N10tG2byZYmovkTEMqEha4WlL02eska0n+BBLmDfL24OqoPucO7eyS3nIN1tMYDjog8Yr2R+LLC02g4aQ7tG6AfYJlIDmLnnEtJAYS\/Ex9Kn1BgJddoX8ACfY\/OyrlAYABh4WTTzSNFMCGbydT9J5UE8LTgDHyO2tIHOXUAgkmwba91vr3jGrXRt7HF+fqNdk4ymalhiQDieeNr1Nmgh5IL3xwm1aMfdRy00BrsrkD0tezEqnEwW\/pi7zdKF+LEEqZm4F+ijK2Pk\/vZACQcGjD6QxqFK2r\/uZK8SV87cbXtPbzmalMBr3P0XMeA4l9WgASjqea\/Sw0pKQ8RNtpi6W5SGkdSJCPs7Vs7CuCI5ud2liWZQWQ8GG6emoHQYPG+s8KzX4WH\/IdXR6xXIP1NG6CCkjQCm7fOScEyxoSgAQNya5IG2tICPtjWYh8wH+BLO1fFuSm8rGf0YSn29K9Jom2bs\/EQNKaY7yOLSbyPNV8b57QkF5IiaIKJ76n1l3X26IlaYGEfrkZm+IAEiKg6545Y2nAIRiA5GnSmv1dG2WDFnRm448PAOlSzTsUuWSA5esN3bU\/6ICUPn88lZtWkm4dKCVMEwAJTm1oMFYvU4BgGUj4szZ6BiDVr5GP9m0oRIWS36fp\/V4TMMISjt\/b\/CUmp9UHA0m7XIaXjRwfFU1Npx0Vj4Xm8ssLq6n30F00f+kpq5siAg3IQ2INiU1G1pAAJPgprPRlKSB57kZ3QJLnTYEnL1OXzxurKJvRQcVObQAn7ljUA0BCtMCuSJsMJO3CWSRF4gCQsmV8k+Z3GUQFx9agcx\/sIIABcLL6YCBpbX8kJeZrmZvORJ5PBaRpFZfRph0XBZSsPhhI7GQHkPDOtXv8LpymWVafpvVziltqThsdO1a\/c1q4nxZI8PGdWjdVzJvEVUWp0htzxIc8b+Fr1G320wpIRgeVMyCxYNv3vC2iOVZ\/ibU+JPxdG\/pHUiQ0jxw3KjuAhMJoz\/23KR3d9JeAktUHA0mbi8T+m7hllx1AAijaHhspmmAnkOBkX\/lSrfum9PJrVLz7Tnp8+kmq8XthmjPsrNCgrKqNZHTsWC3\/tHA\/LZAg\/4Nzh6QCUtGe66lhpacUkNChRgcVAwmkP3c4UmyjfWN1TXry1fHUoMsJ+iN+M8Uve9XSL7EnIHHIv1WX9fRCxaZ0qUxVWl12pjDVWn3WkQ5c2mw5kJD53HrzHhFlA2xkExUwuLjsMqXEpggghdyNp+izSZRlQhvCAuA85RdbPpdYQ8KNf6wVKoAEH1bD4Qcp86QN1OJwHZrdcqtYdmOVOW107Fj+0mnghs6AdGz1h2LexH8TSjWHzCMAqf7jVSh8aR2lIRkdVAwkkJ4Fy0DK03Iu5ctchHZNLvbAJLVi7Mgmmxxp45A\/gFQhTz3K2ri+A0jlX65I2QdZX0aWgQQYFahTzxFp4wgba2QAUuk7h+hI+rJ0r29VAaQqTVda7th2B6Rjw8bTkLiBND7vp1RyzD85ZGb7xOjYMfs8O683urhWu\/3YoUOHnJa61QIJ42XfN11SASlTl7nUOvQlEahRS0eqVjVUz1cGUvSfo4Wv5N6f9YWGdOeZf1Pjwp3ElxgZ3VavLpejbBicHGljIEHz+Fez1nSnbWnadHcx\/dnruHBq42t09pUkij5+zLIxzUACmPGu7CxmIP3Z65hITJSBhHNO\/1rdFse2DKQ1vetT0Q8ec2hIDKSviyyk7B90tiwBL1iBpGdxrbw9Ei8fQcVVbEcmH1ogQaPeOauZmDeJqx4XGlJCh4+pW8ggithTWQHJ6KBiIIH0f60KTwWk2Lpvi+zoQVnfodqffmlpRAedLCdGYlkIR9o45A\/NA0C69GYO2vXHWoeZJrLJpxahtd\/8YjmQEGFDdIsd2wxA+K\/wZwAJIf8VGVoJEPx3dHaRi2R1pE0G0vbxnSnPG7Hi\/fF8tG1i0mfC75Zj4XuWaa\/ascPVGCwTssU3clYlwtvFtbwRKzdRLhftDkj4bd2o8k6B5OsKp0EV9odgt86oTBlDH6MMV5oIDYmBBMHW+S3KsoGv9SHxshDuQI6wteq63imQ4E9pcqg7TRryrmVDnDUkQKbOpC+FoxjaEkfY4L8KbVWOGn+5kaqtG0K7Go4XYPjxrcu2RNpkIP0xdwhlarKdoKW1XHpaPHdw3ECRtZ7c\/2nLtFctkDqWHkQdS1snY8s66+8baZcc6c3Udre4FreGhsT1kxYuXOhWQ8KPB1a+SLfunKNbB0oLDenYixE0pNDntq67dCbLoAfS3sov0ujaS4RgS4wZYtnA1wIJy0LKTytJcQNTRM4TgLRxx0WheXSp2Y1iR0fTH4v20ZkZ58WliHrVe6QdHRsdbdkyEgbS4f6d6NG6rcVzMPFhHgLSMBdlIK0vGSGA9a98v1HRsKwEeFp5yEA6uvpDSldu4wNAQsBh1qWJwhlvxTIFZxoSfIiBelxMjk5VS8uKtWx6C7ShHhLKjzgDUnTvbtT\/9n98XuE06IB0eO9LdGnHMcqe+Xmq0exjmlElLy1uft4hWITFrRj4WiDNfq4vna9zf9V2bLujhJA\/HNqbdsQJIP01ZafwnzCQAIlS9Z6g+sPftqw+EgMp3\/w+VKL9K7Ttsbp06qWydKlbDtEuGUg450CxTiIiV3zDJ7ZE2mQgXUteSYAS8qBYQ2pxqI5oF7RLFJbDYbakrVFzP9BAZRZI7jQjflfZh4QCbRg3F66MpzObfnBoSApI0sgwOqjgQ4ITF4I9G9+fTi9ZQyHZulCh+i8Jn5EMJPiRZIev2QEJH8X4tr+ILOx6a1+nTY2+pVsz42nVO7UeAJKcDMl+nSEjV1m2+p+B1Dd5Iv0em5nWl4igJ+Z2psXlTomQP2DIGhKAtPpcYcpfuz7ln9\/HlkibnBiZpfI1QsABMpCBBO0FBeu0pUq87RejY8fb59h1nV6TzdniWnm7bXe72roC0q2VC+nMqbLCZAOQBmX7iubtmPxAFQu73h33DRoNiYEE0hfY9ANVDO1Om\/M3F0BCAbU1MQscgoV5kBgdbfprDAECSEPfnOnIwgZo9lf+lZZnLyVC6XcTblKXcV\/RriaJtPn14ZSwO0n0J68ta\/fxOLp59pYlWpIMpIGvRdLjPx6ipsmRNP3KPGEq5Thyg7JVfZqqfvsj3ZjYXUALgEbkEYXkeAkJ3smKrZpkIOWolpVunexLbX65TKv77xdaao3fCwn\/DgcCrPhYpHUgYWx4u7i2WrVqVLZs2VS8cLZtkhZIuOBmztWUZ\/1Uapi5Ga1tMpriX+lG7zT4lJb\/stynNeD9AiSrt0FiDQmCvZvnIDVa\/SHdDu1Op0u8SFMrVqRRtZYQbPW4Ep8Sol83i1ahLzIPsqT4Pybvax\/1deQYffb2rwJOBR55lI4OOkofZU9P+4bMo8OFi1L29xvSwXUJtOXvNWwcaet+bxYtqrOVDqxZYGohowykDk0+pyqLNlHZPFto3ZXp1HLzPSr7Www9VrETLeo5hl6MiqCfD2ckGjRJQPuz0YVo9U9nad\/6QtSkcCeRu\/Xlvr6mwCSvZcMHI9PuetR5b3aa9eZKyvp+fzq67qaIgH568Un6clx14Vi\/+MoU2rg9zuuPRTAAyU4NBPfWAgkfZ2iwmDfYngpAytWrOb0+7LPg2bnWlVD1bhTZf2o\/GrFimPA5eFqEykBC2B+5FBDsyWxd6W6lNvTauCZ0\/PJoCi1TgSr1mkwb\/hdDTz9bmP7v4jN0oHgn0\/kvAFLD2Y3owKVNwkcDbazi0zH0S5tP6H+3rtFnk8\/Sx31Xifyfbl83oydjUyjm2h1aeDiZ1gwvIjSmWjWiKGvhInQ9TzqXxdX0DFIGUtujI+nlJl8I7SdH1Wx0Ncskmp4QRjcX7aeNj4bT7vf6UKVpb1KDE5tpyoRDVCpqKN2pe4B2RieIZS5Yb8eOYDP1v+V6SCigByCVWJmRDg5dTf\/6pgvd2H2Hkm7MptCuAyhj+jPiFS\/nKiQ+Ft5qsApInkeKFkiYN6Gty1H9VeF04EoTuvnqeBo\/ujzdzpaf6i6IC+48JD1baRer+riIQnGGKPwObOo4EzcDCREjDPwCC6rRBnpN+JBK9K4vvvi3bo+jBsO7UfLMnfRo+fyU4fnSYuAjW9nMRpIMpM2rfhU+GnzxX2lTlErVn0Iv5rzoyLtBu7F8Il\/iHXq5dCYaUC0bfdkgB23O34IqhnajGnPu0s07YaJUirfbNjGQENIf+PoM4SsDlBION6DZ2YrRgtXn6I8D4XT145cJkbidP64S0a1MF5ZTcqEfqfP+NjT5yR9EEbdHPigoopPOKmF6HvL3z5D9QmhHtr0vCRnlnLKRDnVvSw3id9HTBY9T2Y7fUK6TkRR9JolKv1OL9hx6hGZV+z+v4KyA5Ll3tEDieVNhRmX69koTqjJoEiW+1ow+euctahExPHiBpA1H8t+PHj2aKpsUfpjqwyuLdV+Zwx8VyXzwgbiCEgNpy4DeIiEw1+SnaHtoN4dTGwmLAMXx8q\/Tj3PXUu3Qx+j9j2vTIzky0We3etO5bDVEISrU30bI3oj\/BEAqvja3Y\/U+P6vuSy1oYMolOintDwcgydpelU55qdiY74XW1PXrobTv3EpRlQD\/cdEs+Jpwzc3YWx41RQYS\/EOLv1gqgIB8pALx\/anH7xeo5EtP0LJFY+m\/b+anuHea0q+LdwtgQasse\/Rzar7yU9oX+yfNf\/dLynk4md5a0Ikeudebxp\/pRwmZfqeI1llp\/pL7ZUoQPfR0yNsgAXw5Vn9IC6f+l5r9uN3xEYBGWbD6FmrzSR+qE5pBwPzVPhXp4PbMNK3JFyIKeG3sPAGnPy5t9vRIw+sgPd4wCE\/QAgn5ahgHeb9oSqtSnhJA+qtNU0OrJawSk099SNrdEFwBCSHxMjVKUMjntykq07NUotdByl+kuCi6dudCiNN3z1GhFm2Z8jM1GdqWbvznY4p9qi4VrFCXVo9bIrYhGnngeZqZ8wBtDrnvVC4aloWmlNhBcd92o2O0dJazAAAfp0lEQVStz9GhmoWEyYTdFk5d+ZkSct3PF8JR6LHCdCYp5wPPzXGloPi3xAqL6OAPeQUww+6ep49pJF0u9RP1O36Gao9LFju14vjv6Ol0+3oex33yVk6mWh3foH1L91Lh6JP0VHI8PXemHC2LqC12fICDHuFyVHdEQTVsHYT3x9+dHShOB63w3q9LaPma23Tnaiw1HteOih6Lojkzb1NkmwKUcKD9\/YJ1n+2iU+ePU5FaZahim0p0+ZsvKHzz27Q081+0ssQxKt1xAZW805Ruh3ZL9SgsysUBUBw\/ffy+JpTl773prt+vXiC3D2vq0Ad1+rxIYQc200\/bDlODd3rQtm93UdzB4\/Rx0VJiV5ZRj\/5T+WBWk71UNDqMTv3aUrQVskDtpLhjqfsFz7md\/I88RV\/dTKa3Cxbxue\/Dqgkp38foWjZcK29j78yhjXMYSFNvPUpJeYrSxjH\/oabj3xbz5nixmlT+uSb0y\/uLxHl\/TWytNCTQunGTOvT2V83pj5Aq9H\/VPyQqX4ruXj1LV07ueKDvsxYpIiZv1W3VKfGt98UEz17gKuUvUox+G7eU6pS5Rf3Pv0bzzu2lxSmr70+izEVp0qNvUbpqG2jr89\/SxkzJVCprS+GMTrpZP9Uz\/vrzpMvxlq1IEZFugE7N\/kheGptrKWWI\/oSm3o2lPY\/fpEZv1xJVLDF5Vg35ipKv3f4HSOVKUN0+LWnHzNkUuytBgHNB979owd6CtLV4TiqRp4eAJA7cI9+1O4S24M\/wScnaFu6fMewxSsm5hmKn3aOYhJLUsfhRutL9TXHvM9\/\/QeHpmlJIWGYxyWOGrqDdV45QoRduU7UXB9DlaX9Q+N6y9G3uBbSvfDmq9Xo1AR1EX26k7HWAJk9iJfHnsOv5aF\/8DdFmPrhYPdqHA21MybWasszOQ8W7v0nbdl2lc\/s3CyCte3ckXU3KTeOLlqTQk8\/Ta9eGOfoF7aySNwelvPx\/NDrnfsqU\/VHRN0VTnqJL6f4BUEziHeFrko+y+fPS65nSBR2Q9KxlQ8pAo0aNCHu01ahRw7GxpKtM7U+TDtDJ5LlUc2dLMU42T1lGucpmEUC688v9In7rR7UJXiAxwSFcLPhztX8U+z7uvR9OIUM+EoLpEDWUMhzYKpzBi47cEI5hPrA0AuZP7\/UJ9E2P30VIOaRGSYfJhshamzz\/pvlLT4sQJsysUbWXiAnzeYcO4jbI03k1Ywht2n6RTv112eHnwW\/shN4ae8sRIeNnY0LW33K\/fEbrY1E0+PUStGfn97Ri\/\/9ETeKi7RoLWGIHFG1lRJia5fp2oZ1fN6PbM+NFMuX\/biXQpBsXhAl4fW924eyuE5aBvsxzUDzSXVt4ES2c0iViFtBrDXPSRz0\/oeu\/Z6e7A+eKEHuxd\/fRNxVai6UBKLAPXx0qa2KftOe\/vkCnw38Q+UvwKyBhErIb+OU+OrI\/M9VN35Yypj9LDYvE0XPF9jraEpN4lxYdTn4A2pzaEL4gjn6J+JZif\/qTDi+Y7DDZsMHAinEj6dD3bziWKGCpBxZDL6k5lq60g5meSMX23qWEXYnCTCycPb3TvuExEQw+JCvWsnXo0IG6dOlCo0aNIm1OEsuoz\/JeVLDhrVTzBmMIJlvmkYOpR48ePge7T002jFi9UbYnlq4SAxxf6f91aE25r55xORDvdcwrgNT4SDJta\/Yz\/fnFHAEbTozEpEK29IbZjWnK3r7CyV21QiOa32Wg8BchOuYqkudpAgBIrTb9TvlXzKWm66ZSyW6V6OjGPhS18x6dnt5AgCp0SLzYWkYLJGQnIz8HyZyIgu2MuUozyie5rLntqi0MSgZS21n16NW7i2lS6cy0t2pPKlgmnFCoDv6xBp8dpSWlRggg\/X5hsahRxDVweC8uONWvLvySfprdwOUaN25LhzKZBSQYlDKcGEgjf7lMU1\/9WEBfCyQ848TkOcJ5jr6Ar29jw29Egqmc2e5MRZXlwR+otVSGGvX71DGRIJNAPrTjTm9ipLu1bAw0TyYbAwnzZleLHWKsMpAeCh8SDww9eUj\/nv4fGjR\/IR1ftEBAST60E3Nh1ay0pnRmyg8VvtKPDwCJF7qenTPJcZvfen5ORzLucAsj7UB2BoQfzt+jojNm08ltsUQT+1PjvtUod\/qu9ENoK8eOudV\/biT2IpOjeeyARunQ22V+oIjLuemjzcdFwiSc+Z4OZ22ZGptCRyYUo67fDaDkfX0pslNeKv9kBGUs1lUUqhu+tS298tHTItcH5SXWp3wl1rlBG3u3xnq6WyqeSrQ9Q9NfeZfq1cwnNijQswEAt6UItJcymR1w+iM0A10dXoQi1iXQotZjhaa2Z2J\/h4aEmkxRcxpQ++ceEyDCIZJKW3\/iEUau+iaiRW262HyMA0jQnguH27dTsKd+8vS7Frpml47w81z5Z\/E7a0gTNvSmW1VviKDKifpLxby5fef8w+PU9tQ5\/LsRtRuToXb\/MDr6dzQKBeRPL14j\/CysIfFC19mzEunJPHXFmjPs\/oEcJ3fpBO7ay5NwYOM89OPbU2n1+cJCA9p66H2RTgDn+PxhrenC1k1CCylf9X4FR5hCODgChSUchZ86SXNTLol\/95Ti4E5TgIn2dqe89MqcifTOV88Jc6xGwbcpQ7GutL9rb2Gi9Rvf2QGkPRW+ERE21GVCtAsLhP896VEqUiiL8A1pd1XV038sF5iZyIEa0SI3ZVqXQDmcAKlrtUsCeshofzKkLmUMy0CPzbxkalcW7djBFx8LiwP10EZPrQIS3le+l\/z+LKP1R\/vT\/OxXKP+1O5TrbyDhvFpvDKbd3Z+jH2e1pGmRi2zbJstZn\/jcZNMzMIwACffj8hr4MzaHhK9CBpK8LELen8yKmtaYuNPXjKLvMr0pFrJ2WPA17U+oI\/a1GvhDdWGGQAtBYXsZSPAfNezXg55b24NOn02ioT9gFxLPoX138uN3A5A+XdmLwuZnpWem9aOTo9o4nt2ubxvKNGgmoSLg2qc\/EZOVc45QESFTSDx1avO4eIy8xbOeftOeU75hDso+oZgATJmO71DGq01oQ7\/2Dg0JQMKuLFxlgDeS1KMhumqP0bHjzXvZeY1ek83ZWja0C8tHunbt6tI\/K2tIMNluV7sh3BXwI2LewBqBDze6d3da\/k0lmjRtq+lxYEReQQMkVsvhPD469b8i8oX\/jgzolGrlPeCg1yzSI0iund0nW6RwAp94+l3iaojQOJA70+iNFhQ25ptUQEIksfpLz9HI6+85FuHqeZ5eIMFfhN1NKk5qRxf6tXYU0q\/+UlMqM3mBANKWDp+J2+EcOJK1+8qZbQ\/7kDDgC5YOp5DsXYXZilwx\/H9Gz\/TiEQASn+su30xPe9I6kFizwZq0S5fua80rV64kvRtFGgn7M5DwIcO80QJp1XfFbKkk6q4fgwpIGPj5c74ncnVA+pCaJenqgNeFnwKlQHbGJAjzycotiBhII7J8LOSMwvlcYYCznBE5yr7wvVS7gMKhXb9mXqq27j3L6hCxhgTAZK+alQ5U\/pXOvJIkSsTigIbGpiIDKWFXErXb\/oFYKmJmmYizQSYDKW+JViIfi4EEGc0fU8xRM8oK7Uj++rdr186n4Wo9sAyUc+QoW\/qWd0Wzcl4fIObNha0bhYYU27c9rfimkmUfS73vHlRAwktnOtZb5PxogYTa1nLlRL0C8nQeA+mVYSepUb\/udH7rJvp8yDzHgl6kGTCQ5N1zUZcJZWThQNaT9eypHfidgRQ34BZV7Vvdsb4O2hi0RdQaYiCVHrWVVoa\/LzLgBz32Hf1xaZPlZSYYSGgbKngWKfdhKiChljf8VJ8vPiY+FGa1IwUkPaPkH6c2NCRXQIrr19TxIbdqfOppXVABCRrSnXWtBOGx1AAaUoUFA4TjtPYbq4WTF05sK3xHLFwGEkp3YPcOLsqGqBFrHQBSwWnvCacytBTtIlg9HaXnHLmYf6HwAgTtB2vHuOY4yowwkCpE7qafWg4kwOuTnGttqQwoA4n3ycP7A5B7Xq5HG78uL+R1slJ64Vw34zti+QSDyaanr82c40xDwrxBEIg3iGAgVXhmi081zYcGSA2HbBAb4VnxFZYHAwMJX\/ohEeUcQOI1baj1AyChfC5v581Aqvvdc5Zu0Chvd8ThfDiU5TVlSLSED4eBdKLRZUeETc9aMSMTQQYS5zo5AxIqWsKhb8WHQgHJcw85A9L1Bc3Eh5yBVHZuZxrQq7aIgKptkAxug8S5Jsjqvbm6lvgCQ7DQkJpvGCbqRffaeMiyr7AzIG3aHifC5ehAHAhjcy1vwAlA2k5bhNnEEbY9L9e1zFyTTTZMbABJhi+X7k2MPi2AlOPXr2lr3hl0q1uIaCcXSfM8nPWfIQPp9NhbYnMBJF2iIgMW98KHVOuN1ZRtWEGv0h2ctUQByXP\/aIEEyyJhbhUxb9A\/AJMCkiRHo4PKHZB6Hvu3CKtPzX3Zsq+wHiDhHC6di6Uq6ORTPYuJ6oxwaEOz+rr+s55Hj4EzWENCsh2ijtj2iA+stgekYcoCSFlW\/Jt2FlhGZd9taUuEDc+VgXSkd5II9zOQskxoTePfzk8VJ92vYWWFuRZsPiRvFtdyf8srIrTbI+kBUtO1Pah1y\/pU8\/mFSkPyFkgwT+jP+oL0mHhYbDr2xue0eFss\/dr0nuXmGjqfTTb8GVqSvHMHl87Fb0g229klvYjAiVIcG2fR9M7Wbs\/DQIKmiD\/LkxwQhKMfiZqQT9yfncQmla\/M\/lSMYasjbFogAY7Q0tgkQK3vF6uno85HD4vnY\/mOFYfRsWPFM+26hwwkPYtruR1YxxYREUFxcXFioa0nIGktC1Ro6H64Hz1VuRHVfSFSAcnooGINCZrBrQOlhAbAQPo2TyRN23Kafq6QYjuQtJnNvBziQvJpatJisFhlzz4Urllk5WBmIOGeGGTyJGfHNj8fQIKDf\/AGc0XY3LVf1pBcAenNpJOODQiskIV27MBfF8iHdlkU2mpmcS0gNm7cOLp48SIVKlRIN5Di54SKeYMPBgPpiYqtqGGLTxWQvAUSfCfp4ss7gFS6ei4ae\/MzenvjITqU\/45lZoEzkw3\/xkXy+XeOtCGkjsTD9XOKCy2lSFhWstp\/xM9EnW4c0BZlJzFXUGAgHYttTLkWl6R\/7Z9u2a4n2okv5yFBW4OGhA8FBvzbiYOELIbnvGjph0I7dhjEgQolbXVQvZnarhbXwlTDERMT4yhF4klDwlg5Pyud0NwZSLNyjab4G3UVkLzxA7CGxCU7IFgM9tAC1wSQhlw\/I1atW2UWuAISh\/z5d460IbpWrVRZ+rbjTSpX6wnKGL2HPn26oy1zhIGkXbTJQGIfDoDU\/Kd3qcCWcpZnaPOLaYHEu70wkFDgDb49wMpT3XS9wnKmISGyGKhHUkx0qsXjZtaywaxr27YtDR06lFq2bKkbSBgrV1ZmFEDCBwNjFCsIopM60Qut+ikNyVsNiaNK+BLLQOp85Agd+PmCY5NGKwen7ENChI2LlOEZDKQD8ZuFryliezi9PqEP1bq9md58Y4mVzXDci4HkLL1B1lAAJKx5+ylqpmU757rSkNh8hB+LC9pBQ9p98wTNOnXOUs3V6NixpRNM3NQMkAoXLkwNGjRI9fSzZ88+YLZpndr88eJ5Aw0eQBp\/\/QZ9PWKL0LZ9dQRVHpIMJAgwx60\/hIbU\/NpRS5eLuNKQkA0uH4iuwbHNQOq6+PlUW2zb0clGgGR1TpZRIK24cVgEHKzUXNM6kPSabM4W106ZMsXRBXL1SE8mGy+l0gIJ\/r1d\/3fGlg+5q7EfVEBi1R+CxZHlxm80IOU7EcnxpqyHHmCwhgTNiHOQ5OsQ+seBIvmIZKEqpDdlPfS0BecwkJyZQVDJccDRCw3JSlPJWfvYZGN\/FqJ7MNfwfGhIs68ftNyUTutAghy93SjSLJB4fCA48lHS+wQgWbnuU88YDkogQbAY9ADSqze\/pveun7Ft8jGQtCF\/Fr4zIGl9TXo6Su85DCQ5B4mvhckEIOAAkJydo\/c5es5zBiT4snAASBOS9lr+BQ4GIOmRrZlztCYba8o8b1C6h4FktxatfY+gAhJPMBlIFZO+stxPIQvRE5BgssF0g4YUGf2qWLCoNe3MDC7ttQASHMTOEg15q2pfA4l9FPLzX0roRxOS9lkaYcN7KSB5Hk2egJSc8jv1TZ5o64fcVSuDBkhytq8\/gDR\/6Smn69KwjXeFPHUFkA6GvuOokOh52Hh3hl4gHdxdz1JnsjuTjYEkh+ABpNFHd1puSvNkQ2KgL9dgeddb\/rkqNDRU7LnGq\/1ZC2INGkBqfG00rUhJJ8aIL+UYlEBiwcJko4RPLfdTONOQXPmFkIuEaBuAFNJqrLhUzua2ekgCSNqkSH6GDARfAon9EPLza8e\/JMr2Wu3HwmQbNmyY0JTU4VoCu3fvpnF7RoklRtwHCkgu5GVU7cbOtii3wWaKFkgrpp+0LVIgr\/Z3VvKVgZSr5GFKrPaBWFeHBEq7DndA4lwkPNsfQJKfn+VCc7HLilVr2GR5Akr4D8eXY6uJ\/\/f+4EGZozxMn4qfi9\/jSkyiZ55PR627brCra0TRvMI9ClDMjPN0bff9DUvl437dqiKUnLJX7M4SZ2O4HVrPIy3upgISm9TQkPJcGUrJt4vQ6rablIZkFEjOOhbO23x5j9D+y2MM72BhZETK9ZBgtmkPGUiVek22NcKGZwNI2ixtbpMMhD2\/1LA03O5MZtqytL4CktwWbPAgV2GQf+O0DPxbaNcBlCl3vNNIqZHx4O5cT2V6GQjYrXj3JwNs+4hyGzmhmH2vWiAVuv0M\/aft12kfSLwoMCwsTLy7NjlLzzZIsHG9LUMqdyw2abQzUsBAchU5Qy0kbNAIDQlAsjPCxkBytZ8ZAwFRlP0rX\/QrkBDlc2VaWgUA3Ie3wHJlJnMUtNbQYbTr6CFbzWkjQNo6oLftCYnugIS9Ajvf60oft\/0k7QOJ19Ngd1rt\/lB6N4oMFiBxtjYDSZvNbeXk8wQkLtQGIGG3XCsKohnRCPj5uAZA8rQRpBWyMQKkJet322pOewIS+9igIfkKSHIwiJ8Pkw1A6n13IA1u917aB5J2IMkQGjFiBHnaStsKkw3aADoWGpLd+TYokYu1cvKyEZaBDKTcrcbaahLgmSjT62q\/Oa5U6WsgsdNUCyRfJN0ZAdLn8zfauuUPA8mVIz\/QgDT87jjq0S48+IDE63NGjhxJEydOpLVr1xKySl3trmkWSHLHRv852hbHqd6vt6+B5K5dvgaS2M48vIAAJHKj+PloY+L5prT2X4e83qhTr\/zxsTh9JsllZU7OE4PJ1qH\/EksreOpto9bHhw\/pulfesl02CAblbZnbYbqzSQ8N6Ub8GAKQvHWbGH13Pt\/2sL+8NgcPHTNmDEVFRekCEnJJEJ40eshAOrr6Q9t9Je7aJwMJUTY7Q\/6e5CQDadPYJrY7TbXtkYHki0xxT\/LA7zKQ6nSc61TL1XMfK85hIPgKSNo2y0DKEjeNet8bmPaApHVgr1+\/nuA7wgEYtW\/fXiRhLVy40KER6dWQcI8ZM2bQzJkzDfW33LG+iFa4axzX1oYPaeXdt2w1CfQICev8YLL5G0i+SDvQIw8ZSIXqR+q5xLZzAglIheMWCqd20GhI8BvB9EKS2rZt2xydCPNNrw8J2hQ0JKOZotyxl67Npl2Dx9gerVBA0j9HeeGzL9IO9LSKM+kR9q\/TYa6eS2w7Rx63a55937IaUXobLGtI+fbPpTcLhgcHkGQzTYYRBOOLKJvcsZtfH267La4HSAWrb6Ep+\/qRs1wlvQPGivP8qSGh\/QykrTMq2x7l0yMvBaR\/pCQDKWnOOBrZfExwAEneX5xfV85FsjsPKZCAxMl3ANK7K3r41WnKQIDJtqZ3fb9ojoEKpNtN\/0Ud+i\/VwzDbzuEoJDR7f2hI\/Hw4tS99\/BF93H1CcADJbI+ZjbL5W\/WV3z\/QgISFx4h++RtI60aV97lT3dm45Ex6BSRybCiqgKQZKWaB5O8vjSsgdZrZ2a9RHLTL30DiSgyBBqTj5V+nQRN\/NfstNXW9PG5XPjnQ1L28uVhpSC6kZiWQ\/NGxaQFIK1+q6RffGgNpVa9SfjEZtUOONaSz9ZvbmqWtBxCcFgGTzR\/jtnjpelTntyiChnSu31CaNOwzZbKh44IJSHgfrJdKCfuNBq\/oERAa0q0752hjv\/YKSEhN+bs8zNbijfyeksFAOnc4klY\/+54ehll6Dj8feVBxA75QQGLpWgUklHDYOXiMpZ3mzc0AJGyFNGVfX28ut\/Qa3orot46tLL2v3puxhoQtxQPhYCC1W17A781hIGj3avNVw\/j5Zzb+SNcmThf5g0GTh2RGiGaB5O+O1b57IAEJtaJw+BNIeD62FA+Eg6sxKCDd3\/yBN4tMmTtLAckqDSkQgfT9kU\/o+yMT\/T4H\/Q0k3ptNAcn5UEBahL80JLQIz8futQpIUv8Eo4akgHS\/gxWQ3H+TFJBCQu75\/bOtaUCwAQnrpdbELFAakgKSx6mmgBSEQGLV05+qrzzysDzhj0ubAgJI9+s2F\/WbDynQNCSuxhAIPqRAGLfKZHPyzTCrIQVCx3r8FPrpBMAoKea0n54eeCYbMulxXEiO9ptMAunBrKE9tu5\/yqltlVMb9+Fowf7JEwKpvx\/6tkBDK16mHs1\/MTCibA99h2gEwPNGAclCp7YaZIErAd5KG5EcdQSeBFDc8PyWTVQ4JVlpSFZqSIHX1apFSgJpRwJWuE28eVvbS9h60yh\/CcObtqprlASCUQL+moMKSME4mtQ7KQmYlIACkvIhmRxC6nIlAeskoICkgGTdaFJ3UhIwKQEFJAUkk0NIXa4kYJ0EFJAUkKwbTepOSgImJaCApIBkcgipy5UErJOAApICknWjSd1JScCkBIIWSB06dKDw8HCKjIwUu9fisHsbJJN9oS5XEnjoJRCUQOJttvPmzevYTtsXG0U+9KNJCUBJwKQEghJIgE\/RokUJQGINychW2r6u52umD0NDQ6l58+a0fPlyw1t\/m3mumWtVm81IT\/+1aVHOQQckmGpdunShlStXUqtWrQSQduzYQRMnTqS1a9fSlClTiDWoo0eP0pAhQxw9zMKIiIhIU5MbRdFVm\/VPVG\/OxORWcvZGcsauYTn7WimwbekINKFdu3bR+fPnHT6kU6dO0ZgxYygqKsotkCCMYcOGie2Q1KEkoCTgHwns3r1bfGB9eZgGEms5YWFhot3r16+nmJgYqlatGnXt2pVkp7ZeDQn3AZTwnzqUBJQE\/COB2NhYn1sopoHkTFQcRZN\/S0lJEao2\/CzQmmCi1apVK5XG5B+xq6cqCSgJBIoEbAGS\/HLasL+eKFugCEe1Q0lAScC3EvA5kPB6nvKQfCsC9TQlASWBQJGA7UAKlBdV7VASUBIIfAkoIAV+H6kWKgk8NBIIOCDB54RQY4YMGejQoUMiUpdWDjnpM5DbrI2Mnj17lgYNGkQnTpwI5GaLtskBE0R05fy1QG+8s2VUgdpmbWDKV7IOKCDJUbdly5aJJEpt0mSgd6CvOs6MHBBYwIHJ7Co51cz97bq2T58+jnSStBahdbaMyi45mb0vt5UTmM3ez8j1AQUkzu4eNWoUbdu2jeSIXKB+vbnzrl27JuTOKQ1GOsHf56YFOTuTUVrRSNF2Z8uo\/N3vrp4P2A8fPpzmzJnjWBDvq7YGFJDwBWzUqJHDfMDfsewEWdsAVKAfaWmCyLJEu3GkJfM4LWlIzpZRceWLQBzTstsE7fOlSR9QQNJ+qRWQ7B+uaU3GrG00aNDApxPFTE84W0YVyECSx8SFCxeE6wQWgC8+WAEFJKUhmRn2xq+FvNu3b+8oDWP8Dv69Ii2YmrLfKy05teWe9eVHK6CAlBZ9SFrTJ634kDCZsXg5rZjDztDny4niLXrdLaMKZC1JCyTZleKtLPRcF1BASstRNgg7rfiQ0sJEdjZ4AdECBQo4TIe05vtKKxqSLGdfR2EDCkgYhCoPSc93xNw5zr7avnRcmmm93Pa00mZ+37QCJP64li1bVjTdl\/mAAQckM4NVXaskoCSQtiWggJS2+0+1XkkgqCSggBRU3aleRkkgbUtAASlt959qvZJAUElAASmoulO9jJJA2paAAlLa7j\/VeiWBoJKAAlJQdac1L+PLPCVeVoFtsVwdes6x5s3VXfwtAQUkf\/dAADxfu1DVV0CSl1W4EwPaN3jwYJowYUKaWGQdAF2aZpuggJRmu866hvtj5TwygMeNG0dLlizRVeJCruFk3ZurOwWaBBSQAq1HfNwebfXIefPmiRZw2ZdevXpRxowZRQVP7L2XmJgodiEODw+nbNmyib\/L6+EADqzEx+EukxpZy23btqWhQ4c6KlW6y8LWLrz2sZjU43wkAQUkHwk6kB\/jzmQDkAAt7KnHG33myJFDQAgHdiLGDqeoPqk19dytNdOuS9MCR7su0J9FwwK574KtbQpIwdajXryPJyDhllwLR4aMduGlFiLa6g1y07TnAkidO3cmaGjOHNz+MCu9EKW6xKQEFJBMCjAYLrcCSNOnTxeFvHhLdZYL71isLbXhrDICQ4mvleHkzzrPwdDHaeUdFJDSSk\/Z2E4rgASTzUj5FU\/naus1KQ3JxgEQQLdWQAqgzvBXU6wCktaH5K4InNaH5Kx8sVwUTPmQ\/DU6fPtcBSTfyjtgn8YRLmzjFBMTkyrKpteHhPPkKJsrcw3neYqyaa9VUbaAHTqWNkwByVJxqpvplYDRPCRPJp7e56rzAlsCCkiB3T9B3TqVqR3U3evVyykgeSU2dZFVEtCzTk3POVa1R93HvxL4f6nTeWnrdOFNAAAAAElFTkSuQmCC","height":176,"width":292}}
%---
%[output:50a7ac19]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASQAAACwCAYAAACmeRMNAAAAAXNSR0IArs4c6QAAG9RJREFUeF7tnQ9wVVV+x3+I\/DUBiUh8kT+y0mQ7RR1IEbGIzFbHaeaVrRQzytiFcUGhGcbioOlaBgqTcYpLZdsMA7tIB6YKTpzRNqYozoos2GKkiVKpO2C2KIZkw6oYEkBhI53fkXNz3n33vnPufe\/ed2\/e9844kvfOuffc7\/ndz\/v9fvf8GVRSUnKZcEABKAAFIqDAIAApAr2AJkABKCAUAJBgCFAACkRGAQApMl3hvyErVqyghQsXWie4ePEibd68mRoaGvyfNE81N2zYQHPmzKFdu3ZRfX19zltRXV1NNTU1dOLECVq8eHHOz48TZqcAgJSdfnmv7fQA79ixgyoqKnL6UG\/dupX27duXE8hxm9vb2x2BAyDl3aTy2gAAKa\/yZ3dx+Wv\/7rvvUm1tbXYny1CbATd58uSceF064Oi+z\/Ym4SFlq2Cw9QGkYPUN9OxeHl7pNXGD7CGdBM6RI0doxowZos3Hjh0TIY28hryRAwcOCPi5fc7lZs6cSXV1dVRUVCSqdXR00KpVqyiZTKaElvIaqkjyvIcPH6bbbruNhg4datXnMIsP+7Xt4V2m7wGkQE0y65MDSFlLmL8TmHou9nJ2kElYSdjYy9v\/ljkrCQL1+08\/\/VTA6OzZswJCDDg1Z6ODqPxewkpeyw5CeW2v3wNI+bNXkysDSCYqRbSMCZCkt3Lq1CkriWv\/zARYTiGb6olIr6u0tFR4QRIgdulMgSSBI9vKgNu0aROtXLlSnJJhxx4Tt2vjxo3iM933ToCMaNcWbLMApBh3ve7h5ltz8gjUh5wf0rVr16bkiJw8KBVI0ivp7e2l1atX0\/Lly636lZWVGd+S6dps\/14CZ9SoUfTCCy\/Qww8\/bHlfKpBMvue2Tpo0CW\/ZImzzAFKEO0fXNJOkdhAeUqaQDh6SrtfwfSYFAKSY24fJa3+vIZn9nPz3HXfcYb1lU4cVdHV1CY+DDx77ZM8hjRs3TuSUZMhozz+5hXT2RDhySDE3VMPmA0iGQkW5mMnASJO3bHIwpR1I0hPjN16cbN6yZUvKWzT+TB335PaWjUMs9TsJHfn2TH2DJs\/Jn9nfxuEtW5StMbu25Q1IbOSLFi2i9evXU3Nzc3Z3gdpQAAoMCAXyAiT5i8tvZjjRCCANCFvCTUCBrBUIHUgcXixYsEBMQ7j99tvhIWXdhTgBFBg4CoQOJCkdQraBY0S4EyiQKwUiC6REIuHpHh9MnKUJxYOtOj89fo2n+tkU\/nxIGQ27NZl2im+7O+jSp600buRl+ra7M5tLGNe9ib6gB8qHp5X\/hK6jV1pPUuKGBLX39hmfL9uCs4u\/olmJIeI0jcPvoa+\/\/Iw+bW\/P9rSe6vdUPppW\/vzBX5C0sbD6hhuh6qE26lDnJfrk8nXiozD7R31uOgaX0ou\/7rWa1dkZjs2qOkQSSGwonFuaPn26keGNev8lGvnxPuorGifKD+49TVf3nqbf\/VkdfZOYanQOv4VOnyf68VtX0y3XpS682XWB6PT5QdZpn7mzL62M32tmqndDw3cPn9RClh3226NWtbPTHiT+L+hjWOdRuv711fTNDVOtPuFrhnV9vtbPPriKPvxiEJWO6L\/bfPWNqoeqvbRX+Vn7I\/8WdNeI86vPjdoG+dy0traKt6lhgimSQGIQ8StoUzGY8ncVf0U1xycKoZ8sPyd+iR5vGU7sGQR5XDU6QZ\/M+nuafaaRan9UZbX5y+srqVf5ZR751joae6kjyKaIc28uP0kHe66llzpHpVyLPx9\/xYPc3VEsvp82bRotWbLEWGevjb8jMYR+UvYJPfDBRBo3gmhzxUlxCnl9r+fj8l7bzN4R\/2jc\/OtfWJfjz85cX2n9HXTfyDb\/cvsGWjzoENUcm5jiBc0uPkNPlp+32vPDlhv9SOO5jvrcjC8abPUPPzdjpt0jbGP+\/PkAkgSSqRgrp19D1eUjaNZLn4tOee7uUfRA+QiqbjpDhzoveu4oLxUYSCU1jcRGfe\/UUtq+fbuoziFccXKtdaruF5bRpZMtXk7tq+yhB8dSw\/ELtKn1XEp9\/lwC6bmWXvE9e6JVVVVWm31dMEOlWYmh1JAcQ7NfH0J93Z3EbRD9c+X6fq7ntc1FybU0eHSCul9cZl2OPxuuhNhB941s80d7\/lXowXba3tMfNrOtss3KY8K2Lj\/SeK6jPjdsG7J\/+LnhyIKdAtNn0PPFXSpE2kMyFSMKQLIbdRyAlCsjcjtPEEDy2uYoAEm2WeoBILn3Yt6AlMmw4ughAUjpPQogpWoCIOl\/TgAkvUYZS8iQDUACkHSmBCDpFIroriPwkPQd51bCSw7J61W8DsXgRCnnTB56Z7h41f9fV3JILzvkuLy2xbT8yLuW0pCJlSk5JP5MHabR+9r6UPJ7Ug\/O0aiv9hlU\/6jkkO68kgs1vUc\/5fjNGXJIhsoBSIZCORQLCkheh2L4vwPUDEMBfqV\/bs8m8Xaac1oFn9RGDikYswsKSF6HYgRzdzhrLhSQwxAO1v0VgGQiKDwkE5WcywQNJNM3n\/7vADWDVkA+XwCSodIAkqFQIYZsXvvE\/x2gZtAKAEgeFfZq\/BiH1C8wPCRnY1MXmVNL5GKHXF78jlfODGJvPF6MjpcF9rrLLi+Et2bNGtq5c2fa5p4AEoBEQY8GlhIDSO5AWrp0KW3bti0nu+\/Kq8hNCNra2gIBEsOOD69AYgC73S+ABCABSB5tINfF3R5QCZSenh5r5xS5LjhvdCl3VuH2sMfB2zHddNNN1mabcpcV++abXF5dWlguwyuvV1ZWZp2DN8XkrZy4DbwUMB\/sufHBW0rJv+X65byUMB9ynXH7OXfv3k333nsv8TWclgkGkDxaF0I2j4IpxcPykHhAaJQP+5IiTiEbQ2LdunUWDNgLsXs8HDJNmTJFhD4MGH5dzqGZ9FxkfbuHJNcOb2xspKamJnGN\/fv3EwNMejx8vnnz5lF9fb1YzpmBJHcL5mvKLapk+UceeYTefvttsR8d1507d64os2zZMtFGuTswf\/7mm2+KhRCdPEIAyaPlAkgeBcsDkHhw4ci70tcZ8t\/y3Nbk9Y7OH9xmnVTnIUmg2HMvciHBhoYGsSccA4YBInM7bkByWoBQwo7BxOeQ13rttdeERyPboMKG98xTQzYVrNL74TL2HBZCthzaE4DkX8wwPaTBo8v8NzTgmn3dHSmL4g0EILFXduONN4q1wnj5Z+khAUgBGxOA5F\/gsIDkv4X5qWkKJF3IZuohOYVs7AHxGzO3kE3nIalv8\/jfxcXFjiEbh4F79+6l++67DyFbLswNQPKvIoDkrJ0pkLi2unecmtTmBQPtQOKcj30jTtkCp\/CKv+N8klNS2wlInB+aM2dOWpKbE+E333yztUmGuu8eJ8Tfe+89sfAeJ+E5t6TufYccksfnC0DyKFgeckj+W4ia+VYAQPLYAwCSR8EAJP+CFWBNAMljpwNIHgUDkPwLVoA1ASSPnQ4geRQMQPIvWAHWBJA8djqA5FEwAMm\/YAVYE0Dy2OkAkkfBACStYE4jtZ2mVehOFPTcNfX6fifW6naFBpB0vWz7HkDyKBiApBXM\/trfPmpae4IrBcIEkt+JtXJKCg+gbG5uTrs1AMm0t6+UA5A8CpYHIMk93vy3NNia6r5nfCUdkNSJsFxeLktinyCrThV55ZVXxFifU6dOpc3Gt48L4qkiQUystY91knPueGKwnNBrVxpA8mh7AJJHwfIAJF6D6onKIv8NDbimfTNKp5BNzpbnpjB4GBp8yFBpy5YtAjjqBNn3339f7J7LEOJpHHJemno7qocyadIkMXF2\/fr1tHz5clGMB1PmYmLtpk2baOXKldYcODmSu729XUzahYeUIyMDkPwLGdZIbfaQJhQN9t\/QgGt+1tuXsjusm4ekztK3e0N79uyxYCJDH9XLcfNAnHI\/QUysZW\/oscceS1uEDSFbjo0LQPIvaFhA8t\/C\/NR0mjpin7HPLZPLefCcs0xAyuQhBQEkp4m1AFJItgQg+RcaQHLWLpOHtHXrVmu9Irl2Ea9N5BSy8Zs5nofGnlVLS0uaByXDPxkyccgmV26sqqpyDdn8TKx1Ctn4AtwuhGz+n6G0mgCSfzEBJHcg1dTUkFxtkUupr\/3Zq+FJrLzy45EjR0R+iL2lGTNmkKzH5d0gYF9iVp6Pr+O2suPmzZtJrhbpd2Itn5\/zXOrqlgxBbjNPqHVa+hZJbY\/PF4DkUTClOIDkX7tCqQkgeexpAMmjYACSf8EKsCaA5LHTASSPgoUIJA4FeF94HPFVgLdF53ARG0Ua9iGAZCiUQ7GgQjY2Yh7Xwn2DI\/4K8GYFbdufouryETTrpc+Jh3Gw7fBR3XSGvklMFdAKe6fiQSUlJZejJi+A5L9HggISt4ihxP+ZHnckhtAT04voiV+dFVWeu3uU+P\/LH1+gl49\/bXqarMrxZgR8qAv+82dDJn63+wcfPU3rUtbfzuqCGSqrerT39lkl5efyg+r\/OBNUE6zzspf7YOIsgGSiNIBkopJzmSCB5LVVsxJDqSE5hma\/PoT6ujutX2D7KGqv5\/VSvii5lgaPTlD3i8usavzZ8FuT1t9hbeIp9WCPRJ3a8kD5CAvW3KgJ27q83KLvsuqOz\/CQMsgIIPm2MfHQNxy\/QJtaz6WchD+X88\/CAgKAlNqPAJLerhGy6TXKWII3TCypaUzbmXbYrUkqTn63pxYfYf0KA0ip3QUPyd184SEZPvzwkAyFcigGIAFIptYDIBkq5QdIs8qGircDfHDylONy\/vtQ50XDq\/orBg\/JXTeEbAjZvD5VAyJkYwBxfgRAIuSQbE8AQjaEbF6hmFbeq4cEIPVLiJANIZvpA4iQzVApAMlQKOSQtELBQ4KHpDUSXQEASaeQ+\/fwkKLrIcnxRvZxRhiH1N9nyCH5f\/ZFTSS1kdQ2NSEASa8UgKTXKGMJAAlAMjUhAEmvFICk1whA8qkRXvunChc1IPHLoEnjJ9Bfvvi\/mFybycaRQ\/JJAMJrf7tyUUpqA0h6u4aHpNcIHpJPjeAhwUPyajoAklfFbOWRQ0IOydSE4CHplQKQ9BrBQ\/KpkXwAv7d3PJWe+aDglx8BkPSGBCDpNQKQfGoEICFk82o6AJJXxRCyGSsGIAFIxsZypSCA5FUxAMlYMQAJQDI2FgDJq1TO5ZHUdtcRQAKQvD5lOfWQeP\/xiooKsfsn71jQ0NCQ1h7e0ljdQbS3t1fsZtHc3GyVxTgkr93YXz5Kc9ncgPTy8QvWwv\/+79SsJsYhues0oAdG8tbBU6ZMEdsPJ5NJ173FV6xYQXPnzhXleKtfpwNAMnvYnEoBSKmqAEgFCiT2jrq6uqi2tpZmzpwp9htvbGyk+vr6FEUYXKWlpY57jsuCABKA5F8BAMlUuwHrIU2ePJk2btxI+\/fvFwCSf7e1tQlAqYcM6+Rnu3btSoMWgGRqUunl4CEBSKbWoy5sOKC2QbJ7RG5Asn\/O4duCBQvS8k0SSJxr4h02dQdWjEQOyc1GELJlDtnk0s8qkB5vGU6f0HXx3bnWi4ekyuMGLgkkLvv888\/T9u3bMzIJQAKQACTdz3b6924e0t4b5tMfVf1IVIjtVtqmOSQnIMlQz55D4jwUe0i87W+mA0ACkACk3AGJPaQx0+6hJUuWxBdIJm\/ZOLRbs2YN7dy5UwwJ4JBt3rx5eO3v3ZZcayCHhBySqTkN2BySFMBpHJIdQuo4JLfxSkhqm5oUkto6peKYQ5r10ufU3tOnu7Wsvx\/wQMpaoSsnAJD8KwkPKf4eEoDk3\/4DqQkg+Zc1qkC6dLKFPltaKm4MI7W7UjrYvusIgOTf\/gOpCSD5lxVAgodkaj0I2QyVApAMhXIoBiABSKbWAyAZKgUgGQoFIGmFQlLbXSIASWs+3xUAkAyFApC0QgFIAJLWSHQFACSdQu7fI2RDyGZqPfCQDJUCkAyFgoekFQoeEjwkrZHoCgBIOoXgIZkqBCABSKa24loOQPIvIUI2hGym1oOQzVApAMlQKIRsWqHgIcFD0hqJrgCApFMIIZupQgASgGRqKwjZslYq\/QQI2RCymZoVQjZDpeAhGQqFkE0rFDwkeEhaI9EVAJB0CiFkM1UIQAKQTG0FIVvWSiFk00kIIAFIOhvRfg8PSSuRawHkkJBDMrUe5JAMlQKQDIVCDkkrFDwkeEhaI9EVAJB0CiGHZKoQgAQgmdoKckhZK4Uckk7COABpVmIoNSTHWLeCFSN1vRry9\/CQ\/AuOHFL8ckgAUn+fDSopKbns3\/yDqQkg+dcVQAKQTK0HSW1DpQAkQ6GQ1NYKhZANOSStkegKAEg6hZDUNlUIQAKQTG0FSe2slUJSWychgAQg6WxE+z08JK1ErgWQQ0IOydR6kEMyVApAMhQKOSStUPCQ4CFpjURXAEDSKYQckqlCABKAZGoryCFlrRRySDoJASQASWcj2u\/hIWklQg7JUCIACUAyNBX3YgCSfwmR1EZS29R6kNQ2VApAMhQKSW2tUPCQ4CFpjURXAEDSKYSktqlCABKAZGorSGpnrRSS2joJASQASWcj2u\/hIWklQlLbUCIACUAyNBUktbMWCjkkrYQAEoCkNRJdAXhIOoW855B4ATBed4eP51p6aVPrOf8XMaz5QPkI4jc539s7ni6dbKHPlpaKmi8fv0BP\/Oqs4VmyKwYgAUjZWRARAUj+JXR77Q8gLbNEZUgNvzVp\/d39wjIBzKAPCegJ27pSLoUF2vrlwAJtWVrhVaMTVFLTSHajHnZrkoqTa0M3egAptUPhIcFDyvIRh4eUjYAAEoBkaj8YGGmoFEI2Q6EcigFIAJKp9QBIhkoBSIZCAUhaoRCyIWTTGomuAICkU8j9e3hI8JBMrQcekqFSAJKhUPCQtELBQ4KHpDUSXQEASacQPCRThQAkAMnUVlzLAUj+JUTIhpDN1HoQshkqBSAZCoWQTSsUPCR4SFoj0RUAkHQKIWQzVQhAApBMbQUhW9ZKpZ8AIRtCNlOzQshmqBQ8JEOhELJphYKHBA9JayS6AgCSTiGEbKYKAUgAkqmtIGTLWimEbDoJASQASWcj2u\/hIWklci2AHBJySKbWgxzSFaV27NhBFRUVdPHiRdq8eTM1NDSkaAggmZoUPCSdUvCQ4CFltJENGzbQlClTaNWqVZRMJmnevHm0evVqam5utuoBSLrHDDkkU4UAJO9A4tU8f1P0h8JZmD9\/PnV2dprKnXW50BdoY++oq6uLamtraebMmVRXV0eNjY1UX18fayCNfGsd3Tu1lPbs2SM6MA4LtCUSCaqqqrLanLU12U4QxBK2XtscBSDJNl\/Tto+eLD9PUV8xsmCANHnyZNq4cSPt379fAEj+3dbWJgAlD+kh1dTUGNH5yfJzNKN8PFU3nRGn4L\/5YXi8ZTgd6ryY6+cs5Xxd50msGHnzRz+nn9X+mGSbv7y+MmXFSAZWGMukvnL3JTrYc23amtn\/VPm1tab27o5i8T0\/KPwraKqzVyFnF38l1tT+k\/+ZIe79veS34hTv9Fzre01tr23uqXyUxn9\/OnW\/2L+ELX+mLmEbdN\/INu\/+24UCSLc3XZUi5U30BfESw\/KYv38Itff2eZXbc3n1uRnce5o4\/8jHT4+PpHd6xgx8D8nuEbkBiTvw7ZVzjQUe1nmU+orG0e+q6kSdMQf\/ma75eB99c8NU+n3xOOPz+C341meD6JbrLtO4kf1nOH2e6MMvBlkf2L\/3ey1dPb7vV8c+RP858aGUoo\/\/5mma0H1UfPb7onH0TWKq7lRZf391z2ka9tuj9PoP\/13oM\/5f\/iLU6\/PFjn4+iLouEP3phMvW\/cjP5Adh9Y3UY9X0xhRt\/\/jSh\/Tgh39nfXbuD36QtfYmJ1CfGwZSouFRUU19br7\/1G6TU+WsTKghm6mHxHen\/mKY3O0\/dExOKfZwcRuNLx5sUnVAlem4qpTWFv+N4z2x4f\/3kFtoW\/dPQrvnhnMV9H\/d\/V7qnPEj6M5vPwrt+lG70M9HLhR9YD\/Kvj0tPuI++vOvfxlas9Xnhj019uz\/esjb1vVl1BFWg0IFEt+USQ4prJvHdaAAFIiWAqEDyeQtW7QkQmugABQIS4HQgSS9pEzjkMK6eVwHCkCBaCmQFyBFSwK0BgpAgagoACBFpSfQDigABShyQKqurhbjYoYOHUrHjh2jxYsXx6ab1IR9lBst33aWlZWJZnZ0dIiR8ydOnIhys0Xb5LQj\/veBAwdSxq9FvfFs20uXLqVt27alTZeKWttVncPUOlJAUscpNTU1iUGU9kGTUes42R7ZgXF4SPjFAh88GNVtLFgUdV6xYgVVVlaKHym3Uf5RbDe3Seo8duxYx\/mbUWq3fXhOmG2LFJD4F2TRokW0fv16MbdNfSMX1V9v2Xk9PT2i3+S0mDA7MdtrxUFnp3uMi0fKbWeNJ06cSAykqHtIDPs1a9bQzp07Q\/fkIgUk\/gWcO3euFT7w306Tb7N9AIOqH6cHRNWA281HnMLjOHlI8of2jTfeEPYcdSCpaZOwQ\/pIAcn+Sw0gBYXO\/vPGTWPpbcyZMyc2uS8GfktLi\/Ce45BDUm3i9OnTInXCEUAYP1iRAhI8pOABpF6B9V6wYEHkcxpuqsQh1FTzXnFKatvtJKxIJVJAimMOyR76xCWHxA8zr6pgX4sqXCRmd7U4eHf2t1V8x24LE2anRnC17Y5CcFeiaL32j\/NbNu6kuOSQ4vAgOxk9Q7S0tNQKHeKW+4qLh6TqHPZb2Eh5SGyEGIcU5O\/Pd+d2+tWOy1gkte1xabPs0bgAyW4jYY4HjByQgn8ccQUoAAWiqgCAFNWeQbugQAEqACAVYKfjlqFAVBUAkKLaM2gXFChABQCkAux03DIUiKoCAFJUewbtggIFqACAVICdrrvlMMcpyWkV6r589vaZlNHdE76PhwIAUjz6KdBW2ieqhgUkdVpFphvk9j311FP07LPPpuxwHKgoOHleFACQ8iJ7tC6aj5nzPAL4mWeeoVdffdVoiQt1DadoqYfW5FIBACmXasbwXPbVI3ft2iXuQk6mXL58OQ0bNkys4MkrTPb29orlM3jWelFRkfhbnQ\/H4OCZ+HxkGknNo5bvv\/9+evrpp62VKjONwg5zPlUMu3HANBlAGjBd6f9GMoVsDCSGFm+5ffjwYbEUxahRowSE+Kirq6PW1lax+qQ91Ms018w+L80OHPu8wHwuGuZfWdT0qgCA5FWxAVheByS+ZbkWjgoZ+8RLO0Tsqzeo0tnLMpAWLlxI7KE5JbjzEVYOwK6O\/C0BSJHvouAbmAsgbd26VXhPcuMA2Wq3pTacVkaQUJJ1VTjlc53n4HsAV5AKAEiwhbQF89XQi0M2vx5SJml1S7XY12uCh1QYhgogFUY\/Z7zLXHhITjmkTIvA2XNITssXq+urI4dUGIYKIBVGP2vvUt3Gqb29PeUtm6mHxOXUt2yZVkbUvWWz18VbNm0XDogCANKA6Mb43YTXcUi6EC9+CqDFTgoASLCLvCmAkdp5kz6yFwaQIts1hdEwk3lqJmUKQ62Bf5f\/DweFBw\/JBGGCAAAAAElFTkSuQmCC","height":176,"width":292}}
%---
%[output:33efac5b]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 16.335886 seconds.\n","truncated":false}}
%---
%[output:7176a8a7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASQAAACwCAYAAACmeRMNAAAAAXNSR0IArs4c6QAAEAhJREFUeF7tnU1oHdcVgE9ky+KhWAhnIZw0IaYGUVraRtBo4S6Uli7sGoUKoSaU1obuKmtTTLSoalojaJQaCrVFG5CKTSFpvSkRrkua4qgBUbRRNm0h1ODg\/AglG6EfHvpBLWfCVUejJ72R3tyZO\/d+b2U\/zZt7zneuPt17Z96dR06cOPFf4QUBCEDAAQKPICQHqkAIEIBARAAh0REgAAFnCCAkZ0pBIBCAAEKiD0AAAs4QQEjOlIJAIAABhEQfgAAEnCGAkJwpBYFAAAIIiT4AAQg4QwAhOVMKAoEABBASfQACEHCGAEJyphQEAgEIICT6AAQg4AwBhORMKQgEAhBASPQBCEDAGQIIyZlSEAgEIICQ6AMQgIAzBBCSM6UgEAhAoDAhDQwMyIULF+Tq1asyOztLJSAAAQgUs0GbymhwcFDW19dlZGQEIdERIQCBiEDuI6ShoSHp7++Xe\/fuybPPPssIiY4IAQhsE8hdSKblelO2kydPRofOz89TLghAIBACTgpJZaRTua6uLpmYmJDJyclAykGaEAibgJNCUhGNj4\/L6OiozM3NMUoKu4+SfUAEnBZSX18fMgqoM5IqBBASfQACEHCGAEJyphQEAgEIFCak\/dCbNSSmbHRQCIRFACGFVW+yhYDTBBCS0+UhOAiERQAhhVVvsoWA0wQQktPlITgIhEUAIYVVb7KFgNMEEJLT5SE4CIRFACGFVW+yhYDTBBCS0+UhOAiERQAhhVVvsoWA0wQQktPlITgIhEUAIYVVb7KFgNMEEJLT5SE4CIRFACGFVW+yhYDTBBCS0+UhOAiERQAhhVVvsoWA0wQQktPlITgIhEUAIYVVb7KFgNMEEJLT5SE4CIRFACGFVW+yhYDTBBCS0+UhOAiERQAhhVVvsoWA0wQQktPlITgIhEUAIYVVb7KFgNMEEJLT5SE4CIRFACGFVW+yhYDTBBCS0+UhOAiERQAhhVVvsoWA0wQQktPlITgIhEUAIYVVb7KFgNMEEJLT5SlncFtPbcnaC2vS8ocWaXrYVM4kiLoQAgipEOx+Nqoi2jizIc0zzbL47qK0P9MeienozNHoPV4QqEcAIdUjxM9TE1h7cU1Wb6xKZawi1eGqtLzeIvqe\/r\/ySiX1eTgwXAIIKdzaN5z51tZT0tT0MDqPjo70pSMilZF5qZRURkzdGsYdxAkQUhBlzj7JtbUXpVp9Sb5Q+ZHMt\/xDqi9Vd4go2WJ8lNRd7ZY3134rLS2vS6XySvbBccbSEkBIpS1dMYHrqGht7QWpVoflG02\/l09bfykfNX+0PUJSMek0zbxURMnF7Sc2npC+5T75ufwsGmG1tT2\/PdIqJitadYUAQnKlEiWIw4yKntrakgtyMxrdvFL5\/9qQGSXF1450+tZ6qTVaT4q\/zmxsyK+XH5Pn5G15X56WSmWM0VIJ+oDtEBGSbcIenD8+Kups+YX8dWNEPmhqkt62th3Z6RW2zTOb0YjIXGXT93T9qNZVtpeqVRmuVuVbR0flb5s\/iUZJra2XpLl5xgNqpHAYAgjpMNQC+oyuE+n0TGWhaz5vbYzIk1tb8nxbmzxs2vseIx0tpbmyNrW0JGc2N+V86xflzbVXZXPzTNSOiolXeAQQUng1T5VxfFRkplM6onlxbU0utbbKTHN29xWplFRyz7S3i04LV1dvbAuQRe9U5fLmIITkTSmzSyQ5KlIp6LrRu4uLMlap7Fg3yqJVc+6Zo0ejaWBchix6Z0G4POdASOWplfVIa42KtFEVxhtLSzXXjbIKShe5p5aXdwhP41laeiMSFIveWZF2+zwIye365BZdfFSUXFi+sboqKox660aNBmuk1Hv8+I4p4X6xNdomn3eLAEJyqx65R6Ojj5WVG9Ficq1RiK4ZqZCSkrAVqLalbSbbi8d59OiMtLX12gqB8xZIACEVCL\/opuMLyLUut9tcN9ovd7PIXWtEtrFxRpaXp7gSV3TnsdQ+QrIEtgynNVOhEyce2xWuWTfSH+jVr7xfKiV9Je910pHS4uK70VW49vZn8g6L9iwTQEiWAbt8ejMN2tp6ctcvd17rRnvxSV550+PMIrf+Gxm53LMOH1umQrp586Z0dnbK+vq6jI+Py+3bt3dFNjAwIIODg3Ls2LHoZysrKzIyMiKzs7Pbx3Z1dUWf7+vrk\/n5+cNnxyfrEqglJbNupPcbvd6y8ysfdU+Y4QHxK28vt3RGV9yQUYaAHTxVZkIaGxuT06dPy+XLl+X8+fPS29u7SzSa\/9DQkPT09ETHPXjwoCYShJRvT4lPg77c9pXofiMVkQqp6JfejPndaod8vunvUSh8EbfoithtPzMh6ehoYWFBhoeHpbu7W0ZHR2VqakquX7++IwMVV0dHh1y8eHHPzBCS3aLXOruRUo9My62mbxaybrRXXO2Lf4q+poKM8u8XebeYiZBOnTol165dk+np6UhA5v\/379+PBBV\/mWmdee+1117bJS2ElHc3+Kw9IyVXLqubNSNdTzpbOSd\/rCwUA4ZWcyOQiZCSI6K9hJR8X6dv\/f39u9abjJB0rWlubi43GDQk0fToN9V\/S9FSistIR2zfaV+kPAEQyERIBxkhxZnuJS4jJD12YmJCJicnAyiFOyn+eKlLRjbfKkxKZqH9c5tPRNPHa62fZvplXndIE0mSQCZC0pOmXUOqJSQz1TM\/M0LSdSgdIXGlLd+Oq1OkXy1+Ndo8Le+RUlxGb8tz0TQtvglcviRoLW8CmQkpzVU2ndpduXJFbt26Fd0SoFO2WlfjWEPKuxvsbk8v\/fevfi1XKSVHRk\/L+84srhdfkTAiyExIZpSUvA8pKaH4fUh73a+EkNzofHq39Mbm13ORUvx+qKtNP5Tvb93LfN8lN6gSxX4EMhVSVqgRUlYkGzuPuVt6WnqsSslse6LfrdOnmPxz9c9W9l1qjAafzoMAQsqDconbMHdLn6uclb9U72b+pdakjF5de3N798gSYyP0QxJASIcEF9LHzL7XX2r9tvxr9U5mUorLSHel\/OnW76xskRtSrcqeK0IqewVzil+\/TqJPGjlbOZvZ9h+624BO03RT\/86Wl61tkZsTIprJgABCygBiCKeI7+Z4b+sH0Ub8jTwdJCmjGysrTNVC6Eh1ckRIdILUBMxujro\/0n82vhdJSTd2UzEd5GX2YbL9NJODxMSxbhBASG7UoTRRxDdOMztOHkRKSRkVtStlaYAHFihCCqzgjaabfDqIEUwaKdWSEVO1Rivi1+cRkl\/1zCUb8whssxG\/Tt10tLSflJIy0kDNgydtP80kFyg0kgkBhJQJxvBOktzz2kjp+PFeaW6e2QHETO3iTzWp9Ry28CiScZIAQqJPHIpArbWfWlKqJaM8Hjx5qKT4UOEEEFLhJShvAMmpm2YSl5Le+Kj\/Tz7vjalaeWtuO3KEZJuw5+c3z1CLPyppaWkqevCkvpL3KjFV87xDNJgeQmoQYOgfN1O35EMBPnt22gc7njDLVC303lI\/f4RUnxFH1CFQ63Hb+oTZI0c+iB7oaF5M1ehK9QggpHqE+HkqArWmbvEPmqla0c96S5UMBxVGACEVht6vhms9adZkyFTNr1rbzAYh2aQb2LnjX8CdaW7ezr7ox3IHVoZSp4uQSl0+94I3eyfpVTd9uCNTNfdq5HJECMnl6pQ0NrN30qVHH5U3lpaifZR629pKmg1h50kAIeVJO5C2zKhIR0j64rtqgRQ+gzQRUgYQOcVuAmbvJK6q0TsOQgAhHYQWx6YmoGtJD48ciR5lxAsCaQkgpLSkOA4CELBOACFZR0wDEIBAWgIIKS0pjoMABKwTQEjWEdMABCCQlgBCSkuK4yAAAesEEJJ1xDQAAQikJYCQ0pLiOAhAwDoBhGQdMQ1AAAJpCSCktKQ4DgIQsE4AIVlHTAMQgEBaAggpLSmOgwAErBNASNYR0wAEIJCWAEJKS4rjIAAB6wQQknXENAABCKQlgJDSkuI4CEDAOgGEZB0xDUAAAmkJIKS0pDgOAhCwTgAhWUdMAxCAQFoCCCktKY6DAASsE0BI1hHTAAQgkJYAQkpLiuMgAAHrBBCSdcQ0AAEIpCWAkNKS4jgIQMA6AYRkHTENQAACaQkgpLSkOA4CELBOoBAh3bx5Uzo7O2V9fV3Gx8fl9u3bOxLt6uqK3u\/r65P5+XnrEGgAAhBwg0DuQhobG5PTp0\/L5cuX5fz589Lb2ysjIyMyOzu7TQQhudE5iAICeRPIXUg6OlpYWJDh4WHp7u6W0dFRmZqakuvXryOkvKtPexBwjECuQjp16pRcu3ZNpqenIwGZ\/9+\/fz8SlHmZEdLg4KDMzc05hoxwIAABWwRyFVJyRFRPSJr0xMSETE5O2sqf80IAAg4RyFVIBx0h6XROR0gsbDvUYwgFAhYJ5CokzYM1JIvV5NQQKDmB3IXEVbaS9xjCh4BFArkLyYySuA\/JYlU5NQRKSqAQIdVjxX1I9Qjxcwj4SQAh+VlXsoJAKQkgpFKWjaAh4CcBhORnXckKAqUkEIyQTp48KefOnZO7d+96e1+T7zn6np8aJIQc9zNlMEIKYaHc9xx9z09\/UUPIsbRC0u+yZXWXtv7l0S1Nsjyna2Ni33P0PT8zQnKln2b1u3eQ3xMnR0ja8XRLEv1rwQsCECiGQBHfI3VSSOYvhYqJFwQgUAwBHSHlPUpyVkjFlIBWIQCBIgkgpCLp0zYEILCDAEKiQ0AAAs4QQEjOlIJAIAABr4U0MDAQXeY\/duyYvPfee3Lx4sWaFTdPQTE\/fOedd3ZsqVuGbpI21zLkojGmzceH2iVrEt8zrCz1yipOb4UU3y73zp070V7eyb27FWJyF8uswOZ5nrS55hlTI22lzceH2tWSkW7NU8Y\/io3U3HzWWyHpX9gLFy7I1atXo0csxTeGe\/DgwTY77fxXrlyRW7du7Xo+XBaA8zhH2lzziCWLNtLm40PtDC8j1+Xl5egt82SeLHiW6RzeCmloaEh6enqi57+pgPT\/tZ4BF58aaOE+\/vjj7c+UpZBpc\/UtHx9qV6smTNnK0lMPEGdyRLSXkOLvf\/LJJ9HUTv9K7bXedIAQcjs0ba65BdRgQ2nz8aF2CGknAS9GSGa4+\/jjj0fZ6fz7ww8\/TDVCSnaIvcTV4O+Y1Y+HOkLyoXYIyUMh1Spq2nWIWp06PtWzapKMTn7YXDNqPvPTHDafpJgzDyynEzJlywl0ns2kvVKj04OOjo5oirbXgyvzjPswbaXN9TDnLuIzafPxoXaMkAIZIWmae93LkvwLFL+XZb\/7lYr45UzbZtr7dtKer+jjQqpdkjUjpKJ7H+1DAAIQEBEvFrWpJAQg4AcBhORHHckCAl4QQEhelJEkIOAHAYTkRx3JAgJeEEBIXpSRJCDgBwGE5EcdyQICXhBASF6UkSQg4AcBhORHHckCAl4QQEhelJEkIOAHAYTkRx3JAgJeEEBIXpSRJCDgBwGE5EcdyQICXhBASF6UkSQg4AeB\/wGcbeK3JQBU6AAAAABJRU5ErkJggg==","height":176,"width":292}}
%---

%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88];
dq0 = [0,0,0,0];
X0 = [q0, dq0];
Sim.time = 0.6;
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
    [t, x, te, xe, ie] = ode113(@(t, X) halfleka_dynamics(t, X), tspan, X0, options); %[output:5fff0d1c]

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
toc %[output:56b3496a]

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
plot(t_total,x_total(:,1)); %[output:7765b552]
hold on %[output:7765b552]
plot(t_total,x_total(:,2)); %[output:7765b552]
plot(t_total,x_total(:,3)); %[output:7765b552]
plot(t_total,x_total(:,4)); %[output:7765b552]
legend('x','y','th1','th2'); %[output:7765b552]
title("Generalised Coordinates"); %[output:7765b552]
xlabel('time (s)'); %[output:7765b552]
hold off %[output:7765b552]

% display derivative state of all variables
plot(t_total,x_total(:,5)); %[output:07636841]
hold on %[output:07636841]
plot(t_total,x_total(:,6)); %[output:07636841]
plot(t_total,x_total(:,7)); %[output:07636841]
plot(t_total,x_total(:,8)); %[output:07636841]
legend('dx','dy','dth1','dth2'); %[output:07636841]
title("Derivative of Generalised Coordinates"); %[output:07636841]
xlabel('time (s)'); %[output:07636841]
hold off %[output:07636841]
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
plot(t_total, contact_total) %[output:5a193e01]


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
if Animate ==1 %[output:group:7196e8e8]
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
    fig = figure; %[output:86c5fa58]
    
    ax = axes('Parent',fig); %[output:86c5fa58]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:86c5fa58]
    u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:86c5fa58]
    u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:86c5fa58]
    l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:86c5fa58]
    l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:86c5fa58]
    f = animatedline(ax,'Color','r','LineWidth',0.5); %[output:86c5fa58]
    
    axes(ax); %[output:86c5fa58]
    
    axis equal; %[output:86c5fa58]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:86c5fa58]
    
    hold on; %[output:86c5fa58]
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
        addpoints(b,x_sim(i),y_sim(i)); %[output:86c5fa58]
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
    toc %[output:2d2f712f]
    hold off; %[output:86c5fa58]
    clear fps b f l_l l_r u_l u_r ax fig l1 l2 l3 l4 l5
end %[output:group:7196e8e8]

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
    if t==0
        contact = false; % start in the air
        groundheight = 0;
    end
    isterminal = 1;
    direction = -1;
    ph1 = ph1_calc(q(3), q(4));
    dph1 = dph1_calc(q(3),q(4), q(7), q(8));
    foot = foot_Func(q(1), q(2), q(3), q(4), ph1, q(5), q(6), q(7), q(8), dph1);
    if ~contact && foot(2)<=0 %&& foot(4) <0
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
%   data: {"layout":"onright","rightPanelPercent":42}
%---
%[output:5fff0d1c]
%   data: {"dataType":"text","outputData":{"text":"Time = 0.5990","truncated":false}}
%---
%[output:56b3496a]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 19.253160 seconds.\n","truncated":false}}
%---
%[output:7765b552]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUMAAADCCAYAAADTqKDOAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX9sVtd5x0+D6i6JQ6I0AWwgCEUuq9olS7yE0rSi6dZtIogVjYXM2ha24CXIc6RNjmulHiyMINf11i2ul1GgAk1zE08RFUJepWwLrEtThwFKlaii\/oMkgIGQVg2QMFGRTs8xz8t5j+9973Pfe8977o\/vlSzb73vOued8znm+9zk\/70duvvnmXypcIAACIFByAh+BGJa8BaD4IAACmgDEsIENYenSpWrLli2qubm5ctepqSnV09Ojjh071sCcyG61a9cutXjxYjUyMqLeeustnfdz586lnt\/u7m7V0dGhDhw4oPr6+gIz9+CDD6quri7V1NRU+X50dFQNDw\/LCpNCKGIxNDSkZs+erfr7+3WKrpikkF0kEZMAxDAmsHqDs8EfPXpUrVu3TifD4kh\/k3FNTEzUm7yTeFkRwyCxHBgYUMuXL68poGlDcS2G1B42bNignnrqqUw+HNPmmbX0IIYNqBEWPRdelcvsm2I4Njbm7Fa1PENmd\/LkycpDxFlGIhK2xTDNh1de24ivunBxX4ihC6pWmpJuIEdhg2ttbdUfmZ4ke0MHDx5Ud955p+4y2t1sErAlS5bouOZ3bGyXLl3S31FXnbq\/c+fO1V1Uvsz7RXmG5r0ovtlttbu15ndmGSk\/r732mrrnnnsCvbw47Ox7mmWh\/NX6nr979913dTeYvfV77723wufNN9\/UzMO6yVQG6soTdwpHdUjlI878MOE6ZN40NLBt2zbd\/eY6N+OE1SfFr8W\/Ac26cLeAGDagStkAosa4bM9j0aJF2rheeeUVPZbG6bCR2+kGiRd7VCyGZKRsnCwAdvqcz1piuHLlSi0SFHbfvn1VY2mcbxoHpSEByueyZcsq9zXzzXFJCILGDKXsuCx8T9vTYqGK+p6EiIcs7DS4zBcuXAgcM+R7cBo2BxZ2CV\/KZ636NNOy+afpsTbAPDJzC4hhA6rCNmjb+2Ov6syZM1r82GA5HH1PkyyPPvqoHidjYzK9pj179swYzCdjmj9\/fuRgP6fDKCTGysJge6\/0v11eU3TZC2Lvigy3lvcnFUNbcDkfLMJ33313lSDb39P\/Jnv6385X1JhhlODyJJnpHbIXaE9QzZkzp2Z9mh6r7QE3oEkX8hYQwwZUq9TYWQzNGVPKHnsiq1evjhRDc6aa4oYZGxkmixSFI2+RBMMU21qeIcUxu3b0P3fLWbRttGS0zz777Awjr8VH2k0OGt80hbS9vb0yM253WUn87QdRkKgnFUPbs6TJkrDZehbDsPqkoZIw\/llcmdAAM0t8C4hhYoTRCdgentlYTYMNMkgzddtLivIMzbhBA\/R2epJud9AkkDkWR11dukxRNfMRNAmRxgRKHjzDTZs2VQlyrYdNkGcY1tJs\/mHLk6JbarlDQAwbVP9BS2vM7ql07C2sm0wGENe4zOUp3M0mT0TSTQ7rsgd5WbVEN2rMMKi7anptPM6Y1pghD1HQPeodMwwblyQxpMktkxF75ezp0f+87rRWfdbi38i1lw0yn4bcBmLYEMzTNwladE2fh820ml1PMrBaniF7A+YMI3evaVwuyDO0xy5ptpQmMnhCJWo2OWhmlPNhz9zakyOcz6jZZK4eyaLrNGaTTTGke5tpSmeTw8TQ7vrSsAGLIwmYydN8IPHqALM+zQcCM6q1aL2BzTy3t8qEGJoGjArNbVtCxkEg1wS8iyE9Ddva2nTXgJ6cGzduVLt3766sy8o1XWQeBEAgNwS8iiF107Zu3apovMrlDofc1AYyCgIg4I2AVzGkcaze3l519uxZvaOCrrBucktLi6IfXCAAAsUlcOrUKUU\/Pi7vYkinfvAuCRqs7uzsVNu3b6\/yFEkEaVcArYPDBQIgUFwChw8f1utQfQiidzE0xwh5dnNycrLqKCcSQVoU7AtSkZreXXfdpdavXw+WKVUqeKYEUinFLGknEIlioy+vYsjit3\/\/fn0uXZQY+oLU6EpxeT9+sIBlOpTBMx2OlIpvll7FkADQbPK8efP0hn5ahLxmzZqqUz6yACm96vafEg05rFixQo2Pj3vpivgnkG4OwDM9nqUXQ0JprjMMOtnFN6T0qhspgQAIhBHwbefePUNJ0\/ANSZJHhAEBEEhGwLedQwyT1R9igwAIpEQAYigA6RuSIIsIAgIzCGBtbDWSqDWEvu0cniGMGAQcEMDa2JlQo9YQQgwFDdE3JEEWEQQEqghgbWx1g5CsIfRt5\/AMYcQg4ICAb8N2UKRESUp4SMIkykREZIihS7pIu7QEfBt21sBLeEjCuCwXxNAlXaRdWgK+DTtr4CU8JGFclgti6JIu0i4tAd+GnTXwEh6SMC7LBTF0SRdpl5aAb8POGngJD0kYl+WCGLqki7RLSyDIsK+5sUXNurE1s0wuvzelPnxv+ixB2iJLV60zA+IURCJ0kjBx7hk3LMQwLjGEBwEBgSDDvu7zneq6z\/+5ILafIB98\/1vqg+9v1zfnU+hffPFF9aUvfSnxafQSoZOEcUkGYuiSLtIuLYG8e4ZUcfxGxUOHDlWdL1pPpUqEThKmnntL40AMpaQQDgRiEPBt2DGyGhrUfhd1kjQlPCRhkuQhKi7EMIoQvgeBOgj4Nuw6slwVxXxZG51\/Sd5hkpfTS3hIwiQtV634EEOXdJF2aQn4Nuyk4F944QXFr98IezdRnHtIeEjCxLln3LAQw7jEEB4EBAR8G7Ygiw0NIuEhCeMy0xBDl3SRdmkJ+DbsrIGX8JCEcVkuiKFLuki7tAR8G3bWwEt4SMK4LBfE0CVdpF1aAr4NO2vgJTwkYVyWC2Loki7SLi0B34adNfASHpIwLssFMXRJF2mXloBvw84aeAkPSRiX5YIYuqSLtEtLwLdhJwVPe5NPnz6td57Q0pqHH35Ybd68WU1MTNSVtISHJExdNxdGghgKQSEYCMQhEGTYC26YpRY2z4qTTEPDHr9wWZ04f1nfs7u7W7W3t+uDGgYGBtS8efP03\/VeEqGThKn3\/pJ4EEMJJYQBgZgEggz7L+++Xv1Ve3PMlBoX\/O8PXVDfOPy+viHtS+7t7VWDg4Nqw4YN2IHSuGqofSffT4yscEA+8kMg754hkaau8rFjx9Qdd9yhRbHeLjKlJbFhSRiXLQCeoUu6SLu0BHwbdhrgaaxw7dq16vz584m6yBDDOmrDPFDSjO66Yd02\/7rK7Ra2Xv37tvnX688XGt+bYTnSbUacOopdiij\/c\/BdNTjy41KUVWr8WYdBhzUMDQ2p\/fv3JzqkQcrDtZ1H8c6MZ0gDth0dHero0aMznkJxIbFgsbCRqN137y2aBQvXwvnXKw739skP9N\/824ZGn9N1\/OT7iuLRb77enpr+rixXvcLP3L7z3bfVd777VuFxxW2zWQSSxiwyl4t5DDz9hDpy5LAi22Rnw7TNX\/v1+9Xv\/9k+RS+cb\/SVCTHkwVoqfJBLTiC\/va1Pvfjvo+rw4SMVT43FjAyUjE2LneHFVQTLEDP6jAXsO3veqlTI21cE7vgVcWMBbHSFFPl+vV2fVF\/p+qR6+dWz6msjP1YvH3y3sMXNuxiyc3LgwIHEB7uanuGC659T7545WuWIvHzwrHZSfvHLG9Wv3vF76o87ny2vGPKaJpq+p8uewqeG9cK3V6rZTa\/r701PzRQ3+vz4FeEjcSNhg6hlS2\/oYfXNp9vVfffeqgWxqF3nvIth2q2GeYzu2lQROvth6JuZd8+QXPHVq1erJ598Um3atClUDEdGRtSOHTvU+Pi4OnVq+qU1uPJL4A+\/vEh9c2u7LkARRdG3YWetZUTxaGlp0TPO\/f39qqurq3yeoXma7tjYWNUbuczKZJD0GQnizp07s1bXyE8dBMhLfOjLiypd5yJNskQZfx24ch0liscjjzyi1q9fr8tYSjHkdyw0NTVVVbQ9icIgt2zZop8Y8AxzbRczMm+KIk2u0NBG3rvPUcaf9RqkcfyNGzeq3bt3q7lz56pVq1Zpry1oraFk1jmKB3mG9HoBEsRSiqHdIHwtrcl6wyxL\/lgUqQtNFwljXkUxyvizXqfmUf+1xJDfoNfc3KxGR0dDl+BIeEjCuOTmfczQLBzE0GVV5ydt01PkMcXnrniMeSlFkGFTucx1rFkriznhSO9AaW1tVVNTU+rIkSNq+fLl6ty5c5XPenp6dPZpnJ\/G8Ts7O9XevXshhq4r1fcTw3X5kH4wAVsUaUlOXsYVg9osLy3Kan2bE1m2Z0hrgMnze\/XVVxUNV5nCx94hxLABNQsxbADkjN+ChJHE5L57bq10oUkcs7pWsfvR31Yd656qGv\/Kk2cY1k1+5513ZuxKgRg20Hgghg2EnfFb2d4iTbbQ2GJWutG8jvKuu+9WJ95\/yNtkQNJqhBgmJegoPsTQEdicJ2sK4\/SC+\/d1N9qHx8ieK03+UF7+5h8vqL6vfh1ieKWNSWxYEsZlk83UBEpYQX1DclkBSDsdAtSFpovEiPeZszi69BrtGXCa\/SZPNe9tlru+NGnCEyi0tAbd5HTaa92p5L1h1V1wRKyLAAsU\/eZlOml5jjzuR9sJTeG1lwGhzVZXnYSHJExdDUIYCZ6hEBSC5ZOAKV6fu+cWvSeaLxZIOrjj5VdnHhpB+9vNk1XoMAGOT3HpgAE67CNoEse3YWettiQ8JGFclgti6JIu0s4kARJImpWmcypJIOniI934YA\/79CMah2TRJJGMmsX2bdhZAy\/hIQnjslwQQ5d0kXZuCZhnXdZTCN+GXU+eXcaR8JCEcZlHiKFLuki7tAR8G3bWwEt4SMK4LBfE0CVdpF1aAr4NOyl4yUENfEADbdujK+iUes6HhIckTNJy1YoPMXRJF2mXloBvw04KXnJQg\/k+ZV6Kc+jQocCTsSU8JGGSlgti6JIg0gaBAAJBhj3n2oXq1msXZpbX2YvH1TsXj+v8SQ5qoNeImhefWN\/X1zejjBKhk4RxCQ+eoUu6SLu0BIIMe21bj1rb9kRmmTw\/+XX1\/OSQzl+cgxoovNmtpoOa7UsidJIwLuFBDF3SRdqlJZB3zzDO3uQ0DnelhgIxFJiLb0iCLCIICFQRyHublYphlEfIUCQ8JGFcNjN4hi7pIu3SEvBt2EnBS8SQzjbs7e1Vg4ODga8DMPMg4SEJk7RcteJDDF3SRdqlJeDbsJOClxzU0N7erpYsWVJ1q7D3LEt4SMIkLRfE0CVBpA0CAQR8G3bWKkXCQxLGZbngGbqki7RLS8C3YWcNvISHJIzLckEMXdJF2qUlwIZN7\/mm8wDLftGrQKNeEA8xFLQS35AEWUQQEKgiwMZPbRfXNAF65zm9TCrsvee+7RyeIVoqCDgiQIJIP7imCZAIhgkhfQ8xFLQU35AEWUQQEACBhAR82zk8w4QViOggAALpEIAYCjj6hiTIIoKAAAgkJODbzuEZJqxARAcBEEiHQOnFsLu7W3V0dFRojo6OquHh4Sq6viGlU9VIBQRAoBYB33bu1TOkLT\/m3kYSxlWrVun1SBMTExVuviGhCYMACLgn4NvOvYqhjZf3Q+7du7fKO\/QNyX0zwB1AAAR823mmxNA8KcM8INI3JDRTEAAB9wR823lmxJAPiJycnJzxDgVza9P4+HjNhZvuqwx3AAEQSJsALU4nO4\/aspf2fc30MiGGLITnz59X69atm1FeFkP6gvZ67ty50yUTpA0CINBgAo888ohav369vmtXV5feutfoy7sYRr1Vi4CwGNK+RoJUa0tPowHifiAAAskJkGe4YsUKLYilFMNaXWMTr++xhORVjRRAAASiCPi2c6+eIU2Y0FOgqampipO91tA3pKhKxPcgAALJCfi2c69iKMXnG5I0nwgHAiBQPwHfdg4xrL\/uEBMEQCBFAhBDAUzfkARZRBAQAIGEBHzbOTzDhBWI6CAAAukQgBgKOPqGJMgigoAACCQk4NvO4RkmrEBEBwEQSIcAxFDA0TckQRYRBARAICEB33YOzzBhBSI6CIBAOgQghgKOviEJsoggIAACCQn4tnN4hgkrENFBAATSIQAxFHD0DUmQRQQBARBISMC3ncMzTFiBiA4CIJAOAYihgKNvSIIsIggIgEBCAr7tHJ5hwgpEdBAAgXQIQAwFHH1DEmQRQUAABOoksOCGWWph8yx1zyfmqz\/q\/wf12FeeKudJ1xJ+EEMJJYQBgewRIKGja1nL9Jmly1o+qn+zANLvE+cv6\/\/per\/ti6rjuaMQw7CqhBhmr5EjRyDAomaLnS2ATIpE7\/iFy\/pf+puuV079Qv9Nn89pu1ONjIyU89h\/aXMiMdzRv0H9x86vJX7\/yYkLHwbelivH\/JIrzqxMaZ4RDgSKQCDIszM\/Y7thD88UOxI6Fj76PMjGTEa+nZ7cTKD8y\/rPq3P\/\/a8125cpXjQGYV5cgWk2ULNyzUZw\/PxltfCGWYp+k\/guaL5mulFcEWI7XlQjSTPPSAsEbAJml5XaKrVd+oy6tmYX9pVTlypR6XP26uhDidhFkYcYRhEy3o7n6q1ZYUJZS1BZ4Dj71ID4CkqPGxaFqSXMV7sPVxueKaq2oKbRCAVVgCA5JmB7cn\/wiV+ptEEey6PikdhRmzfblNmNdf3QhhgKGplvSIIsioOYQshiy5+ZAstP54rYNk8\/rc3L7KJwd4Sf0q9MXRVT9ko5PARUXF25CRjWnaU2ZrYbuw3820\/+T38\/7elVt5lGF963neemm+xzYLXRjSLqfty4TTFlIWUP1ez6hKVnGwZ5AZQOeZ9BHmpUvvC9WwISwWPvjj296d9XJylce3dJCEAMBfR8QxJkMfNBTAG1PVEWUOoyhXXhTeE0vc4fXhkk9+1VZL4CYmaQx+zMpSg81EKsuXvL3Fnw8lwPvu0cnmHMRlqW4PagOnmLZJjcpQoTTjZGHmA3Pc08G6rLejeFz5644GUn9ABij72owxwQQ0Er8w1JkMXSBrFFk0DweKc9XsWQzPEpmhxi79Ls2hURqNnN5QeLPYHBXjcxKarohdWtbzuHZ1hEq8tYmWoJpikGZrbNMUsSTLrIMwqa8cxYcWfM1JrCx0tV6PfYTy7qrJPwwWtWCmIoaMm+IQmyiCAJCQTtWqAu9rLWJr3cQzKWqQWTdjMY4ln57MrOB9M7rSfL9moA+t9cm2cuxzIXIpvr8iB8weR92zk8w3osAnG8EWAvkzLAQmR2zfXfVxbc11rPGVSAWjOtYWnlefbWWyWG3BhiqJTatWuXWrJkibp06ZLemzg2NlaFyzekrDUa5CcegVprOzklexG9fQdzGyfWa8bjLw3t2869e4YDAwOqra1N9fT0qJUrV6pVq1ap\/v5+NTExUWHoG5K0MhEOBECgfgK+7dy7GJJXePr0adXX16eWLl2qtmzZovbu3auGh4chhvW3K8QEgdwRKLUYLl68WA0NDan9+\/dr8eP\/JycntTjyxZB27NihxsfHE59ck7tWggyDQMEJtLS06Nlk6hW6OoMgCqFXz9D2BGuJYVvns+o\/j0+f\/lLk6\/J7pzJRvA9\/PhU7H5T3WTe2xI6X1QjX3NQamTXiJAkXmVBAADNt+z5B9cP8pW3ow\/fi17GZTel97KJ9+POZbfyBB1aoFQ88oOZe+0v11BNd5TvcNY5n+Njmf1Jbh3cWwivMmmBcc2O00ddjzGFx4pY\/rtGxkaddrqTikSbDsLIFsbVF8qOL2uvKStx6q+cmv7nwQzW5fUP5xJBgYcywniaDOCCQPwLXRPQaqJs8vLWk3WSqTswm569RFy3Hc65dWCnSrVf+nnPd9GfvfHBc\/37jZz8oWrEzV55ST6BwbWCdYebaZSEzRKJHYkdC9+mb79N\/f\/rjn1XvXDyuTEEMK\/zrP\/2BeuNnL+uvn58cKiQjn4WCGAro+4YkyCKCZJAACdynPv7ZKuHT3t7F49rjO0u\/L75d+Z++o884jCmQlA79f\/+ChyrC+fzk19W0QMJrTKP6fdu519lkKUDfkKT5RDj\/BO5fsFaLH4sX5YgEi0Tu9Z+9rEUwDfFa29ZTEUYS1+HXHk8lXf8E\/eXAt51DDP3VPe6cAgH2\/u6f\/5Du8rJX98ZPf6DF76UTz6dwl\/Ak6P5\/cccz+t4kut\/80ePa08QVnwDEUMDMNyRBFhGkgQR47G9t2xOVMT\/y+F46+ZwiEfQhRpSnzZ\/Zo7vQ1H3GmGL8BuHbzuEZxq8zxPBAwBz\/o64wj\/uRALr2\/uIUl7rPJNLoOsehNh0WYihg5huSIIsI4ojA9KTFWj0+x9dLJ6YF0IcHKCkm5ZkEkfJNXeeNE6sl0UofxredwzMsfRPMJgAWQfayqPv7Xyeey9Ukxadu\/qwWRVrKgwmW6HYGMYxm5N19FmQRQVIgYE+GkOdHXmDex982L92jxzYxlli7kUAMBUbkG5IgiwiSgACLoPairl2ou5ZZGwtMUDwdlbrM3Xc8g25zDZC+7Rzd5KStHPHrJmCPB+axKxyn8DzjTHE2\/nB1Zsc845QpzbAQQwFNgvTM09vU41991MtpFoIsIkhMAnpSZP5DejyNRJC6kFmdEIlZtJrBeV1iHsYReQkTF4j3a5sF5L3b9Jm5e6ceZhBDATWC9PTvjKmLr91QFZqNh4yJL9peRVf1lissghVgbkgQc5EyzQiXRQRtuDSOmBVBtLctmqIn2bMdtLebbfPqQRfTe7rZLulveycQxFBgggTpG7071ba\/2105z9CspDnX3qY33dPFuxCuiuPVTfhmBV19ik2Lp11R5pOOvxNkFUFCCLDB0bgZ1uBNQ\/IliEFrNukz04Gg3Tu63V\/Zwx1kA0FCSdsgKQ7bIdkmXeb2SLuJmHb5uT9tUz3f+hMvPcDCjhlyRfEpJWHiaQsoP+W4gmo9GSXdOrMbUUspSZxZ0OMoalDXJU78RoVlYyvC7HCazBopiOb6Rxa3Rm1bNJlx95varmlfv\/WpNer2322GGNZqYI10n83KMcUpbteBn4hpGk6StHj4IEkaSeLidJdweiyIj730G0kQB8YN8gLpgZTF+miknQfBKqxnmHqrQoIg4JAACSL1DoZ\/9Hhqd6FF3913PqO9LxqfzfqidYihoOp9QxJkEUFAIBEBEq6\/\/cyeVBZm25NUWRdBBufbzuEZJmrCiAwC6RHgQx7++oer69p2aG9hzNsWQIihoC35hiTIIoKAQCoEqLtMV9zDHcwF3XmdpPJt5\/AMU2nCSAQE0iFAovbP9\/9vrO6yeWxYnne2QAwFbcg3JEEWEQQEUiMg7S6b2xnz6g2a0HzbOTzD1JowEgKB9AhELbfhmWLdpS7IPmeIoaD9+IYkyCKCgECqBLi7bB8Oa06SFO1IMN92Ds8w1SaMxEAgPQK83IbXCH5xwUP6KLCinPNok4IYCtqOb0iCLCIICDghYC6cJhEkbzBL73xJs9C+7RyeYZq1ibRAwBEB3tvtKPlMJAsxFFSDb0iCLCIICIBAQgK+7dy7Z9jd3a06OjoqGEdHR9Xw8HAVVt+QEtYxooMACAgI+LZzr2K4dOlS1dvbqwYHB9XExIQiYVy1apXq7+\/X\/\/PlG5KgHhEEBEAgIQHfdu5VDG12JI5btmxRe\/furfIOfUNKWMeZit7S0qJWrFihxsfHKwflZiqDOcsMeKZXYb7tPFNi+OCDD6rOzk61fft2NTY2NsMz3LFjhzpy5Eh69EuYEhkved5gmU7lg2c6HCkVZtnV1VXuk64XL16shoaG1OTkpOrr66sizJDoyYELBECguAQOHz6se4enTp1qeCEb6hmakyUXLlyojA2yEJ4\/f16tW7cuEAIJIv3gAgEQKC4BEkEfQkhEGyqGQVXI44SHDh2a4REWt8pRMhAAgawR8CqGtbrGWQOF\/IAACBSbgFcxpAkTGixtamqqohy01rDY1YDSgQAI+CbgVQx9Fx73BwEQAAEmADFEWwABEACBLEygoBZAAARAIAsEMu8ZmuOKR48eDV16kwWYWcuDhB1PYrW2tursT01NqZ6eHnXs2LGsFcdrfiQszQyGbSDwWogM3VzK0wxnLsdzUZRMi6G5PW\/fvn2hi7JdgMl7mlJ2AwMDuqi00B2z+8G1LmXJsZnjLbfcokZGRqp2U+W9XaWRfylPe3sutdV58+Y5c4gyLYb0VHj44YfV5s2b9cENBKOtrQ2ei6BF1ssOjGfCjcuSGC5atEiRGNpbSwVVV\/ggUp60SaO9vd2Z+NmgMy2GBOMLX\/hCRfzCTrUpfOupo4D1stu1a5e+W9hOoDqykvsocViyoX\/ve9\/TJzBBDGdWv5QnPVRuuukmdfvtt6vm5mZV6m6y7aVADOW6Ug878A3mG4clPUxoN9WZM2cCDx2R12BxQ0p5Urhly5ZVhhpcP6jhGRa0zUmfvlx8Cr9mzRqMcQW0BylLs1uHCZRww5LytMcIXT+sMy2G0rGFgupZomLFYUeNjsZm7EN1E2WgQJGlLMlzWbJkSVXJL126hAeM1RakPKWimVZTy7QYSmed0oJRpHSk7Fw\/bYvAVMrSLCs8w\/Cal\/KkcBs3blS7d+9WBw8e1KtJap1slbStZVoMqXDS9UhJQRQxfhg78mBOnz6tl9MEeTNYazizNUhYQgzlViTlaYZz3S4zL4ZyvAgJAiAAAvUTgBjWzw4xQQAECkQAYligykRRQAAE6icAMayfHWKCAAgUiADEsECVmZWiNHKGmhc5Dw8PhxZfEiYr7JAPfwQghv7YF+bO9ob6RomhdO8q5a+3t1cNDg7qPe64QCCIAMQQ7SIxAVsMEycoSIBOhtm6davas2eP6FQY83QeQfIIUkICEMMSVnqaRbbPQ6T319BFhxTQjpYNGzaoj33sY\/o9N3RmIm22p8MLOjs7Azffk2gtX75cp1FrXRmtP1u9erV68sknK2cvmmsm7bj2boY0GSCtYhCAGBajHr2WolY3mcSQBJPO9eNdBLNnz9ZCSRe9MJxfE2t3r2ttzA\/at2qecGQuLKf7mLsZxsbGvPLCzbNJAGKYzXrJVa6ixJBQFlJEAAAA6UlEQVQKw0eCmQJnHyZrC5i9h9WEYoclIe3o6FBhb1b00ZXPVSUis\/5fIo86yD+BNMRw27Zteu8pv36AqYQddGCLIYVnQeS4pjCy8O7fv1\/VmnnOf22gBPUSgGdYLznEqxBIQwx5nzTvmY7CGySGZhz7JB54hlFE8T3EEG0gMYG0xNAeM6x1tJg9Zhh0YKg5hogxw8TVXPgEIIaFr+LGFJBncg8cOKBOnjxZNZssHTOkcOZscq2zAKNmk+24mE1uTDvI810ghnmuvRLnPe46w6hudYlRouhXCEAM0RRySwA7UHJbdZnM+P8DiGtJoLTpsqcAAAAASUVORK5CYII=","height":194,"width":323}}
%---
%[output:07636841]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUMAAADCCAYAAADTqKDOAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+UVcWZ5ydog0iAAVS6AZFwOmxigqMdYBENag47GYYhIUtQiUfYAUYZJJnk9CCrLARDWEI4k5MgkzHgHJjJohJPMD0sa8aZCDFKGgKMUSeDvR4GsJs\/ATZCg9gK7vlV+z3qVd\/7bt377r11331fncOh33vfrT+\/qvrd76vvq6rL+vfv\/wFJEgQEAUGgyhG4TMiwykeANF8QEAQUAhVPhitXrqQJEyZ06c5NmzbRmjVrIncz51tuPl4V2LBhAw0fPpzWrl1LmzdvjlzHsA+OHTuWli9fTr1796b29nZavHgxNTc3d8kG9Rs5cmTh+46OjtTratu26dOn0\/z58+nAgQM0a9YsSqrfGLvTp09TY2OjKs+vb3Xs2traSsrbtjOMnIlBUpiEqVMlyOaGDHXS4s7fsWMHLVq0KFI\/xDmAMGHvvPNOeuCBB1RdXJFhUJt4wqOOOlEyOSbxYojUOdpDWSFDvNxWr15Nffr0KWDnh2e5bQ56PmkyRP6tra1lKRtBbXDxey7J0GtgugAXZZqT1VU9UG4QGboi6XIwSQvfIM0Q2I4bNy4TGnRQP5eDd5J5l1OvOJ7NJRnyxNcHJ0+ampoahZuu5TAJwKS5\/vrrCRolEsxvyDU0NNDgwYO7vPHZZIIstIK6ujr1HJuVBw8eLJil+J5N03nz5ikz+cknn6SJEyeqZ9j0Mic3EzvnvX\/\/fmUO+iXdxNXNW3M5wcwnaLKb5enl6Kagng\/KB54m3mH74vHHH\/fEF0sMQZqhWZZptvq1Q3954O9XXnmFRowYQV5mcpiXr75UgXy9zGi\/PoS811jdsmVL0fLHm2++STfeeGNhjOsEtmvXLiWLdvj1j1lHHrdjxoyhGTNmFIYCj6FS\/RmEfxwkFlceuSZDJrNjx451WVfSiZIHn25W6wMIYGMQ8O8LFixQn5lQ8TyTJWR5sIHgRo8eXVS2PqCxZnjzzTcXSBdrnLqGsXv37iLTa9iwYSqvnTt3epr\/pmYXxlwKo2Hp5TDhw2wCSfNEwksH7ePfmURMPEyNyqsvwuCrt3nr1q0KP37ZTJ48uagfS7XDj2S9yMv2RWLK8UtUN62D+tAPH16D1l\/APD69yNCvf7hOjJlXf\/G8wnj1wonnFo9fP\/zjIrG48qkKMgRReXUgk4qXeej3NgXBLV26tEhTRGeYGhxPmiAyxLPsAFi2bFnR5DWf5TJ0TZIHAk80JiV8b35XysTxIkMvbfIHP\/hBEdnDkVCKrMwJP3Xq1NB9EQZfLzKEVm06gbwITG+HWc9ShGdLhuZLFO3Sv2OtrVQfmmPVq+xSL0FdM8RY9iJkfezA2YbEWmDQC5bHEeaWrtFn2QnHcyjXZMhvKNa+zDcId3AQGeoa2zPPPENTpkxRC8hsrvLbGvmZZBFEhjD1eBI2NTXRtGnTCpqfaWJw\/b08wV5kZk6U+++\/v4iIdDxKTWg9b24fTxLOw1waMJcQWPthkgnTF2HwNScrk41eHjQmJgW\/dkyaNKnI418KH1sz2etlxPWD1bF3794uVoRZLl7EeiRCqZdYKc3Qr3+OHz9eWJLwqpMfGfr1px\/+5UR6xKUJmvnkkgzNwWm+5U0QbMiQBx2IqH\/\/\/gUT2Rys11xzTSgzGWTIA+bUqVMq7IVDbsKYruVqhsDEzwngRYZ+ISZBpmDYvgiLbyntl0kVffijH\/2I7r33Xs81QMZCtyaCtD8bB0olaIa8FMOhSkFrsmEcKjr+fmFdSRGdTb65JEP9bYvQmqAOtSFD3Qz2chgAbHQwT3aWYXI0NUn97a4vWOuODZPUzYEaROpBJo35vFcoiFfdvNbamBzNl4FphpltMOvoZwb64VtqTcvPJORlBl3LslnbBDn6xQ3ahNagXH092ctEtVkzNGNUvfoDL9UomiH3z4kTJ5RTDxghbpLHpUnopeZWEP5+sZo2xJWETG7I0ATHjIkzTU7dWWJDhrq2YMYv6qYAtA4Qg7kojgHFpqRpgiFvfmua9fZbi7QJ+jXXaWzf4ro3k3E1vc+6jG62B2mGCPIO2xel8A0iVz\/PKAeb+7UD7dbLLeVN1seeiZ1JnnF5k\/WAfX2MoC9svcl+a4b6WjGsFaTz588rcuSXHcjWXBfnSA19fgThnwSpRc0zdTLERJg7dy6tW7eusPuCB1AlLLJGBVqeEwQEgWwjkCoZ8hts4MCBhXUxvIXq6+vVWwehD3BOZHE9IdvdKLUTBASBchFIlQxBfDBrQIasGUIrPHr0qIqbY5UaXtUsepvKBVueFwQEgewikBoZwjyeOXMmPffcc0r7AxlyUOb27dsV+bHm2NLS0iWouLa2lvBPkiAgCOQXgSNHjhD+uUipkSE0wD179hB2g\/CaIXvwWBP0I0OQIExnxAtKEgQEgfwigFhLeNxdEGIqZAivHPb3IkhZd6DYaoYgQXjPXIGUp6F300030Zw5cwTLmDpV8IwJSCJiLLEjC6SYdkqFDL1CNfQwk6A1QyZDVyCl3SlJlidYxouu4Bkfnq6xTIUMdbjM0Bobb7JrkOLrbvc5YckBcY7btm1zYoq4RyDeGgie8eHpep47J0NAGRRn6Bqk+LpbchIEBAE\/BFzP89TJMMpQcA1SlDrLM4KAIBAOAdfzXMgwXH+JtCCQKALXDe5Fh1rP0fjRA+ml3ScSLStrmQsZWvSIa5AsqigigoCKgy03FhYkuHD+xxWan5\/1Yq5QDYohdD3PRTPM1XCTxrhCQGJhg5EPiiEUMgzGUAVbI85QQmsswBIRJwhILGxp2G1iCF3Pc9EMnUwdKTRvCLieyFnH0wYfG5kk2ylkmCS6knfVIOB6ImcdaBt8bGSSbKeQYZLoSt5Vg4DriZx1oG3wsZFJsp1ChkmiK3lXDQKuJ3KcQPOBKXyaVBx52+BjIxNHXfzyEDJMEl3Ju2oQcD2R4wRayDBONGPOK08DLWZoJLuMIOA1Rrv1raXufesyUsPialx4u40uvl18biBvi+V7T3D2KE6bQsKJUzh9ClfZ6vev2DbOZg7byNiWF0VONMMoqMkzgoCBgNdE7nXbXOp1259nEqtzL\/6Qzr24rlA3EB1fucGXbOGO8K1bt9KKFSvo+eefp4kTJ9KWLVsKdxeFaZgN0dnIhCkzrKyQYVjERF4Q8ECg0jVDaIWsAZpmMl\/HgcOZcT1HlGRDdDYyUcq2fUbI0BYpkRMESiDgeiKX2zmlyNC8GzlKWTb42MhEKdv2GSFDW6REThDIMRkGmckwj3EOJrTDKJe12RCdjUySg1DIMEl0Je+qQcD1RI4DaNOBsnPnTnUUP1\/Q5nXnuW25NvjYyNiWF0VOyDAKavKMIGAg4HoiZ71DbPCxkUmynUKGSaIreVcNAq4nctaBtsHHRibJdgoZJomu5F01CLieyFkH2gYfG5kk2ylkmCS6knfVIOB6ImcdaBt8bGSSbKeQYZLoSt5Vg4DriRwn0LIdL040Y84rTwMtZmgku4wgkKcxKmSYkUHlVY08DbQMwyxVKwOBPIxRv73JR48eVTtPEFozc+ZMevTRR6m5uTkUWjb42MiEKjSksJjJIQETcUHA9oU95CPdaWjv7pkE7HD7BXrrzIVC3fyCriGAwxpwUMPKlStp0KBB6u+wyYbobGTClhtGXsgwDFoiKwj4IOA1kb9281X09YbemcTsr\/e003f3ni3UzW873q5du2jhwoW0atUqmjdvnuxAcd2brt8Yrtsv5WcfAa8xWkmaYam9yfjtwIEDNGrUKEWKYU1k9J7NHLaRSXIkiGaYJLqSd9Ug4Hoilwu0n5mMfchYK7zrrrvozJkzkUxkIUOtd9g7VVfXedBlW1sbNTY2qrcNEi\/cdnR0eB4cWekDrdyBKs9nH4E8jFEvBwrIMA7vsg0+NjJJjoRUNEMsvCLBI8XA8uZv\/FZfX6\/IcfLkyYUDJnVV3DVISXaA5J0PBPI8RsvxInPv2uBjI5PkaEmFDM0G6AS4dOlSYtc9HyLZ1NRUdEyQa5CS7ADJOx8I5HWMwnyeMWMG7dixI\/LBrmImlxjjvFi7bNkyWr16NfEtXKbWGOatko8pJa2oVATySoZx9YcNPjYycdXHK5\/UNUN9oRYVWr58ObEmGESG69evp23bttGRI8UX2SQJkOQtCNgg4Hoi29TRpUwQPrW1tcrjvHjxYpo\/fz7t3bs39eqmSobm7VrmwmwQGQIdEOITTzyROlBSoCBQCoGgyV7t6AXhM3v2bJozZ46CKfdkiHVCRLKD+XXnCExm2zVDaJF4Y4hmWO1TK3vtD5rs2auxf410JQVB10uWLKGNGzfStdde6+ng5JxKeZ2D8IFmiGsFQIi5JkPdNDYDNsWbXEnTROrqh0DQZK8k5HRSO3bsGM2dO5fWrVtXkgzZ+dm7d2\/atGlTl3tSbPCxkUkSx1TMZI5f0huixxpKnGGSXSx5p4GA10S+bnAvGlrXK43iQ5dxuO0cHWo9V\/Sc3x0oiA\/GfN23bx9NmDCBTp8+TfwdQuKQEBWC9XwQpxkNgt9tiM5GJnRDQzyQChmGqI+nqGuQyq2\/PJ9\/BLzG6ML5H6eH5n88k43\/9trf0qq1vy3UzW8HiqkZIswGmh\/MZ935iYz8QuOEDGMcAkKGMYIpWSWCQKVrhn57k\/3M5OPHjxeFxQkZJjKsumYqZJgS0FJMZAQqfYwKGRKJmRx5+MuDgsAlBCqdDG3N5ClTpqiIENEMHY3+Sh9ojmCTYlNEIA9j1OugBl4bhNOEHShChikOLLOoPAw0h\/Blougb+t9Cr596ORN1SaISMkZLo2qDj41MEn3HeYqZnCS6krdCAET4zf+8hV47+TItaZ6aS1RcT+Ssg2qDj41Mku0UMkwSXcm7iAzx4YW3nqanW75Dx985nCt0XE\/krINpg4+NTJLtFDJMEl3Ju4tm+MkBt6jv\/sevpubKbHY9kbM+1GzwsZFJsp1ChkmiK3kXkeEDL3xaaYSPjt1C1\/QaSq+ffJl+\/tZTFU+K40cPpC\/86a1068SHnO2rzfpQsyE6G5kk2ylkmCS6krcnGWIN8c4hd9MNA26h4+cO0wutT6n\/K9HBgi13+57\/HJ17fyi9dfbuXJBh2IMazGs99u\/f3+WuFBuis5FJckoJGSaJruTtSYYMyx1D7qIFo75fQOmL266la64cWlHriUyGf\/fMxdxohmEPatDvU+YteXv27Ck6GduG6GxkkpxSQoZJoit5lyRDHZ6fTDqmSBBkCAfL0y2rKwI9mMhNGz9D3\/heO82YtaxIM0Rbrr5yaCbb8bt3Dnd56UQ9qIEvduOG6sfy8Xc2RGcjkySYQoZJoit5W5MhTGesI36y\/3iCxghCRAJBYm0xq95nJsM\/W3icFj3ynSIyvKu+ke6q\/6tMjgLzhWO7A6XUQQ1oKDRDPv9w8+bNhbbbEJ2NTJJgChkmia7kbU2GOlR\/e8evCx+hXa35zVdUSE4W0z1fGEaPrWigiff+h7rmVj+YtJI0wzj2JpdzuCv6VsjQYoS7BsmiiiJSAgEOumZvsg1YvHbIxIjPIESYndBqsuJswTFdIMTZDx3vQoY27cyKTLlk6KcRipkccw8LGcYMaMrZRSFDriJIEKbm8XcO0R1D7lZfw2yGtpiFBK1w\/OirK54Mbc1kr4MasH954cKFtGrVqqIrPfT+sZnDNjJJ9rmYyUmiK3lHMpNLwcYeaGztQ+K1Raw3pm1Kw5P80w2fUYek7j80oKI1Q2AZ9aAG3G00cuTIom4z71m2ITobmSSnlJBhkuhK3rGTITIEId4xuFNL5B0tDDXCc9JIMI1hIj\/57EFFhq4nchptLqcMG3xsZMqpQ9CzQoZBCMnvZSNQjpkcVDjyRoJmiJjFMOuSQXn7\/c6xhfrR+a4nctS2pPWcDT42MknWV8gwSXQl70Q0Qz9Y2dnywltPKZEkYhWZCKERPvjwnkJVeCLjXm+c+yepGAFcBRp0QbyQocWocQ2SRRVFpAQCSWqGerEoB84W3XSO+9gwdpjcNPG5ohbzZMdYleSNAO48xyVSfveeu57nohnKyE0cgbTIUG8IvNBYWwQ5wrGCAyGQygnJwTrh+DED6cktB+ml3Se64AZCxL84E9Ylr6vrRaeaHlHteKE13nhLhCotGPU9pUWXg41Nm0GCfkSI54UMLVB0DZJFFUUkA5qhVxWwjghS5ISdLDCjQSxhdrX4mcdhO37IR7rTuNoaGtK7Gw39SHfC56G9uxeyOdx+gXa2ddCPW87TW2cuKCcNSPjAdzcmFnzOHnrXx6q5nueiGYYdzSIfGgEXmqFZSd4NAhOat8iFmfxNG26joYOvos\/P+kWXy9dLAcLkN672CvrSx64siILoQHz4H+nwmQsFcgRZIuG3o58cTGNuH0PNK5YnRoYoi18aYTAJPRACHhAytEDUNUgWVRSRjGqGftXiMxXhfQ5KvP94ysxfeJrH+vNe5MeEt\/NIB+088h7hf\/7Oq2zk8aX6njSuroY+OnE41TY0KDK8qucs6lOzu0CiIFBOb7VfvESs7dr3mkxQO10Tout5Lpph0AiR38tGgM2wNMJebCsbpk6sFZpOEy6LCfBLH+upTGCT\/H78xju21eoiBzP5q\/fcqsjw1VMz6LVTLxO0TJSJxFpk2AL8yPi991fQmXdH0+VXPKyIF8TNZelleD2vk7PSbNsvFmnC\/AEaMWu+ep5Chlrke0dHh4ri10+7AFhhQcL6DtKh1nNhx4jIJ4BAGOJJoHjfLBGKE3RkGO8yefDhX3fRCkESX7\/5qoL5qzQ\/bb0vjraADO\/\/3OfplR98reRVCTph6WuQJpFhrTIotZ2ZR\/f9py\/TgF4\/pZeP\/nUXcS9y9Crz8k9cQ5f16UGX9e2p\/tdTtz496VDrWbpu8FV08XTn+ugHp9+lvlePoelLXiR4ntNOzjVDHAxZX19PjY2NNHnyZOK9j83NzQUs\/MgQAxX7QocO7kW3jh6o1nSYCE0gmRgPt56lX37oCXxp1++UmJdnMO2OyHN5WSVD3eMMZ8qaV77SxaNqhtKYZjAm8eY33ik4POLuRzhPls++R5Fhmpo1r\/PC0WSzD5zn4j1fuK4wDzHnXtr9OzVHed5BDt8fbj2n5i0Sz9mxDSMUGd49Z1N1kqF+ECSfktvU1ERr1qzxJMMTx\/5dgQvQx4+5+kNgz9KhtnP00q7OcAd+43AGOugIU2DSRKdw50BWiDLuqdyZH5NhWlvlwrYCpPjgqO+r+ET9nD\/dg7zyf\/6rWseb\/rErldkILfDHb5ynckxgm3q6IkPUjQnR70WhEyDmIpQLzC0EpOPvsEpGWAvQBr8wMk41Q\/P8M\/7c0tLieWT4ym\/9FX3jq70VgQHsJ589pICPmpDP0LpeilSRoF0i8WfOl7VKvOUuaZjnFOkebsP\/Yo6X6oOskyHXnQ9jxeRf8qup9Nk\/6qbOKvyHb+2msRc61\/2gBf7qQydI1HEX5jmXZIh6er0oMG\/u\/sIwFfLDygT2Z5czF1FWVZOhqQmWIsOn1s+gAT1fpoO\/PUWb\/v6NghbY1v2awthq6xbfJn1W3RFkO7Su0\/zGv8FXvU2De7+tymxt71v094W3j6jvoWFCU0X6l599oIJmkfDd+VP9FXnyIMKe2kpPuMzJTD37n1QTCemTA8bTuF5\/QXe9NqlkU7v1Cxew3L1vaXnuD1MO3\/s9e82V19H9\/e6jke09qdsNm+mWic\/Suz9+jZp6fpa+92afLl5gLsOrYRd\/f4Rs2gQ5PeEZfHfh7Ta6644aWjz1Qdr\/9H2pmslme\/hFcbrnv9L1\/+V\/U98RbygT2C8APeyYRrA6yDBoy17YfMPIV4xm+OjYn1C\/Hs007OxXCARYd+F40f+lGq2TJJOn\/t2RDwm1rfslMm3r1kmy+ncDOi5X3w3o6E4DOy5XxDiq5izdeEU7nf9\/AwpVOH9qQNHnMB0C2Z5\/cLLoefNz2Pxs5ZMsZ9Cnd9KfnOzWhYRKERPX24vUdBLSiU3\/\/uLv26hbv7pC871IcWjvblR38TjVXThGDe+9qsYVXnbdev83Or7\/dqoZsIy+\/c4o+nXNp4pgNMvEZ7Mt\/Fn\/3qa9ekFTP\/oq\/eUVJxUZ\/vkfHlJloF3vHbq0L7qISPv6t1fHAs8gnyuGNVgND4z1dZ9+jtq3fp7ePPxR9cw\/DnqbXuz+fwP71KvN5kuktnaQyvNTAz6glnXzZM2w1Jrhdxc+QSf\/vo56dG+jn7c+pbYP6R4sAMleLtNjhkh\/\/Xf8zc\/qnrF3L3QOpI736+jdC4Ppzd933taG7xFu4JVQH6Say1vVdZcoq6Z7G+HCHQ4h2Hv8PwqPYsH9ZM376jOHIvTsf6pL1oOv+r3VIDWFWs\/2i\/Sc+RA02LgSlh1+uLIHDfpc55IGNB6\/dPFD7TqusvV+5zAUhKYgIQga\/cFjAOuAt312CF0x7jrVN3BaQItf0jw1zuoU8uoWoNl271tH0++4gh6+878qMrz3zGL1bLe+dQUSYxLWiR9\/d+JcrHFeLIG7Kc9aLayae6YOo6\/e9qbC4qur2uj0iQn0QL+Zam5gLIMQ+VxJEyh+aZR6CfBv0A4fum8S\/e2Sv6hOMgzjTf5G40rqd7y+sIOAO8B2axWbbdiPCfMU5IX7e\/kz\/84dirUjyHTeJHZIEXHN5Z0D7RMDOl3\/ZpiBSbL6ZCs1o\/S4LX1nAj+DyYm4LSb6sDFcicxmy0z5npABn\/iJ5RPhxXhbG\/43t7oBT31XB3LnAGj+u1ufHmod7CHtjMIRl\/1paseC+bUY2DXevoQO\/tNkStsBhaUchPbAYem1JsinkPN2R97qiMMxouxzruo1Qx4AfMKubZwhOgEXkGNblU5gTF7mwALxmUQHGd6bimPkkV479ZJGfl3XwcJP0c4ngmLAwmiyZn5m8KtNgKxX0KsZMAsnAfIKImkbTMohQ24PXjKXAo07NTs98Fh\/6Zhb3bDrw\/xOrzfv\/8V35qTHFaa24SU2WISVwe6X73\/527T7pzeqNcO0EsdXorygLYgconRD\/\/GFE4N4bvFxan5zk9uDOQrNcM7sOdT4w\/uqUzO06dxSbwx9zynywgI4ErQ9aHRMcnzBEC\/2R3lz2dQ1bZkgolWkoQXa8pKBH0l7kWlQmwr7a312Flw\/tpYmzLqBfr58p3Iu6clWs2ay8yI1kB0SfoPGZ5sw4R\/7VoOKHtAPatWf5y1qaWtlXAcXZIiXA2vI+pmNNrjq8xHkiGSeRl4qnytvPEOP\/Gy6kKEfSK7VZ5tBkFcZL7LSv7PRarEOV\/NH9YoMPzh9vgtUpnbLSwIsWEqri4q7rvl47S7hfDnWztV1pWmTIWvJfJ1BVHzN50zLDMoKEiss0Bxdz3On3mRboF2DZFtPkfNGgM1k7O3NSkwmaz8265g41AEpKUdKqXGTFhnqsYNxE6HtvHA9z4UMbXtK5CIjkDUyZPMYDZoy68XAdrk87w9k+J0\/Xk+\/eXFIYmuGOhGW0pIDgSpTQMjQAkDXIFlUUURKIJAlMuRb7VDdIMeA3iQc6uDivmYQ1VOzN9E\/\/+yDRDRTfjFEOasx7kHvep6LZhh3j0p+XRDg8wBdmsmms+SpZw+GMtldOVKSJsOoh9YmMcyFDC1QdQ2SRRVFpAQCaZMhHyCArZQIGsYeduwxxnplVDMQDgBoh2k7UtCWDV\/8Gf1y14lYNcMwoTNpDW7X81w0w7R6uorLSZMM2TECuHkPOP72C50J0y0utMMkyFAnQr8Da8PgEpeskKEFkq5BsqiiiDjWDHWnCM6r5COkWEss90QVNI+1Q\/2Yr6Q7nslwy459VucKBtUnq0SIerue56IZBo0e+b1sBNLQDMPcU1JOg\/j0ljQPWv35ghcpDjJkIsRpM2GDqcvBzPZZIUMLpFyDZFFFEUlRM9TXBLF9Don30KZh9qUddxgHGbIX3VUMoc0EcT3PRTO06SWRKQuBODVDzktfD+TKxbEuaNNQ3pWSlrm87cv\/Rv\/n1X+ObCbzOmpa+Nhg6CUjZGiBnGuQLKooIglqhnzQLgiQt4tBA+RgYRQdNlSm3A5jczmNe4ZBhht3\/Z06ti5MysKukjD1dT3PRTMM01siGwmBcjRDPPvYis7TWmDiwdzDYQ82O0ciVTbEQ\/Au4\/QkXBHAp7SEeNxaNAoZslkMrB58ZE+omErrisUsKGRoAahrkCyqKCIxaoZ8Nw0uFEJQcCcRHlJaYdYmN9YPcdiAzQ1yUQcJjhGzNcn1HSVZXh8UMzniaBAyjAhcRh4LoxmyxxP\/IzwGx2tNmfmL0DetpdV01g6TPGvQhgx1kxhtjxpcnhZuQoYRkRYyjAhcRh6zIUPI8K2EMPEwmTvP1CvvBsSkIUjj5j+Q4U9P\/He1buiV9NjBStMG9fa4nueyZpj0bJH8qRQZsnNk3\/Of065hzcaaoE3XMRmyZpjE2iHIcG+Pb9LyLY91qRIfgpF1T7ENlkKGFii5BsmiiiISYc1QX9\/C4\/AQgzjDXj7uEnwOs4FXGddQ4FTnOPcv864XLzIEfniJ5IEI0Yeu57lohi5nUpWUbWqG7CCBGYyjo2DaHW49V\/Yl5K7g5DU9kCEuQwIhxhVyw2T4b7V\/SYufeLLQxLBnMrrCJky5QoYWaLkGyaKKIhJCM+RYQWwLe2nXiYolQW4yYg7vGHK3ukzssd98JdbTbfzIkM3jLDuXwk4K1\/NcNMOwPSbyoRHQ9w1fN\/gqdZwWL\/Rn5RqA0I0yHuALx\/A1a4q4VQ+pnHVEJsP2hofpvm8+USiVQ46yEG9ZLnb8vJChBZKuQbKooohYaIa8hQ5EmMWDAuLqRJx7yLcw4grMcoKyeU1SJ8O0DqWICw\/bfFzPc9EMbXtK5CIjwJMXGYAE4zhOK3JlUniQzz3kospxqHiRob4lMYXmpFaEkKEF1K5BsqiiiFhohgiizpNZ59dkDrfBrhE4VWx3j3jlx2T4\/sT7afrXnlUieTSR0S7X81z2W11FAAALHUlEQVQ0Q6GxVBDQD1tIpUDHhfAaIpvMr596ibCGGHb9kMmwdtbXadz0\/6UOp0A4TZ4cJ7JmGGKwun5jhKiqiAoCRQjoJjPIMOweZtYyh39tporDzNJNg3F3tet5Lpph3D0q+QkCGgJ81BfiD5HCXkTPzzMZ8m12aRxim3ZHVgUZDh8+nFavXk11dXUK37a2NmpsbKQDBw6ozxs2bKCRI0dSR0cHrV27ljZv3lzUD65BSntQSHn5QQDmMhI0PF4\/DGMugwxnjvkzGvvIYhrwiZ\/kdr2watYMV65cqQbEokWLiImxpaVFfcZv9fX1ihwnT55MU6ZMocWLF1Nzc3NhRggZ5occqrUlbO6i\/WHMZZ0MoQ3mafudORZcz3MnZrJOgEuXLqWjR48qYhw7diwtX76cmpqaaM2aNUKG1cocOWw3O0LCkiHWHKdOuIlunPdd5TRp2viZXDpPqkYzNMc2zGKkZcuWKfN5+\/btivxMrZGf4zfG+vXradu2bXTkyJEcThdpUt4RgMn84Kjvq73LX9x2rVVzcXjsrWMGKjJEfOb40VcrR0reUm1trQqtgVU4f\/582rt3b+pNTF0zXLBgQcEURmt1TTCIDCEPQnziiUvbklJHTAoUBMpAgM1lhNjY7EzRyRBxmjjYIo9kOHv2bJozZ45CNjdkaDpLduzYoUxgJBDhtGnTCk4SlrXVDEGceGOIZljGbJRHnSKgrx3a7EwBGX7sU+\/QxK8\/o857zMr9L3GDCM1w0qRJihBzQ4Z+IGGdsKGhoYtzBCazrBnGPbQkvywjwOEyNjtTELTdUfsv9OXFnQ7FvJxd6NU\/VeFA0U1j3UsMQMSbnOVpK3VLCgGQ3OsnXw4Mwobcmx\/8Iy36m1dVVfK484Qxrgoy5DhCfWDpsYYSZ5jUlJN8s4oA70wJCrPBcWA7z\/2NkGEKHZm6AyVKm1y\/MaLUWZ4RBEohwKYyZPw8y3yWIdYWn\/mn91R2CLzOa3I9z4UM8zqypF2ZRgBxh3cOuVvtTAkiQ1wh8ItfDlIOlDx6kqvKTC53VLp+Y5Rbf3leEPBCQL9Zz+s0G9YMQYY\/erpPbj3JQoYh5oeQYQiwRLSiEMCaoF+IjX7z3qjPtNKh1rMVdXNg2I5wPc\/FTA7bYyIvCMSIQCmvMpMh7mQOew5ijFVMLSshQwuoXYNkUUUREQQiIQAyhDnspR0KGUaCNPJDohlGhk4eFATKR6BUiA2vKdruYy6\/Nm5zcK30CBm67X8pvcoRgFaIcw6RzFOwOfxGyDCdQSJkmA7OUoog4IsAa4cm6fHl9FgzrIYkmqFFL7sGyaKKIiIIREbAL8RGyDAypJEeFM0wEmzykCAQHwJ+jhJojFdfOTT0vSnx1SzdnFwrPUKG6fa3lCYIeCLgFW+I47uQwl4iVakQCxla9JxrkCyqKCKCQFkIeMUb4rsX3nqKnm5ZXVbelfKw63kummGljBSpZ64RME1i3opnc+ZhXoARMrToSdcgWVRRRASBshAwYwr1rXivn+q8cznvyfU8F80w7yNM2lcRCJge5WrbfYJOEjK0GKquQbKooogIAmUjoDtRqi2sRsjQcvgIGVoCJWIVjYDuRKk2T7KQoeXQFTK0BErEKhoBfScKyPB37xwOvCOlohtsVN71PJc1wzyNJmlLRSOgrxtCS6wmT7JohpZD1\/Ubw7KaIiYIlI0A1g1Bgji8ASdcV4snWcjQcugIGVoCJWIVjwA0QiTEGVbLoa7caa7nuZjJFT99pAF5QoDXDXGydbWcViNkGGIEu35jhKiqiAoCZSEAjfCGAbeoPHCncjUl1\/NcNMNqGm3SVkEgwwgIGVp0jmuQLKooIoKAIFAmAq7nuWiGZXagPC4ICALxIFB1ZDh9+nSaO3curVu3jjZv3qxQ3LBhA40cOZI6Ojpo7dq1he+zsrAaT1dLLoKAIFAKgaoiw+HDh9Pq1atp4MCBBdJbuXIl1dfXU2NjI02ePJmmTJlCixcvpubm5gJurkGSISwICALJI+B6nqdqJoP4hg0bpsiQNUNohUePHqVFixbR2LFjafny5dTU1ERr1qwRMkxg\/NXW1tKkSZNo27ZtdOTIkQRKqK4sBc\/4+rtqyBDm8cyZM+m5555T2h\/IcPfu3UpT3L59uyI\/1hxbWloUOZpm8vr162nfvn3xoV+FOWHyQvMWLOPpfMEzHhyRC2M5f\/582rt3b3wZW+aUmmYIDXDPnj107NixwprhwYMHizRBPzJkkPDmkCQICAL5RQAkCOvQhdUSOxkyodXV1ake27FjB7W2tlJDQwPNmjWLdAeKrWbIbw2QoiRBQBDILwIgQRdECERjJ0OvbmJvsf4be46xfhW0ZpjfrpeWCQKCQFYQSIUM9caaoTU23uSsgCX1EAQEgfwi4JwMAW1QnGF+4ZeWCQKCQFYQSJ0Ms9JwqYcgIAgIAjoCQoYyHgQBQUAQSMuBIkgLAoKAIJB1BDKvGcLhgiDMmpoa2r9\/vwrPkWSHgA12ZihUW1ub2hp54MABu0KqRMoGy1KOwiqBybqZtnjqcu3t7V226loXaCGYaTLUt+dt3bpV7VYxd6dYtLEqRWyxgzcfCTt+\/ILeqxJArdG2WPIjXnvwqx1Dvf22eJrbczFWBw0alJhClGky5C18jz76qDq4QQ\/DEc2l9PSKip1g3BXXsFh67cEXMryEgC2eCxYsKGzWSAO\/TJMhwLj99tsLZhs+e51qkwZQlVZGVOwQ5oQkyxGXejwMll578PmoukobQ0nV1xZPvFT69etHI0aMoN69e1NVm8mmliJkaD88o2An+HrjGwZLrz34QobFuNriCblx48YVjvtL+kUtmqE9v1SUpO3blxsF+WnTpnkerltRDU+gsrZY6mad1yHGCVStIrO0xdNcI0z6ZZ1pMrRdW6jIEZFwpcNgh0GHgzTMQ3UTrmLFZG+LZak9+KIdRlszTHOZLNNkaOt1qphZlWJFbbFL+m2bYpMTK8oWS70Cohn6d4ctnpBbsmQJbdy4sXD26ZkzZxJbz840GQJO23ikxGZCBWfsh51+uriXNiOxhl073QZLIUP7yWKLpy6X9LjMPBnawyuSgoAgIAhER0DIMDp28qQgIAjkCAEhwxx1pjRFEBAEoiMgZBgdO3lSEBAEcoSAkGGOOjMrTUnTQ81BzvrVsiYONjJZwU7q4Q4BIUN32OemZHNDfVpkaLt3FfVbuHAhrVq1Su1xlyQIeCEgZCjjomwETDIsO0OLDHAyzIoVK2jLli1kE9Csn85jkb2IVCECQoZV2OlxNtk8D3HTpk0qez5QY968edSjRw91HiWuj8Vm+3Xr1qm7s70234O0JkyYoPIoFVeG+LOpU6fSww8\/XDh7UY+ZNJ81t4DFiYHklQ8EhAzz0Y9OW1HKTAYZgjDXrl1b2EXQp08ftfUPCReG79mzR52naJrXpTbme+1b1bdu6YHlKEffzWCjSToFVAp3goCQoRPY81VoEBmitXwkmE5w5mGyJoGZe4J11ExZEOmMGTMImqmXM8WFKZ+vXs5\/a4QM89\/HibcwDjJ8\/PHH1UnmMKX11NHR4XmSjkmGeIYJkZ\/XiZGJd\/v27Z5kmThIUkDmERAyzHwXZb+CcZAhzGQvgvNrfZCseRKPaIbZH0euayhk6LoHclB+XGRorhmWOlrMXDP0OjBUX0OUNcMcDLSEmyBkmDDA1ZI9e3J37NhBra2tRd5k2zVDyOneZD8TGXJB3mTzWfEmV8tIjN5OIcPo2MmTDhEIG2cYZFY7bIoUnREEhAwz0hFSjfAIyA6U8JjJE\/4I\/H+17Mo23mf4wgAAAABJRU5ErkJggg==","height":194,"width":323}}
%---
%[output:5a193e01]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUMAAADCCAYAAADTqKDOAAAAAXNSR0IArs4c6QAAFghJREFUeF7tnV9oXGd6xl\/ZtZs1Dg5ex\/GYblyR1QoKmzQRG5OLkMD2oohZQYoRrClYIClrOgh6IYwwQq6NMCLooiBUdlcSjSD4Qg0YtFqRiyYo9KIII7V4S6lXCybJRiMn2bRCxhvL2C7vMUcZjWfmfSXP6Hw68ztgrDnznHO+87zP9zvf+SOdhsOHDz8UJhzAARyocwcagGGdJ4DdxwEciBwAhgQBB3AAB4AhGcABHMCBRw4wMiQJOIADOAAMyQAO4AAOMDIkAziAAziw4QCnyYQBB3AABzhNJgM4gAM4ENhpcnt7u5w5c0YuXbok8\/Pz1AcHcAAHdtSBIE6TFYS5XE7W19elv78fGO5oBNgYDuCAOpA4DHt6euTUqVPy0UcfyauvvsrIkFziAA4k4kDiMIz32jpNzmQyov+Y0uHArTsizx0Qif\/XvSqcp5\/1+3h+qc+F8wp1xQ7F6ymer9vbymStp\/D7cvtVvM9b2X49aPP5vOi\/JKZdAUOFoJ4+v\/LKK0l4xDaf0IEv7oh8+Nke+fH3HsiV3+6RH373ofzmDw3y3HdEbv1R5Is7DdF3\/\/ife+X0Dx5EW1OdzlOtLqt6\/e7ogUfL6jyd9PNf\/dnDaD26Pl2ueIrXUzy\/cD2eXdS26Lq0vbrdeNLP8X7p9zrF7dPP+rMuq22MJ\/0cr0f\/\/2nzo+XqfVpcXJTBwcFEgLgrYKgQHB0dTcykNAX05Zdflq6urh318utnW+R2y9ty6He\/ktXv\/0Tur+Zl76HKo\/xCjf784P+WZd+Jlo1S3Pm3X8qeQ8flqRezm8rzzfUZuffpwsa8fc+3PKYpXEDXo+v3TE9nL5SV6XaL2+JZZ6z583\/\/B3ngbMdW1rubtHE29f6BQnGnp10Fw6RM2umi1HJ78YFlJ7380xezoiBR8Bx4\/e1twfDeJwubYLP63tnIpkN\/+\/NNdq3NXJS712c25sXbLufp16NtbggdOX+tbGl0u5VgadV0K+2w1rVbv08im4VeAcPdmpxttlsvObS2tsrs7OyOnYoAQ7tYwFCiy2B6BriTB2pgaGcTRRUdAIa2mcAQGNopkeRNcjUSUVkHgKEdDmCYfD8P5jS5UlySHj7bUUZRyQFgaOcDGAJDOyWMDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gGHKYfjuu+9Kc3OzrK+vR+9DnZqaeiwVjY2NMjw8LMePHy+r44VQdmcKWQEM7eoAwxTDcGhoSJqamqS3t1ey2ay0tbVJf3+\/zM\/Pb0qGAlOnjo4O6enpKakDhnZnClkBDO3qAMMUw1Aht7KyIn19fXLy5EkZHByU6elpGRkZ2UhGPCpcWlqKdO3t7dLd3S1jY2ObRpHA0O5MIStqBcP7q8tyODe9adfXZi7K3eszG\/PibZfzZysQOnL+WlmbdbtPZy9suwxbace2NxL4gkn385q8NzmG3NzcXAS\/YugV1sQzgkzapMAzFHzzgKFdImCY0pFh8UiwEgw1JgrEN954Q5aXl6PT6ps3b25KTwzD8fFxmZ2dlXw+b6cLRTAOAEO7FPUOw0wmI9rP9VJaLpeTxcVF27QqKxIdGW71NFn3XYE4MTFRZRtYXS0dAIa2u\/UOw87OTunq6oqMShUMdYc81wyLrxGWG0HGI0O97qhHDEaGducKSQEM7WrUOwx1ZNja2hoBMXUw9FwLLDUyVCPef\/\/9TTdauGZod6aQFcDQrk69w1AdSrqf1+Q0OS59qecM9XriwMCATE5ORneM4+uLBw8ejBb7+OOPozvLhVPSJtlRRlHJAWBo5wMYphyGdgR8CmDo8ylUFTC0KwMMgaGdkgCGz65GIirrADC0wwEMgaGdEmDo8ihkETC0qwMMgaGdEmDo8ihkETC0qwMMgaGdEmDo8ihkUTVgePf6r+TA629v7Obqe2eFX8cLuepbb1vS9wZqejd563aUXiJpk6q1H\/W6HmBoV56RISNDOyWMDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gCEwtFMCDF0ehSwChnZ1gGHKYVjqvcmlYhHr9Dvem2x3nN2mAIZ2xYBhimE4NDQkTU1N0tvbK9lsVtra2qS\/v1\/m5+c3JaNQd\/To0U0vmI+F\/Nl\/uzOFrACGdnWAYYphqKO9lZUV6evrk5MnT8rg4KBMT0\/LyMjIRjIaGxvl8uXLcvXqVZmamiqbGGBod6aQFcDQrg4wTCkMFXLDw8MyNzcXwS\/+vLS0FMExnhSS586dky+\/\/FJeeumlaDanyXbH2W0KYGhXDBimFIbFI8FKMNQR4+effy4dHR3S3t4u3d3dMjY2tmmkGI8Mx8fHZXZ2VvL5vJ0uFME4AAztUtQ7DDOZjGg\/10tpuVxOFhcXbdOqrKjJq0K3MjIcGBiQycnJCH7loBnDUPddgTgxMVFlG1hdLR0ohqFnW\/dX87L3UCaS6s+8N9nj2u7VdHZ2SldXV7QDqYKh7pD3mqHndDqGoY4i9YjByHB3hR4Y2vViZJiR1tbWCIipg+FW7iYfO3YsOk3u6emRU6dOyejoaMnT5KRMsqOMopIDwNDOR73DUB1K+kZpTU6T49KXes5QrycWnhrHo8jm5uZosStXrmy64xyCSXaUUQDDi\/J09sK2gwAMUw7DbSejaMGkjxjV2o96XQ8jQ7vywBAY2ikJYPjsaiSisg4AQzscwBAY2ikBhi6PQhYBQ7s6wBAY2ikBhi6PQhZtB4b3PlmQfSdaot3i0ZqQq1u9tiV9OaymN1CqZVPSJlVrP+p1PcDQrjwjQ0aGdkoYGbo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkUMgy\/uvwjt3VHzl8rq12b4Q81uI0sI0z6eWIeun7SCrK86QAwNC0SRoaMDO2UMDJ0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGwNBOCTB0eRSyCBja1QGGKYdhqfcmV4qF6nXSF8oXTkn\/mo4dZRSVHACGdj6AYYphODQ0JE1NTdLb2yvZbFba2tqkv79f5ufnSyajp6dHTp8+LTdu3ACGdt\/ZVQpgaJcLGKYYhjrKW1lZkb6+Pjl58qQMDg7K9PS0jIyMPJYM\/f7cuXPR\/LW1NWBo951dpYhh+M31GXnqxayr7Tv1djz+UIOrHDsiSvoMsCZ\/qKGxsVGGh4dlbm4ugl\/8eWlpKYJj8RSD89ixYxVPk8fHx2V2dlby+fyOFIeNVMcBYGj7WO8jw0wmIwpDPXvM5XKyuLhom1ZlRU1gWDwSrATD9vZ2eeutt+T8+fNy4cKFijDULxWIExMTVbaB1dXSAWBou1vvMOzs7JSurq7IqFTB0DsyVN3ly5fl6tWrMjU1JdYNFD3V1iMGI0O7c4WkAIZ2NVbfOyv3Pl2whSlV6MiwtbU1AmKqYKj18lwz1FGh7vj+\/fs3lbj4JkrS1xJSmr8d2y1gaFtd7zBUh5Lu5zU5TdYd2+rd5Big+j+P1tidZzcpgKFdLWCYYhjGcGtubpb19XUZHR2NToX1euLAwIBMTk5Gnwsn6zQ5qeGzHWUUlRwAhnY+gGHKYWhHwKdIevjsayWqcg4AQzsbwBAY2ikJ4FqCq5GIyjoADO1wAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAENgaKcEGLo8ClkEDO3qAMOUw1Dfg1z83uTiWPT09Mjp06c3Zl+5ckVGRkY2yXhVqN2ZQlYAQ7s6wDDFMBwaGpKmpibp7e2VbDYrbW1t0t\/fL\/Pz8xvJ0BfKnzt3Tt55551ovoKxlA4Y2p0pZAUwtKsDDFMMQx0VrqysSF9fnyj0BgcHZXp6+rFRX2FMyumAod2ZQlYAQ7s6wDClMGxsbJTh4WGZm5uL4Bd\/XlpaiuBYbmpvb5fu7m4ZGxuTqampDVkMw\/HxcZmdnZV8Pm+nC0UwDgBDuxT1DsNMJiPaz\/XsMZfLyeLiom1alRUNhw8ffljldT42EvTAsJImhqG2U4E4MTFR7Sazvho6AAxtc+sdhp2dndLV1RUZlSoYbnVkGOvX1tako6PjseTEMNRTbT1iMDK0O1dICmBoV6PeYagjw9bW1giIqYKhlt57zTC+TriwsFD2FJprhnZnClkBDO3q1DsM1aGk+3lNTpN1xzx3kz2nzyGYZEcZRSUHgKGdD2CYYhjGo8Pi5wx1JDgwMCCTk5Mb1wf279+\/KS3FzxomfcSwo4wCGF6UA6+\/LXsPZbYVBmCYchhuKxUlFgKG1XIymfXUy8gQGD5ZvpLu5zU7TX4yWzYvnbRJ1dyXelzXk8Lw3icLcu\/ThWjkFU86krq\/uiyHc9ObLF2buSh3r89szIu3Xc73ry7\/yF2SI+evldXqdoGh28qSwqT7OTB8svqxtMMBYGibxGkyp8l2SgK4y+RqJKKyDgBDOxzAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBjaKQGGLo9CFgFDuzrAEBhGKdEXzhe\/UrQwPkm\/KMaOMopKDgBDOx\/AEBi6XjYPDO3OFLICGNrVAYbAMBoVrqysSF9fn+gL5gcHB2V6elpGRkY2EgQM7c7kVWQyGWltbZXZ2VnJ5\/PexZ5IBwxt+4BhncOwsbFRhoeHZW5uLoJf\/HlpaSmCYzwBQ7szeRVJeAkM7eoAwzqHYfFIsBIMz176J\/nnf5mVxf9YtJOFoqwDOjLs6uyUX\/9657zc93yLPPViVu6v5mXvoYyrOvri+H0nWiLtkfXl6CXyq9\/\/ycayL\/z3L+T3\/7Mod\/\/mF5vWd3Dhl3L4y4WNeV8\/2yK3W759+Xzxxp\/\/4Geu9qjo07\/evK3CBXW72j7v\/hVv9JvrM9E+1vOk2Tzf0yk\/H\/g7WVzc+X6e6EvkvSNDNelg9oL877OPOgfT7nPg6IGH8sWdBjn9gwfyr79vkB9+99Fnnf\/cdySap9\/95g8N0fyfNj+Q\/\/qqQa78dk80\/8ffeyAffrZHbv3x0b7rvKMHRD78rCH6HC\/39395P5ofT1\/cefTdt58fbVO3oZNuxzvF29JltR3a7njbut14O9pO3T\/dTvxz3G5dRn9WrWri7wvb6G1PGnV\/8SfLsjR2dscu4RR6mCgMtSGea4aqUyDqPyYcwIH0OqDXsXfqWnaxi4nDcGhoSJqamqS3t1ey2ay0tbVJf3+\/zM\/Pp7fi7BkO4EBwDiQOw3h0WOk5w+Bco0E4gAOpcyAIGKbOVXYIB3Bg1zkADHddyWgwDuBALRwAhrVwlXXiAA7sOgeCh2F7e7vkcjnZv3+\/3LhxQzo6OnadyUk12ONd\/HjT8ePHo2YuLy9HN7Nu3ryZVLOD3K7Hy8KGq767u1vGxsZkamoqyH1KslFePwt1t2\/frunN1aBhWPhQ9szMTPTbKsW\/nZJkQUPettc7vZuvk\/7GT7mH3kPez51om9fLuC2xj0eOHJHR0VFgWFQkr5\/Fv5ShWT127FjNBkRBw1CPCmfOnJFLly5Fj9oUPobDyKUyBrbrHR4\/7utWvVQPT5w4IQpDRobb97Onp0daWlpqBr\/ilgUNQzXjzTff3Dht0888h+gbC23XO30IXicuR3zr81a8jMH5wQcfRFkFho\/n1eunHlSeeeYZeeGFF+TgwYNS16fJxaMUYOgDoaq24x3+lvZ3K17qwWRhYUFu3brFNcMycfX6qbrXXntt41JDrQ\/UjAz9fNlVSu\/RN94p1Z86dYprXCWq7PWy8LSOGyjlu4vXz+JrhLU+WAcNw61eq9lVtKpxY7finYZOr83wa5Cli+L1Mv6L7YVrWV9f5wBTZKvXTy80q9WVgoah965TtcxI03q83tX6aJsGT71eFu4rI8Pylff6qbqBgQGZnJyUa9euRU+TrK2t1ex6dtAwVDu9zyOlodNVex\/KeVf4l4JKjWZ41rD0HdBSz7sWegkM\/Qn2ZLO4\/9c6l8HD0G8vShzAARzYvgPAcPvesSQO4ECKHACGKSomu4IDOLB9B4Dh9r1jSRzAgRQ5AAxTVEx2BQdwYPsOAMPte8eSOIADKXLg\/wEIc5UnRXe30QAAAABJRU5ErkJggg==","height":194,"width":323}}
%---
%[output:2d2f712f]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 38.088732 seconds.\n","truncated":false}}
%---
%[output:86c5fa58]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7t3U9oHte9N\/BzjRMjYhuTG4pSeG\/QQmQdh9SUFNR1Cy6BIt6bTbSoFkX1TqQimC6CKGrQouBqUSSBunFeRKC8gXZtLbxwgyVoFqV4IWipJQq3mNrBJDW5l\/Nwxzx+rD8z0nOemTPzecAkskdnfvM5v5G+OjPz6N9efvnl\/w5eBAgQIECAAIGMBP5NgMlotpRKgAABAgQI9AQEGI1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQEGD1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQEGD1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQEGD1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQEGD1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQEGD1AgAABAgQIZCcgwGQ3ZQomQIAAAQIEBBg9QIAAAQIECGQnIMBkN2UKJkCAAAECBAQYPUCAAAECBAhkJyDAZDdlCiZAgAABAgQ6G2A2NjbC\/v5+WFhY0AUECBAgQIBAZgKdDDAxvLz++utha2tLgMmsYZVLgAABAgSiQKcCzJUrV8Li4mL46quverP\/+eefCzDOAwIECBAgkKFA5wLMa6+9Fj777LOwvLwc7t27J8Bk2LRKJkCAAAECnQowxXRPTEyUDjCvvvpqiH\/6X3t7eyH+8SJAgAABAgTqERBgjriJNwaX69evh8uXLz8zO2tra2F9fb2eGbNXAgQIECBAoFv3wFRdgYnBZWVlpXffTP+KixUYZw4BAgQIEKhXwArMESswRYCZm5sL29vb9c6UvRMgQIAAAQJPBQQYAcbpQIAAAQIEshMQYASY7JpWwQQIECBAoJMBpuy0u4RUVsp2BAgQIEBgtAICzBHeAsxom9HeCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFkBAUaAKdsrtiNAgAABAo0REGAEmMY0o0IIECBAgEBZAQFGgCnbK7YjQIAAAQKNERBgBJjGNKNCCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFkBAUaAKdsrtiNAgAABAo0REGAEmMY0o0IIECBAgEBZAQFGgCnbK7YjQIAAAQKNERBgBJjGNKNCCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFkBAUaAKdsrtiNAgAABAo0REGAEmMY0o0IIECBAgEBZAQFGgCnbK7YjQIAAAQKNERBgBJjGNKNCCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFkBAUaAKdsrtiNAgAABAo0REGAEmMY0o0IIECBAgEBZAQFGgCnbK7YjQIAAAQKNERBgBJjGNKNCCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFkBAUaAKdsrtiNAgAABAo0REGAEmMY0o0IIECBAgEBZAQFGgCnbK7YjQIAAAQKNERBgBJjGNKNCCBAgQIBAWQEBRoAp2yu2I0CAAAECjREQYASYxjSjQggQIECAQFmBVgSYpaWlMDU11Tvmra2tsLCwcOjx9297\/\/79MD8\/H3Z3dw\/c\/vLly2FlZSXMzc2F7e3tsqa2I0CAAAECBBILZB9gpqenw+zsbFhdXe1RFf+\/ubn5HN21a9fC1atXw\/Xr18Pf\/\/73sLy8HB4+fBhmZmYEmMSNZngCBAgQIDBMgewDTFxRmZycfLqSsrGxEfb39w9chRncdvDjQVgrMMNsNWMRIECAAIHhCWQfYGJgia9iFWXw436qg1Zg7t27d+glpyLArK2thZ2dnadD7e3thfjHiwABAgQIEKhHoBUBpn\/FJa6qjI+PH3pZKF5yive0vPjii+HmzZvhxo0bh8oXAWZwgxho1tfX65kxeyVAgAABAgRCpwJMDDdvvvlm7x6YO3fuhKNWa2JvFAFmcXHxmRUXKzDOHAIECBAgUK9AKwJMmUtIExMTvZt2+y8Z9d8AfNBNv+6Bqbc57Z0AAQIECBwmkH2AGbxkdNhNvAKMk4AAAQIECLRHIPsAU+Ux6oMuIV24cOHQ94KxAtOeRnckBAgQINAugewDTJyOw97Irlh1uXXr1tObdeMKzeuvv96bRW9k165mdjQECBAg0B2BVgSYVNNlBSaVrHEJECBAgMDpBASYI\/wEmNM1l88mQIAAAQKpBAQYASZVbxmXAAECBAgkExBgBJhkzWVgAgQIECCQSkCAEWBS9ZZxCRAgQIBAMgEBRoBJ1lwGJkCAAAECqQQEGAEmVW8ZlwABAgQIJBMQYASYZM1lYAIECBAgkEpAgBFgUvWWcQkQIECAQDIBAUaASdZcBiZAgAABAqkEBBgBJlVvGZcAAQIECCQTEGAEmGTNZWACBAgQIJBKQIARYFL1lnEJECBAgEAyAQFGgEnWXAYmQIAAAQKpBAQYASZVbxmXAAECBAgkExBgBJhkzWVgAgQIECCQSkCAEWBS9ZZxCRAgQIBAMgEBRoBJ1lwGJkCAAAECqQQEGAEmVW8ZlwABAgQIJBMQYASYZM1lYAIECBAgkEpAgBFgUvWWcQkQIECAQDIBAUaASdZcBiZAgAABAqkEBBgBJlVvGZcAAQIECCQTEGAEmGTNZWACBAgQIJBKQIARYFL1lnEJECBAgEAyAQFGgEnWXAYmQIAAAQKpBAQYASZVbxmXAAECBAgkExBgBJhkzWVgAgQIECCQSkCAEWBS9ZZxCRAgQIBAMgEBRoBJ1lwGJkCAAAECqQQEGAEmVW8ZlwABAgQIJBMQYASYZM1lYAIECBAgkEpAgBFgUvWWcQkQIECAQDKBVgSYpaWlMDU11UPa2toKCwsLh4Jdu3YtvPvuu71\/f\/ToUbh+\/Xq4c+fOgdtfvnw5rKyshLm5ubC9vZ1sEgxMgAABAgQIVBPIPsBMT0+H2dnZsLq62jvy4v83Nzefk4jbxjDyySefhBs3boQYfCYnJ8P8\/HzY3d19bnsBploz2ZoAAQIECIxKIPsAMxhCNjY2wv7+\/oGrMHHb8fHxMDMzU8pXgCnFZCMCBAgQIDBygewDTAws8VWEksGPC9GJiYmwvLwc7t27d+Qlpv4ZEGBG3o92SIAAAQIESgm0IsD0r7gctspSBJg\/\/vGP4Tvf+U44f\/586Xtg1tbWws7OzlPQvb29EP94ESBAgAABAvUIdC7AXLx48emNu3G15sKFC8feAzM4NTHQrK+v1zNj9kqAAAECBAiEVgSYk15C6r8B+KCbfotLSIuLi8+suFiBceYQIECAAIF6BbIPMIOXjI66iXfw32KAee+998KHH3544KPU7oGptzntnQABAgQIHCaQfYCp8hh1fA+Yq1evPnMJqX\/1ZhBJgHHiECBAgACBZgpkH2Ai62FvZFfcuHvr1q3e+77EV\/8b2d2\/f\/\/Q+1\/itgJMM5tWVQQIECBAoBUBJtU0CjCpZI1LgAABAgROJyDAHOEnwJyuuXw2AQIECBBIJSDACDCpesu4BAgQIEAgmYAAI8Akay4DEyBAgACBVAICjACTqreMS4AAAQIEkgkIMAJMsuYyMAECBAgQSCUgwAgwqXrLuAQIECBAIJmAACPAJGsuAxMgQIAAgVQCAowAk6q3jEuAAAECBJIJCDACTLLmMjABAgQIEEglIMAIMKl6y7gECBAgQCCZgAAjwCRrLgMTIECAAIFUAgKMAJOqt4xLgAABAgSSCQgwAkyy5jIwAQIECBBIJSDACDCpesu4BAgQIEAgmYAAI8Akay4DEyBAgACBVAICjACTqreMS4AAAQIEkgkIMAJMsuYyMAECBAgQSCUgwAgwqXrLuAQIECBAIJmAACPAJGsuAxMgQIAAgVQCAowAk6q3jEuAAAECBJIJCDACTLLmMjABAgQIEEglIMAIMKl6y7gECBAgQCCZgAAjwCRrLgMTIECAAIFUAgKMAJOqt4xLgAABAgSSCQgwAkyy5jIwAQIECBBIJSDACDCpesu4BAgQIEAgmYAAI8Akay4DEyBAgACBVAICjACTqreMS4AAAQIEkgkIMAJMsuYyMAECBAgQSCUgwAgwqXrLuASOFfj66\/8IX375f8O5c\/8vnDnzl2O3twEBAgQKAQFGgHE2EBi5QAwu\/\/rX2+GFF26HBw92wqVLb\/SCzNmzt3t\/50WAAIHjBAQYAea4HvHvBIYu8OWX\/xm++OJXYWzsF+Hx45+Gc+c+DvHv4sdjYx8NfX8GJECgfQICjADTvq52RI0WiKsv8RVXXGJ4KV4xxMTw4lJSo6dPcQQaI9CKALO0tBSmpqZ6qFtbW2FhYeFY4Onp6TA7OxtWV1fD5ubmgdtfvnw5rKyshLm5ubC9vX3smDYg0HWB\/\/zyy\/DxuXNHMjx+\/P4zwWVw4zKrMP\/x9dfh7X\/969h9dX0+HD+BNgtkH2D6g0icqONCSdxmYmIiLC8vh1deeaUXUASYNre4YxuVwPuPH4cYYH7y0kvh9gsvHLrbYgUmBpl42ah4xeBS9mbeGGB2HjwIvxgbCx+NjY3qEO2HAIEGCWQfYOLqy+TkZJifnw+7u7thY2Mj7O\/vH7kKc+3atXD16tXeNFiBaVA3KiVbgbga8unDh6UDRbEK03\/vS7yc9NJLP+ndD1PmVezz6oULRwamMmPZhgCB\/ASyDzAxsMTXzMxM77+DHw9OyZUrV8L7778fbt261QsxZQLM2tpa2NnZeTrU3t5eiH+8CBAIoVgNiZeO4upLmVd8AunJk7d7Ky7FU0jx7+L9L1WeQoqrPj99\/DgIMWXUbUOgXQKtCDD9Ky5xRWZ8fPxpoBmcrvjv8RXvaTnuclNxD8zgGDHQrK+vt6sTHA2BEwjE8PL\/\/\/nP3me+cenSCUYIIa7GnObJo0\/\/+c\/w9pMnQsyJ9H0SgXwFOhVg4v0y77zzTvjggw\/CW2+9VTrALC4uPrPiYgUm34ZX+fAEYnj51aNH4f98\/XX4wcWL4S9nzgxv8IojxRAT6zhpiKq4O5sTINAAgVYEmOhY5hJSvLx09+7dcOPGjeAppAZ0nxKyFih70+6oDjKGmPi6evHiqHZpPwQI1CiQfYAZvGR02E288d6XuJJy\/vz557hv3rzZCzWDL49R19iZdt1ogfi00a+++KJ3z8txj02P6kCKe3Funz0rxIwK3X4I1CiQfYA5yWPU0dsKTI1dZ9dZCzT5EeaqT0NlPRGKJ9BxgewDTJy\/w97Irni\/l\/jE0eAKiwDT8c53+CcSKG7aje\/zUvaJoxPt6BSfVISYJq0OneJwfCoBAocItCLApJpdl5BSyRo3R4H+m3abfrNscYnL49U5dpqaCZQTEGCOcBJgyjWRrbohUNy0W\/cTR2W14z06McgIMWXFbEcgLwEBRoDJq2NVW4tA0544Kovg8eqyUrYjkJ+AACPA5Ne1Kh6pQO43xnq8eqTtYmcERiYgwAgwI2s2O8pP4CS\/JqBpR1ncePzXM2c8Xt20yVEPgVMICDACzCnax6e2WSCnm3aPm4c2BLHjjtG\/E+iagAAjwHSt5x1vSYF4E2y8fJTLTbvHHZbHq48T8u8E8hIQYASYvDpWtSMRyPWm3eNwihDjyaTjpPw7geYLCDACTPO7VIUjFSjeQ+UXY2Pho7Gxke57FDuL4eynjx97vHoU2PZBIKGAACPAJGwvQ+cm0ORfEzBMy+Lx6rZcHhumjbEI5CIgwAgwufSqOhMLdO1pHe8Rk7ihDE8gsYAAI8AkbjHD5yDQpieOynoXxxy3v3rxYtlPsx0BAg0REGAEmIa0ojLqFMjt1wQMy6q4ZHb77FkhZlioxiEwIgEBRoAZUavZTVMF4k27McDE394cf8t01165v9Nw1+bL8RIoBAQYAcbZ0GGBrty0e9wUe7z6OCH\/TqB5AgKMANO8rlTRSASKm3bjqktcfen6y+PVXe8Ax5+bgAAjwOTWs+odgkAXb9otw1Y8mdTVy2lljGxDoCkCAowA05ReVMcIBdr2awKGSefx6mFqGotAOgEBRoBJ111GbqRAW39NwDCxY4iJL49XD1PVWASGKyDACDDD7SijNVqg7b8mYFj4xU29\/\/7yy8Ma0jgECAxZQIARYIbcUoZrskARYN64dCn85cyZJpdaa20uI9XKb+cESgkIMAJMqUaxUXsEXB45ei67\/r447el0R9J2AQFGgGl7jzu+AQFv3HZ4S3hfHKcLgXwEBBgBJp9uVenQBLznyfOUfjfS0NrLQARGIiDACDAjaTQ7aZ6A+zyenZOu\/j6o5nWmigiUExBgBJhynWKr1gkUl0s+Pneu8+\/EW9zcHN\/ALnp4ESDQfAEBRoBpfpeqMJmA3wEUgl+pkKy9DEwgqYAAI8AkbTCDN18gXkp6+8mT0NX3PHEprfk9qkICBwkIMAKMM4NA2HnwIPz1zJnOvfOsR6Y1P4F8BQQYASbf7lX50ASKS0ldugfEI9NDax8DEahFQIARYGppPDttnkCXHq3227ib138qIlBVQIARYKr2jO1bLNCVd+n1yHSLm9ihdUZAgBFgOtPsDvR4gS5cVuni5bLjZ94WBPITaEWAWVpaClNTUz39ra2tsLCwcOBMXLlyJSwuLobz58\/3\/v3+\/fthfn4+7O7uHrj95cuXw8rKSpibmwvb29v5za6KCZxAoHhPlKsXLoTbL7xwghGa+ykemW7u3KiMQFWB7APM9PR0mJ2dDaurq71jL\/5\/c3PzGYuJiYmwvLwc7t271ws4xccPHz4MMzMzAkzVzrF9qwXa+mhxW4+r1c3o4AgcIpB9gImrL5OTk09XUjY2NsL+\/v6hqzD9DoOfO2hkBcZ502WB\/\/rHP8Lts2db82i1R6a73M2OvY0C2QeYGFjiq1hFGfz4qEkTYNrY0o5pWAJtepfeLtzbM6x5Nw6BXARaEWD6V1xiKBkfHz\/0slAxMcX9MHfv3j10taZYgVlbWws7OztP53Rvby\/EP14E2i7wqy++CHHl4o1Ll8JfzpzJ8nA9Mp3ltCmawLECnQwwxf0vUafMTbyDijHQrK+vH4trAwJtEMj90WqPTLehCx0DgecFWhFgqlxCKhte4pjFCkx8cql\/xcUKjFOpSwLFpaRfjI2Fj8bGsjr0nGvPClqxBGoQyD7ADF4yOuom3jJPHvXPgZt4a+hIu2ykQI7v0uuR6Ua2kqIIDE0g+wBT9jHqKBbDzYULF468bCTADK23DNQygdweQY7378QVmB9cvJjt\/TstayGHQ2CoAtkHmKhx2BvZFSsut27dCn\/4wx+eeRO7QvHRo0fh+vXr4c6dO8\/BWoEZaq8ZLHOB4kmej8+dC\/GXPjb51eY342uyu9oIjFKgFQEmFZgAk0rWuLkK5PBodU5BK9c+UDeBJggIMEfMggDThBZVQ9ME4qWkt588Cf\/+8stNKy14ZLpxU6IgAskEBBgBJllzGbi9AjsPHoS\/njnTuHfpLR6Zjpe42vZ7nNrbTY6MwMkEBBgB5mSd47M6LdDE3+jskelOt6SD76CAACPAdLDtHfIwBJr0aHXxyHQTV4WGYW0MAgSeFxBgBBjnBYETCzTlXXo9Mn3iKfSJBLIVEGAEmGybV+H1CzThlyR6ZLr+PlABgToEBBgBpo6+s88WCdQZIDwy3aJGcigEKgoIMAJMxZaxOYHnBep4l16PTOtEAt0WEGAEmG6fAY5+aAL\/9Y9\/hNtnz47s0WqPTA9t6gxEIEsBAUaAybJxFd08gVG+S28T7r1p3gyoiEC3BAQYAaZbHe9okwrEp4HiPTFvXLqU7BcoemQ66RQanEA2AgKMAJNNsyo0D4HUj1Z7ZDqPPlAlgdQCAowAk7rHjN8xgZTviFvnE08dm0aHS6DxAgKMANP4JlVgfgIp3qXXI9P59YGKCaQUEGAEmJT9ZewOCwzz0WqPTHe4kRw6gUMEBBgBxslBIInAMFdMPDKdZIoMSiBrAQFGgMm6gRXfbIFhPFrtkelmz7HqCNQlIMAIMHX1nv12ROA0l5I8Mt2RJnGYBE4gIMAIMCdoG59CoJrAzoMH4a9nzlR+l97i0tEPLl5M9r4y1Y7E1gQINEVAgBFgmtKL6mixwEkuJRWPTP\/kpZfCx+fOtVjHoREgcBIBAUaAOUnf+BwClQWqPFpdXDq6\/cILIQYYLwIECAwKCDACjLOCwMgEyr5L72numxnZwdgRAQK1CggwAkytDWjn3RIo80RRvHQUV2viyktcgfEiQIDAQQICjADjzCAwUoGjfh1AmYAz0mLtjACBxgoIMAJMY5tTYe0VOOgSkXfbbe98OzICKQQEGAEmRV8Zk8CRAsVKy+2zZ58+Wu2RaU1DgEAVAQFGgKnSL7YlMDSB\/ker46CfPnzYu+\/FI9NDIzYQgVYLCDACTKsb3ME1WyBeSnr7yZPem9R5ZLrZc6U6Ak0TEGAEmKb1pHo6JhDfpTe+3rh0qWNH7nAJEDiNgAAjwJymf3wugVMJeGT6VHw+mUCnBQQYAabTJ4CDr1cg3rgbXx+NjdVbiL0TIJCdgAAjwGTXtAomQIAAAQICjADjLCBAgAABAtkJdC7ALC0thampqd5EbW1thYWFhUMn7fLly2FlZSXMzc2F7e3t7CZXwQQIECBAoK0CnQow09PTYXZ2Nqyurvbms\/j\/zc3NA+dXgGlr2zsuAgQIEMhdoFMBJq6+TE5Ohvn5+bC7uxs2NjbC\/v7+oaswAkzu7a1+AgQIEGirQKcCTAws8TUzM9P77+DHg5NcBJi1tbWws7Pz9J\/39vZC\/ONFgAABAgQI1CPQuQDTv+ISV2TGx8efBprDAszg38dAs76+Xs+M2SsBAgQIECAQBJgSAWZxcfGZFRcrMM4cAgQIECBQr0DnAsxJLiF5CqneJrV3AgQIECAwKNCpADN4ychNvE4IAgQIECCQp0CnAozHqPNsUlUTIECAAIFOr8DEg\/dGdk4CAgQIECCQv0CnVmCqTpf3gakqZnsCBAgQIDAaAQHmCGcBZjRNaC8ECBAgQKCqgAAjwFTtGdsTIECAAIHaBQQYAab2JlQAAQIECBCoKiDACDBVe8b2BAgQIECgdgEBRoCpvQkVQIAAAQIEqgoIMAJM1Z6xPQECBAgQqF1AgBFgam9CBRAgQIAAgaoCAowAU7VnbE+AAAECBGoXEGAEmNqbUAEECBAgQKCqgAAjwFTtGdsTIECAAIHaBQQYAab2JlQAAQIECBCoKiDAlAgwi4uLYXx8PPz+978Pe3t7VY1Hvv2rr74avve972VRb061xonMqV61pjv12LLN7etBuhmrb2QBpkSAWVtbCz\/60Y\/C3Nxc2N7erm+2Su45p9\/hlFOtkT+netVa8oQ5wWZsT4BW8lPYloSyWRBgBJhaT4OcvlgJMOlaRR+wLQRy6oWcak3XYfWNLMCUCDC\/+93vwve\/\/\/0QV2J2dnbqm62Se47L29evX8+i3pxqLZaM2ZZsxAqb6YMKWBU3ZVsRrMLmOdnG2x9yuAWiAr8VmKOwiuaMKduLAAECBAjkKhB\/AF9fX8+1\/APrtgJzzHTGEBP\/eBEgQIAAgVwFrMDkOnPqJkCAAAECBFolYAWmVdPpYAgQIECAQDcEBJhuzLOjJECAAAECrRIQYFo1nQ6GAAECBAh0Q0CA6cY8O0oCBAgQINAqAQGmVdPpYAgQIECAQDcEBJhuzLOjJECAAAECrRIQYE4wnRsbG2F\/fz8sLCyc4LOH+ylXrlwJ8ZdNnj9\/Pjx69Kj3Drx37tw5cCf928YNbt68GW7cuDHcgo4ZrUq9ExMTYXl5OXzzm9\/sjbq1tTVS8yq19h927I\/4mpmZaazt0tJSmJqaelpfHb0waPb666\/3\/qruWoq6pqene7\/\/7MUXXwz3798P8\/PzYXd398A5jXNe1P\/VV1+FlZWVsLm5OdL5r1JvUVhxjt27d2+k51aVWpvwdatKvf3b1tULI228GncmwFTEL75Qjfqb6WFl9n+zPOob5+AXqmvXroUf\/vCHI\/9CW7beeLxx2wsXLvS+cXzjG9\/oBbVPP\/10ZKGrSq3F\/ETXd999N\/z5z38eeYApW2+s8erVq0\/Dbl290G9W1POtb33rmdoqnp5D27z\/fPn1r3\/dC9KHfZOPYXBycvJpwIkfv\/nmm0f+MDG0Qv93oCr19u+7CLKj\/HpWpdZi24cPH\/bOpxgOZmdnw+rq6sgCYpV6i7BVfJ2qoxeG3VtNHk+AKTk7RWPGRB1fn3\/++Uh\/YjmozMGTJZ7c7733Xvjwww+fW4UZ3Hbw45IMp9qsar0\/+9nPwm9+85uRfaHqP7gqtfb\/NPvzn\/88vPLKK+Fvf\/vbSAPMSeot6q6jFwZXX+LH8RtU8c3i1q1bIwuqBzX14DfKGPK++93vHrkK079yM+pvsiepN857PMfiCtPdu3dH9vWsSq1HfU071RejCp9ctd7+ua8jcFU4tOw3FWBKTmE82V977bXw2WefHfnTWMnhhrLZ4Mlx1Mly0ApM\/0\/hQynomEGq1Du4SjCK+vr3UaXW4vPiT1vxNT4+3vvvKC8hnaTeJgSYwb6s65LGYH8dtEpV9nyp45vWSeqNK3Z\/+tOfQlz1GuUlpCq1Dq5ujfrrQNxflXoPWoHpX52ro\/4271OAqTi7TfkCG8se\/Omk+InqqFWL4hLYcdf0K7KU2rxKvcV2pxGLAAAEl0lEQVRPvHEl46233uqNP8pl7iq1xtqi\/fvvvx8++uij8OMf\/7iWANO\/+lamF\/qD16gveRT7PmjFpQn3mA2uuFRZCei\/9HnYPTOlTpgKG1WtNx7PO++8E375y1\/2+nbUAaZ\/Neso2xhgih8I6rpHqqpt\/717dVxKrtA22W8qwFScwlwDTPGTQbFUXMdPiVVCQXEvSRFaihvjPvnkk5FcWqhSa2yh+E0r2sabouu4ibdqvUXbF8513TjbtgATv+F++9vfHvm9ZVW+yUbzeKnzt7\/9bS0rylVqLe7RKfqzjvu1qtQ7+HWq7pXkit\/esttcgDlgyvqf0Bh8sqeuADN4J378xr69vf3MDW1HhZKDTqSU32hzqve0tRY\/zX7wwQe9p1RSuharPcWTZ8XKVJVeaEp4iXW06RJSXeGl6mWO+LUgrrj133M06hWYwZvID7s8N3gJqY6vv1UuITWh3uxSyCkKFmAq4tVxAh1W4uBlgqOWYkcdYA6quUq9Bx3LKC8tVKl18JHk4thHeZmuSr3FN7w6nkI7qC\/657VJN\/H2X5I77ibeup82GTxfjqq3\/5Hv\/vkY1eWOKrUOHkcd\/VGlXgGm4jfUU24uwFQEbFKAKS5dxP\/Gn6aO+sn\/oEtI8T0uRnVJpmAu+6jvoHMdl7zK1jrYQqlXYA5r2bL1jvpy3HGnWH+4zvEx6jouawyaVnnUt\/9z6\/h6VqXWwZti67gkU6Xegy4hNeUHhePOwxz\/XYCpOGt1nPBHlXjUm60VN8AVT8MUJ1d8bDK+6rjvoUq9g29kN+p6q9TaP0d1BZiy9R62YjTKm6QPCn113aR52PnVf74Mrqb1n1uHrWiMul\/L1lt3gIn7r1Jrf1\/X9cZwVeot7iuLx1lXvRW\/rWW7uQCT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtgACT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtgACT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtgACT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtgACT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtgACT7dQpnAABAgQIdFdAgOnu3DtyAgQIECCQrYAAk+3UKZwAAQIECHRXQIDp7tw7cgIECBAgkK2AAJPt1CmcAAECBAh0V0CA6e7cO3ICBAgQIJCtwP8AnmyDYbJuaGsAAAAASUVORK5CYII=","height":194,"width":323}}
%---

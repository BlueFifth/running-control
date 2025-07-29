%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88];
dq0 = [0,0,0,0];
X0 = [q0, dq0];
Sim.time = 0.4;
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
    [t, x, te, xe, ie] = ode113(@(t, X) halfleka_dynamics(t, X), tspan, X0, options); %[output:81daff19]

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
toc %[output:302a6b26]

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
plot(t_total,x_total(:,1)); %[output:849a3604]
hold on %[output:849a3604]
plot(t_total,x_total(:,2)); %[output:849a3604]
plot(t_total,x_total(:,3)); %[output:849a3604]
plot(t_total,x_total(:,4)); %[output:849a3604]
legend('x','y','th1','th2'); %[output:849a3604]
title("Generalised Coordinates"); %[output:849a3604]
xlabel('time (s)'); %[output:849a3604]
hold off %[output:849a3604]

% display derivative state of all variables
plot(t_total,x_total(:,5)); %[output:4355ce97]
hold on %[output:4355ce97]
plot(t_total,x_total(:,6)); %[output:4355ce97]
plot(t_total,x_total(:,7)); %[output:4355ce97]
plot(t_total,x_total(:,8)); %[output:4355ce97]
legend('dx','dy','dth1','dth2'); %[output:4355ce97]
title("Derivative of Generalised Coordinates"); %[output:4355ce97]
xlabel('time (s)'); %[output:4355ce97]
hold off %[output:4355ce97]
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
plot(t_total, contact_total) %[output:12936223]


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
if Animate ==1 %[output:group:87994b13]
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
    fig = figure; %[output:614fb6f9]
    
    ax = axes('Parent',fig); %[output:614fb6f9]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:614fb6f9]
    u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:614fb6f9]
    u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:614fb6f9]
    l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:614fb6f9]
    l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:614fb6f9]
    f = animatedline(ax,'Color','r','LineWidth',0.5); %[output:614fb6f9]
    
    axes(ax); %[output:614fb6f9]
    
    axis equal; %[output:614fb6f9]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:614fb6f9]
    
    hold on; %[output:614fb6f9]
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
        addpoints(b,x_sim(i),y_sim(i)); %[output:614fb6f9]
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
    toc %[output:23a4215c]
    hold off; %[output:614fb6f9]
    clear fps b f l_l l_r u_l u_r ax fig l1 l2 l3 l4 l5
end %[output:group:87994b13]

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
        if abs(foot(4))>1e-2
            value = foot(2);
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
%[output:81daff19]
%   data: {"dataType":"text","outputData":{"text":"Time = 0.3990","truncated":false}}
%---
%[output:302a6b26]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 13.557568 seconds.\n","truncated":false}}
%---
%[output:849a3604]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMUAAAB3CAYAAACzOoSOAAAAAXNSR0IArs4c6QAAFANJREFUeF7tXQ1sVtUZfteyDrsiDCtCoQJbSDHqCBA01jnRZC5hxEhYlBEnLPwIY5ixMGWMwGSsUSTDQBh\/mslmnJI5HGs0yxLUMcaQAFucQVIXx09bqMCoVmSVsuU55b19v8P9\/b5z+917v3OSpv36nXvue973fc77c\/4+M3DgwP+RLZYDlgMOBz5jQWG1wXIglwOJAsXIkSNpzZo1VFNT41B55MgRmjlzZtHldv\/999OCBQto7969tHnzZkUnyuLFi+n9998viD7ut197TzzxBN155529ypdbb72VVq1aRc3NzUoGzz33HA0dOpSWLVtG+\/bty7vPmzZtol27dtH27dvzbiPOBxMDCl0A6DQrIpSu2MCQoFiyZIlRmQSBAsqIOhs2bFCKxLz68MMPjYDSqzNuMim04wD3bbfd5vSl0PbieD4xoAg7CskRs6WlRSkFCo\/cnZ2dNGLECOro6HBGNN0CvfnmmwTFZkVHO7BOp0+fpm3bttHChQupqqpKtcuWys9SDBo0SI2o\/Ay3j+fR1vTp01VbkiZWODxz9uxZ9f2FCxeuUPKwYJR8ke+Rg0tFRYV6zwsvvEDr1693aEMf6+rqVF9fffVVZRFRF3y5+uqrXS3FLbfcovq1f\/9+GjNmjKov+y3pgUwA6Ouuu87hBdPR2NiY4x14taH3KQ4wcJuJAEXYEYkVDEJ96623HNP++OOPK8ZWV1cr5qOwqwPll4BjYaKNU6dOqXoAA7tBqHvy5EkFGhasrOvmPq1YscJxK6ZMmeKMhJIO3eXCMzz6cz1JBwsozMgq60BJpWs3YcKEHF7IuqykDHwePAAEuEjMK\/7ejY9u3w0fPpxmzJhBK1euVN3AgMFW7eGHH86xFEGyAb937NiR48bFCQi0nUhQ6CM7jxLz58\/P8WnB0H79+tHatWtp0aJFalSDMFkQ8IU3btyYw1AG4IEDB+jgwYM5CsPMlqM4j2gMIC9QYKTVRzNdofF5\/Pjx9Pzzz9ODDz7ojMB+7lMQKNyelYMHYgDprsgBCDzAaM+WQ7dKfjEFA4ZHdjc63ay6BMXRo0cDZaNboLgBkRhQ+CkFjyQw9xh9ZBDOLgl\/h88Y8dmdASikOyAZKl0FKDosgwQjFAWFlcYPFDwas3siQQywyAJXAi7DPffcYwQUblZWggIglMGxaVAwoNwsENyv1atX06OPPqpYANlIULCFZL4xn6T1kfyTrlWc4EiEpUAHvWIK+X\/dUjBjdFBJUOiWQjJTHxn1z1K5\/EDB2ScJKggWbphbUKkHyqDJK5sVFFPEYSk4sRHGUriBYtKkSQ4Q29racvrmZym8FJ3lwLFJ3FmrxIDCLaPC5pdHXhkPuMUUbpZCTyW6xRRsKaQScJwCy+QXU7Drxu+WPrx0z\/KNKXjA8Ms+SUXLJ6Zgxc4npnADxbhx45zBgOMWToqEjSmklYYnEDYRY8KCJAYU6IzbPIU+OgRln3T3CaAIyj4xKECDnsWBacf3fgquZ59YATDiemWfJE1+2ScWsj5PId+h061\/x9YGfZH8lJYQioci46kw2Sc3UHCsgMwa3ocfFMR7CMI5uwV3iAcLdou9sk94nt9lQvH92kgUKOLurG3fciAMBywownDJ1ikpDlhQlJS4bWfDcKBXQDFkyBDCjy2WA3FyoLW1lfBTaIkdFAADAixkJGyxHMiHA58ta6c+ZR8SfleWH1dN9Clrd5q6eKm\/+vuNfZ209CfbCgZG7KAAGLD0AlP9JlCcD1PT9szYsWNp9uzZJc2z2ppK+sot1XT7hGupbvgZJcJjzefV7+PNH9Oxlu6\/r6+pVL8x+FZWf42+PWejyhQWUnoNFEjDFUpsIR1N07MQMCbAMBtfagPJ9UMradp9w+mxBTcoEAAAv3nlGB1r\/pj27D\/tKUYefE3omQVFmtCSYVp1MPzmlaP04itHHesQ1HULiiAO2e9TwwE3MKzecDgy\/YkEhZw1lTOqJomNzKkMPZDFDN7tE6rpW\/cNp9qhlco1CgMGrwyTST0z4j5hacDy5cvVBh0s1sI6FSxLxtIBk8RmSMcjdcVm8HrYhbjULWljUs+MgAJraLBE2W3LqEliI2lShirbDF63MDkr5xZMm9QzY6CYOHGi2uSDRWBu7tMzzzxTktkUE9g0KXAT9BSrDS8+wJLiO8yHJSb7xDvKQBSvn29qalIbd7gjYCSA8eyzzxaLp6l9rwVFt+i8+DBr1iw1r4OSGFDAfYKlkPucQSDcKWn64Q+WWt7dBBItKPxBwfM6AEZiQCEDbd7kolsKE8SaULA0tmFB4Q8KPyuSj7yNxBR4sdxMIw8wswLNRyy5z2SBh\/opKYMHD458lpcfH0zyyBgovERvktjC1SudLbjxsKz\/ECrv33OSYtJ61tXeQpfac1es4mRA7MJDQiafw+0sKJIm5SLS46YMlXfMoco75haRKv9Xn9+9hc7v3ppTCW42TvbACR\/5HLtpQZFYcfc+YVmwFNiT3tDQQO+++y6NHj2ali5dGvkMXguK3te9xL4xCy7oyy+\/TJx8QQrfxhQbNhhJlSVWa2MmLAugMMEiaylMcDEjbVhQpDQla7NP8SHQgsKCIj7tSmnLFhQWFClV3fjItqCwoIhPu1LasgWFBUVKVTc+si0oLCji066UtuwGimH9yqm2qjyxPTre0UUnPupy6JO7Mf02pfl1KLUpWUzMjBo1yllG3lujHDbAuxWcHxS2XD\/082GrutbDMSz5luMt50nSKj9XXzealvz4qZy5nkXjPk8\/GN99L18Sy88PdNDagz38kECQAIlCeypBwYcXyLvbZEdOn3o3hwdSCaRCYiM7F1Z2PvSq1kVxuQ7OCfICB9oL+j6KgHqz7vmLtXTi42k5oEibpcAyD9zzh7379fX1pbHMg9e2fPDBB+oWG95wBFD8eut8GnzVa4F6xCfAoaI8BU5\/sOekuO5T4rh4jdQYddNcxo4dd4WlSGN\/4EXcfPPN9Pbbb6tdmVGLl6VI5HZUdA4dRsE9c3IXHjqyZWODOgv0T6+9QK\/84S85vGCFlYCIyqys1+8tFzRuPsKTmDNnDm3dujWvi+W9+JDI7ajoLK7KxcrHyZMnXwEKPkvWbkfNT+2yBArWE74nMApH\/CwFjhlN1HZU\/eopdBQnekydOtWe+xRF6h51swAKt6vEorImlYE2OqkfYpAFgUYVnun6lofdHLWgMK1ZKW7PgiLloNB1zwq0cDRaHlpQFK5FGWshC6BAyv6hhx4i3E+OGBRZSr6mWIpLJm30YDy17pO1FOYRmQVQQNnvvvtumjdvnicoOBjX7wFnjlpQmNet1LbopgyYuY+yhKW3O4\/5Jzn3hD3auEAel8ejYI92XV2d+huXxjc2NipLcvjwYSe9by1Fb0stRe9zA8WjC25QV2AltTy54XDOfRO6peCDC\/TFgdZ9SqpEE0ZXFiyFl\/sk\/w+2W1AkTPmSSk6WYwoLigKvck2q0sZNV1ZAgUO2ESecPHnSyT5ZUFhQ5IWfLIAir45rD9nskwkuZqQNCwo7eZcRVTbXDQuKFIICs5Vr1qxReWgUez+FOUCgJQuKFIJCHpiL49ZxpSu2Hco77+xNRvkDJQugCFrmoQ+smNDTl4GkOqaQt9ZwR+ztqKUNiqBlHrxzEwOp1z0WqduOyiLXL5rnjuB7eztqfsDIgqUIWuYhrQIfcrBx48acy11StR2VRc0m8I033nBMH3cELpXdjmoOFIOuqqVrr6rNr8FeeOqDT45T2yfHnTeFXebhpkPcSKq2o4Jo3UIEdaQX5JKZV7gpwwOjFtMDo36Y2D6+1PQUvdS0xhMUvHRcgsVLh8LokklrauQiSL+7zEwSm1gNiJkwNx6m3VLooNi1axfNmDGDVq5c6XkfXqoCbQTWvAyY9QNLhG32yQxasjCw8EF5Xss8+vbtm6NDHR0dtGzZslAxhem0tRFL4Sf6LAjUjGrn34rlYQrnKSwo8lf4ME9aUFhQhNGTkqpjQWFBUVIKH6azcgL00KFDYR7JZB2cGYs4w211hMmBw8YUKVAfVgYIvtQL5row59Xa2prDCguKEtQMAAM\/pV4ABh0Qqcw+bXpyBT33dAMdOliY6cftOEFF3p4TVNd+nx0OpM5S\/PL7U2ng7nWRJADlxsUkpgraA6jcrsRyA5sE13FxTRXTc6LjkkNaTl0BXAtQU9ILbid1oIClWPXTVXTw0MHg3rnUiHq3mxuYhlWVub671gV4eJ4VWt4YBPDcNqRCtZMPaIPAI8HHoGMwO2B0AWheTE3RQzx7P6iylvC3LP8881f18XMjLhCufDCxRcEG2gUqhw5ACWD5nQ5KCUaux8+GtZBuIOP\/AWBu1oytYhqs2I0D6+nuYdPoxmvqHTDIRYYQHUDC\/\/vi16toyfbpauFpIcWCohDu9cKzDBAoMayUtFYSaAwyCbAw4GJw7G3tdHqzt\/VT4rYBLAdo2o2ncXQfSn7XsAformHTHIV\/\/cSLBIvwztluqyALW5Gv3TSVvvmjr9IjP344OaDgc0BBMK97wt8mfb04hFBqbbI7iN86qCSIYLX8QAUQoQ5fDcyuH4NIvzI4iM9Qbqz6BSAw8r9z5q+068SLrkBwa8uknhmxFHLJLwiWqx1NEhvEWPt9fBxwAxOsEwOHYy2dAt3KMHj+1vop\/berhvpX3Ep3DZ1GN11Tf9ka7KHXT7yUsxcjTK9M6pkRUMBK3HvvvWq2sa2tTR1iwBuNTBIbhjm2TvE5oANIgmfcoBHUebGGmj+aT\/86N1hZm4o+zVRduZMudP1OWR4UlWC4nFTwyvTJnmIOZ8mT62jeY48nw32SV3qBUICiqanJWTq+4qkN9IPNr7lOuhRfhMmmoKu9lcr790za4XPc5dK5nnd0tbeo110q4L1wjRAs3zTwdsc9QpuIFZraf0tfGnDKceUkgKQLFybbd\/aOR+g7T7+cDlDMW\/kLevrvZdTaejJueebd\/qVzLVQ2oPuInmIVqfwmadCBVUjbaAu80sF56TJ4+P9faGujG\/qMVEDg7BFiBezIQ7ygZ5HC0OQV43xj0iSaNXs2dVUNSk5KNoz7ZPdohxF78euUCasEasr7dw8UZQO6rRUDt6x\/Tc\/fA2oUUDCoVHeW0w3lI2nmsWtU\/TMVF2l3+Xv03tAv0OGufzv1lPW5DC4GFPe+G3jd1iqMpYLrlLgrg22gXXxlLjYF7CIt\/PI6ZQXgGr350Z\/pTEXXFYBSILsMNgdkA3pAJsGhW0+2RNJaAVQAxmMPTaJNy7+bDPcJnZApWXmQlQ20i62u8b8faVQ+RAFgkAcW5Pt2tljSUkmA6JYK3zXUdyULFF6dt6DIVy2S\/xysw\/e+vE6lU5FGRbyQT6xgoqcm9cxIStavUyaJNcE820bhHOBZZ1gHgGD9Px4JPclW+NvdWzCpZxYUcUkpg+1KMHQvu9hjxFUywarUgWLdzzbTT1etoj\/u+b2J\/ts2epkDcj0SXo24IZ9Z5zjJTh0o1sz9FZ3bea3DE5hcuboRX7Sd7zliESOQXthXlfVQp8+AizlVy7XPcQoizrbHjh2rrsFy22UW53u7zvVRzV8814ewVJuXYJypeJ\/KB3yqLAOOxExa8du\/HZXW2N0nEPujhT9RkziygOlcoNj\/PdqXyvtfpK72PsSC6Wr\/rKrCn6N2TtavGP5Jd1uX2yykraw+q\/OZB5h+X\/0PgX9JH3C89m9HlVfsoABBJvcX64KT4MLIBkuC37LoG1OiMqlU6ktrjIEq6SDQ5eK1fzuq\/HoFFFGJsvUtB4rJAQuKYnLfvjuRHLCgSKRYLFHF5IAFRTG5b9+dSA7EDgqvbaqJ5EYRiJIXILodPw+SJA87OzvVqRXbt28vArXFf2UYfjGVfCnpzp07r7hU0q8nsYLCb\/Vs8dmbDArkzbLyAk0p2OXLl9O2bdsUEFB\/\/PjxV9zdkIzexE9FEL8kBXxvittNq0UDhd8+i\/jZl\/w38KjHuxTlDkZcbuJWcPlJ0I0\/ye95fhRG4Rd4WV9fTxUVFc7W6LBvjdVS+G1TDUtgluvplx7KQWTfvn2uXZcjZZZ549a3sPzi6+a2bNlCc+fOtaBIk6KEFTL3KYwlSVP\/o9Iall9wmw4cOECNjY05h2iEfV\/slsLrlI+wBGa5XhR3oJQtBOtAGH5xcF1VVZWjOlHiilhBYQPtYEiHCRxRBwUXa5Z6CcMvHUTyXvcw\/IsVFCDAa5tqGOJKoY5MMba0tNDixYsJQTYDAYvccGgwAkYuXqlby6\/cgcPvono\/XsUOilIQlO1jtjhgQZEtedreGOCABYUBJsompM8bd7aIsyzr16937QXej4m+mTNnGu5ltpuzoDAs397KEoVV+CDgGO5+JpqzoDAoRplUOHLkiMqVT5w4kdauXUuLFi2i5uZmGjNmjAqacV3B4MGDqa6ujmTgjBlrDqy9AmoEkA0NDbRjxw619EMG6+iOvAoB7U2ZMoWWLl2qAnhbgjlgQRHMo0g13NwnBgUaQnZp8uTJNH36dKW8mzdvdg6khpLjeFFewIZRvl+\/fk5GignRFV2+k2dzV69eTZgV1wEUqTMlWtmCwrDg\/UDBa5z09Uu8EBCWhCc7odBe65x01ynIZbMuVDQhW1BE41dgbT9Q8CSSHyhgQWRxWyruBgJeEYpn9dlbC4pAseVUsKCIxq\/A2oWCAjEIT+B5vcwvyJarCHjPhQVFoNgsKKKxKFrtQkChxxReeyf0mELuw7AxRTR5udW2lqJwHua0wBkot+xTkPuEtU0y++S1y04PnvVFcNJ9stmn6AK2oIjOs0Q8Yecp4hODBUV8vI295aBYISxwYic0ZS+woEiZwCy58XPg\/zLOpTiPp6huAAAAAElFTkSuQmCC","height":0,"width":0}}
%---
%[output:4355ce97]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMUAAAB3CAYAAACzOoSOAAAAAXNSR0IArs4c6QAAFXhJREFUeF7tXQ1wVtWZfkncQGMI2ygBwt+gYMZxLYTwUxAW6RTbImLT6eCWdZbsyoIOlTY7LGQo1ZHFVCnduCIFXO3o1NGSsZsuy9hid\/2B0RQw\/q7DptHRAkkIBuQnIgRDd54T3sv5Ts797j357v2++92cM8MA3z0\/73nf9znv+57fAUVFRX8mmywHLAccDgywoLDaYDmQyIEBTU1NjqV49tlnafPmzUY8GjduHG3atEmUWbVqFX300UdG5Tnztm3b6KWXXqK6ujp66qmnaOTIkbRu3Trat29fn+rzW+jee++lxYsXi+xNTU1UWVmZUJT7V1JS4vyuy+e3vSDzMe2QW3t7O61YsYIaGhqouro65Wb8yBVyKi0tddp69dVXA2k7GfHTp0+nDRs2UEtLCz3wwAOB6B76ev\/999PWrVuFvjmW4qGHHqI5c+ZQX4CRqgTQ9owZM2jLli0CFOlMECyYomtbFgCDZdGiRUL5AH4VQOmkG23JoDAdzLxoTQYK\/lZYWOgMXOnii04mXn3x+q4Owg4oVCYUFxcLRBYUFIg6GSwsCIyWGCU+\/vhjysvLE3lefPFFMeryaKV2gIGHvF1dXUIRhw0b5ozU3E55ebmwFBD0kiVLEqyQ3AF80NGoMoHp4L7wiCbTI\/eRy\/uxWKol4bpZSQ4ePEjXXnut4KNsYVSa3PiLMi+88IIAIvOZ20hmKbh9LiMPdrJ17OzsdBRbpunEiROCDefOnevlAfgFo2xJWltbE+qReS\/ToOqX2n\/UAzDqLAVGe+jNhx9+SBMnTnR0jAdamR5us6KiQhgDWScTYgpWAlUZly9f7ozkrMQsYBlMtbW1VFVVRWfOnBGjqCo0KPj69esFAVDm06dPC0bJ9avuE4gGSOBKcTkdQ9Q6WLFVYKoCdbMUfkckGTjTpk0TAJfdmY6Ojl59PHDgQILZ98PfV155RQwSMr0sC9V92r59e0L9rCzg4dixYx03S5ePrSb4ByAy\/bJb7Gew0A1ekBv0QvYMVF4sWLBA8FDVL7ZKzGN8V90n9BMDNfixf\/9+x81iXbz55puFLKZOnZpg7WVajx07dtl9AhP44+7du+m2225zRnwedZ5\/\/nmhayx4CEm1MBAwK\/E999zTKzaQRwgePZKBgoWotq360DKNsiuhgkClF4zUuU8qKFSLgJHmmWeeoTvvvFOMWmA8l2lsbKQ333wzgfF+RnUdfxnc8sivWlk3UCAO0o3QsqsKeUBeal+SuU9eoNANKAyE5557jubNm5dg\/WXeqPrF\/dZ5HzpQsCxVsDGoZWvBwJP7A33TWgpUOHfu3F6uGMw2FCAZKBiFCJpnzZrlKAx3HELauHEjrV692mFMMlAAuQjkYX2QOADnEUMlUg321HjFLyj8KAUPHuyiMC2yyWdh6gSvo13lLysY2oC7OXny5F5WWxdoyy4S2mFw8Ggqtw2Q7dq1i2655RZHXn767zYRoioy2mI5qO3gW5CgYP1gvUH97I3ATYJsEFBzsI7BTLX22phCjQ1kBnqNvMgLJYa5gyAx+rHZT0awPHqpIxEzFMLjUVnH+F4o1gSjfkEhW05V+EyfOrrK7av0+Z0pUvmr\/l8GuZv7JM8+yXECBgwk3aQG52OXluXISiW7T14xRRiWgic2ks0+yW6iDAp267kvHC+zLrlaCrmjQLM8zarzeTlw040o7CLJAZROmH7cJ0yRya6DW7tBxxRgoKooEIzaN9lF1MUUOkthwl8ZXPX19WKEY6vhBgrOx0J3A2RfYwo\/s0+pxhSqnP3GFG4DrwwYDq517pOIKdzWKbxmR5KBQjc9J9eHER9\/kOTgD8LGaDZ8+PCEWEQnBFlp1Rky1WK49YWtgduULL7r1inYp8ekgNfskw4UsJx++cs08noAZoWKiopEMCn7324xFrt28syX2+yT3Jdks0\/MX3WdQl2\/0fnvXFYXW2LQ0VkhmVd+Zp+gU6r7xG49+MH6x1aRg3vQBr7aFW2dz2V\/69cc8AQFEI2RmxeqVNTyKrY8+qRjZbNfS812PlQOJAUFmz\/ZLOK3o0ePEvujmD\/HnPB9991HTz\/9tCCW1yPC3qIRKmds5f2WA66ggIVAoIY5bCR5Hn7nzp1iRomtCOblFy5cKOID9uV4sQllR4wYIf7YZDkQJgfa2toIf1JNnu4TLIMMCrYICDIBigkTJtBbb71FZWVlYj4YCTNXzc3NYnMYwACwYH7dJsuBMDmABVPMzqUKjNBBATBg0SkIYsNkaJTqxgCzdOlSyzMDoTDPsDUF4EglGYMCym3iPjEogiA2lY5mU1lY1\/nz54uNgKmOetnU71RoDVLPjEABok0D7SCJTYVptmy8ORCknhmDQp6SdVsQkrcpB0lsvMXaP3p309Sr6bGaKVQ273eBdjhIPfMERaqUB0lsqrRkU\/m4ztgBFKtXXE+3V+7tkzjcZpiC1DMLij6JJtxCdsbOnb9uM0wWFOHqZMZrtzN2ehEkm2GyoMi42oZLQJACDpfS9NaejC9B8sy6T+mVq6\/WghSwrwazJJMFRZYIKgwy4wYKzFji3AmOj\/b1CiTw2YIiDG3LkjotKPSCsqDIEgUOg8w4gEI+sISDQThj\/8EHHzh75HDox3Q3tQVFGNqWJXXqhJ8zZATlDrl8S2HUutJ9qpUunrq8QxWbRZGwKRRnbfh6GZxyu+GGG2jo0KHiAguT4wUWFFGTehrp0Qk\/f\/Y\/Uv7sZWmkwqyps3sfp7N7\/90phO1AOFLAR2\/lmEL+ZtKKBYUJt2KWN26WAmf2cVnA2rVrxcV3+fn5wlLgggWTa1ItKGKm6CbdiVtMgVtdcNMg\/uCCNr6lz8YUAexzN1GsbM4bB1CEwX9rKcLgapbUaUFhp2SzRFXTR6YFhQVF+rQtS1qyoLCgyBJVTR+ZFhQWFOnTtixpKW6gsHufFMWLm4DTgau48cyCwoIiZdzoQDFqcC6NLshNue6wKjjc2U1HznQ71bvtfRo\/frxzsR7eKDHZ6mGnZMOSXhbUqxN+1eQr6Z\/Ke94fjGL618ZOqn3zM4c03d4nXKuKBTsAAc8W4PZJk8c0LSiiKPk00RQHS+G29wnbPD799FOaNGmS3eZhL0Pzj6g4xBSypZD3PvGWcbwNgb1QJoeOrKXwr0OxyxkHUMgxBe99AgiQ5LuGTYRnQWHCrZjljQMo3EQCsNTU1Bi7TqgvsqBQX91UHzoH8fKjLXEWcFhYjCvP\/L5N7sbXyIJCfdkIHUBn3R5tiauAwwKE14gYZrtRrzuyoOALluVnaWE93B5t4Y488cQT9hZtn1pnBxI9o9z4ghsV8Q3voAQxoWN075P6oic\/CYw5Zz6DqwZS3BH8DmA8+eSTPlWj\/2azoDADxV133SXe80BKOyhUUv28ZMQCxrsWuAfUvrfgDfa4gULe5oHpWTz9hoEUb17LHgdzxi0YT2Yp8J4HgJFxULDbdODAAcL8s+7Nu7gJ2FulU88RN54xKOApQHG3bt3qCgr2RvDeNV7Aks9wRzKmAMHyfhV+Dw+dtIF26mDgGnTCHzMyn0aX5AfXSMA1HW49S4dazjq16vY+4e6nKVOmEO6BwjuJuOoGZ7YBAJ6xXLNmDb399tti+4d6sUEkQYEey1OyHFPg7h75d\/toS2oapxM+3nRYs+L61CoOsfTDWw7Sxi0HnRZ0e59qa2sTLAXHofAy+LYPrHCbuk9Bz9gZBdp94WncXIG+8MC0TBwshW7vk5v7pG4tt6Aw1Zh+kD8OA4lu7xNiBF1MYUHRD5Q61S7GARS6vU8ARVVVFRUWFhImZ3JycsTskwVFqhrTD8rHARRhiCmygbZpZ62ATTmWfOObeW3xKWFBER9ZGvfEDiR6lllQGKtSfApYUFhQxEebA+pJ3EBhss0Ds1Zz5swRnGxqako4w20tRUAKlo3VxBUUXts85GOrWMRT37GwoMhGbQ6I5jiAoq\/bPGQWwmq0tLSIh1+8Vq2D5Jld0Q5IkYOsRifg4i+NpqFfGh1kM4HW9cnnh+nY54edOlPZ5oFKdIfZrKUIVGTZVZlO+HdMWEV3TPjnyHZkR\/NPaUfzJoe+VLZ56A6yWUsRWdGnh7C4WQqOFby2eSDmwIo3zluwyyRz3FqK9OhfJFsJ0j\/OVAf7ss0DTwrfeuutCST73XEdJM9sTJEprUnSbpACjmD3+kyStRR9Zl32F7Sg0MvQgiL7dbvPPbCgsKDos\/LEtSCDAoEnjm3a1MMBXGXjdo1NkAOJjSkiqHEsfAjapkQO4EYY3Ayj3gpjQdEPNAXAwJ9MJVyS8FhNOTXtWEKHWj6jx979gTEp3\/\/KvyWUwQIkL\/Dh30j4v1vd3\/v2GLq+7RF6\/8TrzhoIwKC7JsmCwlg8toApB26aejXV1VbQvpoNtPndlfTykR2mVdANRTPFgmNx\/iUAnAUAVop65o66QwDi\/eOvJ6yEy43gBpOXt62kd7ZW0Y\/\/UCHA4ZYsKIzFYwuYcmDnU7Np5MlKOvCfE+nul6eYFg8sP8D5L1+tp5MfXkffeWGYBUVgnLUVGXEAI3TDjr8VI\/Rv3\/tvYSkymcrHl9KPrttD6lYSmSZrKTIpoX7QNu6YWv7N2325Lelix71feVS4XD98Z1zCpWvcfiRBIV+GZt+nSJeqBN8OrMSqOffTqJOV9L\/HX6f79lUE30gfa\/yP+e3Ucc3PaNljG3vVEDlQ2Pcp+ijliBXDjBAC42\/d+HV6ev8vEna9RoHU9dPracr4Unrwj39Nrx3oSCApcqDw8z6FfBs034s6ZuSVNHpkz\/2o+E1Nr+3vcL7j2+FLd5ViipCTeodpFISXjTQAEOu\/Wk9jSvIpf\/Y2uvnuRyPXDdC4be4bVLTwwV70RRIUyd6nePTB7XSk9RHKzz1EN00bmsDsPx08QQAHUkvnEOfbgMKBzr8PtZ6lGddd8BSSfMHvYQk4KI8kf2eQnfv0qoR6ZcCdO5H4zZOAEDIMKjpOYdKBw0Hw1WEhGs7+nKp\/\/h6p98KG0K0+VwlrMWva1dRZvpZ+WfOGU093QbFY7V66Yat48iGVFMiKNixFMlA8cvdm6vjNeBr05eNUXPqKoPf0hWa6ou0gDbyihUq6j3n24Xx3iciTUziIzn\/R8+8LV44Tfw\/Ab5e+d31RQhdPn6ecwoGkKjzyYmrPpkQOHM\/7gvbmfkBtY3fTr5b+Hy362RDa8\/t36cKhxsiwatTgXJoxIk8AePigh2niPbU0ZOh79MX77XThDz0n\/k7MXkl\/\/8ivowOKZM97wVI8\/5M9VHT+Gvqrq2a6MnpgbmvCN1Z0\/hGLPbwSqv4mH4U8draHSRgFkUYNziG57rwrLrczuiBX5OHfjpzpvlQml0YX5CR8Y1Di74FfPi6+nVcsTTItQhmT\/HJdsGBQDKavNXcYjbpEu67NI509\/dB+u9THw2e6aU31GPqf3X+mP9JQWnRbu8he91\/DCOXbiyZR98k2uniqlbpPtYm2W3OLxb+RLp5so5y\/HOH8nTukZwU+Z0jPoIXEv8l0cHn+DfVzueEn3hb1T8\/vEP1F+sG1p8XfaLs1Zxidbv8JjS45R+e\/82uaNuwQXbg4hJoOXUVDhk6jf1j+UDRAYRpos2LjzDGvdqLTOoWXmcnKjt\/UM8HJlDGsbyw0t\/oZcMna19Ux6hIYXeu9pCy67140oQzTdbizm762biZ9vK9NjLgTfjidLjQcckbeZHRDQYWi5vQsqL3xFzdSycV25\/9tucU0QvIA+Bv+vniqTQtofJO9BoDwjbwbnfp3Dvq6+Hdp50Ba\/ckk2jS+nU6e\/y19d3IHfbesg45+\/i2qrooIKECofZ8iLOiFW+9bv\/8mPfebP4lJDKxP3F65x4m9VIAxmPA7A3c0\/i2BVDcQAHz4HX\/LqaG1i7g8ANDQdsGxhA1tXUk7zmfW4SG8emYPDRz7OS1f9bXMP+\/lR1xBzgr4ac\/mMeMAg2LW1KtFwYWVe80qyGBu7K2CO45JAoDjmm8UUHXd4mi4T8n4YkGRQa3x0TRA8dqBT+h73x5L31\/bKKxGNqZv3HQ7\/XjdOlr5o+UWFNkowCjRDFBgjQjT1WXzfhcl0oxoCXLwDWRK1loKI\/lFKjOD4rX9n2SV66Qy0YIiUmqV3cQwKBYu2dNr60Q29SwrQWHPG5upWFlZGR09elR7ysyspuS5n3y4WJzwm\/937wRZbdrrSnZ+25SY0N0ne97YVCTpzT\/qyl+JBo989jfpbTiE1tzOb5s2FTooQFCmzxubMsXmz04OuJ3fNu1NWkBhSpTNbzmQSQ5YUGSS+7btSHLAgiKSYrFEZZIDFhSZ5L5tO5IcCB0Ubme3I8mNDBClXlmPgzL79u1LoETmYVdXF+Gdh7q6ugxQm\/km\/fCLqcTubdwmuHPnTu17F269CRUUybaUZ5690aBAfsZK94KPzEMAAfnLy8vFKTMVPNHoUbhUePFLbh38LC0tJfmNCz\/UhQqKZGe3\/RAX9zw86jU3N1N1dbXYfs8nGPE6qC7hVaAlS5bQ+vXr+x0oTPgFXs6cOZPy8vJcX0bKiKVIdkw17grvp38sZH7OSh5E3KyA7oFEP23FIY9ffsG6rl69mh5\/\/HFatmyZBUU2Cd+vkLlPfixJNvXflFa\/\/OJHKHft2kWbNm2KHijczm6bMiSO+U3cgf5sIVj2fvjFwXVBQUGCypjEFaHGFDbQ9oayn8BRfpPau8Z45\/DDLxVEbq+tZiSmQKNuZ7fjLTr\/vZOnGFtbW2nVqlWEIJuBgE1uuEgOASOnzs7Ofjv75MUvTFhEHhT+1cPmtByIBgdCdZ+i0UVLheWAGQcsKMz45Zlb9nnDni3iWZbNmzdr6UL7WOirrKz0pNtmuMwBC4qAtSFds0R+Fd4LOAF3PxbVWVAEKEZ5UqGpqYkaGxvFCnVtbS1VVVVRS0sLTZw4UQTNeMNj+PDhYhuCHDhjxZoDa7eAGsFmTU0N1dfXiz1QcvCJ7sjvg6C+iooKWrt2rQjgbfLmgAWFN4+McujcJwYFKsLs0oIFC2jx4sVCebdv3y4WmLDVA0oub2DDKD948GBnRooJURVdbpNXczdu3Ci2gagAMupMP81sQRGw4JOBgvc4qfuXeCMgLAkvdkKh3fY5qa6Tl8tmXSgzIVtQmPHLM3cyUPAiUjJQwILISbdVXAcC3hGKsurqrQWFp9gSMlhQmPHLM3eqoPDaJQsCkgXZ6lZz5Leg8BSbBYUZi8xypwIKNaZwOzuhxhTyOQwbU5jJS5fbWorUeZhQA89A6WafvNwnbFGQZ5\/cTtmpwbO6CU52n+zsk7mALSjMeRaJEnadIjwxWFCEx9vQa\/aKFfwCJ3RCs6wBC4osE5glN3wO\/D80CkS\/OCZQYQAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
%[output:12936223]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMUAAAB3CAYAAACzOoSOAAAAAXNSR0IArs4c6QAACTRJREFUeF7tnV9IW1ccx3\/qtEMyutmuM8JaAi0ORkvbbIQKhcKeJq0Pe+iDe1Co2kEQ+uDElaCthCJdGIUQmFZBH+aDD3vIRPZSSCesyDCUlj0UH4QVEudaOqk46tY6zi03JGk0N7n3+Pvd+L0QKOk55\/c73\/P93PPnxqSqoaFhi3BBASiQUaAKUMANUCBXAUABR0CBPAUABSwBBQAFPAAFdlbA0ZnC5\/NRJBKhRCJB0WgU2kMBVyrgGBSBQIDC4TB5PB6anp4GFK60A5JWCjgChZohhoaGaG5ujrq7uykej+dA4fV6Sb1wQQGdCqTTaVIvu5cjUJhJmLNFNhQKhlAoRKdPn7abK+prUODh0yq6db+6YMtXTr6i4wesP8Za3SD65l5Npi1V\/87jKlIxduP6aH2R7k0N2wZDOxQKhlgsZiytnKB4N8TljnHq1Cnq6uraFc2Wz1yjV3+n6N8\/FnO6XXvYT9XvNpHv3jXLcjz399CTuiZ68eAn2nfiAtXsf7062Jgfs9xGuQXVzfe7y5\/T9a+DlEwmy23GqLdrUASD9pO11VMXVVYD3NraaixHdd9I3gvGDRNvzN\/OUaj+bLdh7GexNsvKec4PGSCs\/fAV7f\/ye6o94jfqPrnxqeU2yi1o3nyd8BmgKHcUKqQeoHhzIAFFhZi73G4ACs1QFBoYJ6e1cgce9bZXAFAACvCRpwCgABSAAlAU9YCjewosn4rqLa4AZgrMFOJMyZ0QoAAU3B4UFx9QAApxpuROCFAACm4PiosPKACFOFNyJwQoAAW3B8XFBxSAQpwpuRMCFICC24Pi4gMKQCHOlNwJAQpAwe1BcfEBBaAQZ0ruhAAFoOD2oLj4gAJQiDMld0KAAlBwe1BcfEABKMSZkjshQAEouD0oLj6gABTiTMmdEKAAFNweFBcfUAAKcabkTghQAApuD4qLDygAhThTcicEKAAFtwfFxQcUgEKcKbkTAhSAgtuD4uIDCkAhzpTcCQEKQMHtQXHxAQWgEGdK7oQABaDg9qC4+E5CoX69SF34JaMiw4zfpxDHQU5CgAIzhWyHMmQHKAAFg+1khwQUgEK2QxmyAxSAgsF2skMCCkAh26EM2emCQv2m9tsnzhs92tO\/o11oTHH6xOD0EkICCswUJdhlbxQFFGVCEQgEKBwOk8fjoVQqRX19fbS8vJzT2uTkJDU3NxvvbW5uUiwWo5mZGcJMIRsuQFEmFMrwKysrNDo6SpFIhBKJBEWj0UxrPp+v4PuqAKAAFBW3pzBniXg8boAwMjJCjY2N1NnZmRltVWZwcJCmpqaM2SH7MqEYHx+nubk5SqfTsl2yx7KrhJnC6\/UaN99QKETBYJCSyaStUSz6O9r5hldQHDt2LGcJ1dvbS+3t7ZlEHj16lIHGhEL9pwJjYmLCVsKo7KwClQDFpUuXqKuryxBGDBTZw2QupZaWlmhgYCCzfFJ7EkUwZgpnTW23tUqAQs0Ura2tBhi7BoUy9E7Lp\/yBUXsQdaklFvYUdm2rt34lQOH03rXo8kkFLLbRVssnv99vQJC\/BwEUek1tt3VAUebpU\/aRrLlfUO\/19\/fTzZs3aWFhwQDHPJIttKdwYlqzawDUf1MBQFEmFHbMhJnCjnr66wIKQKHfZS6LACgAhcssqz9dQAEo9LvMZREABaBwmWX1pwsoAIV+l7ksAqAAFC6zrP50AQWg0O8yl0UAFIDCZZbVn67TULxcS9P67HXCn6PuMHZ4eKff2HYiAArMFHb8U5F1AQWgqEhj2+kUoAAUdvxTkXUBBaCoSGPb6RSgABR2\/FORdQ9e\/Y025sdoY\/52Tv\/qz3bTvhMX6FmszXK\/1Vfx4\/TJglw4fbIgEmMRQIGZgtF+MkMDCkAh05mMWQEKQMFoP5mhAQWgkOlMxqwABaBgtJ\/M0IACUMh0JmNWgAJQMNpPZmhAAShkOpMxK0ABKBjtJzM0oAAUMp3JmBWgABSM9pMZGlAACpnOZMwKUAAKRvvJDA0oAIVMZzJmBSgABaP9ZIYGFIBCpjMZswIUgILRfjJDAwpAIdOZjFkBCkDBaD+ZoQEFoJDpTMasAAWgYLSfzNCAAlDIdCZjVoACUDDaT2ZoQAEoZDqTMStAoRGK3t5eam9vNyLcvXuXBgYGjH\/jy9AYHW8htILi+ex1evFgNqc0viHQgng7FQkEAjQ4OEhTU1NGsY6ODhoeHqaFhQVAYVNb3dUBhaaZQs0SbW1tFAqFaHV1lSKRCCUSCYpGowYUQ9\/GaHxigpLJpO4xrpj2lW7pdNp46bxefDFKnsUxavhrMSfM2tHztHb0Au378bLl8Ov+HjpUT\/TO4hg99\/fQs\/f9Rt3DP1tvw3KwvIJer9fwXzAYtO2zqoaGhq1yEzHrKSjOnTtHfX19xlsKiqWlJWMJpZI90zFIv1R\/YjcM6mtQ4FD9Fl05+YqOH8i1wcOnVXT115qSI145+ZI++3CL7jyuolv3a4x2b7S8LLmdciqom244HLZ9I9EOhepc9X4v1R55fdfAJU+Bj99KFUzqSW0T\/flPaflmt\/X7f010cDNFH9SX1ka5pZ2aWR2DYrvlU7kdRD0owKWAI1DstNHm6hjiQoFyFXAEChU8+0h2enra2GTjggJuVMAxKLbr\/HbPL9wolo6cfT6fcTDR1NRE6+vrxgmKOsrOvrI13NzcpFgsRjMzMzrSEd+mFb3MTqgVjNp4x+Pxkm7SWqHAsqq4x0ZGRqixsZE6OztpcnKSVlZWMg8+Ve1sDRUIqrzf7y8IT\/Fo7i9RTK\/sHio9m5ubqdSVi1Yodnp+4f7hsd8D865nHl9nH20vLy8XDHDx4sWch6P2s3BPC6XopbRsaWmhurq6zDMzqz3VDsV2zy+sJljJ5cxBNh90Zt9E8pdQpg7Zd8pK1qZQ36zqpWbX\/v5+Ghsbo56eHkDhJqNYHWSzT1ZmEjf1v9Rcreqllk2Li4s0Ozub8+kKq\/G0zxR4frH9UJSyHNjLM4SpoBW9zM21x+PJEb6UfYVWKLDRLn5vsrJxVGXUZX7yuHirlVvCil75EJnLU6uqaIVCJYHnFzsPRfYRYyqVMj4\/pjbZJgjq8zzqQ25qw2he2x3dWh10N5crplf2jSN\/uWW139qhsJoIykEBKQoACikjgTzEKPA\/\/kfC3rw9BgQAAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:23a4215c]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 22.944420 seconds.\n","truncated":false}}
%---
%[output:614fb6f9]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7t3U+IXNedL\/AzmthCWBLGz4R2QmL6gfA6Mo4ICXRmG4IGQxAz3lgEtAht7YTTGJGFaYJiehFQehHUDZ2NMghDGC+yjXrhhWLUDfEiZLQQOETdBN4QIgVhJ6P3OPW4Tem6\/txbVb+qe6s+BcaWdftXpz7nV1XfPvfcqn964YUX\/m9yI0CAAAECBAi0SOCfBJgWzZahEiBAgAABAh0BAUYjECBAgAABAq0TEGBaN2UGTIAAAQIECAgweoAAAQIECBBonYAA07opM2ACBAgQIEBAgNEDBAgQIECAQOsEBJjWTZkBEyBAgAABAgKMHiBAgAABAgRaJyDAtG7KDJgAAQIECBAQYPQAAQIECBAg0DoBAaZ1U2bABAgQIECAgACjBwgQIECAAIHWCQgwrZsyAyZAgAABAgQEGD1AgAABAgQItE5AgGndlBkwAQIECBAgIMDoAQIECBAgQKB1AgJM66bMgAkQIECAAAEBRg8QIECAAAECrRMQYFo3ZQZMgAABAgQICDB6gAABAgQIEGidgADTuikzYAIECBAgQECA0QMECBAgQIBA6wQEmNZNmQETIECAAAECAoweIECAAAECBFonIMC0bsoMmAABAgQIEBBg9AABAgQIECDQOgEBpnVTZsAECBAgQICAAKMHCBAgQIAAgdYJCDCtmzIDJkCAAAECBAQYPUCAAAECBAi0TkCAad2UGTABAgQIECAgwOgBAgQIECBAoHUCAkzrpsyACRAgQIAAAQFGDxAgQIAAAQKtExBgWjdlBkyAAAECBAgIMHqAAAECBAgQaJ2AANO6KTNgAgQIECBAYGEDzM7OTjo8PExra2u6gAABAgQIEGiZwEIGmBxeXnnllbS7uyvAtKxhDZcAAQIECGSBhQow586dS+vr6+mzzz7rzP7HH38swHgeECBAgACBFgosXIB5+eWX00cffZQ2NjbSvXv3BJgWNq0hEyBAgACBhQowxXQvLy9XDjAvvfRSyv903w4ODlL+x40AAQIECBCYjYAAM2ATbw4uV69eTWfPnn1qdra2ttL29vZsZsy9EiBAgAABAou1B6buCkwOLpubm519M90rLlZgPHMIECBAgMBsBazADFiBKQLM6upq2tvbm+1MuXcCBAgQIEDgSECAEWA8HQgQIECAQOsEBBgBpnVNa8AECBAgQGAhA0zVaXcKqaqU4wgQIECAwHQFBJgB3gLMdJvRvREgQIAAgaoCAowAU7VXHEeAAAECBBojIMAIMI1pRgMhQIAAAQJVBQQYAaZqrziOAAECBAg0RkCAEWAa04wGQoAAAQIEqgoIMAJM1V5xHAECBAgQaIyAACPANKYZDYQAAQIECFQVEGAEmKq94jgCBAgQINAYAQFGgGlMMxoIAQIECBCoKiDACDBVe8VxBAgQIECgMQICjADTmGY0EAIECBAgUFVAgBFgqvaK4wgQIECAQGMEBBgBpjHNaCAECBAgQKCqgAAjwFTtFccRIECAAIHGCAgwAkxjmtFACBAgQIBAVQEBRoCp2iuOI0CAAAECjREQYASYxjSjgRAgQIAAgaoCAowAU7VXHEeAAAECBBojIMAIMI1pRgMhQIAAAQJVBQQYAaZqrziOAAECBAg0RkCAEWAa04wGQoAAAQIEqgoIMAJM1V5xHAECBAgQaIyAACPANKYZDYQAAQIECFQVEGAEmKq94jgCBAgQINAYAQFGgGlMMxoIAQIECBCoKiDACDBVe8VxBAgQIECgMQICjADTmGY0EAIECBAgUFVAgBFgqvaK4wgQIECAQGMEBBgBpjHNaCAECBAgQKCqgAAjwFTtFccRIECAAIHGCAgwAkxjmtFACBAgQIBAVQEBRoCp2iuOI0CAAAECjREQYASYxjSjgRAgQIAAgaoCcxFgrl27llZWVjqPeXd3N62trfV9\/N3HPnjwIF25ciXdv3+\/5\/Fnz55Nm5ubaXV1Ne3t7VU1dRwBAgQIECAQLND6AHPhwoV06dKldOPGjQ5V8d+3bt36HN3ly5fT+fPn09WrV9Of\/\/zntLGxkR4+fJguXrwowAQ3mvIECBAgQGCSAq0PMHlF5cyZM0crKTs7O+nw8LDnKkz52PKfy7BWYCbZamoRIECAAIHJCbQ+wOTAkm\/FKkr5z91UvVZg7t271\/eUUxFgtra20v7+\/lGpg4ODlP9xI0CAAAECBGYjMBcBpnvFJa+qLC0t9T0tlE855T0tzz77bLp582a6fv16X\/kiwJQPyIFme3t7NjPmXgkQIECAAIG0UAEmh5tXX321swfmzp07adBqTe6NIsCsr68\/teJiBcYzhwABAgQIzFZgLgJMlVNIy8vLnU273aeMujcA99r0aw\/MbJvTvRMgQIAAgX4CrQ8w5VNG\/TbxCjCeBAQIECBAYH4EWh9g6lxG3esU0qlTp\/p+FowVmPlpdI+EAAECBOZLoPUBJk9Hvw+yK1Zdbt++fbRZN6\/QvPLKK51Z9EF289XMHg0BAgQILI7AXASYqOmyAhMlqy4BAgQIEBhPQIAZ4CfAjNdcfpoAAQIECEQJCDACTFRvqUuAAAECBMIEBBgBJqy5FCZAgAABAlECAowAE9Vb6hIgQIAAgTABAUaACWsuhQkQIECAQJSAACPARPWWugQIECBAIExAgBFgwppLYQIECBAgECUgwAgwUb2lLgECBAgQCBMQYASYsOZSmAABAgQIRAkIMAJMVG+pS4AAAQIEwgQEGAEmrLkUJkCAAAECUQICjAAT1VvqEiBAgACBMAEBRoAJay6FCRAgQIBAlIAAI8BE9Za6BAgQIEAgTECAEWDCmkthAgQIECAQJSDACDBRvaUuAQIECBAIExBgBJiw5lKYAAECBAhECQgwAkxUb6lLgAABAgTCBAQYASasuRQmQIAAAQJRAgKMABPVW+oSIECAAIEwAQFGgAlrLoUJECBAgECUgAAjwET1lroECBAgQCBMQIARYMKaS2ECBAgQIBAlIMAIMFG9pS4BAgQIEAgTEGAEmLDmUpgAAQIECEQJCDACTFRvqUuAAAECBMIEBBgBJqy5FCZAgAABAlECAowAE9Vb6hIgQIAAgTABAUaACWsuhQkQIECAQJSAACPARPWWugQIECBAIExAgBFgwppLYQIECBAgECUgwAgwUb2lLgECBAgQCBOYiwBz7dq1tLKy0kHa3d1Na2trfcEuX76c3njjjc7fP3r0KF29ejXduXOn5\/Fnz55Nm5ubaXV1Ne3t7YVNgsIECBAgQIBAPYHWB5gLFy6kS5cupRs3bnQeefHft27d+pxEPjaHkffffz9dv3495eBz5syZdOXKlXT\/\/v3PHS\/A1GsmRxMgQIAAgWkJtD7AlEPIzs5OOjw87LkKk49dWlpKFy9erOQrwFRichABAgQIEJi6QOsDTA4s+VaEkvKfC9Hl5eW0sbGR7t27N\/AUU\/cMCDBT70d3SIAAAQIEKgnMRYDpXnHpt8pSBJjf\/e536Vvf+lY6efJk5T0wW1tbaX9\/\/wj04OAg5X\/cCBAgQIAAgdkILFyAOX369NHG3bxac+rUqaF7YMpTkwPN9vb2bGbMvRIgQIAAAQJpLgLMqKeQujcA99r0W5xCWl9ff2rFxQqMZw4BAgQIEJitQOsDTPmU0aBNvOW\/ywHmzTffTO+++27PS6ntgZltc7p3AgQIECDQT6D1AabOZdT5M2DOnz\/\/1Cmk7tWbMpIA44lDgAABAgSaKdD6AJNZ+32QXbFx9\/bt253Pfcm37g+ye\/DgQd\/9L\/lYAaaZTWtUBAgQIEBgLgJM1DQKMFGy6hIgQIAAgfEEBJgBfgLMeM3lpwkQIECAQJSAACPARPWWugQIECBAIExAgBFgwppLYQIECBAgECUgwAgwUb2lLgECBAgQCBMQYASYsOZSmAABAgQIRAkIMAJMVG+pS4AAAQIEwgQEGAEmrLkUJkCAAAECUQICjAAT1VvqEiBAgACBMAEBRoAJay6FCRAgQIBAlIAAI8BE9Za6BAgQIEAgTECAEWDCmkthAgQIECAQJSDACDBRvaUuAQIECBAIExBgBJiw5lKYAAECBAhECQgwAkxUb6lLgAABAgTCBAQYASasuRQmQIAAAQJRAgKMABPVW+oSIECAAIEwAQFGgAlrLoUJECBAgECUgAAjwET1lroECBAgQCBMQIARYMKaS2ECBAgQIBAlIMAIMFG9pS4BAgQIEAgTEGAEmLDmUpgAAQIECEQJCDACTFRvqUuAAAECBMIEBBgBJqy5FCZAgAABAlECAowAE9Vb6hIgQIAAgTABAUaACWsuhQkQIECAQJSAACPARPWWugQIECBAIExAgBFgwppLYQIECBAgECUgwAgwUb2lLgECBAgQCBMQYASYsOZSmAABAgQIRAkIMAJMVG+pS4AAAQIEwgQEGAEmrLkUJkCAAAECUQICjAAT1VvqEiBAgACBMAEBRoAJay6FCRAgQIBAlMBcBJhr166llZWVjtHu7m5aW1sb6nXhwoV06dKldOPGjXTr1q2ex589ezZtbm6m1dXVtLe3N7SmAwgQIECAAIHpCLQ+wHQHkUw2LJTkY5aXl9PGxkZ68cUXOwFFgJlOs7kXAgQIECAwKYHWB5i8+nLmzJl05cqVdP\/+\/bSzs5MODw8HrsJcvnw5nT9\/vmNoBWZSraQOAQIECBCYnkDrA0wOLPl28eLFzr\/Lfy5Tnjt3Lr399tvp9u3bnRBTJcBsbW2l\/f39o1IHBwcp\/+NGgAABAgQIzEZgLgJM94pLXpFZWlo6CjRl1vz3+Zb3tAw73VTsgSnXyIFme3t7NjPmXgkQIECAAIG0UAEm75d5\/fXX0zvvvJNee+21ygFmfX39qRUXKzCeOQQIECBAYLYCcxFgMmGVU0j59NLdu3fT9evXk6uQZtt47p0AAQIECIwj0PoAUz5l1G8Tb977kldSTp48+TmvmzdvdkJN+eYy6nFay88SIECAAIE4gdYHmFEuo86cVmDimkplAgQIECAQLdD6AJOB+n2QXfF5L\/mKo\/IKiwAT3VrqEyBAgACBOIG5CDBRPE4hRcmqS4AAAQIExhMQYAb4CTDjNZefJkCAAAECUQICjAAT1VvqEiBAgACBMAEBRoAJay6FCRAgQIBAlIAAI8BE9Za6BAgQIEAgTECAEWDCmkthAgQIECAQJSDACDBRvaUuAQIECBAIExBgBJiw5lKYAAECBAhECQgwAkxUb6lLgAABAgTCBAQYASasuRQmQIAAAQJRAgKMABPVW+oSIECAAIEwAQFGgAlrLoUJECBAgECUgAAjwET1lroECBAgQCBMQIARYMKaS2ECBAgQIBAlIMAIMFG9pS4BAgQIEAgTEGAEmLDmUpgAAQIECEQJCDACTFRvqUuAAAECBMIEBBgBJqy5FCZAgAABAlECAowAE9Vb6hIgQIAAgTABAUaACWsuhQkQIECAQJSAACPARPWWugQIECBAIExAgBFgwppLYQIECBAgECUgwAgwUb2lLgECBAgQCBMQYASYsOZSmAABAgQIRAkIMAJMVG+pS4AAAQIEwgQEGAEmrLkUJkCAAAECUQICjAAT1VvqEiBAgACBMAEBRoAJay6FCRAgQIBAlIAAI8BE9Za6BAgQIEAgTECAEWDCmkthAgQIECAQJSDACDBRvaUuAQIECBAIExBgBJiw5lKYAAECBAhECQgwAkxUb6lLgAABAgTCBOYiwFy7di2trKx0kHZ3d9Pa2lpPsHPnzqX19fV08uTJzt8\/ePAgXblyJd2\/f7\/n8WfPnk2bm5tpdXU17e3thU2CwgQIECBAgEA9gdYHmAsXLqRLly6lGzdudB558d+3bt16SmJ5eTltbGyke\/fudQJO8eeHDx+mixcvCjD1+sbRBAgQIEBgpgKtDzB59eXMmTNHKyk7Ozvp8PCw7ypMt3b5Z8szYQVmpr3pzgkQIECAQF+B1geYHFjyrVhFKf950NwLMJ4ZBAgQIECgnQJzEWC6V1xyKFlaWup7WqiYpmI\/zN27d\/uu1hQrMFtbW2l\/f\/9ohg8ODlL+x40AAQIECBCYjcBCBphi\/0smr7KJtzw1OdBsb2\/PZsbcKwECBAgQIJDmIsDUOYVUNbzkmsUKTL5yqXvFxQqMZw4BAgQIEJitQOsDTPmU0aBNvFWuPOqeDpt4Z9uc7p0AAQIECPQTaH2AqXoZdQbI4ebUqVMDTxsJMJ4sBAgQIECg+QKtDzCZuN8H2RUrLrdv306\/\/e1vn\/oQu2JqHj16lK5evZru3LnzudmyAtP8BjZCAtMU+OqTJ+mbf\/97+uXx49O8W\/dFgEAPgbkIMFEzK8BEyapLoH0CObz851\/\/mvK\/f3LiRHrvxIn2PQgjJjBHAgLMgMkUYOao0z2UuRbIoeLfPv00\/funn3ZWRyYZLoraP3z8OH1y7FgnwJw\/dSp9+Mwzc23qwRFouoAAI8A0vUeNj8BAgXxK52d\/+9vRMX88diydP316Imo5rPzs0aP0lSdPOsHowy98IX3w8KEAMxFdRQiMJyDACDDjdZCfJjAjge6VkRws3jp5srM\/JYeZ\/\/XCC2OP6u3Hj1Ox6vKvp093Vl\/y\/8urPF97\/vmx6ytAgMB4AgKMADNeB\/lpAjMQyCEih4l86z5llP9\/DjA5YOTAMcqtOxiV97p88Ne\/dlZjBJhRZP0MgckKCDACzGQ7SjUCgQLlcPEfx49\/Lqj8n\/\/+7\/TWc8+NdKVQsVG3HIyKh5QDTL5N6hRVIJXSBOZeQIARYOa+yT3A+RDoPqUzaKPu\/l\/+0tlgm0NM1Vuv01HlFZx8TK7tCqSqqo4jECsgwAgwsR2mOoExBcobaYddYZRPIeVTSVX3wVStXwQYVyCNOaF+nMCEBAQYAWZCraQMgckKlC9fzisqVS5drrMPptdG3X6PIm8QzlcgjbO\/ZrJCqhFYbAEBRoBZ7GeAR99IgWF7UYYNetg+mEEbdfvVdgXSMHV\/T2C6AgKMADPdjnNvBAYIlFddisuX66IN2gczajiygbfuLDieQKyAACPAxHaY6gQqCowaLHqVLz7Yrnsjb5WNuoOGmkPRpD\/ltyKNwwgQ6CEgwAgwnhgEZiowqVWX7gdR7G0pNvJW3ajbD6LYwDvq5dkzBXbnBOZUQIARYOa0tT2sNgj0+0C6ccfevZE3f0dS+RN169YvNvC6AqmunOMJxAkIMAJMXHepTKCPQPeqS3FaZtRPzu2HnDfy5q8Y+OY\/\/jH2Z7eUV3RMLAECsxcQYASY2XehESyUQNUPpBsHpTjlk0PRJPatuAJpnNnwswRiBAQYASams1QlUBIo9qFMYkWkH273yk5xTNUPtBs0Ya5A0s4EmicgwAgwzetKI5orgUEfSPfkyVfTp5\/+Wzp+\/D\/SsWOfjPW4yxt1\/3js2Nhf7FgMKJ+O8hUCY02PHyYwcQEBRoCZeFMpSKAQ6Hf1Tw4uf\/\/7N9Mzz3yY\/vKX\/fT881\/rBJkvfOHDzv+re+v1ibqTunLIVwjUnQ3HE5iOgAAjwEyn09zLQgkM+xqATz\/99\/S3v\/0snTjxk\/T48Q\/T8eO\/TPn\/5T+fOPFeZathn6g7yhc7lu\/cFUiVp8OBBKYqIMAIMFNtOHc2\/wLDPpAur77kW15xyeGluOUQk8NL1VNJw+4n1+31gXZ1Z8AG3rpijicwHQEBRoCZTqe5l4UQ6L76p9\/XADx+\/PZTwaUMU2UVpriffJn0WydPpn6XYE\/i8ue8gfcrT550vsTRjQCB5ggIMAJMc7rRSOZCIH+IXL50ud+tWIHJQSafNipuObjU2cybT+0M+3bqOt9M3W+8rkCai7b0IOZQQIARYOawrT2kpgsUqzDde1\/y6aTnnnursx9mkrdh30w96L6KlR5XIE1yRtQiMBkBAUaAmUwnqUKghkC+Aukf\/\/hmZ8WluAop\/7+8\/2WUq5AG3fU4G3ldgVRjUh1KYMoCAowAM+WWc3cEnhbIqzF1rjyq65c38ubTTaPsYSmuQMo\/O+mvOqj7OBxPgMDTAgKMAOM5QWCuBcbZB+MKpLluDQ+u5QICjADT8hY2fAKDBYpVlLeee27g5uJeVWzg1V0EmisgwAgwze1OIyMwIYFRvwog75+ZxJdBTuhhKEOAQJeAACPAeEIQmHuBUTbyTuqrCOYe1wMkMCMBAUaAmVHruVsC0xPIG3nzXpg630ztKwSmNz\/uicAoAgKMADNK3\/gZAq0SGGUj7yQ+xbdVSAZLoGUCAowA07KWNVwCownU\/UA7VyCN5uynCExLQIARYKbVa+6HwEwF6u6DcQXSTKfLnRMYKrBwAebatWtpZWWlA7O7u5vW1tb6Ip09ezZtbm6m1dXVtLe3NxTTAQQINFcg74P56v\/8Tzp\/+nSlQY565VKl4g4iQGBsgYUKMBcuXEiXLl1KN27c6MAV\/33r1q2ekALM2P2lAIHGCBT7YKps5PUVAo2ZNgMh0FdgoQJMXn05c+ZMunLlSrp\/\/37a2dlJh4eHfVdhBBjPHALzI1AEmPOnTg39FmtXIM3PvHsk8yuwUAEmB5Z8u3jxYuff5T+Xp7kIMFtbW2l\/f\/\/orw8ODlL+x40AgXYJVN3IawNvu+bVaBdTYOECTPeKS16RWVpaOgo0\/QJM+f\/nQLO9vb2YHeNRE2ixQNWNvHkD71eePBnpCyBbzGPoBFolIMBUCDDr6+tPrbhYgWlVjxssgSOBqh9o5wokTUOg+QILF2BGOYXkKqTmN7IREqgiUOUD7YoNvD85cSK9d+JElbKOIUBgBgILFWDKp4xs4p1Bx7lLAjMWGLYPxhVIM54gd0+gosBCBRiXUVfsCocRmGOBYftgiiuQvvb88+mTY8fmWMJDI9BugYUKMHmqfJBduxvW6AmMK5D3weTbW88917OUK5DGFfbzBKYjsHABpg6rz4Gpo+VYAu0QGPaBdjbwtmMejZKAADOgBwQYTxAC8ycwbCNvPsX0y+PHbeCdv6n3iOZMQIARYOaspT0cAsMF+m3kLTbw5tNLOcS4ESDQXAEBRoBpbncaGYEggX4beX2FQBC4sgQCBAQYASagrZQk0GyBfh9olzfw\/vDx41TlCx+b\/QiNjsD8CwgwAsz8d7lHSKAk0G8fjCuQtAqB9ggIMAJMe7rVSAlMSKDfXhdXIE0IWBkCUxAQYASYKbSZuyDQPIFe+2Dy5l5fIdC8uTIiAr0EBBgBxjODwEIKlD\/QzlcILGQbeNAtFhBgBJgWt6+hExhdoLxh1xVIo1v6SQKzEBBgBJhZ9J37JDBzgfJGXht4Zz4lBkCgloAAI8DUahgHE5gnge4PtMsbeL\/y5EnKX+LoRoBA8wUEmOAA89JLL6XvfOc76de\/\/nU6ODhodEcYa9z0sG2mbfdG3mlcgdSmPsgz1qbxtmmsbbONe\/aOV1mACQ4wbfo+JWMd78k06KfZNtO2+EC7vOqSw0z0FUht6oM8Y20ab5vG2jbbuGfveJUFGAHmSKBNLwBtGmvbXqwWybbYB3P+1Kn0wcOHKf\/7w2eeGe9VNfg1JWxwPQq3qRfaNNa2vSZMs+fq3JcAU+HFZmtrK+3v79dxPTo2L2tevXo1jVNjpDse4YeMdQS0ij\/CtiLUCIeNY\/u\/P\/kk\/fy\/\/qvzxY05zHz\/X\/4l\/fGf\/3mEUVT7kXHGWu0eJntUm8bbprHmWZr2ePMWhqZvY6jbvQLMALGiwXKydyNAgAABAm0VyL9Eb29vt3X4PcctwAyZzhxi8j9uBAgQIECgrQJWYNo6c8ZNgAABAgQIzJWAFZi5mk4PhgABAgQILIaAALMY8+xREiBAgACBuRIQYOZqOj0YAgQIECCwGAICzGLMs0dJgAABAgTmSkCAmavp9GAIECBAgMBiCAgwizHPHiUBAgQIEJgrAQEmYDovXLiQ3nzzzfTuu++mO3fu9LyHa9eupZWVlaO\/u3nzZrp+\/XrAaEYvubOzkw4PD9Pa2troRSb0k+fOnUvr6+vp5MmT6dGjR51PN+5n231svvtp29YZ6\/LyctrY2Ehf+tKXOlK7u7tT964z3u7pzP2RbxcvXpzQLA8vU2esTXyOZbNXXnml80Cn3Zf9dPPr1erqanr22WfTgwcP0pUrV9L9+\/d7Ht49\/s8++yxtbm6mW7duDZ+4CR5RZ7zF3RbPs3v37k31+VVnrLN+3ZrgFE2tlAAzYeqiYfOTu9+b7OXLl9P58+eP\/j7\/+Xvf+95MXgz6PfzihWoWb6i9xtT9ZjnojbP8QjUL26pjzY8zH3vq1KnOm8YXv\/jFTkj74IMPphpm64y3mJvs+sYbb6Q\/\/OEPUw0wVcfaxOdY95i+\/vWvP\/UaMOGXocrlup8vP\/\/5zzthut+bfA6EZ86cOQo4+c+vvvrqwF8mKg+k4oF1xttdsgiz03w9qzPW4tiHDx92nk\/5feTSpUvpxo0bUw+IFaeiEYcJMBOchuJJkl\/Uv\/zlL1d+YhfJe9pvXL0eejGWHMDy7eOPP57qbyyDxlT4DFrhKltO27Z8f8PG+qMf\/Sj94he\/mNmLVJ3xdv82++Mf\/zi9+OKL6U9\/+tPUAswoYy3GPO0+GBbCizes27dvTzWslsdVfqPMIevb3\/72wFWYosYs3mRHGW+e+\/w8yytMd+\/endrrWZ2xVlm1n+Bb1dyUEmAmOJXf\/\/73029+85v03e9+t9ZvV014ce1+oX\/55ZfTRx99NPC3sQmyDS1VfiEY9MLZawWme7Vr6J2NeUCdsZZXCca865F+vM54izvIQT3flpaWOv+e1imkUcbalABT7stZndIoN0mvlaqqz5dZBJhRxptX7X7\/+9+nvOo1zVNIdcZaXt0a6cm8gD8kwARMet03plksxQ572E15gc3jLP92UvxGNWjlojgFNuyc\/jCHun9fZ6zFb7t5FeO1117r3NU0l7hHsc32b7\/9dnrvvffSD37wg6kHmO69ZVX6oDt0Tft0R3fv9FpxacIes\/KKS52VgO7Tn\/32zNRNPRrlAAAELklEQVR9\/gw7vu548+N5\/fXX009\/+tNO3047wHSvZg2yze8BxS8ETdsjNWxOZvn3AkyAfp0AU+wlaMqGvoKjrQGmWM0qloqn\/Vti3QCT95EUoaXYP\/X+++9P7bRCnfHm3shvWtk2bzif9ibeumMterkJz7F5CzD5Dfcb3\/jG1Pft1Qkw2Tyf6vzVr341kxXlOmMtth8U7wOz2LsX8FYYXlKAGYG4vFu8\/Ftz1QAz6xfW7qs0ylf2zCrA9LLd29t7akPboFDSyz7qjbZNY81tPu54i99m33nnnc5VKlGukxhrk8JLHss8nUKaVXjJjnVOy+Rj86pbPsU5i9ezOmMtn0KaxXhHeCuc+Y8IMAFTUCXAND1hN+kJVD5VMGgpdpoBplfr1Blrr8cx7dMKdcZbviy5ePzTOk1XZ6zFm12Tru7rntsmbeLtPi03bBPvrE93l58zg8bbfcl393N1WlfO1Rlr+XE0pT8C3h4nWlKAmSjn\/y82LMDM4lRB3YfZpABTnLrI\/86\/TQ36zb\/XKaT8GRfTPC1T9VLfsvG0T3cVPVF1vOUeilyB6devVcfaxOdY2y+jbsIvXXUuTe7uoVm8ntUZa\/lCjmHvIXVfz+f1eAEmYGZ7NV+xSSu\/Aff7TXbaGzgHPfRZPOEHjWfQB5h12+YaxZtXvmwy36a9v6jOWMsfZDftsZZP15RPJZZtu+doFgGmqm1Tn2Nt+yC77vnvt6Ix7Z7tfn6XV\/\/69eusXs\/qjLW7t2f1IYEBb4ehJQWYUF7FCRAgQIAAgQgBASZCVU0CBAgQIEAgVECACeVVnAABAgQIEIgQEGAiVNUkQIAAAQIEQgUEmFBexQkQIECAAIEIAQEmQlVNAgQIECBAIFRAgAnlVZwAAQIECBCIEBBgIlTVJECAAAECBEIFBJhQXsUJECBAgACBCAEBJkJVTQIECBAgQCBUQIAJ5VWcAAECBAgQiBAQYCJU1SRAgAABAgRCBQSYUF7FCRAgQIAAgQgBASZCVU0CBAgQIEAgVECACeVVnAABAgQIEIgQEGAiVNUkQIAAAQIEQgUEmFBexQkQIECAAIEIAQEmQlVNAgQIECBAIFRAgAnlVZwAAQIECBCIEBBgIlTVJECAAAECBEIFBJhQXsUJECBAgACBCAEBJkJVTQIECBAgQCBUQIAJ5VWcAAECBAgQiBAQYCJU1SRAgAABAgRCBQSYUF7FCRAgQIAAgQgBASZCVU0CBAgQIEAgVECACeVVnAABAgQIEIgQEGAiVNUkQIAAAQIEQgUEmFBexQkQIECAAIEIAQEmQlVNAgQIECBAIFRAgAnlVZwAAQIECBCIEBBgIlTVJECAAAECBEIFBJhQXsUJECBAgACBCAEBJkJVTQIECBAgQCBUQIAJ5VWcAAECBAgQiBAQYCJU1SRAgAABAgRCBf4f1U79UuGVZTMAAAAASUVORK5CYII=","height":0,"width":0}}
%---

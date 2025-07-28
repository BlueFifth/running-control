%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88];
dq0 = [0,0,0,0];
X0 = [q0, dq0];
Sim.time = 0.40;
tspan = [0 Sim.time];
options = odeset('RelTol',1e-10, 'AbsTol',1e-10, 'Events',@(t, X) collision_event(t, X),'MaxStep', 1e-5);
tic

Animate = 1; % 1 = drawnow

t_total = [];
x_total = [];
te_total = [];
count = 0;
% Using ode113
while tspan(1) <Sim.time %[output:group:1875f705]

    % solve (with event detection)
    [t, x, te, xe, ie] = ode113(@(t, X) halfleka_dynamics(t, X), tspan, X0, options); %[output:85c843d4]

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
end %[output:group:1875f705]
toc %[output:53ad3cb2]

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
plot(t_total,x_total(:,1)); %[output:45d9e1dc]
hold on %[output:45d9e1dc]
plot(t_total,x_total(:,2)); %[output:45d9e1dc]
plot(t_total,x_total(:,3)); %[output:45d9e1dc]
plot(t_total,x_total(:,4)); %[output:45d9e1dc]
legend('x','y','th1','th2'); %[output:45d9e1dc]
title("Generalised Coordinates"); %[output:45d9e1dc]
xlabel('time (s)'); %[output:45d9e1dc]
hold off %[output:45d9e1dc]

% display derivative state of all variables
plot(t_total,x_total(:,5)); %[output:0083aff1]
hold on %[output:0083aff1]
plot(t_total,x_total(:,6)); %[output:0083aff1]
plot(t_total,x_total(:,7)); %[output:0083aff1]
plot(t_total,x_total(:,8)); %[output:0083aff1]
legend('dx','dy','dth1','dth2'); %[output:0083aff1]
title("Derivative of Generalised Coordinates"); %[output:0083aff1]
xlabel('time (s)'); %[output:0083aff1]
hold off %[output:0083aff1]
%%
%[text] ## Forces
coords_total = zeros(8, length(t_total));
foot_total = zeros(4,length(t_total));


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
      

    foot_total(:, i) = out.foot;
    controller_total(:, i) = out.controller;
    constraint_total(:, i) = out.lambda;
    contact_total(:, i) = out.contact;
end
%%
plot(t_total, contact_total) %[output:2d1e3a2e]


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

sim("data_inspector_Baleka.slx");
%%
%[text] ## Animate (drawnow)
if Animate ==1 %[output:group:4d0156b3]
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
    fig = figure; %[output:154e240f]
    
    ax = axes('Parent',fig); %[output:154e240f]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:154e240f]
    u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:154e240f]
    u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:154e240f]
    l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:154e240f]
    l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:154e240f]
    f = animatedline(ax,'Color','r','LineWidth',0.5); %[output:154e240f]
    
    axes(ax); %[output:154e240f]
    
    axis equal; %[output:154e240f]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:154e240f]
    
    hold on; %[output:154e240f]
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
        addpoints(b,x_sim(i),y_sim(i)); %[output:154e240f]
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
    toc %[output:0ef3d5a4]
    hold off; %[output:154e240f]
    clear fps b f l_l l_r u_l u_r ax fig l1 l2 l3 l4 l5
end %[output:group:4d0156b3]

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
%[output:85c843d4]
%   data: {"dataType":"text","outputData":{"text":"Time = 0.3990","truncated":false}}
%---
%[output:53ad3cb2]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 10.863725 seconds.\n","truncated":false}}
%---
%[output:45d9e1dc]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACxCAYAAACGEnurAAAAAXNSR0IArs4c6QAAHKxJREFUeF7tnX1sF0d6xyeQOA1xSJRLABuMxR\/Iiu6UCHwJItEduT+i63GIBglBgnoFBbgIESJd5VBEETQpoQ5xXxTOiigmB\/\/QxlJERBFNFV0LzeXFcIbSJkqRqxJwsIG8KGAOJKdw1TM\/Pz\/mN96X2fXs7uzudyXL\/nnn7fnOM5\/fM7Ozu7fce++9vxc4oAAUgAKOKXAL4ORYj6A5UAAKSAUAJzgCFIACTioAOMXoljlz5oitW7eK+vr6au6BgQHR1tYmTp8+HaPEZLPs2bNHzJgxQ3R2doozZ87Itl++fNl6e9etWyeWLVsmjhw5IjZs2OBp1JIlS8TatWtFXV1d9fy+ffvEjh07khVBKZ206OjoEBMnThSbNm2SZ5LSJDWjClgR4BSxU3kAnjp1SqxYsULmZljR3+TsPT09EUtNNrkrcPKCV3t7u5g3b14g0GyrkzScyB\/WrFkjXnzxRSe\/rGzrmVR5gFMEZRlCSUQdEZoROakKp+7u7sj5TTMERU6s3blz56pQNy3XdjodTja\/TPLqI7Y1tlEe4BRBRZNpCxfHA6CxsVH+S420OFo4duyYeOihh+QUR58WElBaWlpkXvUcO\/\/w8LA8R1NLmq5NnjxZTqn4UOsLi5zUuii\/Os3Sp2HqOdVGas\/JkyfFww8\/7BkFRdFOr1O1hdoXdJ7Pffnll3LaxtHsI488UtXns88+k5r7TevIBpp6ku6UjvqQ7COdGe7ch6w3TWV37twpp4vc52oev\/6k\/EH6R3DPwiUFnCJ0KTtk2BqJ\/s3c3Nwsnf3DDz+UazFcDg86vVwvmHDEwXCiQcODhQekXj63MwhOCxYskIOW0h48eLBmLYbbTetoNIWlds6dO7dar9puzksD02vNyVQ7toXr1CMRBkfYeQIDT7H1MtjmK1eueK45cR1chq4Dg9ZEX2pnUH+qZen624zoIri5M0kBpwhdoQ8wPTriqOPChQsSRjyAOB2dp0XzZ599Vq6zsHOrUcX+\/ftHLc6Sc0+dOjV08ZbLYZNMBg8PVD26o8+6vSoEOUrg6IMGUlB0ZAonHYDcDobi7NmzawCpn6fPqvb0WW9X2JpTGAD5oocaPXGUpF9wmDRpUmB\/qhGdHiFGcM1CJgWcInSr6eBjOKlXpKga\/qZetGhRKJzUK4GU18\/5aaAwNCgdRVM0gFX4BUVOlEeditBnnkYyRHWJaBC9\/vrrowZdkD6m0zqv9TEVbK2trdUrj\/oUi2CsfzF4QXascNIjL1r89rsaynDy60+a2vvp7+KV3wjDZcxJAacIEuoRkOo86gDyGiBqNXoUERY5qXm9Flz18kymiV6L+upaDk3N6FAhp7bDa1HZxoJ4HiKnLVu21AAyCP5ekZOfy+n6+23HiOCyuU4KOEXsPq+tBOp0ynTtxm9aRw4Z1dnVy\/E8LaRvapNpnd8U0ysKCYJg2JqT1\/RKjWp4ncrWmhNPqamOuGtOfutaBCe6WKFqxFErR0L0mfe9BfVnkP5p7v2KOAxSSQ44xZDZaxMmFeN3JUudKpHDB0VO\/G2pXsHh6SCt63hFTvraF12NooVpXiAPu1rndeWJ26FfGdMXu7mdYVfrWGaTTZg2rtapcKK61TJNr9b5wUmfqtE0l2FFQFH1VL8g+Oqr2p8qoFmjoE2sMdw1t1kAp9x2HRoOBYqtAOBU7P6FdVAgtwoATrntOjQcChRbAWfh1NDQIOgHBxRwQYFbx11KvRn\/d+Pu1OsMqnBwcFDQT1pHpnCixVQ6+AZaNpqgRLt7ab+Oa8dtAU5667jLwuv8bbeMduzbxl32NS1sIATl9Ss0qN2U59uRgaCmo\/+F5dPr88sTpyzX+j6L9lwe\/p44f+0nWVQ9qs7jx4\/LvW1pASozOHldkmc1CEq0mXCsQjQ1TpBFTp9a+c1\/nz13VTz2yP2VzyNp6O8mJZ0KgKgDVO9Vqk8\/OCo8+tv\/8XS8swOj85iUa+rF\/R5tMs1rmm727Fli\/k9\/KnZ37U7NoU3bZitd0jauX\/uAnEHM\/5OTtpocq5xZs2aJVatWyd33BKk0jkzgRJfD169fL+0bGhoaFTkRnN7YuUHs7uoSX144Jc6e+91NmKigqQHLnaNAFCSgCoz+c7+rJlWhUJvmJiyoPerRr4HEC0ZecJo\/f744dOhQYQeuHFSwcUzjmL5Yf\/lyq2iaeqeY9cQ7YyprLJk5YCg8nGg6d\/78eTFlyhSplz6tIyHeemOBmFj38U1oKN\/0OkwYBhQNEDimT62AiiHC8DCBxlg6EHmhQFIK\/HJbq4zyF654L6kqAsstBZxoMxzdW7Zx40ZBO2394ETTuq6urkJHFpl4GSrNpQIUQZ149w\/F+0e\/SBVQfGGK14ELGznRTuZt27YJusWCbtr0WxBnSpMXEaB2796dS4dCo6GATQUee\/g+cWDvD8UrnZ+K7Z2f2izat6yVK1fKtSY+Cgsnr1sXyGj9URHqgjgtvqV1dSCV3kYlTiiQ160qTz85XTz1ZLN4bmOv0Nc6xyKs3zYB1qk0C+IsYljklCalx9KxyJsvBVzeqpKVkmHbBEqx5qSKDzhl5YrlrtfWVpWiqGgSFZUOTn6dm4UQRXE02BGuAPyrViMTPUzShCsfLUUm+5zCmpiFEGFtwvniKAD\/ApxiezOcJ7Z0yGigAPwLcDJwE+8kcJ7Y0iGjgQLwL8DJwE0Ap9giIWNsBQAnwAnOE1sBZExSAS84jbu7QYy\/u\/ICVBeP65cGxI1LlceVqFe56Qb6xYsX17zwM2r7TWBtkiZqvWHpsSAephDOF04Br4E24QerxYQf\/NxZW6++9\/fi6nu7ZPv4Tot3331XPPHEE9U7LuI23gQ8Jmni1u+XD3CyrSjKc16BvEdOJDC\/6KK3t1e+RXoshwl4TNKMpQ1eeQEn24qiPOcVyGKg2RZFf4XWWMo30cMkzVjaADjZVg\/l5VKBLAaaTaHUG+jpeVkUPY3lHXcmepiksWkjlYXIybaiKM95BbIYaDZFeeutt0RfX5+czlEEtXr1arFr1y75pI84h4keJmni1B2UB3CyrSjKc16BLAaay6KY6GGSxraNgJNtRVGe8wpkMdBcFsVED5M0tm0EnGwrivKcVyCLgeayKCZ6mKSxbSPgZFtRlOe8AlkMNJdFMdHDJI1tGwEn24qiPOcVyGKguSyKiR4maWzbCDjZVhTlOa9AFgPNZVFM9DBJY9tGwMm2oijPeQWyGGg2ReFXq\/FWguXLl4uXXnpJ9PT0xKrGRA+TNLEqD8gEONlWFOU5r4DXQJt213jRVD\/e2bb3X7kuPh+6LttHN\/u2trbK9z22t7fL9z\/q736MYogJeEzSRKnTJC3gZKIS0hRKAa+B9ovZd4o\/ba131s6\/6b0i\/vZ45U3T\/Mbs7du3izVr1mCHeJq9lgWl07QPdWWrQN4jJ1KPpnanT58WDz74oCBIxZ3SUVkm480kje1eReRkW1GU57wCWQw026LQbStLly4VQ0NDY5rSAU4Re6YIzhPRZCRPUYEi+Bfd\/NvR0SEOHz48ppt+AaeIjlcE54loMpKnqEAR\/Isip7FepWPJTfQwSWO7CzGts60oynNegSwGmk1R6GrdsmXLxJEjR8b8oDlEThF7Ju\/OE9FcJE9ZAfhXreBBekyfOkEmfurJZrHoj98Qa9euFfTq8jQORE5pqIw6nFIAcAqH09NPNoumqRPEn619QCb+3\/5bxS33PCN+vmYj4NTZ2ZkqpZ0aPWhMogoATv5wapn+lXj6yenisUfuF2fPXRX\/8PYZ8f7RL8S169PlG14QOc2enboQiY4GFO6UAnmHE23C3Lx5s9i7d6+YPHmyWLhwodi0aZPnXieTq3qsx++\/eUO0NH8lofTcxt+K9499We23LDTDtM6pYYPGpKFAFgPNpl3qo3mD4MRvaKmvrxf79u3z3XLAevzm3VfE2\/\/0mxoocbuz0Axwsuk1KCsXCngNNFr4bWqsLP66ePQPXJURDR30DPHGxkYxMDAgTpw4IebNmycuX75c\/V9bW5tMt2XLFnHo0CH5jPEDBw6EwiloygY4jXhFFkK46JBoUzIKePnX+rUPVBd\/k6l1bKW+0vmp2N75qSxEj5xoWwFFRkePHhVbt26tARFHT4CTgf68R4OTeoWbgJOBkEgSW4G8R05+07qLFy+O2jUOOBm6iXo3Nd2oSKDyWswDnAwFRbJYCuTdvwCnWN0eLZMf1fPuPNFUQOq0Fci7fwFOKXiM3wsB2Xm6urrkgt7g4GAKrUEVZVEg73DiL3VaBOcFcdpKkMS0rqGhQfAP1VGKfU68\/4LfXKoODHYe+h8Bavfu3WUZN7AzBQXyDifbEgXpsXLlSrFq1apqlYWHE4PJ71k0LBZdeaD7eBA52XbHcpcHONX2f5AeHDXNmjVLQqrQcOKQtLe31\/eOajhPueGRtPXwL3M4ccosNEt1E2bQVM5rWpcmpZMeECjfHQWyGGjuWD+6JSZ6mKSxbWOqcKIFcAJOXV1djR36XqcshLAtLMpzV4G8+5fJvXUcCNBOcjpOnTrl+zhfEz1M0tju8VThZNr4LIQwbRvS5V+BvPuXyb116iujwpZSTPQwSWPbMwAn24qiPOcVyGKg2RTF5N46ejOLeqgv4tTbYqKHSRqbNlJZgJNtRVGe8wp4DbRJdzSJ++9ocrbtX1zrFxev9cv2Rbm3jtKr08Du7u5RNpqAxySNbfEAJ9uKojznFfAaaEtntomlM19wtu1v9r0q3uzr8IQT3wLmtQkzyvOc8FQCg+7PgtIGzUKSgihQtMjJD05hERN3p8l4M0lj2z0QOdlWFOU5r0AWA82mKCb31tHjU9avX2\/0NmATPUzS2LQRa0621UR5uVAgi4FmUxiTe+taW1tFS0tLTbV+r5Iy0cMkjU0bASfbaqK8XCiQxUBzWRgTPUzS2LYR0zrbiqI85xXIYqC5LIqJHiZpbNsIONlWFOU5r0AWA81lUUz0MElj20bAybaiKM95BXig0eN46HlIZT\/oyQNhz2oCnEa8JAshyu6gZbKfByP5GY6KAvRoInpEkd\/jibIYk4ic4J2lVICfU1RK4z2MJigFPTcNcELkhLECBZxUAHACnJx0TDQKCgBOgBNGARRwUgHACXBy0jHRKCgAOAFOGAVQwEkFACfAyUnHRKOgAOCkwKlr0xrxL92\/EsePj22T3OdD18W0u8ZXvYs+xz36r4TnHUv5cduFfFAgaQUAJwVO+55qEXf2\/WvSmoeW7wc3Ah6fU4GkglAvPAhcBD4+z2Xw5\/6h66LprvGCfnsdHw1+KwGstscLpABnaHcjgY8CgFNC07ogYJh4Y1P9zcgrKH1QPdPqx\/lmJfDoh1dZ1A4VOmq7dDjpgKPy1TRcXxWAI1GhCuMPB4YlFOnQwfj5lRue0Si3DyA08az8pAGcEoJTflxg7C31gxqXrJ\/XoamCUk2rgzBKS1VQqXBVI0MuT40CJRSVqTSAF0V1u2kBJ8DJrkelUJoOO4aYGqV5RY1zG+uq01BqJuUzjXC9YOcFOo7uPhwcluUjqovvEIAT4BTfewqW0ytq4\/+psONIzy+9vmaoyuQ1pdWnsAQ4NV2ZordxdzeI8Xc3inH30O8G8evtz8iX4tJNwmkcuPE3DZVRR2YKBEFOrsONrAVSJMeHVxTndfGD06sXM3htjqO2PICNIETHbc2t4rbprRJE9Dcf1y8NioaGKeKv5l4XL74AOInOzs5UKZ3Z6EHFTiugw039TGDjq6g8TZ3bUOe5dYWnuaqxflBLCmhqJEQQokMFEUGIjm\/P9FZ+n+0VN74ZlL8xrcO0zumBisZFU4BBpq7DqdEa\/U1wU9N57cljsDHMqp9HtpboU88vZvykEgmNREHj7mmUEFIPBhABSYWQn4WAE+AUzfuRupAK6LCiCG38PY3S1uZp08S0+vHi8yvX5e\/vf\/tfvhoMjJ9UPTcwbrLMQ8e\/f\/SR\/M1TWnVbCGfQ19bo+VcvdP5jqrMZrDkV0r1hlKsK8PoOLTTTQYvNdFBkM27kf\/yZp1l\/8OCCUeZQ5EMR0Y1vBgSlu\/\/0P8s0U77+jwp4KCIbWU\/zu2hgenW0Cqxn3gacsgghXXVmtMsdBRgsFXiMhgu3lCCjTqPUaRWBhM7xb8rDECLQ8FH936XK\/+gzrf9U\/h4QN0bWh2yr4weshT\/8vvjZpr8DnAAn2y5X7vKCoOIXtVRBMzKd4nQqTFQAMUzoPENGBYx6ntd7kgJMEr2dxZjMZFq3Z88e+TbS4eFheVWuu7u7Rs8shEiiQ1FmdAW8QEKl8PRHhYT8\/0gEc+PSgFwAViMSr4VgvUVBUJGgGYlcvj1T2dtDUQtHTUlGMNGVSzZHFmMydTi1t7eLmTNnira2NrFgwQKxcOFC+Vqanp6eqrpZCJFs1xajdBUcFUhUpjY6PFSAqOsoo8AyEpXoV5LC1PKaBtEVJz5unq9MgwhsPCViwOQpagnTI43zWYzJ1OFEUdP58+fFhg0bBL\/z\/cCBA2LHjh2AU4iX6XDQAeEFCb9IQ61KhwNFHDJq+GaguhmvsuYxIBdh9YPXUPg8f+Z0Xmsp1XPKmsrN9BWoMEhkWxJaY0ljYBehjsLDacaMGaKjo0McPnxYwog\/9\/X1SVjxQUJsebVTdO3eLbfKB72yJqzj1W\/3sLR+gzsonz4QeXObVx6vCEGdhnCeIAAERRkcMfgtuKptCgJGBQY3F2Jr890EhwoPAMTEu\/KXhl+hZfLiTdvWpRo56ZFSEJxmrn5d\/Lrf\/zEjQUJ4Dfib38S1V0vUNYco0ws1n1db1MGvnvfLxzAIS6tOT\/Toohp5IMqwPU5KW97KlSvFqlWrqvYX9t66qJHTuo0vjylqqh3oNy\/TmngaphEmKiFN0RXgyGnWrFkSUoWFE3Uk1pyK7s6wr4gKFH7NiToNV+uK6LqwqegKlAJOHD1hn1PR3Rn2FUmB0sAprNOyECKsTTgPBcqsQBZjMtWrdaadm4UQpm1DOihQRgWyGJOAUxk9DTZDgYgKAE4jgmUhRMS+QnIoUCoFshiTiJxK5WIwFgrEUwBwQuQUz3OQCwokrADgBDgl7GIoHgrEUwBwApzieQ5yQYGEFQCcAKeEXQzFQ4F4CgBOgFM8z0EuKJCwAoAT4JSwi6F4KBBPAcAJcIrnOcgFBRJWAHBS4PTayzvFzr\/eK5+EGef44lp\/YLaLIefj1Ik8UCApBSbd0eRbdBq+DDgpcHr5x93i2sm7PDuEOoM6S+2UoM6L4zBBHX7xajD4qL5Pvn5fTLpjuqz64rWz1b\/1tnws03k7HtsZp\/16HhMHtq2hjXYHlcH6mtYT1A9chkka0\/pspLv\/jiYxaUKTr4+odah9\/MlXH0i\/U4+Pv\/pA0Je2iS\/obQectMjpL7duFSciRk7UmWEHdbbfETZAwwaESf1Ud1AbGH56Gvq\/\/r+w9lbgWIG5rSOOc9uq27Qcky8Q07LC0tGAN+33sLL081Q2fYHJfvT5UmSf4D4mH+X2qOe43\/iLncqjL9F\/+\/zNUGABTgqc6H12aT4SNKrTID0UyJsCBCU1EvvuvY9VozKKqghUb\/Z1eJoFOAFOefN3tDfnChCwfjRtqfjRtKeqSyU7Tj4vPvn6gxrLACfAKeeujubnWQEC1XMPvia+951HBUVSv\/zP56vTPcAJcMqzb6PtBVHgu\/c+KtY99JqMpN7se1VO9QAnwKkg7g0ziqDA0pltYunMF2T09N\/Xj4ilrzya6jownudUBC+CDVAgIQV4TWrZw78Q3\/nZgHj+z5+NvfcwahMBp6iKIT0UKKECP37sj8RfdGxA5JTF\/LaE\/gaToYCxAlmMSUROxt2DhFCgvAoATlgQL6\/3w3KnFQCcACenHRSNK68CgBPgVF7vh+VOKwA4AU5OOygaV14FACfAqbzeD8udVgBwApycdlA0rrwKAE6AU3m9H5Y7rUAp4LRu3TqxbNmyakfs27dP7Nixo6ZjshDCac9A46BAxgpkMSZT3YQ5Z84csX79erF9+3bR09MjCFQLFy4UmzZtkp\/5yEKIjPse1UMBpxXIYkymCiddfYLV1q1bxYEDB2qipyyESNszGhoaxPz588WhQ4fE4OBg2tWnUh9sTEXmVCrJYkxmCqclS5aI1atXi127donu7u5SRU5ZdHYqXqxUAhvTVjy5+rLoy8zgNGPGDNHR0SH6+vrEhg0bPNecurq6xIkTJ5JTPMOSKaqg6SxszLATLFRdhn4kmdjONJ\/rnyic1MXvK1euVNeWGExDQ0NixYoVo1yEhSBa44ACUMANBegdkrQMk9YyRKJw8pKU15l6e3tHRUxqegIU\/eCAAlDADQUISmmBiSxOFU5BUzk35EcroAAUcEWBVOFEC+A0Z62rq6ux32uvkysCoR1QAApko0CqcMrGRNQKBaBAHhUAnPLYa2gzFCiBAoBTCToZJkKBPCrgHJzUdalTp055bjXIm9BRbdqzZ484f\/584NVM1zQwsZEviDQ2NsrmDwwMiLa2NnH69GnXzPFsj4mNlJH6r6WlRZZx5MiRwvWjKo7fRmobHeoUnNTbWQ4ePOi7SdOG4WmVEdUmduw8ObWpje3t7VJ22nSbtyu3pjbS3r7W1lb5pep3e1Zavhe1HlMbuVzuw\/vuu090dnbW3OURtW6v9E7BiSi8fPly8dJLL8kbgcmZZ86cmatvV11kU5vUjalURp4iJ1MbdW3y1L9xbcxTFBzVRuq\/5uZmQXDSb0ErHJzoW+fxxx+vwsjvqQU2DE+rjDg25cmhScc4NvL0h3573SWQVv+Y1hPHxrxFTlFsZJC988478skihYeT\/k1aBDjFsSlvcIpjY976NqqNlH7evHm5WleLYiP5KN3lceHCBc+b902hH5TOqWldFHLbMD6NMuLYlDc4RbWR0i9evDiRdYqk+jSqjdyOPE1dTW1U19VKsyAedc6blCPaLDeOTXmDUxQbabDSgrH+gEGbmidRVhQb1frzFCGa2qhejWRbh4eHrX\/ZOBU5Rb1akIQT2i4zjk15g5OpjXkaqLofmNpI8J0yZUp1HY36Mi\/raqY2qtqUJnIio033ktiGSJLl+dnkB6G8wSmo31RbvL5x87TXybQfVTvzZJ9pP5YWTklCAmVDASiQHwWcmtblRza0FApAgaQVAJySVhjlQwEoEEsBwCmWbMgEBaBA0goATkkrjPKhABSIpQDgFEu2cmRK89I\/7zjW3\/6sKm2Sphw9Uw4rAady9LORlfq9YGnBSd1xHNRQ\/Y3RRkYhUW4VAJxy23X2G57Fjar0NIZt27aJ\/fv3Gz1yQ33sin0FUKJLCgBOLvVGhm3RHwRHL52gg+44p1tN1qxZI26\/\/Xb5cgp6WBy9h5DuRKc3NtfX18vP6i0pfOMrlRG0EZE2Ni5atEhs3Lix+tC5oE2M+v1fGUqGqhNWAHBKWOA8FR80rSM4EcDooWLHjh2TDwKcOHGiBBId9LJFfhehPh0MuoVDv91Dh4++W57auHnzZrF3716jSCtP+qOttQoATvCIqgJhcKKE\/OwlFTj6Uy11oOg3lOqL3OqD9fgt0X6vC8ti6gkXyUYBwCkb3Z2s1Qacdu7cKaMqfk44G+p317rXfYTqa+wpvwoqBuHhw4dF0JU9JwVGoyIpADhFkqvYiW3AiZ4PHuXG5bC0+iNWEDkV2wdV6wCn8vR1qKW24KSvOQU9w0lfc\/J6GqP66GasOYV2Y2ESAE6F6Uo7hqhvfzl37lzN1TrTNSdKp16tC3oQWdjVOj0vrtbZ6ec8lAI45aGXCtzGqPucwqaBBZaqdKYBTqXrcvcMxg5x9\/rEhRb9P+yZrt31P8ThAAAAAElFTkSuQmCC","height":177,"width":295}}
%---
%[output:0083aff1]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACxCAYAAACGEnurAAAAAXNSR0IArs4c6QAAH9tJREFUeF7tXQ2QF0V2f4IuJ2yQ8CW7fKyEMujFqMsGKaQ8iFVULpzZyIUCpUygDohSHKZyhdyGI3BQSJTbSy4FG44AF0g8iJQ5dI\/acCFVt+gRAlsLhTFyZGOoFfcTMAor4kYw9Qben\/73zkx3z39memb+b6oodnf66\/369W9ev37dfdvQoUO\/AH4YAUaAEUgYArcxOSWsR7g5jAAj4CDA5MSKwAgwAolEIDHk9NJLL8H06dP7gLRnzx7YvHlzYPCo3ELLcWvArl27YPz48VBXVwf79u0L3EbTjFOmTIENGzZAaWkp9PT0wOrVq+HYsWN9isH2TZw4Mff33t7e2NuqK9vcuXNh2bJlcPbsWVi4cCFE1W+E3aVLl2DFihVOfV59K2LX3t7um15XTpN0MgZRYWLSpjjTJo6cRBKhzjh8+DDU1NQEwiXMDsUB9Pjjj8Nzzz3ntMUWOalkogGIbRSJi8gqCqIO1DlCpqSQE35samtrYfDgwTnsvPAsVGZV\/qjJCctva2sr6OOvkqGQ94kmJzdFKUTYQvLKg6eQsgrNqyInW6RZiFxx4auynBDbqVOnJsLCVPVzIXhHWXYh7RLzJpqcsKGyspASl5SUOHKIVgANSjTB77nnHkCLCx+cLmK6qqoqGD16dJ8vIpn4mBa\/muXl5U4+mga1trbmplH4d5pKLV261JnW7d27F2bOnOnkoamCPNiIaKnsM2fOONMXr0eckonTMXn6K5ejGnxyfWI94tRFLAfrRzxlvE37Ytu2ba744pRYZTnJdcnTLC85SIfIZXDq1CmYMGECuE3rTD6G4tQa63Cb9nn1IaZ309X9+\/fnTdffe+89eOihh3I6LhLK8ePHnbQoh1f\/yG0kvX3kkUdg\/vz5OVUgHfLrTxX+YRFS6siJyKWrq6uPX0L8ypEyiNNAsUNRcOwUer98+XLndyI4zE\/khWmp85FwJk+enFe3qGDoc5o0aVKOBNFHJpJqU1NT3lShoqLCKevo0aOu01XZ8jEx700sELEeImA085E0SbHxI4Dy0Xsa1DIe8kfErS9M8BVlPnDggIMfkf8TTzyR149+cniRnhuZ6BK7nI4+auJUUNWHXviQD1P8IJJ+upGTV\/9Qmwgzt\/6icYX66oYTjS3SXy\/8oyAmLDMVlhOBiMThBigNcrfpjNfXBgln7dq1eZYUAiJbOKTEKnLCvOTQXbduXd5gkvNSHaKlRR1Mik8kgX+X\/+ZnkruRk5u1tXXr1jzyRcewH3nIA3D27NnGfWGCrxs5odUpO\/XdCEWUQ26nHwHpkpP8UUO5xL+RVePXh7KuutXt91ESLSfUZTeCFHUHF0\/wIStJ9cEjPcKxJVq8cS6qpIKciMHJOpGZmgBXkZNo0bz22mtQXV3tOARpekVfMyxPHrwqcsKpCQ2K+vp6mDNnTs4ykk1iar\/bSpsbuciK++yzz+YRg4iH3wATyyb5SGmpDHkqK095yTqgQW\/SFyb4yoOHBr9YH1oUNEi95Jg1a1beiqofPrrTOrePA7UPrfITJ070sbLlevHDKK70+n1U\/Cwnr\/7p7u7OTaHd2uRFTl796YV\/ISvpKosr0eQkK4v8FZSF0yEnUgIkhqFDh+amdLLyjBw50mhah+REHfjhhx86y\/wUYmAy1SrUckJMvJy6buTktaSumrqY9oUpvn7WIZEc9uErr7wCzzzzjKsPibAQrW2VdaTjEE+D5USuAwrNUPn0TBzkIv5eYSwq4tF5n2hyEr9GGEqgAliHnMRpm5sDGEFDwGnwURoiK9nSEr9+ogNSdFTLJCsrjopkVSa4nN9t6dutbW6+GiIrmZzlaYMsg9xGr2mLF75+PhGvKQxNi0UrRMc3hmTlFbekE0qA9Yr+SLcplY7PSY6Rc+sP\/MgFsZyofy5cuOAs0iBGGLdFeikTrN\/YUuHvFSumQ0B+aRJHTnJj5ZgceYokOr91yEn8msrxU6Lpil9lHKiykxM7mKY+8pQBy6avitxuL1+WThCgPM\/X\/cqJq0WEq7y6J6YRp5kqywmDPk37wg9fFdl5rTxR8KmXHCi3WK\/fap2oezJ2MpmFtVonBvCKOoJ9obta5+VzEn2NaM3jc\/XqVYes6OOD5Cf7VWklXBwfKvwLJSK3\/JGQk2ogUsfH6VyLAjwukxFgBKJDIBJyQsbGB6diRFQtLS3O7\/ju3nvvddgbl4TRKR3lvDU66LhkRoARiBKBSMhJbrBISDj37ezsdIiKTEVc3YrS6x8lgFw2I8AIRINALOSE0zh8KP6nsbHRISPZqhJFLCsrA\/zHDyPACCQDgY6ODsB\/cT2RkxM6I2nqhkLhKgdZSl7khKSEUz2Ma+KHEWAEkoEAxm\/h+I2LoCIlJyQmDEakFQkiI5XlhKSEeeIEIu7ur6yshMWLF7OMcQMfcn3F0I8IGcmJuyCQpOJ4IiMn9DPhRlvZ2Y1TPJXPicgpTiDiAFuetmIoQkNDQ2xfIpYxfATQys96PyJqNsZkJOQkTuXkQ9B0VutsABG+2nKJjEB2ELAxJiMhJ7fgPzGITRXnZAOI7KgRS8IIhI+AjTEZCTkVCo0NIAptM+dnBLKMgI0xyeSUZY1i2RwEOCxFrQiqMAEmp5sY2gBC3X2cIo0IcFiKXq+pwgRsjEm2nPT6jlOlFIFiCEsptGt0wgSYnNhyKlTPOL+EgI1BlbZOIIxeevEFWDTnNqf51QvfyhPDBo5sOaVNk7i9RgjYGFRGDUxAYsLoi49+BBe6fgnTHhkBL9edhk11p3Ots4Ejk1MClIObEB0CNgZVdNJEUzJhNGbQP8JTi34MT8+ugKefrIBhX\/4Jk5MMOStUNEpYjKVmTZfkLWBh9KlMTuNGD4ItG6ugesGbcKTpglOFDRzZcgqjd7mMxCJgY1BFCUbU5DT2wb92mn\/y0FfhSNN5+OaqZiYnsUOzplBRKiuX7Y+Amy71u6sM+t914+LUJD7XPm6H6x\/nH01CuyrouN2DBw86e1fxwduD5E32JnKJPqdHf3e7kxUtJ3FqZ2NMsuVk0oucNnUIuA2qgY8tgYGP\/XFiZbny1t\/ClbdukAQ+4l5VOmsdrzbDy0Y3btwIhw4dcm6cxhuD8RYg08eNnJCYxKkdk9NNVG0AYdqhnD4dCGTBcqLDGtFCkqd1dJpsc3Oz6+3ROr1EGLW+u8VxiNMjTu1sjEm2nHR6j9OkFgEbgypssPzIyeRORK92EUa\/OPQyvLDm9VwycWpnA0cmp7A1ictLFAI2BlXYAKimdTidwzOl0HoKchY\/YbRn11rYvO1fcs0Xp3afXhvnHAAZ5xlrTE5haxKXlygEskBOCKjsED969KhzOiXdaoQW1JIlS2D79u3GficvcsJ6aWr3o9e+YHJCQLKiUIkapUXaGNYldceL21feOPCLvAw0tdv7eit8bdbX4A++cSD9x\/SqIfFOwQpVCHqcV0SAdUmtDyqMLr77daeQS70PMDmpwFLDzSkYgRsIsC6pNUGFUf2ux5z9dhevPgpPLd7DllPczjd1F3KKNCKgGnhplCnsNhNG3\/rhP8MvS3\/LKf6zt38KV98+4ASDTps8HP7p734PLvX+BpMTK1TY6le85WVNl6LcvoLkdOp\/OuCOcVVwR8WN6PP\/a22Gq\/9xAO6+E+DVH3yHV+uyplDFSw32Jc+aLkVJTmKYAG3xuaNikhNNf+3jDmh4ZgSTU9YUyv4QLd4WZEWXvPbW0R2QGEqwYMECWL9+PcjXsal6XwcjnTSqekzfW4lz4quhTLuJ0wdFwG1QjfmV\/jC2tH\/QIiPPd67nGnxw+VquHq8gTEyAm39xWwveBzlq1CjnZ9NHh3h00pjWq0ofOznxpZqqLuH3YSLgNqj+dNIg+FZVaZjVhFrWXzb3wF+d+CRXptf2lePHj8PKlSth06ZNsHTp0oIjxP2iv4uCnIJcRz5u9ECno6ZNHnHj\/0eGO\/+PKx8I77dfyf3spyGUjtK833YjHz3nhN\/fb7ulGPj+3M065DyhamRKCkNfhOpRHUfSb4hOGf5p+imOPLn+cTtgmocnjILvPfVwnq8kbZaT3946fHf27Fl48MEHHZIyndJhX+oQj04alV6Yvo\/VcpKdefQ7heBT48WI1ZmPfuqQ0NjRgwBJigjiXNsnOWLSERrLEB8sjx4iP69y3Eip\/11lufpHD\/rYyYptypHfTULD393y\/\/t\/3QGUr+P2+\/Kqbuu5K+\/3\/kPK4QPpb2ICHIi4wqJ6+g3xPsPo+kftTnZcpUHnJz0op+5D+SgP\/o4\/i\/+blCu2w6sN1G6v9785\/Is+5KQrT1LSeU3rcB8d+prmzZsHly9fDjSlY3K62ct0vEN9fb2zQdGPnH74Z9+F\/m3n4WLJNWcQ7\/t5LzSdOJXTlzuH3VKdAf3boL3\/3coB1Tn0YVd9wwEk5ifSwMSjS28Qj\/PzTRK68feP8soak\/fuVh7K1\/bJXU5+sTyxgKsf3hKI0ua9\/99h8KVfvdin\/e+d+zXnb1dvvhf\/R5L70tBbecQ6MB3WY\/KIJEukMHLgWM8irn2Uf2CanFA+UM2kLbppSyo+hTmrvhLrKpNu20zSuTnExTHU2NgYaNOvipzoQlK6\/y+zG39NLKcXf2cfXH\/nstN\/7f1HwrDe22FA\/3b47Nqtr\/\/Fks+h\/Fp3Xh+X3N4GvZ+PBvwfH\/Fn\/B3LoEcsS\/y72\/v2frfIT1YqbB8NtJF39h2sogUgvu\/+9Fwf\/cT39Hevn8VMbvXpKr1b\/ZS3+8o5QOKh\/5EYkdBUj5zOjVBVZei+12nPmPuGw7A\/6kg9OXlhUsgqHZVJM5X\/3rkSzrecgnOCM37opJnOnrrPyh5wkmeWnFA4E5\/Thg0bnFB5vCoZH3EgjhBIgL7ebgN15J3jPHVdLENnQJx3IROvfBOGdHoWOWFIl+s79IWgFej1nOu5Dm7l4soO5pXJ108mcTUI0+EKET3iOyoX3x1t73WSjL1ZF6UXldmtzg96rrs2RW6DmEhsj\/h3vzxuldjwlejoUhhpcLo3f\/58OHz4cOCD5rAdIjkN6HzHWSmcWlaS06mcXnzj9WyTE6\/WmaulSBBybq8lca88Y0r7+TZAJh5KHGYbzBHwziESM6aSyQu\/+APm\/kWsgypM+eIoy4\/AafGgclIl\/OHqH8SKY6wOcQKa45ziULlk1xGE7FAiU9Id+esPwYw\/+X6sgyrZyPdtnY51qZMmbLmtkJNKCBtAqNrE79OJAOuSut90MNJJo67JLAWTkxlenDplCNgYVFFCJC4qYRDmmjVrYPfu3XD33XdDdXU1rF692jXWyW9Png5GOmnClpvJKWxEubxEIWBjUEUJgEgyXV1duaN5\/ciJQnhKS0thz549fUIOdDDSSRO23ExOYSPK5SUKAbdBhUG3Y6Wg3CQ1GnckyIG7XmeIl5eXQ3t7O5w8eRKmT58Oly5dAvrbihUrHLHWrl0LDQ0NDpFRjKEorw7x6KQJG0Mmp7AR5fIShYDboFq57H749rL7E9VOsTEv152GTXWnc3\/yihCXLScMK0DLCKd7GIYjEpEcAM3kFLD7bbB0wKZytoQjkAXLyWtvnde0rru7G2pra0GMGmdyCklRmZxCApKL0drUmnSYmJwS1ENMTgnqjJQ3JQu6pDuto9U6tpwiVNosKFSE8HDRBghkRZfcNv6Sbwmd4OQQx1ACJicDBTFNmhWFMpWb04ePAOuSGlMdjHTSqGsyS8GrdWZ4ceqUIWBjUKUMIi2\/nA0cmZzSpkncXiMEbAwqowYmILEORjppwhaFySlsRLm8RCFgY1AlCgCNxuhgpJNGoyqjJExORnBx4rQhYGNQRYmR6d46So9R4\/icOXOmz3G+OhjppAlbbiansBHl8hKFgI1BFSUApnvrxCujKBCzubk573A6HYx00oQtN5NT2IhyeYlCwMagigKAoHvr8GYW8RFPoqW\/E0bfXfES\/OzIG67Nt4Ejk1MUmsRlJgYBt0GFxzmbHtEcp0B4HLR4vrtuEKbf3jpsP1pOdMTKvn37ciIRRq9tfBO+\/9NvMzn5dbYNlo5T+biu+BBw06V5966Aefe+EF8jDGt6teV78GpLbS5XGNtXdM5zuvj3ZfDE1nuYnJicDDWWkwdCIAuWU6Hk5GUxydO609s+h3\/41y3w8w9e7YO1DYOBp3WBVJ4zpQUBG4MqbGx0p3Vue+vEK8u9bgMmjNBy+tmRetj89vNMTl6dmAWFCltBubxgCGRFl4LurauqqoKJEyfmgSdfJUUYoeXU23onbD71vHNnIT4PDJ0Gvz1mHlwsOQv3P9sfnv\/Os851bXE8bDnFgTLXYQ2BrJBTlAASRn\/z5z+GadcW5u6HfOfivznV\/ueHR2DMfSNivzmZySnKXueyrSPA5KTuAhGjD06fd1Yy5RVDGzhGQk5yVCqecYznGVPMBd9bp1YYThEOAjYGVTgtj68UHYx00oTd4kjICaNS8ampqQEiqpaWFud3vvE37C7k8vwQoEG1Y8cO58wjfvoiUFZW5lwptWzZMk9\/UmbISRZfJCS8CaKzs9MhKq9zjW0AwUqbTQRo4KFO8eONADq58VKEjo4O10Q2xmQklpMsHcVprFu3Lu\/gddmqonzi1w6vtPECjJWNEdBBAAkK\/6Xt2fJiFbzffiXvJpYwZRDLxzHmNs4IOx3rKsy2YVmRk5MYo4EVilfWqMgJ06M5vnPnzrDl5vIYgcQjUL\/rMYecvrmqOZK2Yvn4VC98y7P8RYsWweLFi3Pv\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\/tP0yDzLoyyrKkqX+DypgmK9hURuy\/iooKQHLKvOXkF7xJx62kYbCKbQwiU5oUGmUNIiNNf\/B\/tw3iSevnIDKmzXIykZGI7ODBg87uj8yTk\/wlpW0v4llQSVNaVXuCyJQ2cgoiY9r61lRGTD99+nRIk1\/NREbU0ebmZujq6ioOn5MJc6tIISnvg8iUNnIylRHTz5kzJxI\/RVT9biojtSNNU1ddGUW\/WtE4xE3nvFEpYpjlBpEpbeRkIiMOVjzrK23WsImM8rQepz1pkFdXRlq0EeXs7e0N\/WOTKIe46WpBmCQSVVlBZEobOenKmLapnKgTujKKp76mza+mK6OIS9FYTii0bixJVGQSRbleMnmRUNrIya\/fRFncvrhp8sno9qMoZ5rk0+3HoiWnKMiBy2QEGIH0IZCoaV364OMWMwKMQFQIMDlFhSyXywgwAgUhwORUEHycmRFgBKJCgMkpKmS5XEaAESgIASanguDLduY4l\/4p4hgvwfB6dNJku0eKSzomp+Lqb19p5b1gcZGT3yWsYoOxfStXroRNmzblrrbn7ssuAkxO2e1bY8lsbFTF0xg2btwI+\/fv1zoPSDx2xVhAzpAqBJicUtVd0TVWPghuz549TmW09QKv9BowYACUlJQA3lHY09Pj7ETHg8bEW53p9Aja+Ipl+AUiYmDj7NmzYdWqVc41Yvj4BTHK+7+iQ4RLto0Ak5PtHkhQ\/X7TOiQnJDC8xZmu+xo8eLCzZwwfvGwRd6njQXLydJBufHY7GkXe7iGTjxwtL147FvbJiwnqCm5KVPfWMbLpREBFTigVEYxIOPKpljKhyBtKRXTktEhO8+fPB7Tc3JzjNqae6ezN9LeaLaf092FoEoRBTtu2bXOOV6br6alxXrvW3fYREkFRXpGoiAgbGxtdySs0MLgg6wgwOVnvguQ0IAxywmmdycZlVVr5iBW2nJKjL1G3hMkpaoRTVH5Y5CT7nPzOcJJ9Tm6nMc6YMSN3YzT7nFKkUAU2lcmpQACzll28\/aWtrS1vtU7X54TpxNU6v4PIVKt1cl5ercuaxnnLw+RUPH2dSElN45xU08BECsmNCoQAk1Mg2DhTmAhwhHiYaGanrP8H2GvmN6BVTtMAAAAASUVORK5CYII=","height":177,"width":295}}
%---
%[output:2d1e3a2e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACxCAYAAACGEnurAAAAAXNSR0IArs4c6QAADqFJREFUeF7tnUFoVNsZxz8ppPAQ5Fkr6kakSFatjILSlS6LSEAIWbgxkGQVspMQJMQiQUSykyzaJGA2LkQQ5sngpqK4Cq+O0J1ECLToaC3dBIRmY\/kunDCZl0nO5J5753zn\/gYk5r07d873O\/\/7y7lnxnyHjh49+l14QAACEIiMwCHkFNmMMBwIQCAjgJwIAgQgECUB5BTltDAoCEAAOZEBCEAgSgLIKcppYVAQgAByIgMQgECUBJBTlNPCoCAAAeREBiAAgSgJ9E1OIyMjcvPmTbl7966sra1FCYdBQQAC\/SPQFzmpmCYnJ2Vra0tmZ2eRU\/\/mn1eGQLQESpfT1NSUDA8Py8uXL+XixYusnKKNBgODQH8JlC4nV+5+t3UnT54U\/cMDAhCIg0Cr1RL9U9YjSjmplPR27\/z582Vx4HUgEB2Bv\/3rkPz+N9\/l+A9xDK3ZbMr8\/HxpgopSTiqlxcXFUkGUPf21Wk3Gx8epsWzwgV+vyHnc+OOf5ejXv8uRD88Dj7r307k6da9YJVXGI2o5lQmiDNjtr6Grw6tXr0qj0SjtJxE1hidQ5Dz+OFmX\/\/3jJ\/n2Zin8wHs8o1swlHlNIqceJ4nDIVAWAeTUp182t9eGeD8sXVbgeB0I+BJATn2S014ThJx848txKRNATsgp5XxTm2ECyAk5GY4vQ0+ZAHJCTinnm9oME0BOyMlwfBl6ygSQE3JKOd\/UZpgAckJOhuPL0FMmgJyQU8r5pjbDBJATcjIcX4aeMgHkhJxSzje1GSaAnJCT4fgy9JQJICfklHK+qc0wAeSEnAzHl6GnTAA5IaeU801thgkgJ+RkOL4MPWUCyAk5pZxvajNMADkhJ8PxZegpE0BOyCnlfFObYQLICTkZji9DT5kAcgoop0ePHsng4GDWZlxbOz158uQX2Tlz5owsLCzIqVOnuh7Hr+lN+ZKjNl8CyCmQnO7fvy9nz56VW7duybVr12RoaChrjLm2trZjLlRg+hgdHRVtTb7bccjJN74clzIB5BRITiqdz58\/y8zMjFy6dClrFlmv1+Xhw4fb+XGrpvX19ew47cAyMTEhS0tLO1ZZyCnlS47afAkgpwByctJ59epVJqNOCbVPhs8KCzn5xpfjUiaAnALIqXOltJecNEwqqMuXL8unT5+y28CNjY0dGXNyWl5eTrojbsoXFrXlJxCDnLSjsfuj2zTmOv76rpx6va3T6VVBrays5J9pzgABYwRikNPY2JiMj49vkzMnJx25z55T5x5TtxWWWznpvlWz2ZRWq2UsVgwXAvkJxCAnt2qq1WqZpEzKyWcvabeVkxb79OnTHRvn7DnlDzZnsE8gBjk5iv24Jg8dDbDn5ArY7XNOuh81Nzcnq6ur2Ttybn\/q8OHD2dNev36dvXPX\/ugHCPtRpoLUCCCngHIKFQ7kFIok57FMADkhJ8v5ZewJE0BOyCnheFOaZQLICTlZzi9jT5gAckJOCceb0iwTQE7IyXJ+GXvCBJATcko43pRmmQByQk6W88vYEyaAnJBTwvGmNMsEkBNyspxfxp4wAeSEnBKON6VZJoCckJPl\/DL2hAkgJ+SUcLwpzTIB5IScLOeXsSdMADkhp4TjTWmWCSAn5GQ5v4w9YQLICTklHG9Ks0wAOSEny\/ll7AkTQE7IKeF4U5plAsgJOVnOL2NPmAByCiin3Roc7JYdd5z+PxocJHx1UVouAsgpkJx8WkPpTLUfd\/z48R2dWdxM0uAgV6Z5ciIEkFMgOfk01dS+dffu3ZNnz55lbaK6PZBTIlcXZeQigJwCyMm3Hbn2rJuenpavX7\/KuXPnsonjti5XfnlywgSQUwA5uUaZ9Xo969zbrc24O+7jx48yOjoqne3JO2\/rlpeXpdFo0I484QuQ0roTiEFOrh25fp2dnbXXjryXlVN7999uEnO3dTptKqiVlRUyDIHKEYhBTmNjYzI+Pr7NfnJyUprNZilzEawdue+e08LCgrx69WrPFZaT0\/z8fAai1WqVAoMXgUBMBI7d\/lm+vfmrfHuz1LdhuZVTrVbLJGVSTr28W3fixInstm5qakqGh4dlcXFxxwY5G+J9yyIvHBGBGOTUz3fQg62ctIjdPuek+0ztt3Ltx+nfHz9+nK2i2h\/IKaIrhKH0jQByCrAhHnr2kFNoopzPIgHkhJws5pYxV4AAckJOFYg5JVokgJyQk8XcMuYKEEBOyKkCMadEiwSQE3KymFvGXAECyAk5VSDmlGiRAHJCThZzy5grQAA5IacKxJwSLRJATsjJYm4ZcwUIICfkVIGYU6JFAsgJOVnMLWOuAAHkhJwqEHNKtEgAOSEni7llzBUggJyQUwViTokWCSAn5GQxt4y5AgSQE3KqQMwp0SIB5IScLOaWMVeAAHJCThWIOSVaJICckJPF3DLmChBATgHltFuDg70ypMfrQzuxtD\/4HeIVuPIocV8CyCmQnHxbQ7kZ0bZQN27ckPfv3yOnfWPKAVUkgJwCycmnqaYLmLaLmp6ezr7d3NxETlW88qh5XwLIKYCcfNuRu9lwItPmmtzW7ZtRDqgoAeQUQE66EtLW4fV6fc8245qxkZERuX79uty+fVvu3Lmzp5yWl5el0WjQjryiF2fVy45BTq4duX6dnZ21147cd+Wkx927d0+ePXuWtR\/fb0Ncw6mCWllZqXpOqb+CBGKQ09jYmIyPj2\/Tn5yclGazWcpsBGtH7rPnpKsmLW5gYGBHcZ2b4u7dOl2NKYhWq1UKDF4EAjERiEFObuVUq9UySZmUU6\/v1mkI9ls5lQkiplAyFggogRjk5GaiHx\/vCbZycrIZHByUra0tWVxczG7ddD9qbm5OVldXs+\/bH8iJixAC3QkgpwAb4qED1g9Lh66B80EgLwHkhJzyZojnQ6AQAsgJORUSLE4KgbwEkBNyypshng+BQgggJ+RUSLA4KQTyEkBOyClvhng+BAohgJyQUyHB4qQQyEsAOSGnvBni+RAohAByQk6FBIuTQiAvAeSEnPJmiOdDoBACyAk5FRIsTgqBvASQE3LKmyGeD4FCCCAn5FRIsDgpBPISQE7IKW+GeD4ECiGAnJBTIcHipBDISwA5Iae8GeL5ECiEAHJCToUEi5NCIC8B5ISc8maI50OgEALICTkVEixOCoG8BJATcsqbIZ4PgUIIIKeActKGBZ0NDjpnbWpqSm7cuLH9nx8\/fpw14mx\/8DvEC8k6JzVGADkFkpNPayjtxDI9PS0PHjyQtbU1UVENDQ1lnUT1e\/dATsauIoZbCAHkFEhOPk01O2ews405ciok45zUKAHkFEBOvu3IOzOiHYAnJiZkaWlpR087t3LSVuSNRoOOv0YvLoadj0AMcnIdf\/Wr3uGU2eg2SFPNzhWQk9X6+rrMzMzsOkN7HePkpE9UQa2srOSbZZ4NAYMEYpDT2NhY1obcPczJqdeVkzt+c3NTRkdHfxEbJ6f5+XlpNpusnAxeWAw5P4EY5ORWTrVaLZOUOTnpNPjuOblV1tu3b7uuqtgQzx9szmCfQAxy6uc+cJDbOi3A5906n9s9PRdysn9hUUF+AsgpwIa4m4bdPuekK6W5uTlZXV3NDtNl4cDAwI6Z6\/ysE3LKH2zOYJ8Acgoop1BxQE6hSHIeywSQE3KynF\/GnjAB5IScEo43pVkmgJyQk+X8MvaECSAn5JRwvCnNMgHkhJws55exJ0wAOSGnhONNaZYJICfkZDm\/jD1hAsgJOSUcb0qzTAA5ISfL+WXsCRNATsgp4XhTmmUCyAk5Wc4vY0+YAHJCTgnHm9IsE0BOyMlyfhl7wgSQE3JKON6UZpkAckJOlvPL2BMmgJyQU8LxpjTLBJATcrKcX8aeMAHkhJwSjjelWSaAnJCT5fwy9oQJIKc+yGm3RgjtGeN3iCd8xVGaNwHkVLKcfFpIISfv\/HJgwgSQU8ly8mm+WQU5aSfVq1evSqPRSLajMTXmMydyKlFOvm3LnZyWl5fl3bt3+WY40mfrhTs7OyvUGOkEeQ6ryHn855\/+Ikc+\/CRHPjz3HE1xh7k6TbYj98HiWpHX63V5+PChdOsArCB+NzIn\/\/3tBZ\/TcgwEkiTw72+H5PgP36OpbfLHn2V+fr60lX6wduQ+BH1XTnquX\/\/hmvzqyEmf03IMBJIkcGyrJf8ZiOca0BVcq9UqjXWpctKqfPacSqueF4IABKIlULqcfN6ti5YWA4MABEojULqc3OppcHBQtra2ZHFxUZ48eVJawbwQBCBgg0Bf5GQDDaOEAAT6SQA59ZM+rw0BCHQlEJ2cRkZGRD9LMTAwIO\/fv5fR0VHz09drTe1vGlgp3qdG927tqVOnsrI+ffokt27dko2NDRNl+tTYvm2hf3\/9+rXMzMyYqE8H6VujK0iPn5iYkKWlpeDbM1HJqf1zUM+fP5eFhQVZX183NbmdKey1JvfvDi2F2rdGfTNEH3qxdvuMW6xXsW+NU1NTcuHCheyHaufn+mKtzY3Lt0Z3vJvDY8eOFbJ3HJWc1MI3b96Uu3fvytramrS\/s2flp2tnAH1rchO9ubmZneLz589mpOxbYycbS\/N70BotrYJ7rVHn7\/Tp06JySn7lpD91rly5sr3U1++Hhoayf+ahsrL4OEhNlgKtc3KQGt3tj361cOt+kBqtrZx6qdGJ7MWLF9k1mrycOn+SpiCng9RkTU4HqdHa3PZaox5\/+fJlU\/tqvdSoGX379q18+fKlGntOvZjbyirqIDVZk1OvNerxw8PDhexTFJWLXmt047B06+pbY\/u+WmU2xHu95y0qiCHPe5CarMmplxr1YtUNY2u36r3U2J4fSytE3xrdmzbtdRbxgeqoNsR7fbcgpESKOtdBarImJ98aLV2oB33XVeV74sSJ7X00nUsr+2q+89jOpjIrp4N8zqIoqYQ8b7fPjnSTkDU57TVv7bXs9hPX0medfOexvU5L9fnOY2XlFFIKnAsCELBLIKrbOrsYGTkEIBCaAHIKTZTzQQACQQggpyAYOQkEIBCawP8BYVqhVs6CBQwAAAAASUVORK5CYII=","height":177,"width":295}}
%---
%[output:0ef3d5a4]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 24.164203 seconds.\n","truncated":false}}
%---
%[output:154e240f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACxCAYAAACGEnurAAAAAXNSR0IArs4c6QAADsFJREFUeF7tnU9IVdsawNc1U6RuRBdCgxs0iIZxG9SgQY0jfAQiryY6ceR1Jl0JMZCIbjgzRxbkxPuQIF6Ew8hBAwfZWBwIXVIp3iXyhtgf3+Pb9y3v9nQ8Z53jt\/dea53fhrjp3a71rd\/3+Wutfc5Z67sjR47813BBAAIQ8IzAd8jJs4wQDgQgkBBAThQCBCDgJQHk5GVaCAoCEEBO1AAEIOAlAeTkZVoICgIQQE7UAAQg4CUB5ORlWggKAhBATtQABCDgJQHk5GVaCAoCEEBO1AAEIOAlAeTkZVoICgIQQE7UAAQg4CUB5ORlWggKAhBATtQABCDgJQHk5GVaCAoCEEBO1AAEIOAlAeTkZVoICgIQKExO3d3dpqenx4yOjpr5+XkyAQEIQGAHgULkJGLq7+83nz59MsPDw8iJooQABL4hkLucBgYGTFdXl3n27Jk5e\/YsMyeKEgIQKEsgdznZKKot6zo6Ooz8WV1dTf5wQQACjUXASzmJlGS5d+bMGXP\/\/n3z4MGDxsoKo4UABIo74KDSzEmkNDExYW7dumUWFhaYOVGoEGhAAl7OnKyc5KG5yIkLAhBoPALIqfFyzoghEAQB5BREmggSAo1HoDA5VULNsq7xCpERQ6CUAHKiJiAAAS8JICcv00JQEIAAcqIGIAABLwkgJy\/TQlAQgAByogYgAAEvCSAnL9NCUBCAAHKiBiAAAS8JICcv00JQEIAAcqIGIAABLwkgJy\/TQlAQgAByogYgAAEvCSAnL9NCUBCAAHKiBiAAAS8JqMrp4cOH5tSpU8mpKrKT5czMzDeDPnHihBkbGzPHjh3b9T52JfCyVggKArkSUJPTnTt3zMmTJ83g4KC5fPmy6ezsLHvskwhMrt7eXiMnsZS7DznlWgN0BgEvCajJSaSztrZmhoaGzLlz55L9v588eWLGx8e3B25nTUtLS8l9so94X1+fmZyc3DHLQk5e1gpBQSBXAipystJ5\/vx5IqNSCaVH5DLDsnKSk1dmZ2c54CDXkqAzCPhBQEVOpTOlSnKSYYugLly4YFZWVpJl4PLy8g4aVk7yTY6G8qNQiAICeRNQkZPrzKnWZR1HQ+VdDvQHAX8IqMhJhuPyzKn0GdNuMyyeOflTIEQCgaIIqMnJ5VlSuZmTnE336NGjHQ\/OkVNR5UC\/EPCHgJqc7Oyp9H1O8jxqZGTETE1NJa\/I2edTBw8eTCjMzc0lr9ylL+TkT4EQCQSKIqAqJ61BICctkrQDgXAJIKdwc0fkEIiaAHKKOr0MDgLhEkBO4eaOyCEQNQHkFHV6GRwEwiWAnMLNHZFDIGoCyCnq9DI4CIRLADmFmzsih0DUBJBT1OllcBAIlwByCjd3RA6BqAkgp6jTy+AgEC4B5BRu7ogcAlETQE5Rp5fBQSBcAsgp3NwROQSiJqAqJ5ejoYSmvU\/+zpYpUdcXg4NA3QTU5OSy2ZxEmb7v6NGjO\/Z6sqNgy5S688kPQiAaAmpyctmmV3bCvH37tnn8+HHZAzeRUzR1xUAgsGcCKnJyPeBAdsG8fv26effunTl9+nQSfKVlHUdD7Tm\/NACBYAmoyMn1aCh735s3b5ITf6sdqilUORoq2NoicAjsiYCKnGqZOaX3E692+gpHQ+0pt\/wwBIImoCIn+wqc63Hk1U4G5oF40DVF8BBQIaAmp1perWtvb0+WdQMDA6arq8tMTEzseECOnFRySyMQCJqAmpzs7Kna0VDp++Tv09PTO86sk+8hp6BriuAhoEJAVU4qESEnLYy0A4GgCSCnoNNH8BCIlwByije3jAwCQRNATkGnj+AhEC8B5BRvbhkZBIImgJyCTh\/BQyBeAsgp3twyMggETQA5BZ0+godAvASQU7y5ZWQQCJoAcgo6fQQPgXgJIKd4c8vIIBA0AeQUdPoIHgLxEkBO8ebWeWTHt7bM66Ym5\/u5EQJ5EFCVk+vpK3Zgcr9csn1K+mJXgjxS\/1cf5z9\/Nk\/W1xM5\/XzggHmxf39+ndMTBCoQUJOT635ONhbZy+natWtmcXERORVYojJrevX+fSIn+fuvbW3mbltbgRHRNQT+IqAmJ5fTVyx0e9CBfL2+vo6cCq7G\/\/zxRzJrkuvex4\/MogrOB90rysl1D\/H0ck629JUdMVnWFV+KVk6\/tbYms6d7f\/5pzn\/5wiyq+NQ0dAQqMyfX01eEtJy4cuXKFXPjxg1z8+bNinLiaKh8alOWdSKm9HLu+saG+WVjI5lF\/ePQIR6Y55MKekkRUJGT68yp9FDNag\/EJU6Ohsq+XkVO8iDcLu1sj8yismdPD7sTUJGTNO\/yzElmTf39\/aalpWVHRKUPxe2rdRwNlU\/pPvnwwbzet+8bOdne7SzqRXOz6Tx0KJ+g6KXhCajJqdZX66zQeOZUfA3KQ\/DjX79WFI+85UDuk8v1LQdbW8fN5uY\/TWvrv0xT0+viB0oEQRFQk5OVjcvpK5ZQtWWdzLIWFhaCAhpisC5yKp1FyTOq0mWgvUek9PnzebN\/\/wvz\/v0rc\/jwT4mkmptfJN\/jgoALAVU5uXTocg9vwnShpHePLNuubm6anw4fdmq02hs3Nzevmo8f75m2tl\/NxsYvprX1NyPfk6\/b2u469cFNEEBO1ICxz5R+OHLEmcZuD8tl1iSXzJRETPYSQYmYWN45I274G5FTw5eASWZNsrSrRU7pZV561rWxcX2HlErxpmdPIkV5yM5HZijCcgSQE3WxLSdZ1u31A8B25iSSkqWcvURKpQ\/Gd3sLAymBgBBATtTB9od\/NeQkOO3sKf2sSZZ4Bw78nDx\/spfM1mTWVc+MjbTFTwA5xZ\/jqiO0H\/7t\/P57lSWWvFL35cv5ZKZkX62T78nzpvSrdbZfedVPXv3jgkCaAHKiHhIC6c\/XaSKRWVSlV+hY2mnSjqst5BRXPuseTVZyqhaQfRivtaSs1h\/\/PxwCyCmcXGUaabkP\/2baYarxosSY1\/jopz4CyKk+btH9VJHLq2qf7YsONgNyIoCcnDDFf1ORgmBpF3991TNC5FQPtQh\/ppbP12UxfJZ2WVANu03kFHb+1KIvWk4yc5OLLVnUUhp8Q8gp+BTqDKDWD\/\/q9Pp3KyzttImG356qnFyOhrKnrlh009PTZnx8fAdJdiXIv7Dq+fCvdpSytKu0FYt2f7TnNwE1OblsNmdPXbl7966Zn583IqrOzk4zPDycfG0v5JR\/0ezlw79a0fJxFi2ScbSjJieXbXpLkZUejICciisqH5ZVNgatj9EUR5OeNQioyMn1gIPSgGVP8b6+PjM5OWlmZmaYOWlktM427AZyRb9Tm6VdnQmM8MdU5FTL0VCWoRXa0tKSGRoaKvvMiaOh8qs47Q\/\/1hs5S7t6ycX3cypyqnXmZO8vd9qvILbPnOTvHA2VX9H58F4jlnb55dv3nlTkJIN0feZkZ1kvX778ZsZU+syJo6HyLZ+i5SSzN7n+\/eFD2XP08qVBb0UTUJOTy6t1lZZyaRC8WldMWWT14V8rnR+\/fk2OO5dLjjuXS46k+nFra\/v7duTs8VRMDfjUq5qc7Oyp0tFQck+5QzVL3+uEnIopkVo\/\/FtNOmkJpUdktwL+vakpOcxTLvle8nVTU\/I1+4oXUwM+9aoqJ62BISctkrW1k\/7wr4gnPdOxsxuZ6ewmHSsZ+a8VT6l0ft+3b8\/7lNc2Ku4OlQByCjVzinHL2wiufvqU7CWeLLX+v\/SqNNux0tme9SAdxYzQlBBATtRBQsDOmpIlVXPz30stpEOFFEQAORUEnm4hAIHKBJATFQIBCHhJADl5mRaCggAEkBM1AAEIeEkAOXmZFoKCAASQEzUAAQh4SQA5eZkWgoIABJATNQABCHhJADl5mRaCggAEkBM1AAEIeEkAOXmZFoKCAAQKkVO1I6TYlYDChAAEcpeTy6Z0yInChAAEcpeTy3a+yInChAAEcpWT60EIyInChAAEcpWT6xFSVk4cDUWBQqBxCeQqp1pnTpIWjoZq3OJk5I1NIFc5CepanjlxNFRjFyejb2wCucuJV+sau+AYPQRcCeQuJzt7Kj1CKh0wD8Rd08d9EIiXQCFyqoYTOVUjxP+HQPwEkFP8OWaEEAiSAHIKMm0EDYH4CSCn+HPMCCEQJAHkFGTaCBoC8RNAThnluKOjw1y6dMnMzs6a1dXVjHopttlGGKMQboRx+jhGr+Uk7w5\/9epVsb+BdfYuyR4eHk7e4R7qGKoNvRHGaOVELk3yj2ye\/9B6KSdb9PKWAi4IQMAPAnl\/lMxLOdl\/rURSXBCAgB8EmDn5kQeigAAECibg7cypYC50DwEIFEwAORWcALqHAATKE0BOVAYEIOAlAeTkZVoICgIQQE4Z1UB3d7fp7+83LS0tZnFx0fT29mbUUz7Nuo7HHvtlo5qbmzNDQ0P5BJlhL+lNEjPsJremq43HhzwipwzKIb1X+tOnT83Y2JhZWloK9pfUdTyl2zBngLaQJu0vakyilf3UdhuPL3lEThmUu8wyenp6zOjoqJmfnzfp3T+Xl5cz6DHbJl3HIxIbGRkxU1NTZmZmJtugcmjd\/pKur68nva2trQX7D4zE7zoeX\/KInDIo8oGBAXPx4kUzODhoREbydWdnZ\/JxFpFVaJfreNJLPxnjysrKNoPQxlwab7VlUGjjqzQeX\/KInDKoqtKZUuhych1Pepxv375NlrMy6wj9eZuUSCPJyZc8IqcM5OQ608ig60yarHc8oUs5DbOR5FRaREXlETll8Ovs+owmg64zabLe8ZRKLZPgcmq00eWUfkyRE3KDnDIg7frqVgZdZ9Kk63hk+dfe3p4s4+zD15BfpWzUmZMveUROmfw6G+P6vqCMuldvdrfxlM4o0u+PieH9XRZk7DMnH\/OInNR\/jWkQAhDQIICcNCjSBgQgoE4AOakjpUEIQECDAHLSoEgbEICAOgHkpI6UBiEAAQ0CyEmDIm1AAALqBJCTOlIahAAENAggJw2KtAEBCKgTQE7qSGkQAhDQIICcNCjSBgQgoE4AOakjpUEIQECDAHLSoEgbEICAOgHkpI6UBiEAAQ0C\/wNEuSD8k46+DwAAAABJRU5ErkJggg==","height":177,"width":295}}
%---

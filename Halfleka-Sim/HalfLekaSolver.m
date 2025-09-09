%[text] ## Solver
clear
% Starting values:
BusDeclaration;
q0 = [0, 0.8 , 4.10, -0.88];
dq0 = [0,0,0,0];
X0 = [q0, dq0];
Sim.time = 0.445;
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
toc %[output:1f5916bf]

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
plot(t_total,x_total(:,1)); %[output:2cc372a0]
hold on %[output:2cc372a0]
plot(t_total,x_total(:,2)); %[output:2cc372a0]
plot(t_total,x_total(:,3)); %[output:2cc372a0]
plot(t_total,x_total(:,4)); %[output:2cc372a0]
legend('x','y','th1','th2'); %[output:2cc372a0]
title("Generalised Coordinates"); %[output:2cc372a0]
xlabel('time (s)'); %[output:2cc372a0]
hold off %[output:2cc372a0]

% display derivative state of all variables
plot(t_total,x_total(:,5)); %[output:88a04439]
hold on %[output:88a04439]
plot(t_total,x_total(:,6)); %[output:88a04439]
plot(t_total,x_total(:,7)); %[output:88a04439]
plot(t_total,x_total(:,8)); %[output:88a04439]
legend('dx','dy','dth1','dth2'); %[output:88a04439]
title("Derivative of Generalised Coordinates"); %[output:88a04439]
xlabel('time (s)'); %[output:88a04439]
hold off %[output:88a04439]
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
plot(t_total, contact_total) %[output:14f8d1ba]


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
if Animate ==1 %[output:group:5e4641c9]
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
    fig = figure; %[output:229918ab]
    
    ax = axes('Parent',fig); %[output:229918ab]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:229918ab]
    u_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:229918ab]
    u_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:229918ab]
    l_l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:229918ab]
    l_r = animatedline(ax,'Color','r','LineWidth',0.5); %[output:229918ab]
    f = animatedline(ax,'Color','r','LineWidth',0.5); %[output:229918ab]
    
    axes(ax); %[output:229918ab]
    
    axis equal; %[output:229918ab]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:229918ab]
    
    hold on; %[output:229918ab]
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
        addpoints(b,x_sim(i),y_sim(i)); %[output:229918ab]
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
    toc %[output:06ea418d]
    hold off; %[output:229918ab]
    clear fps b f l_l l_r u_l u_r ax fig l1 l2 l3 l4 l5
end %[output:group:5e4641c9]

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
%[output:81daff19]
%   data: {"dataType":"text","outputData":{"text":"Time = 0.4440","truncated":false}}
%---
%[output:1f5916bf]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 30.204329 seconds.\n","truncated":false}}
%---
%[output:2cc372a0]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAR8AAACtCAYAAACJInuHAAAAAXNSR0IArs4c6QAAGzRJREFUeF7tXW1sVUd6nsAGBa9JKja4\/sAhCFGyaiMVaExDusLKTyuKSquyKj8CCpCksojaymURRVhQFxFqZau1nA8+tpBV3Y1FxAoR9x8FpSwlFFMtu0ooP9Di+IOPRQs4kDoLW71z\/V7Gw7lnzjn3zJk59z5Hsu71na93nnnnOe\/7zsw5j8yePfu3AhcQAAJAIGMEHgH5ZIw4mgMCQEAiAPKBIgABIOAEAZBPAtiXLVsmurq6RG1tbbH0iRMnxObNmxPUZqfI\/PnzRXd3t6y8o6NDvPTSS2L16tWir69P9PT0pNLoxo0bjXXu2rVLrFixotjehQsXxNq1a1Np31SJLl8UeU11cjrV1dTU5NWYR5Xdl3wgn5gjsWrVKtHe3i5OnTpVVDxWap8ISCefS5cuxeypObtpMh84cECQHL29vaK\/v18wad+6dUsSog2ZVKlN8pl7GJwjSAeS1lXN5UA+MUY\/zoRmxafqx8fHxdatW8Xp06cFK+5nn30mFixYIK0n1RrgNhobG6VkKqHRZKa7LU1eSicr5sqVK5IMZ8yYMSW\/yfJRLRJVPqqklOyUppYjuRctWhRoTUWdoLoVqRM418P9Y8uN+\/fYY4\/JfhOORHJLliwpWlq6fCoZHT16tGgZTkxMiKeffnrKOFGdhDf1Tx1D+q5avTx2YZip9YyMjGRCvDHU2llWkE8M6KNOKDXf+++\/P8X9ee655yRZXL9+XSrh66+\/Lp5\/\/vmidcAEQ2TV0tIyxa1hJdYn4PHjx6UrpVoaZ86cKel2MWGR9Xb48GE5mYaHh6U7FEV2slgory6PCiWRlNqvIJiZeLht3VLR8Vbr5P49+eSTRew4fyn5gsiHy5N8qkVLeVtbW+UY8ZhxvbpcYZip7q6Ku08ueowpkGpWkE8MOHWl0+\/afFfTCYUmzdKlS6X1M2\/ePKnkrMjqhPj000+nEAHXf\/bsWeniqcREVhRfqnVAd3GyAKKQD1kTuqWhk4Yq+8qVK6cQSphbE4V89PK6tabjqJLV9u3bp5Ar4am3GRbzYcvn8ccfl+PCFg0TIWOrWi1s5YSRIrmXKmZ8A6H60oy3xVBbb7OCfGIMjX6n5qL6pOns7Cya65yHSSHoDsuBYN2F4rKs9Dr5sDxEIuxysLURRj5sJbFLobp36mTTZSeXRrVmyiUfnSyCcCQ3k11WG+RDfSTrpq6ubgrxs3tJ2L\/77rslrUO+KahYUp36TYDdaJDQgwkH8olBPqViPqY7ttqEftcMs3x00XTy0Sd\/kFvCk6vUahfXwZNFJxjdlaKVK76Dh5FPFBfVhuUTJl+pmI9OPrpVpROTyfIppVJMaHqMLYYKVlRWkE\/M4dTjClScrQV2uzhGQDGVUjEfXi3TJ6Ap5qNaAuok4NgNW0Fhlg\/JrC67q22yWxgme5SYD+MSttqlT+okMR8mV5IpScynlOWjYkLuJpFaKbcrLOajuo76mNhe7Yup2plnB\/kkgDxon0\/UFaMwy4fcoSirXeyGqMRH32\/cuCFmz54tLRN1NSdon4++\/0aNR4St3KhpYatdDKvejr7ao2Opx0WC4lkUVyllhYbJF9XyUQPvROZkFdIfbxGgvtEeKnKluD9sWVKarguqK8sWJvWh2i+QT7VrAPoPBBwhAPJxBDyaBQLVjgDIp9o1AP0HAo4Q8JJ8GhoaBP3hAgJAIDsERkdHBf1ldXlHPkQ6FFClJV9b16PTbk6p+hvTbgn67ev7T8jfa6Zflp+PTrul5ZtaTk3U89qS3VTv1\/cfN2XxMv03k9iz\/F\/\/tjAWPCa\/uf948buXHagAoQYHB+V+pqwIyCn50CoAXeopZyId2jAXF4TmxhpZ11NNNfLv8vAd8Zd\/Ok\/+1txUSCOCYOKhdM6v6w2n0e9Dw1+KyyOFvKUuIswli5eIjwc+lu1mcVEfk16qvEnrSLPcU5NjR4Tz6CM3RXPTN4vVB\/WTCIlIamj4TnFs5DgN3xEnz1xPU7TEdS1evFisX78+th4nbrDMgiwv7b4nEsrickY+vOypP2KByGfPuzvFwR++XQThKUUZX2h5skAakwpLihqkoEwCpJR0EYHQb6SwfF2eTBuaTEsKOBNmlgOXVFYqlyd5aWwXL14itne+KT744dviG5NWq7zJNNZIouLxp\/FVv9PY\/9tPLsubz8lPr0nIsiKnPGHsSieckA\/t7di0aZO4du2aoFPJuuXzo71\/Jepn\/vtD86uURXLy08LdjsikXCJJMqnJkmhraxMDAwOZmaxJ5OQylSgvk84Lz80pWrp\/8lzhRvVCy5wpcBVuQgWLlr7\/+Ce\/nNSf9KzWvGHsgiydkA+5W3RYknbr1tfXP0Q+ZPl8\/PGAdGPGJgNgWbkz5UxqlPUXASIncs3JiiZLiMmKXfMH1nCBmP7zzPWCWzf8ZWbWkiv0pBu+ZImMtWZpvWdOPuRu0QlvsnZo92sQ+VDMh659+\/aJ\/fv3uxoTtFslCDAxsYVEFpNqLak3vpNnrgnV0s7KjbM5FOvWrZPxKboqmnyCTk2rcR814EyBr6wi7zYHF3XnEwHdlYtjLblw\/5OizC4iEVBFk48KUJjlkyUISQcN5fKBgI19YwUXrqZoIb0wGV\/SEeEVOQ54y3iTYfXUFqph+3iqJubD4IJ8bKkZ6lWD67b3jeUF7bB9PFVHPkGD5gKEvCgP5IyPQNJ9Y\/Fb8ruEaR+Pi3mXecDZNEQuQDDJhPT8IgB9KoydCQdTug0NAPnYQBV1eoOAi0nlTecVQUw4mNJt9AnkYwNV1OkNAi4mlTedB\/nEGwooSzy8kDscAegT3K7IcwTKEhkqZIyAQN71ifbFjY2NyVcnBa0OR4AAMZ+0QIpaD\/IBgVKB1mlPNIjpTxTeCOvjde\/miLh\/88Fzdd577z35DGl6x1jS99ybSNiUbgMnxHxsoIo6vUEgaFLVfGeDqPnOa97IqAty55M94s4ne4s\/80Hs3bt3y1duJ7lM5GJKT9KmqQzIx4QQ0nONQNCkypPlQ2\/p2Llzp\/j888\/FM888I7Zs2SLfdhv3MpGLKT1ue1Hyg3yioIQ8uUXAxaRKE6yPPvpIXLx4ETGfNEEtVVfelSULjNBGdASgTwWsTDiY0qMjHj0nLJ\/oWCFnDhFwMal8hMmEgyndRp9APjZQRZ3eIOBiUnnTeUUQEw6mdBt9AvnYQBV1eoOAi0nlTedBPvGGAsoSDy\/kDkcA+oSYT+Q5AmWJDBUyRkAA+gTyiaAm0UCKXBEyAoEIqzzVApKJhE3pNnBCzMcGqqjTGwRcTCpvOo+YT7yhgLLEwwu548d85s6aLpprp3sL3dD4PfHF7XtSPn7NVE9Pj1Df\/BJXeNO8MqXHbS9Kflg+UVBCntwiEDSp\/mbJN8XfLq31tk9vnx0X3x8svGlXJRyViOIKbyIXU3rc9qLkB\/lEQQl5cotA0KTKk+VDZ7s6OzvlSzaXL1+Os102NdEFA9vsD+p2i0Al6BM9x+fZZ58V58+fl2e8klwmHEzpSdo0lYHlY0II6blGwMWkShuwVatWiQ0bNoi9e\/eK\/v7+RNWbcDClJ2rUUAjkYwNV1OkNAi4mVdqdJ\/JZuXJlYpeL5DHhYEpPu09UH8jHBqqo0xsEXEyqNDtPAefVq1eLvr4+QSteSS8TDqb0pO2GlQP52EAVdXqDgItJ5U3nFUFMOJjSbfQJ5GMDVdTpDQIuJpU3nQf5xBsKKEs8vJA7HAHoUwEfEw6mdBt6BsvHBqqo0xsEXEyqNDtP+3xeeeUVsX37dvnqnOHh4ZKxn7BNiCYcTOlp9onrAvnYQBV1eoOAi0mVZudppevFF18Ub7zxRknyIYLq7u4WjY2NJQPTJhxM6Wn2CeRjA03U6R0CLiZVmiDQA+SJVE6cOCGrra+vF4sWLZLfeQXs1VdfFePj46KtrU3uhA5aFTPhYEpPs08gHxtook7vEAiaVE811YjmxhrvZGWBhkbuiMvDd+S\/uuVD5EMvDgw6ZAq3yzCk9AK0rq4uUVtbONhHjK5uGXfBwN5qIQQrG4EgfdrU\/m3xvfZvl123rQre6v1M7O79LJB8OOajkhLLAfIxjIj67mkCsL29XRw6dKhoKoJ8bKl0ddZbaZYPyCclPeZA2fHjx0E+KWGKaqYikPebGd+g6S2lY2NjxdUuWD5lajoBuGbNGrFjx47iO6jzrixlQoLiKSMAfSoAasLBlJ7ysMjqnC21U+xn27Zt4uDBg1NO6jII+\/btEwMDA2J0dNRGv1FnlSDgYlL5CG0YDg0NDZKctm7dKsMgg4ODmXTBCfkEWTzcWwaJ\/icC2r9\/fyZAoJHKRADkY7Z81q1bJ9avXy8zVjT5mB4PwMpCK2LEwLB8KpMUsuoVyMdMPmT50B4hIqCKJR91J6aqfOrjAqAsWU3L6mgH+mQmnygxIRva4sTtCusIlMXGMFdvndAnkE9k7YeyRIYKGSMgkHd9inKwlB84RnCMjIyIjo4OQUvz6mXCwZQeAerYWWD5xIYMBfKEgItJlSY+poOltGq8adMmsXv3brldhU6+06U\/aN6Egyk9zT5xXSAfG6iiTm8QCJpUdTObxZyZzd7IqAty7e6QuHp3SP4c5WCpWp6soKamJpBPktF1wcBJ5ESZfCAQpE\/fXdghvrvw77ztwIcX\/0l8eLFbyhfnYCkRT2trK9yupCML8kmKHMoFIZB3y6eU26UfryB3i0+8R8UhTkzIhnbB7bKBKur0BoG838yikI96WLsU8CYcTOk2BhTkYwNV1OkNAi4mVZqdNx0sPXbsmNwYOGPGjGKz+mNqKMGEgyk9zT5xXSAfG6iiTm8QcDGpvOm8IogJB1O6jT6BfGygijq9QcDFpPKm8yCfeEMBZYmHF3KHIwB9KuBjwsGUbkPPYPnYQBV1eoMATyp6QsK5c+e8kStrQejwaNgjM0A+ERg660FDe\/lGgCcdTa5qv+gpEfS0iKAnRYB8QD7VPj+s9J8IiP6q\/SLSKfWIGpAPyKfa5wf67wgBkA\/Ix5HqodlqRwDkA\/Kp9jmA\/jtCAOQD8nGkemi22hEA+YB8qn0OoP+OEAD5gHwcqR6arXYEQD6T5PPeW52i6x+6xOC58t8f9MXte9WuV+g\/EDAiAPKZJJ9\/+es\/F7M\/+YERsHIylCKlU6MTYu6s6bJqyjN0+55onjVdfn4xfl\/MrZ1WbJb+53z849B4gexAeuWMDspmjQDIR7F8dn3vzbLf2cUkog+kSiBqGpGMeunlm2sL6aXqLaUwKhExufFvTGpctvj7+D0QWNYzsIrbA\/nkMOZjIiiV6Jjc1DJEaKXIjIhItcJobpBlxQR1avTrB1bYpHupplfxXELXYyIA8skh+cQcY2N2JhvVymIC08krjLhU148JiciKCQ0WlnEoqioDyAfkk1jhddL644ZHZV1EWGpamJWlxqvIHZQENn6\/EPuCG5h4bPJQEOQD8slMT4mE9BgWWVjPNz54HOfzDQ++l4pJkTVF5f5r0gWkmBau\/CEA8gH5eKe1pdxAJikiKDU2pZIUExFbUURQsKC8G2IpEMjHEQh+qkO+pGJLij45RqUSlN4bjjn1\/+9dmcSWE8jJzbiDfEA+bjQvo1Z1guJ4FLt3Qat7ZD3xdgRKh1tnZ7BAPiAfO5qVk1qDyIlE\/4vfm1kIfmtbD8hKOjVSiDHBpStvkEE+IJ\/yNKiCS6uxJ17JI7dO3W6gbiPQiYmggdVUWkFAPiCfCqYPu11jq0klJmpRX7FTtw3AnXswJlVDPvRe6RUrVsie9\/X1iZ6eniIKLkCwOy1Qu2sETO4cy6cTUzVtH3Ax7zJ\/dQ69\/nXNmjVix44doqWlRbS2toqOjg5x6dIlqQMuQHA9OdC+OwTU\/U5kNelBcI410ae6U5x\/rxRXzsW8y5x8yOpZuHChJJy6ujqxbds2cfDgQdHf3w\/ycTcH0XIAArorF4WY2JUjUsrTkw2qhnzq6+vF2rVrxbJly+R7hI4cOVJ0vdSXvA0MDJR9sh2zCgjYQKDUtgH9vJ16hk5dmfMpAE6vFaJ5F\/ZSQRsYOrF8TOTzZ5vfEf\/8P1MfbxGl8\/dujhaz3f\/1iJj2O41Rinmdh\/qR5qViNP2J8HdZqXnTlEGt6\/7N9PqXhbw6DkEY8socBbvnTj6G5Y++Pi+L8qcaZ2IiGpv9h\/Lns48+Kz\/\/e\/KTvpMu3\/lkj\/i\/nx1NfSjWrVsn1q9fL+ttb28X9HLBLC4n5GNyu97Y8Y5464PqsnqmPeGeKE1klJZCht0U4tw0spJX7zeRHLXNZKd+N2FE\/Wu8f1U03rsiP+laOnFeNN6\/IubNnSsa7xV+44sIiNJGpv2u6D52UZz8\/JemJmKnk+XT1tYmCaiiyQcB59i6gQJVhoDq0lHX+cDv989+aW2vUlXEfAhMXmqfmJgQvb29xWAzpbkAocp0G90FAg8h4GLeZe52mcbdBQgmmZAOBCodARfzDuRT6VqF\/gGBCAiAfOB2RVATZAEC6SMA8gH5pK9VqBEIREAA5APyiaAmyAIE0kcA5APySV+rUCMQiIAAyAfkE0FNkAUIpI8AyAfkk75WocaKQKBuZrO4enfIWl9APiAfa8qFiv1CgMiErjkzm0VdTeH7H8x+ofgbfbl2d0j8\/reWC87b87M3xX988aGVjoB8QD5WFAuVZo8AEYZOLPS\/JJlvLZ8i0M9\/9dMiAVHC1TtDknjk97uXixbPL371U2vWD8gH5JP9LEGLiRFQCUa1WlRyIVeJyISsGyYVIhQiHL5+cePB98TClFkQ5APyKVOFUDxtBNjlYfenbuZT0qJhgiFy4XgMk8uxL35cJBsfiCUKJiAfkE8UPUEeCwjoVkyQi8QBX3J\/fn7jZMEtujMk8kIwYbCBfCbJ5wf\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","height":173,"width":287}}
%---
%[output:88a04439]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAR8AAACtCAYAAACJInuHAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QVcWVPoIQhEEMIDIzIBKCxJiNkREQjEHdTSU7ZZGQNWjYqkAVqOsi7moRZAk7JCNShKXMRqAQxRRkU6Aj6yhBkk1SChpUHGYwJIYgVcvyMzPIX0AQEMVsfY3n2a\/n3tv3vnd\/3zu3imJmbv+c\/vr0d885\/XdB7969\/0ryCAKCgCAQMwIXCPnEjLhUJwgIAgoBIR9RBEFAEEgEgVSSz8qVK2nYsGF5gLS1tdGMGTNo9+7dgYGaMGECTZs2jV577TWaNWtW4PycYfDgwTR37lxatmwZbdmyhSBndXU1zZkzR\/0e9YP6Fy1aRFVVVXT27FlaunQpNTQ05FWrp+EXO3fupMmTJ0ctnu\/yWUZkQJ\/eeuutNHHiRFq9ejUtXrzYdzleCadPn24tk\/Wia9euqqiTJ0\/G1pejRo2iefPmUWtrq+obP\/L6BQZlQS+L0XW\/dRWTLrXkow9q7qh33323YAIqBiTOGzfZmDLzYAEBO5GJqdDIb8sTBi5ByzDJp5APiq1O22Dm95s2bcoNUvQvZHMidVt9Qd879VXQMpzSh\/WhDUMWWxmZIB80wlQm7ryKigrVRv5qMviwlGAhHD58mH7961+rryAsH3xp9K+s2Vnm15CVc8GCBTR27FhVF1sdtbW16guDr\/WkSZPUO7bOTKLS89usON3y47T9+vVTX0pur1MZfsmRsTS\/9tz2HTt20JAhQ1RdutVkWlXmwAUW+EAAd\/THG2+8kSczl2WzfMw+0C0it35HW3SMUResZydrKgj5OfUFk6WXnNwXOh7vvPOOssBhaaH\/Lr74YkfLZ\/369crCZV274oorOlhlTvq4Z88eR7zd+tvELE7LD3Vnhnx0kli+fHmuczDY7777bho9erT6YuFBB4N0mAj0vI2NjaqDmpub1RcPnch5m5qaVLkbN25UhGJ+Cc3Brf8+fvx4qqmpUWY7HjeTmgckm9vm10Ev0yzHy4rx+yX1wnHEiBF52Om4wr3TZRs5cmQeifMg1Qc7\/nbgwIEcziBvvNcHl+l2me\/g5rIVfPDgQWu\/s1XoJA9j7dc68NMX7MrresRY6eTHhAfCgY4wfkzI+seVMejbt2+eTnNdkB8fu\/r6+pyusVfAfain5ZCDOW70tDwu3PTSZsUU8j6T5NPS0pIXw2FlWrt2LfHXRY\/vmMoGperZsyctXLiQZs6cmWex4Bf9i6bHVrzIZ9CgQUomyIBHt66c8qF+M4blRCC6UjOxOrldZl7TSuGvGkiSyRaDBOUzaXIbuHx9QJikyfUxibtZXU6Wih\/ygfVkWnduViowB0Hp7fJyu\/yQj60vhg8fnlefmd7Ew6zTK+bD+DBRmR8hHuhO1rRJPiYpOvU3LDHdii2ESArJk0nyYdfJbDAANIlJJxMmJCjmbbfdRi+++CLdcsstuUA0KwQ6A1aUqWBe5MNf5RMnTiixzK81BpP+OJm4ToPCL\/l4uRK63Pfcc0+HYD4TLJObjhOTqO4y6O3gL7eJjU5+sHZ0QvYiH1icupuAfExCHJh26nf8LUzysfUFu9w82RAF+aBN+ECxy20Gp4GL+QE1ycdp8sb8oOqTO3GSUGbIR\/+SOVk3Xia121cHnYC4BgcYza+lkymtB8Kd4joYAChXN1\/9xmJsX1svywfv3Oox3UN9kOoD2cTJy\/IxCcD2pXdyK3hwuc126VaT24fFtALY7fOyfPzEfGx9Uajlw1alH8vHjXx0rE1X1Gb5uFkojJfbLGohlo0tTybIx1QWNIoDcm4xHy+3iwcqGF837Z1iQ2wFmTEPp6l23V3TYx+FDmLT3LbNXDnNCrJpzpYWu1bAxysGgHiYOYBtMR+nGUqQ8A9\/+MPcEgFbzMd077xIyynW5yfmA1z9zHYVG\/PR8Sgk5uNGPvpH8bLLLlMuPutxkJiPSfp+P5I2UvH7PrXkY1vnY5vtspGPk\/LppISfjx49Sr17987NmOjuAAYRYiVeCqav\/Sl0tkufbbKRD2R2Wudjfs1ss11ObhfcIT+zXfqaJ73NID8QuRPpmYPAnEXSMfCa7dLb5TXbZVrJvM7H6auvuy3meim32KCbFarL7ne2y8nt0suBzPiHB9izJaTHzHR31XT39T5CGWGutbKRUKLkg47Fk6YFcDbA5L0gIAiEg0Bi5MNfqbStvg0HVilFEBAEbAgkQj4wGzHFfejQIerWrZtYPrZekveCQAkikAj5wN3C+hDES\/r379+BfCorKwn\/5BEEBIH4EGhvbyf8i+uJnXzgbiFQizgPgl0m+YB0EDjDVKY8goAgEB8CWCOHlflxEVDs5OO06EmP+4B0sO4mThCK7d5rr72Wpk6dmhmZRd5ie9yeP6sYY5U+SCiOJ3by0RvlZPkw+cQJQrFAZ01mkbfYHrfnF4ztGAn52DGypoCriOX2GzZsiM1ktQrlkUDkLQY9f3mzhnESZJko+Th1YxIg+FMnSSUIlC4CSYw7IZ\/S1SdpmSDgGwEhHyI1y4WAc5ZiPr57WBIKAilFIIlxJ5ZPSpVBxCoMAVkj5o6b1zoeIR+xfAobcZJLISBrxLwVwWsdj5CPkI\/QSBEIZHGNWBHNDZSV1x25hTOEfIR8AimUJM5HIIkBlJU+sGFjex9FOyXmEwWqUmYiCCQxgBJpaAGV2rCxvS+gSmsWIR8rRJIgKwgkMYCiwgYnP+C8bZwCGca9ZjZsbO+jaKeQTxSoSpmJIJDEAIqqoUI+USHrUW4pKVAC8JV1lVnXHf2YWhyziptQVqxYQffff3\/eXXJ8F1qQzrZhY3sfpC6\/acXy8YuUpEs9Ak4DqFOvSurcK\/\/aojQ15NzxNvro+PkzdLDRGg8f3n\/TTTepq3Pw4PLEM2fO0LFjxwq6g91GLrb3UWAm5BMFqlJmIgg4DaDuN95J3W+8KxF5\/FR66pXH6dQrT6ikfMgeDus33S4cVo8LH2fPnl1QDMhGLrb3ftoSNI2QT1DEJH1qESgly0cnG1waiGOHt2\/frm4QKeTCBRu52N5H0elCPlGgKmUmgkASAyjMhuoxH1xxc\/jwYVqzZo26Y2zdunUEiwjWkcR8wkRdKyvrChQRLFKsDwREd9xBsmFje+8D\/sBJxPIJDJlkSCsCSQygtGJhymXDxvY+inYK+USBqpSZCAJJDKBEGlpApTZsbO8LqNKaRcjHCpEkyAIC\/S4aSAtuX01X3X2hnAXl0GE2crG9j0IHhHyiQFXKjBUBEM+9X3yUhg+\/lvp8t13IR8inMP1LgoELk1RypQWB24fOoJsH3EGbO6+kf37oH4V8hHwKU00hn8JwK9dcV\/ceQ9OveZRe2v8U7er5YskcwSt7uxLQaCGfBEDPaJXsbh06vY8Wb7+vpM7\/FvKJQCkBKm4jraioUKVv2rQpb6+KkE8EoJdokdO\/+Chd3WcM1b0+ng6e3udIPgN6dqaBFZ1Ti8C+k+do\/4lzSj63jaU4fbCxsZEaGhry9n8FaZRtXNneB6nLb9rYA876Ck0sIQewa9euVas38SQBgl+wJF16EIC79dD1jfTvr4+nt46+6qo79w\/vQQ\/UnP\/QpfF5pPkk\/bjlPSWa28bSu+++W71fvnw5zZ8\/P0dEQdpjG1e290Hq8ps2dvLRBWOm37hxo5CP3x6TdGS6WwyJ0wDKkuXjtrEUe7twsNiOHTvoqquukr1dYYwBWD6TJk2i+vp62rJlS2DL5\/Lq7jSwqjvdMPJSws+XV3WnNc\/tpTXP7QlDPCkjpQjw7Ba7W17kk9ImOIqlWz7mLvbHHnuMhgwZktvjFbRdNsvG9j5ofX7SJ2b5IPZTV1dHq1atUr6sqUA4RMnr7nOQzZKHa2hgdQ\/a1\/oe7W07pcgHRLS39RTdO3srbW467AcDSZMhBGD1PHbzVnp613\/Q07sW5UmexAAKEzqnjaV8hAY2l44bN47mzJmT+1AHqdsLG1w5hPcoO87LOhMhHyeLxyQf\/A4CevLJJztgDOJ5fuVX1N+\/MfllRTb8MCmBhO6d3SxWUBANzUDa+lGN1K\/7QPqnl67rIG3WyccLfpBPTU1NQS4XyvXCZsqUKTR16lRVfUmTj+1QJAYJM2K45Ay3LJrPupU3Kovn2q\/+yrW\/bhjRl5bMv05ZReMmv5KBYSUi2hC4ecDthBkuPcis5ylV8oE7Nnr0aLWGSfcSbHj5xQaWT21trSKgkiUf3azUgVm9erXvgPN3vjmIZk67ypdbBSto22++TpvfOCQEFERTU5gW7lb99Y301pFX1Zoep6dUySeM7rBhY3sfhgxmGYm4XV4N8QKBXSrEd+BS+Xk4D9KKBeQHsXSmcQsy+\/26p7NV8UllIxfb+ygkzRT5sNVjxnlswLAFhFkwv6RlK1Pex4cAB5lf2v+0q9UDaZIYQPGhUFxNNmxs74ur3Tl3ZsiHg8ybmw4VRCBCQFGoTzxl8kpmpyBzqVo++vYKrPPBTRaI9+AWi2XLljnOeCGs4bYI0UYutvdR9HRmyIcDyMVMoaOMdau+Qj9auoMWLt0RBZ5SZgQIPFv7jmuQuRzI59Zbb6Xq6mq1wtmNfHjbUteuXR0D0zZysb2PoFspM+SDGa4w4jZhkFgUHSFlOiPgNbVu5khiAIXZb057u5599ll1gDz2Qr7wwgv02c9+lnr27ElVVVWEQ+Z53c+DDz5Ib775ppqK531gQYg5CewyQT7sMoVlsWC27MFpV9G4SS\/LQsQwR0\/IZfH+LcxuId5je5wGEK+Ct+VN6v2+tlO5dWpue7t0y2fRokV5t5c2NzfnZorF7SqyF50UCIHmJfNrQiULlHfDiEs7LFIsUnzJHiICQawet4Azf2hCFCvUovQPqtveLje3C2TV2toq5BNWjziRj59FhYXUH5YrV0jdkscbAadd6zbMSsny0RfjCvnYej6k96YChe1y6WLyDJqsgg6p80IsJqjV42b5hChS5EW57e0C+UycOJG2bt2q4j082yWWT8hdYpIPz1BFFZ+RGbCQOzCE4gqxekqBfEKAzrUIW0DZ9j4K2VIfcIbPjpiP1z6uYoGJmuCKla\/c8sPqwVO3ZXygpicxgAIJmGBiGza291GInnryiSsuwyRXzDqiKDqo3Mos1OoRy8dbU2zkYnsfhR6mmnwOv\/NntTE0rCl2G4CYAYOVFZWLZ6tf3hMVEuth3JIYQFnpMxs2tvdRtDPV5HNR571qRXKcZAACwqFk936\/Oe+coCjAlzLzESjG6hHLRyyfoseTzsB\/N\/p05PEeJ4GjmtovGpwSL6AYq0fIR8in6OGhk88P7uuhjkdNYid6XLGmogErkQKKtXpKjXyCbizFtPvYsWOVNuzcubPDiYc2t8r2Pgo1S7Xb9ZufXxFbvMfNAsLf5RygKFQvv8xirZ5SJh\/bxlLzdFB9pbTfeJiQj3Ymy4KHv0c\/Xdgv1niPOcTkILLoSQc18JXHi39\/X+4OrkJqdhpAOAvo0osGFlJcLHlw2youPMRTzMZSXVhz8aEfYhby0chn9cq59IN\/qUiUfNBp+i0ZUa41ikXTU1pJGFaP2wDDCYi3D\/1eSltOebdwFLuxFI1EGf379xe3q5AeZwbe86clNOzyI5EuLgwiHwehZR1QENTsaflsZlyF42fnuleJWbd8it1Yqt8GbOJks2xs7+09GTxFamM+A3o8RdtaWlIVb+Gd8EJAwRXNLUdYVo8f1yI8qaMpye3SQNvG0vXr15N+1IaTdDZysb2PosWpJZ\/BPR+nR5a9nroTB\/mIBrkTrHh15LOZ3a7CCVpDEgMoqIxe6QvdWIoysfFUf\/QbYfwQcxLYpZJ8Hl82n0A+aR3gcidYOEPO79nMfmtLYgD5lS3pdDZsbO+jkD+V5PPT5bNoYI+nEg82ewGu34wa1\/aPKBQgqTLDjPVwG5IYQEnhF7ReGza290Hr85M+EfLRF0Q5mYdPrZhIfbq9Sn0+\/6yfNiSaht0wXNkc9EqfRAVPuPKwrR4\/rkXCTU60ehu52N5HIXzs5KPf0z5y5Eh1JciMGTNo9+7dqn0AAeTz5+3PpyrY7NcKAglJQNpbVf3ewxVU4XkArVixgrZt2xY0e0mnx5XIOGze7TrksiAfWD1Dhw5VhIP7iOrq6mjVqlW5O6gBwvMrb0zdTJcfzeRYEFwyXNH8u6bDqQuY+2lH1Glg9WDhX9Dzemxy8QCDDsnTEYGWlhaaN28etbe3d3hZNuTDi6D4rqF169bl3dW+4WfXpHKmy69C6yQESwg3pe77+H+zDFgBeDAY+3UfSPw7\/tbvosutVR48vbdDmj8eeVWVdfDUPsIKWv3h1bTWgiNKEJXVw+KCgPCv0KdTr0rqcnkNdRlUQ517fVLOuePnB+wHe5rpg73N9NHHvxdaTxL5QDpOxMMex9KlS10toyjkjd3t0ldgOpHPyOuG0C9\/dg2t+a\/D9GrTOXru\/c87tpuVQX\/50bGOjH7ueFuH\/HEpDiygO745SF3Tg+fM0T70h1eq6YU1vRXZfKHPGPV3EAKTDn4GaTB52Dod6fjRicuWz0ZIkMF8TCLzqqNb7yOOr3ue\/pJqm+320SDyF5pWEc2gGkU2IBr8zA\/0i4kG\/8elM4W2pZh8IGtYPl5uWTHlu+VNhHxsbte8q35OlfSf1Lf789TWuZ+Sva3TZbS1y9\/QL7r9LbV1viwKLMiJ0Liij451JDFdCLe8fc9eSKOP9qCvXX41XXzmSypLt08foUuGvK1+7jVkF2GggnTWvdmeO0OouuK4spZa3+tF1T2O56rC7\/pz\/ZUf5H4F2fHTv\/8oQhl4cD7Rmb\/0UT+jTH2v05m\/9M7lwbuB1d3p\/aPn0\/KDkwVAnLDiUAf+54d\/x\/9ch1fnQJZOX2igr\/\/bDyLpQ6dCQTKde1VRp0vOWzVuRPPR8Tb6YE+LsmzK6ZkyZQpNnTpVNdktJhQFHrGTj5+A89zP\/YJe2v8UbW7\/MbUfaKdvD+1Go6u60ujKrgqD\/SfOUcPbp+n19g\/otfaz6m9QMPOBwpkPFLBjOm8zvZNDOVyGbprr5X7rU7fQmKM9qM\/ZCxWxvF1xhnZWvE9Hun6o\/h\/Q45giBxAL\/h952V41KPh3t85uPfkJ+SCf\/jvyMDmZfweBAKPWk5fkijaJTM9jvnOTx4uw9TxM3rcNP0wL\/76FbvzJKEViGPAoA23nsmDBOlmsbjIwseA9ykF\/cb\/o1gzew4pBPUw0qKeUrRo\/pAHLp7a2VhFQSZMPwOCp9rNnz3a4VxrmH8jnt3\/8b8JNlfozoGdn9SvI6IGaCkVCeEwi8gN4FGng9tw84Ha1kRGE89aRV+nF\/U8VtVPbTU4nsvVqkxMRO6V3ImendG6k61imQd57Fv6BZv5yOK1t6ZtL3umSTwij0L7RiRBkp1ynj60YEFq5WTRBcCyLgLMNEIDw8NcaaM\/rx6xxAZARrKFHxl6simWL6JldZ3LEZKsvjPcm6cBqe3rXojCKLskysDbqyyP6Oi6l0ElVJ0ydFM3YHltJ5W7BFKMsQj4fr\/NZdNfP6Ni6S+lbG\/zHdkBEbBHpRPTjlveK6RPPvCCdq\/uMIUwdw9IR0vEHNV9\/jSNK9PiRv9ySKgoEhHwM8sGMSNCpYSYhxIgGVpx306Jwy0A8937xUTVzI6QTfDjgVpLNTYcSOSI3uLSln0PIRztMrP2hz6iYTzFnvERlDfEBVVhPs2T7fYEJsvRV2d5CvqYoC1to7K3JfgohH418\/vijC2j7\/77RIehcaDeDiB4Y3oO+feVFuXjQIy3v0TNvn\/ZdJMd2bh5wh1g7vlFzTii3xBYJYMjZhXw08tmx\/EPa1rItNPLhvmJraMKVFxF+9huk5l3YKCeMU\/dC1p1MFic3hKSn24R8DPK55OCVgYLOQbvSr1umx3fqXh8vblZQoF3Sc+A5zkshQxK95IoR8tHIZ+38l+mGc5PVdHvQoHNQzfCyhs5+WEX11zeqLQ9hb4QMKmcpppfAczp6VchHIx+stJzT\/5cU1hGbfrtYt4beP1dF2w\/8ivae\/An968vz\/RYh6QIgINPuAcCKMKmQj0E+d\/V6XK0SNlc6R9gHuaJ59zWI5x+GPan+jq0cz7x9JlCQOg5Zs17HkT99K9HLIbOOXxjyC\/kY5HPDh5MVrnGTDxMPAstYqay7ZdzRWDsU90rqMJQsjWXItHvyvSLkY5BPr4ND1erhICudi+1GntVyWzjoN0hdrBzllB874hH7SeuFAeXQF0I+Bvm8\/3\/d6KHrG2OL+5gWj03psK\/s+souapMrHkzbP7Dp3dxOe1t+ef8JAnwpo9wKm4xWCPkY5INjH5+tfSfvStmouoYtnkJiTPqWDhCS05EfUcldKuXKosNke1LIx4F84HZFHfdh4gljOt1tb5nEh+yDSxYd2jGKKoWQjwP54HwcEFBU6330BYRhH+2px4f0s4eEiJyHkCw6jIpa7OUK+TiQD2CD61XsJlMn+ONcuQx37NtXdlN7yzg+hBmzKI\/8sKtc+lKI9ZNMnwj5uJDPYzdvjWS9D3anY5NonFsmzPiQEFH+YBPrR8gnGQRcyAduF9yvMKfcmXgW\/\/6+SI459QOgW6B6\/8mPynohI9b94KD5cZNf8QOjpAkBAbF8XMjn6t5j1JR7WK4Xypt+zaOp2p3utL8MR34gVsSH5IegY5koQqyfjt2E2UA8m5sOR9KHQj4u5AO0w3K9OIDNq5cj6ckiC3VbyFhOgWqJ\/XyiRFiE+fzKr0R68qOQjwf5hDHrFXQRYZEcEkp2EBGOg2249dN5gepSJyJZ9\/OJ+uDAfViD35j8cmRnXgv5eJAPz3rhWNVC9nplkXhM9nJaUV3KM2ay6pmISTjqrSdCPhby4bOTgwaei1m9HIr5EkEh9w\/vQQNxY0cJT93znq8fLd1BC5fuiADFdBeJ9i95+PwVzlEH38uCfPh+9oqK8\/uhNm3aRLNmzcppgQ0ErPkJYv2wxRMkT7pVMl86jg+VKhHB5cBd9+V4zU4c7hZrk23cRTEmYr8ueeXKlXTgwAFFOLg6GYeGrV27lhYvXqzaZwOBYz9+DhljSynNweUwO9VpxgwzZa+1f5DpqftyDD7HbfXZxl2YesplxU4+eiMGDx5MixYtoo0bN\/omH+SvH9Wo7sty2w6h3yBaLsRjKkcpbXYtt+Azz27ta30vcnerrCwffZDA8pk0aRLV19fTli1b8iyfFStW0IYNG6i9vd2RdJmA9HN39KttkEku8zsPndvU\/evtH2RmDVE5BZ\/jdLegH5WVlcrjmDNnjvJEcJpEHE9ilg9iP3V1dbRq1SpqaGjoEPPBH0BATz55\/ghTp4fdKrzDIfMgH\/yPYzFg8UR98HwcHRR2HVldQ1QuB47FNbul69WUKVNo6tSp6k8lRT6I8QwbNkw1bPXq1cq9crJ4TPNv3rx5ioHdLB9Oz9YOEw2IR0jHH2Vl7XjYUl\/5HMdiQifNgOVTW1urCKikyMdsLIhn\/PjxNHv2bNq9e3cHLJIIfPkbqqWdiqfsHxl7sWooH4iWtl33pex+YU\/bDSMuVTN7cT9JjLtY3S4OMFdVVeVhyxYR\/pgECHF3dJrrcwpUQ960LGZk92vNc3vUmc+l8sCqQ6zn3tlbI9u\/5YVVEuMuVvLxoyhJgOBHrnJM43T8h5q6bzub6DlEpTb7Ffe0upMuJzHuhHzKkVUKaLPbra5JHf9RKvGfpOI8pgoI+YjbVQAtxJ\/FjYjinrrPevyHt08MrO6RSJxH1xwhHyGf+JmkyBqTnLpnd2XzG4diW4xXJFx52Xk9j584D3Dmc8DDlIHLEvIR8olCr2Ir04mIcCDaM2+fjkwGxH+WzL+OEIDO0uZTDjBDZsju9uhWZpRYCvkI+UQ2SOMuWD\/+Q7+5I4qp+6wFoP0EmEE6DwzvoU4tQJAfGPLJllH0pZCPkE8UepVomXHtus9KANqLeIAV33KiXz4Zx8FxQj5CPokSRdSVu93cgS86f92LkYEX6fmJoRRTT6F53Wa2THcVWDzz9plI3VWZ7XLoxSQYuFBlknyFI+C0mHHfyXNqDVExs2YgIFhB4ya9nMhiPTdEzJ3qTjfbJnlhQBLjTtb5FD5+JGdICOiuGdwNntnBqupC1hGljYCYeHDg2z3ffZEGVHSiCR+fQMlWTtI3lAj5iNsV0nDOdjFOs2ZBrSImoKjPPrYhrRPP6Se35o4widutsskp5CPkY9ORsnvvFCcCCLzx1ctF4yB0nGdAc9B4dGUXuv2W\/vSprw2lPTuO0qaVb6X6REkhHyGfsiOXoA32IiO4LvtOnFMxI1hKICheBxTFqYB8rdF5wumSO8z\/gos\/RRd+vh91GX05xUl8QbHU0wv5CPkUoz9lmZfJCI3XD9FnMEBAIIPP3PY5RUZrnttLv\/vtfvUaBMVWFKdHefqDO9P4byAYPDrpcH4mu9ZOXWjwV69Q1z3\/rulwZhY+CvkI+ZQlgYTdaCaH6z8mi9FVXdXFiyAFWCPn9h+ns\/+zi\/a1nsqRiZsMICx9awOTDCwsBMP5OmvEdu745iB10wa2e9z7\/ebILvgLGy+UJ+STEAhRdKaUmU4E2A2DlfTKb\/errQ1\/fff9DsLq+6iYcJxaBNLBAWDYp4Una9s8uE1CPkI+6RyxJSgVB6PRtL2tpwKTBls6KAfP5qZDysVCWVl8hHyEfLKot5mWmXeWg0x0IsLPTz23hwZWnf87P98ZP0jFc24YeakiGpDOmsY9qVrQWEiHCPkI+RSiN5InBATYkvnyiL6E83WYjMyi2UpCXGdz0+EQak5HEUI+Qj7p0ESRIo98YP3sazuVWZfKT3cK+Qj5+NETSSMIhI6AkI+QT+hKJQUKAn4QEPIR8vGjJ5JGEAgdgbIjH9xmimfy5Mk5MJMAIfSelAIFgYwhkMS4S+xIjenTp9PEiRNp586dQj4ZU1QRt\/QQKBvyGTVqFM2cOZMOHTpE3bp1E\/IpPV2WFmUMgbIhH7hbzc3NVF1dTf379888+VToa20aAAAFAUlEQVRWVlJtbS1t2LCB2tvbU692Im\/0XZQ1jMuCfOBu1dTUKMJZsGCBK\/msWLGCtm3bFr2WhFADFG3OnDmUFZlF3hA63VJEVjGeNm0atbS0RA8QEUUe84GVM2zYMNWY1atXK+Lh37mFetyHOw1MLI8gIAjEhwBIZ968ebFZ75GTjxd0TpYP0oOA8E8eQUAQiA8BhAziDBukknzig1tqEgQEgaQQSJR8kmq01CsICALJIyDkk3wfiASCQFkikDryQRxo7NixuQD14sWLU9MxvDASAm3atIlmzZrlKhvS3nTTTTRjxgzavXt3Im3wI6+e5uzZs7R06VJqaGhIRF5U6kfmCRMmEGZlunbtSidPnlQzjVu2bElEZj\/ysmBY34aA7rp16yhJvfYjs44x5DcXA4cBdqrIBw2eNGkS1dfX08iRIxMfvDrAUJy6ujpatWqV+jPL6aT0TKBtbW2JkY8fefU0IBzIjdnIpAazH5kHDx5MixYtoo0bN6oB7LRFJ4yB4acMP\/Lq5fDML2Z9kyIfvzLH8fFMFflA+YcOHaoGbL9+\/XKDPckvMSsPOmPcuHFqYB48eDBvAOgKhnRYPImH25KE5eNXXl12nfyTsCQKkRkD+sCBA55WqB8iKSRNEHmRdsyYMcpaY+IspM5i8\/iV2W0mutj69fypIx9e8ZwWE1UnH3aj8Dd8fXft2uWq9DqRJkU+QeRFm+JQOC\/l1b+2NozZLYCrmJSl5lde3k70+OOP01133ZU4+dj0gq3Lqqoq1V1RueNCPj6p3K+icXFZI584zGwb1EEx5hgRW6RxW2t+5eXtROvXr3e1mG3YhPXer8ymNR8Fxqkjn6y7XWkiHz9uYhosnqCurekq3nnnnfTEE0\/EHij348KwBV9RUZHHH0nFffzIbBIdrMwoME4V+ZRKwJkHdJIxH7+BRVhoeLxm7sL66trK8SMzXIL58+dTY2OjIpskLUw\/8uptNoPlNjyieO9H5rgwThX58KDFVHtUfmYxHapPUepfLqcBnOSg0C0JnJmEx0le7OXhKWvOk6apazeM0zrVbtOJNJAPu6peeoEPkY5xVGMxdeRTDDlIXkFAEMgOAkI+2ekrkVQQKCkEhHxKqjulMYJAdhAQ8slOX4mkgkBJISDkU1LdKY0RBLKDgJBPdvoqMkn1LQpxLDbETMr48eNp9uzZrptueWFeUnugIgNbCs4hIOQjyqA2Z8a1P8pcQ+IGP29JWLhwYWI71kU1okVAyCdafFNfun6ECY4JaW1tzZ0mMHfuXDpz5gwNGTKEsEIXxyqApJzWYelroLx28+sXCAAcr\/UkaVoAmfqOzKCAQj4Z7LSwRXZzu0A+2KGPjZuDBg1SCxKxSRY3j+hHWejL75uamtT+pRMnTuRdicQy6+6UuejOiZhs7lnYWEh58SEg5BMf1qmtyYt8IDTIxunsHz6BwFzN7RY3Ml0u24pfcb1SqzKhCCbkEwqM2S7Ei3w4FmQjHz59kpFw2qZhloG0+sZLM4+QT7b1yia9kI8NoTJ4Hwb5mDfPOsFmCzabFpOQT2krn5BPafevr9YVSz7mkQsor2fPno5HyOoxH9MSkpiPr+4qmURCPiXTlYU3hGe8nGa7\/LhdqFmf7fLaGW8SjFc+me0qvE+zkFPIJwu9VEIy2lwvbqq4XCXU6S5NEfIp\/T5OXQtlhXPquiQRgYR8EoFdKhUEBIH\/B0ljf32yobHaAAAAAElFTkSuQmCC","height":173,"width":287}}
%---
%[output:14f8d1ba]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAR8AAACtCAYAAACJInuHAAAAAXNSR0IArs4c6QAAECFJREFUeF7tnU9oHOcZxj+3xJggU+qDqshthQ6q2qOtJoFcHNqeHHDJxWD1IIP\/gDG6CaMaV6UimBx0E4bajsG65CAMBtfRWfKl6CAFehOm6KZVZV8KbkoNTco7ZcRo9WdmZ5+d753d30Igtr5559nnfb6fvplZ73fs1KlT3wVeOIADOFCxA8eAT8WOczocwIHEAeBDEHAAB6I4AHyi2M5JcQAHgA8ZwAEciOIA8IliOyfFARwAPmQAB3AgigPAJ4rtnBQHcCAafD788MMwMzMTFhYWwuLiIp3AARzoMQeiwOfixYvh5s2bidX37t0DPj0WOt4uDpgDlcPHVjw3btwIL168CJcuXQoPHz4EPmQRB3rQgcrhk3psq59r164dCJ\/33nsv2H+8cAAHqnOg0WgE+6+qlzv4GHTu3LkTzp49W5UHnKdCB3a+CeH3f\/1+hWfsnlON\/+zb8OufdO6fYq6vr4fPPvusMgC5g49Bx+4DVWlCu\/E8c+ZMuHr1am00x9T7za\/+GP7x72PhP3\/7S2Hb7RfSJ+c\/CV8tfVXZxCgs7pCBndD8w1dr4Ufvtqvs4OPTTNi9WINQFS+38KnShHaNToFZF80x9f7gd38O\/\/1nI7x5\/qfCtsfUW1hk08C6aY6hF\/iUTVfmOPstd\/78+bC0tFSL38wx9ZaBT0y9ZeNRN809BZ\/DmhrDhLIB47jWHSgDn9bPwhGtOhBj3kVb+QCfVuPRHeOBj88+Ap8QkqdcdsO5LvdPfEbJryrg47M3MeYdKx+fWehaVcDHZ2uBDysfn8kUqgI+QjOFpYAP8BHGyWcp4OOzL8AH+PhMplAV8BGaKSwFfICPME4+SwEfn30BPsDHZzKFqoCP0ExhKeADfIRx8lkK+PjsC\/ABPj6TKVQFfIRmCksBH+AjjJPPUsDHZ1+AD\/DxmUyhKuAjNFNYCvgAH2GcfJYCPj77AnyAj89kClUBH6GZwlLAB\/gI4+SzFPDx2RfgA3x8JlOoCvgIzRSWAj7ARxgnn6WAj8++AB\/g4zOZQlXAR2imsBTwAT7COPksBXx89qX28LHdSG3Lm76+vrC1tRWmpqbC5ubmPrcfP34cRkdHk79fWVkJ09PTu2NimOAzDt2pCvj47GuMeSf9JkODyvb2drh\/\/36Ym5sLy8vLYX5+fo\/bk5OT4cKFC8nGgENDQ\/t2LY1hgs84dKcq4OOzrzHmnQw+6arn2bNnCXA+\/\/zzMDAwEC5fvrzHbdsmeWJiIszOzibwSf9\/dXU1GRfDBJ9x6E5VwMdnX2PMOyl8ZmZmwsLCQlhcXEzgMzIycuCllwHIviD+9evX+34ewwSfcehOVcDHZ19jzLvK4ZOF0vvvv3\/oZdcXX3xRm034fMbJpyrg468vtsGhwcduhVS5a4wUPnazOe+yK70vZDeZh4eHk3tDL1++3L3pnBLYWmQAevTokb9uoai0A8CntHUdO\/DKlSvh6tWrSf1awseEF7nhnF359Pf3J0\/HUmBl7\/nY39uG9Y1Go2OmU7h6B4BP9Z7nnTHd2tkAVFv4ZB+1b2xs7N5sNuDYK32kzqP2vDh078+Bj8\/e1vqej8rSGCaotFMn3wHgk+9RjBEx5p3sno\/KsBgmqLRTJ98B4JPvUYwRMeYd8InR6R4+J\/Dx2Xzgw4cMfSZTqAr4CM0UlgI+wEcYJ5+lgI\/PvgAf4OMzmUJVwEdoprAU8AE+wjj5LAV8fPYF+AAfn8kUqgI+QjOFpYAP8BHGyWcp4OOzL8AH+PhMplAV8BGaKSwFfICPME4+SwEfn30BPsDHZzKFqoCP0ExhKeADfIRx8lkK+PjsC\/ABPj6TKVQFfIRmCksBH+AjjJPPUsDHZ1+AD\/DxmUyhKuAjNFNYCvgAH2GcfJYCPj77AnyAj89kClUBH6GZwlLAB\/gI4+SzFPDx2RfgA3x8JlOoCvgIzRSWqj18iu7Vblsmj4+PJ9Zlv2je\/hzDBGEPKZXjAPDxGZEY8076NapFts7Jbpe8s7MT7t69G54+fZrscgp8fAZTqQr4KN3U1ao1fIru1d68jU6zfTFM0LWQSnkOAJ88h+L8PMa8k618DD5F9mo3+AwNDYXBwcFw\/PhxLrviZC3aWYFPNOuPPHFPwMcuzU6ePBmmpqbCUTuWsle7z5C2qwr4tOug\/vie2avdVj4DAwPJbqbs1a4PkveKwMdfh3pmr\/bsDWdrA3u1+wtjJxUBn066W652T+3Vbqufc+fOJU6trKzs7uHO065y4anTUcDHZ7dqfc9HZWkME1TaqZPvAPDJ9yjGiBjzTva0S2VYDBNU2qmT7wDwyfcoxogY8w74xOh0D58T+PhsPvDhn1f4TKZQFfARmiksBXyAjzBOPksBH599AT7Ax2cyhaqAj9BMYSngA3yEcfJZCvj47AvwAT4+kylUBXyEZgpLAR\/gI4yTz1LAx2dfgA\/w8ZlMoSrgIzRTWAr4AB9hnHyWAj4++wJ8gI\/PZApVAR+hmcJSwAf4COPksxTw8dkX4AN8fCZTqAr4CM0UlgI+wEcYJ5+lgI\/PvgAf4OMzmUJVwEdoprAU8AE+wjj5LAV8fPYF+AAfn8kUqgI+QjOFpYAP8BHGyWcp4OOzL8AH+PhMplAV8BGaKSxVe\/gU3as99cz28LKXbaOTvmKYIOwhpXIcAD4+IxJj3km\/RrXIXu2p9ZOTk2F8fJwdS31msWOqgE\/HrG2rcK3hU3SvdnPIxt66dSu8evUqnDhxgpVPW7Gp18HAx2e\/ag+fInu1m\/W2QlpbWwunT5\/e3b2Uyy6foVSrAj5qRzX1egI+drk1NjaWrHayWyc3w4e92jWh8lYF+HjrSAg9s1e7rXpGR0f3dGBjY2P30islsA0wAD169Mhft1BU2gHgU9q6jh3YM3u1Zx08auVje7ivr6+HRqPRMdMpXL0DwKd6z\/PO2FN7tadmHAWfmzdvJvDh1V0OAB+f\/az1PR+VpTFMUGmnTr4DwCffoxgjYsw76ed8FKbFMEGhmxrFHAA+xXyqelSMeQd8qu5yj58P+PgMAPDh33b5TKZQFfARmiksBXyAjzBOPksBH599AT7Ax2cyhaqAj9BMYSngA3yEcfJZCvj47AvwAT4+kylUBXyEZgpLAR\/gI4yTz1LAx2dfgA\/w8ZlMoSrgIzRTWAr4AB9hnHyWAj4++wJ8gI\/PZApVAR+hmcJSwAf4COPksxTw8dkX4AN8fCZTqAr4CM0UlgI+wEcYJ5+lgI\/PvgAf4OMzmUJVwEdoprAU8AE+wjj5LAV8fPYF+AAfn8kUqgI+QjOFpYAP8BHGyWcp4OOzL8AH+PhMplAV8BGaKSwFfICPME4+SwEfn32pPXzSLZP7+vrC1tZWmJqaCpubm3vczo6xH6ysrITp6endMTFM8BmH7lQFfHz2Nca8k36Hs20IuL29He7fvx\/m5ubC8vJymJ+f3+N2OsaAc\/HixWBb5Dx58mR3XAwTfMahO1UBH599jTHvZPBJVzTPnj1LQHLQnlzNtg8PD++DVAwTfMahO1UBH599jTHvpPCZmZkJCwsLYXFxMYHPyMjIgZdeqf228pmYmAizs7NhdXU1+evUBPZq9xnSdlUBn3Yd1B\/fFXu1twIfWyllx6eWsle7PlyeKgIfT934v5ba79XeymXXQSueZviwV7u\/kCoUAR+Fi9oaXbFXe5EbzgaeTz\/9NNy+fXvfk7DsZRd7tWsD5qUa8PHSib06an3Px95K9jH6xsZGuHz5cvIO7f6PvdKnYIODg3ve+ZdffsnTLp+ZlKsCPnJLJQVrDx+FCzFMUOimRjEHgE8xn6oeFWPeyZ52qcyKYYJKO3XyHQA++R7FGBFj3gGfGJ3u4XMCH5\/NBz782y6fyRSqAj5CM4WlgA\/wEcbJZyng47MvwAf4+EymUBXwEZopLAV8gI8wTj5LAR+ffQE+wMdnMoWqgI\/QTGEp4AN8hHHyWQr4+OwL8AE+PpMpVAV8hGYKSwEf4COMk89SwMdnX4AP8PGZTKEq4CM0U1gK+AAfYZx8lgI+PvsCfICPz2QKVQEfoZnCUsAH+Ajj5LMU8PHZF+ADfHwmU6gK+AjNFJYCPsBHGCefpYCPz74AH+DjM5lCVcBHaKawFPABPsI4+SwFfHz2BfgAH5\/JFKoCPkIzhaV6Bj72hfLnzp1LrMt+ebz9OYYJwh5SKscB4OMzIjHmXeVfo5rds+uDDz4IH3\/88Z5dTWOY4DMO3akK+Pjsa4x5Vzl8stso9\/f379u1NIYJ7cYh3XRtaWkpNBqNdst1\/PiYesvAJ6bess2om+YY8y4KfAYGBpI9vZp3Oc1edtle7V9\/\/XXZ3ld6nAXtzp07oS6aY+r9+y+uh\/53Qzi59qBwj2LqLSyyaWDdNKd6q9ys0x18zITf3vhDePG9X5btO8c5d+A3P\/4uXBr91rnK3pO3vr4ebJvyqlbvUeAzMjKS3Oc56LLLWv7OT8fCO0Nne6\/7PfKOf\/6vtR55p\/V6mwadqsBjzlQOn7wbzvVqF2pxAAfKOlA5fExo+qj97du34d69e2FxcbGsfo7DARyoqQNR4FNTr5CNAzggdMAdfI76AKLwfZcqNTk5GcbHx5NjV1ZWwvT09KF1bGzzZ5hKnbSNg4rozY7xsBItotku3e2pzPHjx8ObN2+SJ42rq6ttOFX+0CJ60+oHPd0tf+byRxbRnPXYzrSxsZE8oVa+XMHH8\/0gC87MzExYWFhI\/J+YmAizs7MHhj4F6NbW1p4PUCobl1eriN7sGLv0Nd1jY2PRJnMRzcPDw2Fubi4sLy+H+fn58Pjx48QK9cTI89d+XkRvto5pHR0d3fep\/iLnUo0pqrmKX56u4JP3AURVA8rUsWZcuHAhmZg7Ozt7JkC2no07ffp08lfpU73Nzc0yp2zrmKJ6syfJwj\/GSqKMZpvQ29vbR65C2zLyiINb0WtjP\/roo2S1loKzU7qOqltUs83F9PN4ndLpDj5HfQCxUyYUqZv9TWDj7bfvy5cvDw19FqSx4JNe9hXRa2OqCFzexCiqOb0ssEvFWJddRTNhq41bt26FBw8ehOvXr0eHT57H6epycHAwaVenLseBTxHyhBCKBi0tVzf4VLHMzrO6VY+tXvY3edWrtaJ6bXW2trYWnj9\/fuiKOc8b1c+Lam5ezaerfqXH7uCT9wFEVRNarVN0ueoJPkUuEz2seFLPWvXYjrMV0LVr18LDhw8r\/8hGEb3pTea+vr49kWv+NodW81h2fBHNzbU75bEr+HTLDed0Qse851P0xqKt0Ox11JO7skFv9bgimu2S4O7du+Hp06cJbGKuMIvozXrQfLO8VX8U44torspjV\/BJJ61910+nrjPbaWD2EWX2N9dBEzjmpMiuJNKPBhyk1\/4tT\/rIOj3G06Prwzz2+qg9LxMe4JNeqh6VC\/tFlPW4U3PRHXzagQPH4gAO1McB4FOfXqEUB7rKAeDTVe3kzeBAfRwAPvXpFUpxoKscAD5d1U7eDA7UxwHgU59eoRQHusoB4NNV7eTN4EB9HPgf4Kzeb\/PYbH4AAAAASUVORK5CYII=","height":173,"width":287}}
%---
%[output:06ea418d]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 25.191118 seconds.\n","truncated":false}}
%---
%[output:229918ab]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7t3U9oHde9B\/BTkcSY2CLkdaEU2qCF8LoKrRcpqLtCFwqFIl6ziTZaqd6JVBTTRdBCKdo5WhRJoG6ch8imWXRtLbJwQyRoll4IErBEaYOJHUwS4\/c493XU6xv9mat7z505M58BEyuae+bM5\/yu\/NU5M3O\/9\/LLL\/9vsBEgQIAAAQIEMhL4ngCT0WjpKgECBAgQINAREGAUAgECBAgQIJCdgACT3ZDpMAECBAgQICDAqAECBAgQIEAgOwEBJrsh02ECBAgQIEBAgFEDBAgQIECAQHYCAkx2Q6bDBAgQIECAgACjBggQIECAAIHsBASY7IZMhwkQIECAAAEBRg0QIECAAAEC2QkIMNkNmQ4TIECAAAECAowaIECAAAECBLITEGCyGzIdJkCAAAECBAQYNUCAAAECBAhkJyDAZDdkOkyAAAECBAgIMGqAAAECBAgQyE5AgMluyHSYAAECBAgQEGDUAAECBAgQIJCdgACT3ZDpMAECBAgQICDAqAECBAgQIEAgOwEBJrsh02ECBAgQIEBAgFEDBAgQIECAQHYCAkx2Q6bDBAgQIECAgACjBggQIECAAIHsBASY7IZMhwkQIECAAAEBRg0QIECAAAEC2QkIMNkNmQ4TIECAAAECAowaIECAAAECBLITEGCyGzIdJkCAAAECBAQYNUCAAAECBAhkJyDAZDdkOkyAAAECBAgIMGqAAAECBAgQyE5AgMluyHSYAAECBAgQEGDUAAECBAgQIJCdgACT3ZDpMAECBAgQICDAqAECBAgQIEAgOwEBJrsh02ECBAgQIECgtQFme3s7HB0dheXlZVVAgAABAgQIZCbQygATw8u1a9fC7u6uAJNZweouAQIECBCIAq0KMNevXw8rKyvhm2++6Yz+p59+KsB4HxAgQIAAgQwFWhdgXn311fDxxx+HtbW1cO\/ePQEmw6LVZQIECBAg0KoAUwz35ORk6QDzyiuvhPinezs8PAzxj40AAQIECBCoRkCAOeMi3hhcbt68Gaanp58Znc3NzbC1tVXNiDkqAQIECBAg0K5rYPqdgYnBZX19vXPdTPeMixkY7xwCBAgQIFCtgBmYM2ZgigCzuLgY9vb2qh0pRydAgAABAgSOBQQYAcbbgQABAgQIZCcgwAgw2RWtDhMgQIAAgVYGmLLDbgmprJT9CBAgQIDAaAUEmDO8BZjRFqOjESBAgACBsgICjABTtlbsR4AAAQIEaiMgwAgwtSlGHSFAgAABAmUFBBgBpmyt2I8AAQIECNRGQIARYGpTjDpCgAABAgTKCggwAkzZWrEfAQIECBCojYAAI8DUphh1hAABAgQIlBUQYASYsrViPwIECBAgUBsBAUaAqU0x6ggBAgQIECgrIMAIMGVrxX4ECBAgQKA2AgKMAFObYtQRAgQIECBQVkCAEWDK1or9CBAgQIBAbQQEGAGmNsWoIwQIECBAoKyAACPAlK0V+xEgQIAAgdoICDACTG2KUUcIECBAgEBZAQFGgClbK\/YjQIAAAQK1ERBgBJjaFKOOECBAgACBsgICjABTtlbsR4AAAQIEaiMgwAgwtSlGHSFAgAABAmUFBBgBpmyt2I8AAQIECNRGQIARYGpTjDpCgAABAgTKCggwAkzZWrEfAQIECBCojYAAI8DUphh1hAABAgQIlBUQYASYsrViPwIECBAgUBsBAUaAqU0x6ggBAgQIECgrIMAIMGVrxX4ECBAgQKA2AgKMAFObYtQRAgQIECBQVkCAEWDK1or9CBAgQIBAbQQEGAGmNsWoIwQIECBAoKyAACPAlK0V+xEgQIAAgdoICDACTG2KUUcIECBAgEBZAQFGgClbK\/YjQIAAAQK1ERBgBJjaFKOOECBAgACBsgKNCDCrq6thZmamc867u7theXn51PPv3vf+\/fthaWkpHBwcnLj\/9PR0WF9fD4uLi2Fvb6+sqf0IECgp8PTpj8LXX\/93uHTpf8LY2GclX2U3AgQIhJB9gJmbmwsLCwthY2OjM57F33d2dr4zvjdu3Aizs7Ph5s2b4R\/\/+EdYW1sLDx8+DPPz8wKMdwOBEQrE4PLtt6+H55\/\/KDx4sB9eeunHnSDz3HMfdf6fjQABAucJZB9g4ozK1NTU8UzK9vZ2ODo6OnEWpnff3q97sczAnFc+vk\/gYgJff\/2b8NVX74XLl98Njx\/\/Lly69H6I\/y9+ffnyHy\/WqFcRINAqgewDTAwscStmUXq\/7h7Nk2Zg7t27d+qSUxFgNjc3w\/7+\/nFTh4eHIf6xESDQv0CcfYlbnHGJ4aXYYoiJ4cVSUv+mXkGgjQKNCDDdMy5xVmViYuLUZaG45BSvaXnhhRfC7du3w61bt04d9yLA9O4QA83W1lYb68U5EzhT4Ddffx3ev3TpzH0eP377meDSu3OZWZgfPX0aXv\/223OPZbgIEGiuQKsCTAw3r732WucamLt374azZmvikBcBZmVl5ZkZFzMwzX1DOLOLC8Tw8t5XX4XfvvjimcGimIGJQSYuGxVbDC5lL+aNAWb\/wYNzj3Xxs\/FKAgTqLtCIABORz1tCmpyc7Fy0271k1H0B8EkX\/boGpu7lq391EYiB4i9ffhk+ev75Tqg4bytmYbqvfYnLSS+++NvO9TBltrcfPw6\/e\/w4zF692jmujQCBdglkH2B6l4xOu4hXgGlXYTvb0Qp8+OWX4YdPn4Yfv\/RSqQPHO5CePHm9M+NS3IUU\/1+8\/qWfu5DicV9\/8kSIKaVuJwLNEsg+wPRzG\/VJS0hXr1499VkwZmCaVezOJo1AXDqKsyFx5uUiMyFxNmaQO4\/6DU9pFLRKgMCoBbIPMBHstAfZFbMud+7cOb5YN87QXLt2rePsQXajLjfHa5pAcS3Ku5cvhz9evlzJ6RXLV5+PjYXZ8fFK+uCgBAiMXqARASYVmxmYVLLabYJADA7vPXrU19JRqvOuQ5BKdW7aJUDgZAEB5ozKEGC8bQicLhCXjeLy0UWXjoZtG2+r\/vDhQ9fDDBtWewRqKiDACDA1LU3dqrNAERaqXDo6ycedSXWuGn0jMFwBAUaAGW5Faa3xAnW\/5qS4qPeN8fHw2dhY48fDCRJoq4AAI8C0tfad9wUF4sPq4gxMnQNCDDFxc1HvBQfZywhkICDACDAZlKku1kWgeNpu3R8eV1zU+9FzzwkxdSke\/SAwZAEBRoAZcklprqkCRSiIn3VU5mm7VTvU9Tqdql0cn0BTBAQYAaYptew8Egvk+MA4dyYlLgrNE6hQQIARYCosP4fORWDQp+1WeZ7uTKpS37EJpBMQYASYdNWl5UYINOEhcTnOHjWieJwEgYQCAowAk7C8NJ27QJ2etjuopTuTBhX0egL1EhBgBJh6VaTe1EqgeNpunW+ZLgvmzqSyUvYjkIeAACPA5FGpejlygSbexdPEcxp5YTgggZoICDACTE1KUTfqJFA8bfej55\/P4pbpfuzcmdSPln0J1FdAgBFg6ludelaZQA5P2x0EJ55fvLOq7g\/kG+QcvZZA0wUEGAGm6TXu\/PoUyOVpu32e1nd2d2fSoIJeT6BaAQFGgKm2Ah29VgK5PW13UDx3Jg0q6PUEqhMQYASY6qrPkWsl0KRbpsvCujOprJT9CNRPQIARYOpXlXpUiUBxy3T8nKN48W5btuKi3nje8XOebAQI5CEgwAgweVSqXiYVaMLTdgcB8nEDg+h5LYFqBAQYAaaaynPU2ggUS0exQ7Pj47Xp16g7Eq+Hef3JE3cmjRre8QhcUECAEWAuWDpe1hSBJj1td9AxcWfSoIJeT2B0AgKMADO6anOk2gm4\/uPZISke4Pf52FirZ6NqV6g6ROAEAQFGgPHGaKlAk5+2O8iQtv16oEHsvJbAKAUEGAFmlPXmWDUSKJ62++OXXqpRr6rvimBX\/RjoAYEyAgKMAFOmTuzTMIFilsGj9L87sK4JalixO53GCggwAkxji9uJnS5gmeRkGy7eNQTyERBgBJh8qlVPhyrg2SfPcrqdfKjlpTECyQUEGAEmeZE5QH0FfBbQf8amrU8irm916hmBswUEGAHGe6TFApZM\/n\/wObT4TeDUsxUQYASYbItXx4cj0PalpDZ+iOVwKkcrBKoVEGAEmGor0NFrIdDmJ9D+5uuvQwxxbfsQy1oUnk4QGEBAgBFgBigfL22KQLGEEj+NOf5D3pbN0lFbRtp5NlGgEQFmdXU1zMzMdMZnd3c3LC8vnzpWN27cCG+++Wbn+48ePQo3b94Md+\/ePXH\/6enpsL6+HhYXF8Pe3l4Tx985ETgWiDMR8eF2bXo2TJtnnpQ+gdwFsg8wc3NzYWFhIWxsbHTGovj7zs7Od8Ym7hvDyAcffBBu3boVYvCZmpoKS0tL4eDg4Dv7CzC5l7f+9yvQpn\/QLR31Wx32J1AvgewDTG8I2d7eDkdHRyfOwsR9JyYmwvz8fKlREGBKMdmpYQL\/+uKL8NFzzzX6wwzbumTWsFJ1Oi0XyD7AxMAStyKU9H5djO\/k5GRYW1sL9+7dO3OJqbseBJiWvztaevrFJ1Q3eSmpTTNNLS1jp90CgUYEmO4Zl9NmWYoA8\/e\/\/z387Gc\/C1euXCl9Dczm5mbY398\/LofDw8MQ\/9gINFUg\/gP\/+pMnIX7Q42djY406zTZe69OoAXQyBP4t0LoAMz4+fnzhbpytuXr16rnXwPRWSww0W1tbiohAowX2HzwIn4+NNWopydJRo0vWybVMoBEB5qJLSN0XAJ900W+xhLSysvLMjIsZmJa9S1p6usVSUrytOt5e3YQt3mUVzyvOLNkIEMhbIPsA07tkdNZFvL3fiwHmrbfeCu+8886Jt1K7Bibv4tb7wQWa9JReS0eD14MWCNRJIPsA089t1PEZMLOzs88sIXXP3vQOjABTp1LVl6oEmvCBj3Hp6C9ffhk+ev75Vj2or6qacVwCoxDIPsBEpNMeZFdcuHvnzp3Oc1\/i1v0gu\/v37596\/UvcV4AZRQk6Rt0FiqWkdy9fDn+8fLnu3T2xf8XS0Rvj4427KDnLAdFpAkMQaESAGYLDiU0IMKlktZubQM5LSU28lie3+tFfAikEBJgzVAWYFCWnzVwFcnx2SrF01LS7qXKtIf0mMEwBAUaAGWY9aavBAjl+8GGcOYoX71o6anBhOrXWCggwAkxri9+J9y+Q0508OQau\/kfEKwi0V0CAEWDaW\/3O\/EICOSwlxfDy3qNHnfObHR+\/0Hl6EQEC9RYQYASYeleo3tVOIIen2RZLR\/EhfPHWaRsBAs0TEGAEmOZVtTNKLlDnD3y0dJR8+B2AQC0EBBgBphaFqBP5CRQf+PhfL79cm84XS0c\/fPrUxwXUZlR0hEAaAQFGgElTWVpthUDdPvAxXmQcl48sHbWi\/JxkywUEGAGm5W8Bpz+IQJ2WkiwdDTKSXksgPwEBRoDJr2r1uFYC8TH9ceYjfsLzZ2NjlfUth7ujKsNxYAINFBBgBJgGlrVTGrVA1R\/4aOlo1CPueASqFxBgBJjqq1APsheo8gMfc7itO\/sBdgIEaiggwAgwNSxLXcpRoKoPfLR0lGO16DOBwQUEGAFm8CrSAoF\/C4x6KSmnjzZQJAQIDFdAgBFghltRWmu1wCjvBLJ01OpSc\/IEggAjwHgbEBiqwKhmReLdT\/Ham3j3k40AgfYJCDACTPuq3hknF0h9XcqoQlJyKAcgQODCAgKMAHPh4vFCAqcJpFzeiW3\/5csvOx\/SGJ+4ayNAoJ0CAowA087Kd9bJBVI9pbdYOnpjfLzSB+clB3QAAgTOFBBgBBhvEQLJBIa9lFSEojjz8v6lS8n6rWECBOovIMAIMPWvUj3MWmBYH\/hYLB19PjYWZsfHszbReQIEBhcQYASYwatICwTOEBjWUlJ8UF68eNfSkXIjQCAKCDACjHcCgeQCcSnp9SdPLvyBj8VFwZaOkg+VAxDIRkCAEWCyKVYdzVvgoktJMby89+hR5+QtHeVdA3pPYJgCAowAM8x60haBUwUuegGupSNFRYDASQICjADjnUFgZAL9fuDjKD+aYGQIDkSAwFAEBBgBZiiFpBECZQXKfuBjsXT0w6dPfVxAWVz7EWiRgAAjwLSo3J1qHQSKpaR3L18Of7x8+dQuxTuO4oxNvHA3PnXXRoAAgW4BAUaA8Y4gMHKB85aSLB2NfEgckEB2AgKMAJNd0epwMwTOekrvsJ\/g2wwxZ0GAgBmYkjUwPT0d1tfXw+LiYtjb2yv5KrsRIFBG4LRZFktHZfTsQ4CAGRgzMN4FBCoTiGElfjjj7NWrnetcUn6KdWUn6cAECCQREGAEmCSFpVECZQW6l4ssHZVVsx8BAo0IMKurq2FmZqYzmru7u2F5efnckZ2bmwsLCwthY2Mj7OzsnLi\/JaRzGe1AYCgC\/\/rii\/DZ2FhnBqaYjRlKwxohQKCxAtkHmO4gEkfpvFAS95mcnAxra2vh+9\/\/fucaFwGmsfXtxDIRKJaS3r90qXPbtI0AAQLnCWQfYOLsy9TUVFhaWgoHBwdhe3s7HB0dnTkLc+PGjTA7O9uxMQNzXon4PoG0Ah5Yl9ZX6wSaKpB9gImBJW7z8\/Od\/\/Z+3Ttw169fD2+\/\/Xa4c+dOJ8SUCTCbm5thf3\/\/uKnDw8MQ\/9gIEBiOQPygxzfGxzvLSDYCBAiUEWhEgOmecYkzMhMTE8eBphchfj9u8bbo85abimtgetuIgWZra6uMr30IECBAgACBBAKtCjDxeplf\/epX4fe\/\/334yU9+UjrArKysPDPjYgYmQSVqkgABAgQI9CHQiAATz7fMElJcXvrkk0\/CrVu3gruQ+qgSuxIgQIAAgZoJZB9gepeMTruIN177EmdSrly58p0huH37difU9G5uo65ZteoOAQIECBD4t0D2AeYit1HHczcD4z1AgAABAgTyFcg+wET60x5kVzzvJd5x1DvDIsDkW7R6ToAAAQIEGhFgUg2jJaRUstolQIAAAQKDCQgwZ\/gJMIMVl1cTIECAAIFUAgKMAJOqtrRLgAABAgSSCQgwAkyy4tIwAQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJmAACPAJCsuDRMgQIAAgVQCAowAk6q2tEuAAAECBJIJCDACTLLi0jABAgQIEEglIMAIMKlqS7sECBAgQCCZgAAjwCQrLg0TIECAAIFUAgKMAJOqtrRLgAABAgSSCQgwAkyy4tIwAQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJmAACPAJCsuDRMgQIAAgVQCAowAk6q2tEuAAAECBJIJCDACTLLi0jABAgQIEEglIMAIMKlqS7sECBAgQCCZgAAjwCQrLg0TIECAAIFUAgKMAJOqtrRLgAABAgSSCQgwAkyy4tIwAQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJlAIwLM6upqmJmZ6SDt7u6G5eXlE8GuX78eVlZWwpUrVzrfv3\/\/flhaWgoHBwcn7j89PR3W19fD4uJi2NvbSzYIGiZAgAABAgT6E8g+wMzNzYWFhYWwsbHROfPi7zs7O89ITE5OhrW1tXDv3r1OwCm+fvjwYZifnxdg+qsbexMgQIAAgUoFsg8wcfZlamrqeCZle3s7HB0dnToL063d+9rekTADU2ltOjgBAgQIEDhVIPsAEwNL3IpZlN6vzxp7AcY7gwABAgQI5CnQiADTPeMSQ8nExMSpy0LFMBXXw3zyySenztYUMzCbm5thf3\/\/eIQPDw9D\/GMjQIAAAQIEqhFoZYAprn+J5GUu4u0dmhhotra2qhkxRyVAgAABAgRCIwJMP0tIZcNLbLOYgYl3LnXPuJiB8c4hQIAAAQLVCmQfYHqXjM66iLfMnUfdw+Ei3mqL09EJECBAgMBpAtkHmLK3UUeAGG6uXr165rKRAOPNQoAAAQIE6i+QfYCJxKc9yK6Ycblz507429\/+9sxD7IqhefToUbh582a4e\/fud0bLDEz9C1gPCRAgQKCdAo0IMKmGToBJJatdAgQIECAwmIAAc4afADNYcXk1AQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJmAACPAJCsuDRMgQIAAgVQCAowAk6q2tEuAAAECBJIJCDACTLLi0jABAgQIEEglIMAIMKlqS7sECBAgQCCZgAAjwCQrLg0TIECAAIFUAgKMAJOqtrRLgAABAgSSCQgwAkyy4tIwAQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJmAACPAJCsuDRMgQIAAgVQCAowAk6q2tEuAAAECBJIJCDACTLLi0jABAgQIEEglIMAIMKlqS7sECBAgQCCZgAAjwCQrLg0TIECAAIFUAgKMAJOqtrRLgAABAgSSCQgwAkyy4tIwAQIECBBIJSDACDCpaku7BAgQIEAgmYAAI8AkKy4NEyBAgACBVAICjACTqra0S4AAAQIEkgkIMAJMsuLSMAECBAgQSCUgwAgwqWpLuwQIECBAIJmAACPAJCsuDRMgQIAAgVQCAowAk6q2tEuAAAECBJIJCDACTLLi0jABAgQIEEgl0LoAs7q6GmZmZjqeu7u7YXl5+VTb6enpsL6+HhYXF8Pe3l6qMdAuAQIECBAg0KdAqwLM3NxcWFhYCBsbGx2m4u87OzsnsgkwfVaT3QkQIECAwIgEWhVg4uzL1NRUWFpaCgcHB2F7ezscHR2dOgsjwIyoCh2GAAECBAj0KdCqABMDS9zm5+c7\/+39uteuCDCbm5thf3\/\/+NuHh4ch\/rERIECAAAEC1Qi0LsB0z7jEGZmJiYnjQHNagOn9\/zHQbG1tVTNijkqAAAECBAgEAaZEgFlZWXlmxsUMjHcOAQIECBCoVqB1AeYiS0juQqq2SB2dAAECBAj0CrQqwPQuGbmI1xuCAAECBAjkKdCqAOM26jyLVK8JECBAgECrZ2DiyXuQnTcBAQIECBDIX6BVMzD9DpfnwPQrZn8CBAgQIDAaAQHmDGcBZjRF6CgECBAgQKBfAQFGgOm3ZuxPgAABAgQqFxBgBJjKi1AHCBAgQIBAvwICjADTb83YnwABAgQIVC4gwAgwlRehDhAgQIAAgX4FBBgBpt+asT8BAgQIEKhcQIARYCovQh0gQIAAAQL9CggwAky\/NWN\/AgQIECBQuYAAI8BUXoQ6QIAAAQIE+hUQYASYfmvG\/gQIECBAoHIBAUaAqbwIdYAAAQIECPQrIMAIMP3WTK33f+WVV8Ivf\/nL8Ne\/\/jUcHh7Wuq86F4Lxyq8KjJkxq4uAACPA1KUWh9IPn181FMaRNWK8RkY9tAMZs6FRjqyhpo6ZACPAjOxNNIoDNfWNOgq7Ko5hvKpQH+yYxmwwvype3dQxE2BKBJjNzc2wv79fRd05Zp8CcXr75s2bwZj1CVfR7sarIvgBDmvMBsCr6KXFmC0uLoa9vb2KejH8wwowZ5gWgx7Tq40AAQIECOQqEIPLyspKo64NFGDOqcYYYuIfGwECBAgQyFUg3tTQtBsbBJhcq1G\/CRAgQIBAiwUEmBYPvlMnQIAAAQK5CggwuY6cfhMgQIAAgRYLCDAtHnynToAAAQIEchUQYHIdOf0mQIAAAQItFhBgWjz4Tp0AAQIECOQqIMDkOnL6TYAAAQIEWiwgwPQx+Nvb2+Ho6CgsLy\/38Sq7phK4fv1658FMV65cCY8ePeo8gffu3bsnHu7GjRvhzTffPP7eN998E9bX18POzk6q7mm3hEB8T127dq2z5+3bt8OtW7dKvMouoxCYm5sL8cmtL7zwQrh\/\/35YWloKBwcHJx56dXU1zMzMHH\/vvPfjKPrvGP8RmJycDGtra+HOnTuNeo8JMCWrvPhBu7u7K8CUNEu9WxyTuM3Pz4fuv5903PgDdmJiorOvrR4CMVTOzs52gudPf\/rT47+fFkLr0et29KL4B+\/evXvhT3\/6U+cfv\/j3035588tdfeuiGMsf\/OAHjfslQYA5p+6K3\/Ljb+xx+\/TTTwWYGrxXi3H58MMPO79RxN8W33rrrfDOO++cOAvjB2wNBq2nC92hs6m\/IdZPvVyP4vtpYWEhbGxsdGYpY9j8+c9\/fuIsjLErZ1rFXsUs2j\/\/+c8wPj4eip+XVfQlxTEFmBIB5tVXXw0ff\/zxub+FpBggbZ4s0PsDtvfr7lf5AVu\/Kur+DT\/+Vt\/7df163K4edc+OxRmx3q+7NeIvE3\/4wx\/Cn\/\/8Z0uyNSuTX\/ziF50ePXjwoLPcLsDUbIBG1R0\/YEclXe44vTMuZ\/0Q7b5WpmjdUmA551R7nRQqzZKl0u6\/3d4Zl7NmOLuvlSmO5Hqm\/s1TvqJ3xjrlsUbZthmYktoCTEmoEe3WT4ApfsB+8MEHx8tN8eLE4usRddlhugQEmHqXQz8BJu7761\/\/+vii+N6v632m7eidANOCce6+kr73KnoBproC6J1BibMn8aPhu9foz1pCOqnn5130W93ZtuPIlpDqPc79LCH1nomflfUbWwGmfmMy0h55U46U+9yD9S4ZnXcRb2+DlivOJU6+Q\/cYuE4pOXdfB+h9P511Ee9pAaZpt+z2BViznQWYmg3IqLsjwIxa\/Pzjlb2N+qTfJrunvM8\/kj1SCLiNOoXqcNrs5zbqOHM9NTV1fIdS\/Pq1117CDFhzAAACPklEQVQ787lMw+mlVsoKCDBlpRq6nwBTv4E960F2vc996X2QnYsM6zGeHmRXj3E4qRdnPciu9\/3VvfzuIZH1G1MBpn5jokcECBAgQIBASwXchdTSgXfaBAgQIEAgZwEBJufR03cCBAgQINBSAQGmpQPvtAkQIECAQM4CAkzOo6fvBAgQIECgpQICTEsH3mkTIECAAIGcBQSYnEdP3wkQIECAQEsFBJiWDrzTJkCAAAECOQsIMDmPnr4TIECAAIGWCggwLR14p02AAAECBHIWEGByHj19J0CAAAECLRUQYFo68E6bAAECBAjkLCDA5Dx6+k6AAAECBFoqIMC0dOCdNgECBAgQyFlAgMl59PSdAAECBAi0VECAaenAO20CBAgQIJCzgACT8+jpOwECBAgQaKmAANPSgXfaBAgQIEAgZwEBJufR03cCBAgQINBSAQGmpQPvtAkQIECAQM4CAkzOo6fvBAgQIECgpQICTEsH3mkTIECAAIGcBQSYnEdP3wkQIECAQEsFBJiWDrzTJkCAAAECOQsIMDmPnr4TIECAAIGWCggwLR14p02AAAECBHIWEGByHj19J0CAAAECLRUQYFo68E6bAAECBAjkLCDA5Dx6+k6AAAECBFoqIMC0dOCdNgECBAgQyFlAgMl59PSdAAECBAi0VECAaenAO20CBAgQIJCzgACT8+jpOwECBAgQaKmAANPSgXfaBAgQIEAgZwEBJufR03cCBAgQINBSAQGmpQPvtAkQIECAQM4C\/wefxK0WEMKPhwAAAABJRU5ErkJggg==","height":173,"width":287}}
%---

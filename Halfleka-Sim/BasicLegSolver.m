%[text] ## Solver
clear;
clc;
% Starting values:
% BusDeclaration;
q0 = [0, 0.7, -pi/2+0.1, 0.6];
dq0 = [2,-0.05,0,0];
X0 = [q0, dq0];
Sim.time = 1.1;
tspan = [0 Sim.time];
options = odeset('RelTol',1e-10, 'AbsTol',1e-10, 'Events',@(t, X) collision_event(t, X),'MaxStep', 1e-5);
tic

Animate = 2; % 1 = fanimate, 2 = drawnow


t_total = [];
x_total = [];
te_total = [];
count = 0;
% Using ode45
while tspan(1) <Sim.time %[output:group:7d860795]

    % solve (with event detection)
    [t, x, te, xe, ie] = ode113(@(t, X) basic_leg_dynamics(t, X), tspan, X0, options); %[output:1c5d8d47]

    % Store results
    t_total = [t_total(1:end-1);t];
    x_total = [x_total(1:end-1, :);x];
    te_total = [te_total(1:end);te];
    if ~isempty(te) % Impact
        count = count + 1;

        % calculate post-impact velocity
        dq_post =dq_impact_Func(x(end, 1:4), x(end, 5:8));
        
        % update initial conditions to restart solver
        X0 = [x(end, 1:4), dq_post'];
        tspan = [te(end) Sim.time];
    else
        break
    end
end %[output:group:7d860795]
toc %[output:029ac02b]


%%
%[text] ## Display generalized Coordinates
% display state of all variables
plot(t_total,x_total(:,1)); %[output:6482b1bf]
hold on %[output:6482b1bf]
plot(t_total,x_total(:,2)); %[output:6482b1bf]
plot(t_total,x_total(:,3)); %[output:6482b1bf]
plot(t_total,x_total(:,4)); %[output:6482b1bf]
legend('x','y','th1','th2'); %[output:6482b1bf]
title("Generalised Coordinates"); %[output:6482b1bf]
xlabel('time (s)'); %[output:6482b1bf]
hold off %[output:6482b1bf]

% display derivative state of all variables
plot(t_total,x_total(:,6)); %[output:96a38533]
hold on %[output:96a38533]
plot(t_total,x_total(:,6)); %[output:96a38533]
plot(t_total,x_total(:,7)); %[output:96a38533]
plot(t_total,x_total(:,8)); %[output:96a38533]
legend('dx','dy','dth1','dth2'); %[output:96a38533]
title("Derivative of Generalised Coordinates"); %[output:96a38533]
xlabel('time (s)'); %[output:96a38533]
hold off %[output:96a38533]
%%
%[text] ## Forces
coords_total = zeros(8, length(t_total));

foot_total = zeros(4,length(t_total));
grf_total = zeros(2, length(t_total));
contact_total = zeros(1, length(t_total));
forces_total = zeros(2, length(t_total));
dth_c_total = zeros(4, length(t_total));

Controller_total = zeros(9, length(t_total));
% Controller_total(9, :) = repmat(States.Contact,1, length(t_total));
ddq_total = zeros(4, length(t_total));

Mddq_total = zeros(4, length(t_total));
C_total = zeros(4, length(t_total));
G_total = zeros(4, length(t_total));
J_total = zeros(4, length(t_total));

groundheight = 0;
% controller_t = 0;
% diff_t = 0;
% th_old = [0; 0];
for i = 1:length(t_total)
    % States
    coords_total(:, i) = x_total(i, 1:8)';
    x = x_total(i, 1);
    y = x_total(i, 2);
    th1 = x_total(i, 3);
    th2 = x_total(i, 4);
    dx = x_total(i, 5);
    dy = x_total(i, 6);
    dth1 = x_total(i, 7);
    dth2 = x_total(i, 8);

    out = SolverFuncRaibert(t_total(i), x_total(i, 1:8));

    foot_total(:,i) = out.foot;    
    Controller_total(1:9, i) = out.controller;
    
    
    % t1 = Controller_total(1, i);
    % t2 = Controller_total(2, i);
    % fy = 0;
    % fx = 0;
    if out.lambda(2) >=0
        grf_total(:, i) = out.lambda;
    end

   
    % Contact logging
    if i == 1
        if foot_total(2, i)<=0 && foot_total(4, i) <0
            contact_total(i) = 1;
        end
    else
        contact_total(i) = contact_total(i-1);
        if contact_total(i-1)==0 && foot_total(2, i)<=0 && foot_total(4, i) <0
            contact_total(i) = 1;
            groundheight = foot_total(2, i) + 1e-5;
        else 
            if (foot_total(2, i) + 1e-5)<groundheight
                groundheight = foot_total(2, i) +1e-5;
            end
            if contact_total(i-1)==1 && foot_total(2, i)>groundheight && foot_total(4, i)>1e-3
            contact_total(i) = 0;
            end
        end
    end
    
    % J_total(:, i) = transpose(J_calc_func(x, y, th1, th2))*grf_total(:, i);

    ddq_total(:, i) = out.ddq;
    Mddq_total(:, i) = M_calc_func(x, y, th1, th2)*out.ddq;
    C_total(:,i) = C_calc_func(x, y, th1, th2, dx, dy, dth1, dth2);
    G_total(:,i) = G_calc_func(x, y, th1, th2);
    dth_c_total(3, i) = out.dth_c(1);
    dth_c_total(4, i) = out.dth_c(2);
    forces_total(:, i) = out.u(1:2);

end
clear t1 t2 footdynamics ddq x y th1 th2 dx dy dth1 dth2
%%

plot(t_total,foot_total(2,:)); %[output:5158e8f4]
legend('foot_y'); %[output:5158e8f4]
% % hold off
plot(t_total,foot_total(4,:)); %[output:06358434]
legend('dfoot_y'); %[output:06358434]
%%
%[text] ## Load into Simulink
BusDeclaration
time = t_total;

dth_c_sols.time = transpose(time);
dth_c_sols.signals.values = transpose(dth_c_total);

coords_sols.time = transpose(time);
coords_sols.signals.values = transpose(coords_total);

forces_val = [grf_total;foot_total;contact_total];
forces_sols.time = transpose(time);
forces_sols.signals.values = transpose(forces_val);

cmd_sols.time = transpose(time);
cmd_sols.signals.values = transpose(Controller_total);

Mddq_sols.time = transpose(time);
Mddq_sols.signals.values = transpose(Mddq_total);

C_sols.time = transpose(time);
C_sols.signals.values = transpose(C_total);

G_sols.time = transpose(time);
G_sols.signals.values = transpose(G_total);

J_sols.time = transpose(time);
J_sols.signals.values = transpose(J_total);

Accel_sols.time = transpose(time);
Accel_sols.signals.values = transpose(ddq_total);

Dist_sols.time = transpose(time);
Dist_sols.signals.values = transpose(forces_total);


sim("data_inspector_basic_leg.slx");
%%
%[text] ## Animate (fanimate)
if Animate ==1
    % Import important coords
    LegParams
    clear cby cl1x cl2x b grav mb Il1 Il2 Ib ml1 ml2

    x_sim = @(t) deval(sols, t, 1);
    y_sim = @(t) deval(sols, t, 2);
    th1_sim = @(t) deval(sols, t, 3);
    th2_sim = @(t) deval(sols, t, 4);
    
 
    
    
    x_knee = @(t) x_sim(t) + l1*cos(th1_sim(t));
    y_knee = @(t) y_sim(t) + l1*sin(th1_sim(t));
    

    x_foot = @(t) x_knee(t) + l2*cos(th2_sim(t) + th1_sim(t));
    y_foot = @(t) y_knee(t) + l2*sin(th2_sim(t) + th1_sim(t));
    
 
    
    fanimator(@(t) plot(x_sim(t), y_sim(t), 'b*'), 'AnimationRange',tspan, 'FrameRate',60);
    axis equal;
    pbaspect([1 0.2 1]);
    axis([-2 2 -0.2 1]);
    hold on;
    fanimator(@(t) plot([x_sim(t) x_knee(t)], [y_sim(t), y_knee(t)], 'r-'), 'AnimationRange',tspan, 'FrameRate',60);
   
    
    fanimator(@(t) plot([x_knee(t) x_foot(t)], [y_knee(t) y_foot(t)], 'r-'), 'AnimationRange',tspan, 'FrameRate',60);
    
    hold off;
    % writeAnimation("HalfLeka_Hop.gif",'framerate',60);
    % vidObj = VideoWriter('HalfLeka_Drop','MPEG-4');
    % open(vidObj)
    % writeAnimation(vidObj,'framerate',60, 'SpeedFactor', 0.1)
    % close(vidObj)
    playAnimation('SpeedFactor', 0.1)
end
%%
%[text] ## Animate (drawnow)
if Animate ==2 %[output:group:93817c37]

    % Import important coords
    LegParams
    clear cby cl1x cl2x b grav mb Il1 Il2 Ib ml1 ml2

    fps = 30; % Desired frame rate
    dt = 1 / fps; % Time interval between frames
    t_interp = t_total(1):dt:t_total(end); % New time vector
    
    % Interpolate all states in x_total
    solsy_interp = interp1(t_total, x_total, t_interp, 'linear')';
    
    
    x_sim = solsy_interp(1,:);
    y_sim = solsy_interp(2,:);
    th1_sim = solsy_interp(3,:);
    th2_sim = solsy_interp(4,:);

    x_knee = x_sim + l1*cos(th1_sim);
    y_knee = y_sim + l1*sin(th1_sim);
    
    x_foot = x_knee + l2*cos(th2_sim + th1_sim);
    y_foot = y_knee + l2*sin(th2_sim + th1_sim);
    
    fig = figure; %[output:1ef3f307]
    
    ax = axes('Parent',fig); %[output:1ef3f307]
    
    b = animatedline(ax,'Color','b','LineWidth',1, "Marker", "*"); %[output:1ef3f307]
    u = animatedline(ax,'Color','r','LineWidth',0.5); %[output:1ef3f307]
    l = animatedline(ax,'Color','r','LineWidth',0.5); %[output:1ef3f307]
    
    axes(ax); %[output:1ef3f307]
    
    axis equal; %[output:1ef3f307]
    axis(ax,[(x_sim(1)-1) (x_sim(1)+1) -0.5 1]); %[output:1ef3f307]
    
    hold on; %[output:1ef3f307]
    tic
    drawnow
    
    for i = 1:length(solsy_interp)
    
        clearpoints(b);
        clearpoints(u);
        clearpoints(l);
    
        % axis equal;
        % axis([-1 11 -0.5 1]);
        % body
        addpoints(b,x_sim(i),y_sim(i)); %[output:1ef3f307]
        % upper
        addpoints(u,[x_sim(i), x_knee(i)],[y_sim(i), y_knee(i)]);
        % lower
        addpoints(l,[x_knee(i), x_foot(i)],[y_knee(i), y_foot(i)]);
    
        axis(ax,[(x_sim(i)-1) (x_sim(i)+1) -0.1 1]);
    
        drawnow
        pause(dt);
    
    end
    drawnow
    toc %[output:42ae0281]
    hold off; %[output:1ef3f307]
    clear fps dt
end %[output:group:93817c37]

%%
%[text] ## Functions
function out = basic_leg_dynamics(time, q)
    solv = SolverFuncRaibert(time, q);
    out = [q(5:8);solv.ddq];
    out = reshape(out, 8,1);
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
    foot = foot_Func(q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8));
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
clear Animate ax b C_total contact_total Controller_total
clear ddq_total dq0 dq_post dth_c_total fig foot_total
clear forces_total forces_val G_total grf_total groundheight i ie J_total l l1 l2
clear Mddq_total options out q0 solsy_interp t t_interp t_total
clear te th1_sim th2_sim time tspan u X0 x_foot x_knee x_sim x_total xe y_foot y_knee y_sim

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":30.8}
%---
%[output:1c5d8d47]
%   data: {"dataType":"text","outputData":{"text":"Time = 1.1000","truncated":false}}
%---
%[output:029ac02b]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 36.231692 seconds.\n","truncated":false}}
%---
%[output:6482b1bf]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAALsAAABxCAYAAACN49K4AAAAAXNSR0IArs4c6QAAFK9JREFUeF7tnQ9wFcUdx38kBiQkUCkEEhIytoPAWLUQlaaxdehYrUCrtIjWPy3jH8QBGUdpwQ7ajiCjCNQpUhSkg3XGVtCitIPt1KlaSwE1CLY2RXQYCUkMxKIkJAgGOt9Nfo\/N5u7d3r299+7e251h8njvbu+3v\/3s737729\/t9bnyyitP7dixg7isX7+eRo8enfg\/fzh+\/DitWrWKNmzYIL7i4xobG2nevHm0b98+8f1DDz1El156KbW1tdHChQsJdU+fPp1mz55Nffv27VXvnj17aMaMGUnrlOvtVUH3F8888wytXLmSzj77bFq2bBmVlZUR1+303YQJE2jx4sVUVFTkWOVrr71GCxYs6NEmpwP5uvjNq04cIx\/vVJ8sq1db+fdk+lX7J5mMcp8l62O\/bZXl4\/Y7fafWy32QrH04R2bozjvvpOuvvz6hOm7\/7bffTn0GDx58SlWqeoKqBL+w43inTpSB8lKu\/Ds+QyZAPXPmTE+wnWBPNridgFQVrg5+rk9tJ+TcvHkzTZs2TQx2L9i5HrUP8L0KrtxvTvqVIVD7WDVqTnW7GTS\/bU0VdlxP1Qf0OGLEiF6G1Un\/bHQdYXezKPZ7q4E4a8AX7Ko10LVScVaQlT17NKANO0DHbYNvCXxbUV2R7FGNbUm2aUALdva5nnvuOTEJlP3d4uLiHhPUbFOQbU\/2aEALdrfmwtpb2LMHhmxvSWDYOcTo5beXlpYS\/tliNRBUA01NTYR\/qRbfsMshoGShLQgGyOHjjx8\/PlU57fk5rIGdO3eKNZFUgfcNuxpjHThwYGLSqvYHIMdClAlBnfp63LhxdOutt9r6kwyEbNERFiUBfSolMOy4KE9ct23bllhtlIVh2E0I6tRIW79311sdndaRhT0JL3EHBU0Lqw08F2NX9cknn6S3337be\/RpHCH76Cbl14IdfjqWu+XcGMjM\/rvbJNWkoNaya1DicEgYfRD2XKyzs5MefPBBeumll4wOVi3YOd8AuuSkL3ZhWlpaXOPsYSjaukn+oA+jD0zOxZBsN2TIEFq3bh2NGTOGJk6cSPn5+SJxED66Sfm1YGf1qukCXqunJgV16mJYmEmTJtGWLVtSnqlnY\/0cETOtI7lfd33QRPmDynyNwM5PG+nkp12hRM7ChNFEpmpdXR1dcMEFmYfdV4tC9Bf9ymGPN6sBGfb\/Dqiiwm\/M9HWB9tfXUPvraxPnsDuMzEtY+Pvuuy+zsOuk6KotDtuy+9KwPdiYBkxadgglPwMB72HOnDmZg51vNUeOHOnls+OhDX74wsJujKdIV2TKiAHs4cOHU\/\/+\/Wn37t0i0XDQoEE0YMAAWrRokXBPTV0LCtXy2d2iMRiRVVVVnotKYcXZI01EFgsnA3hw726qKMr31dr6tk460NopuMF84tChQ3TVVVfRrFmz6KabbqK8vLzMWXa3lgD26urqXiFJPt7kqPSlTXtwqBqQ+\/VS2kN3Vzk\/2ugmxIraNvrlzqO0ceNGYc2bm5tp6tSpYpHyrrvuoj59+ojfVqxYkdSyw+O4\/\/776amnnko8Lpqs4VqW3a0CNcfdzY1RFxxMJfaE2qO2clcNmLLsmAeCoc8\/\/1ykfeA50ZqaGjrjjDOImeGYvuodsGuNRx3V9R83wQPDrvPwBitFvTgaglm3LfHUgKk7NmBfvXo1FRYWCugnT55MZ511lvDh1SLDzuzh+V6UtWvXhmfZdRaUIITb4oO17PGE3LR7+vzzz4vYOsqxY8cIK6d4kB3AHz58WPzlwrBzVPDVV18V7s9tt90WHuy6oMuw2wlqvOF2c0\/Rry3N\/6WKskJfDaxvbKf9De1ix4lHH31URF\/wD3H2oUOHUkFBAb388ssi3n7DDTeIUOSuXbvojjvuENfhUCVf9P333xcTW6\/iy42Rg\/\/yXjFuFzF1u\/NqhP09vRqQ+\/Wy6g6aP3usLwEeXlVHS1fV0SWXXEJLliwRVr1fv36E7FkEPOCHq6HH+vp6MYGVI4CVlZViAGBCu337dsfMW1kwbdgZdK8HNuTKLey+GIjNwXK\/pmLZZdgBLHxv3kxLzY0B7PPnzxd7Be3du1eADfjhxnz00UfC7\/cywFqws+uSbAHJqadyDfaS\/hVCDQc76ok\/s17w3bmDvx4boJMJWj52KP1i2YJELDxoo1588UUqKSkRp8NXx+IkJqqw7MiVQYIYF7bs8rVCgd1tSzy+cKZSfIMqGecBRgDot+C8c7\/4dXH+uYNrqKSwC3A30PkaKvx+r+s2iILUk+o5fSs76Is\/akoZ9ltuuYVuvvlmOnXqFN1zzz1iq0QkFwJ21Wc\/evQoXXbZZT1EB+zYEQ6hS1h3t5V8PknLsgdVThQtO6CbWH5tAtSVu+fSu\/\/7p2cT+bxrR\/0kYb1fOfB7Kuk\/kv6Gv93QH2yvp0Md9WIg6VpyHB+nMm78+IRlP1B3iIZ239F028D64Tg7fPYrrrhCnM6wqz47LD\/2EFUt+9y5cwXscHHkPUudZAkEO98+vOKbUYKdYZ1Yfp3Qw7N7H6GvDK4R4M965UJPK3\/n+b8SFv3dj\/8p4NYZILqdH9Xj8gaVilTcgpFVlPeFUsofVEp5g8roq18eTo9c91Vh2Ue1fovYAOi2A7p\/du8yEY154oknRCSmvb1dpA8sXbpUWHbVZ3eCnd0ghCHvvfdez8v7ht3PylVUYAfoj098SwCtWvIHJmwSlnjlO3MdlSWfe\/\/2qZ6DwlPjETkAIKMgFx0gn\/ykifqdP6X7u1IqqKzqIWlnd\/75yU8a6bwhpxKwp2rZ5QknLohdoisqKnrBrvrscK0xWHRXT1G3L9jlbTTcdrGVNRQF2K8dNY9gzeFywJqoBZYdVvu+7VN7WWsGnS1RRDjVEgMwM8g4Ada5C+5SyvtC10IOPnNhmE98WCu+OrG\/VgwAFPlhC\/yf+zXV506R8YjQYUNDA61Zs0ZcC7F1fO8UjYFHgeKVpuKmIG3Y5fQACOf0TKp6kUzD7gU6ywurrw4GgD7n\/F\/Ru\/\/b6jhItIgL8SAZZl2QYZUB9clPu\/92wwyw\/ZSwn0GFS3PjjTeKp8+YIac4u5ePrrZJG3b5RLeUXzfYM5EIxhZbxyrDsuP4728ZJpoA0B\/42ibh179y4Fk\/HBg5VgaZ\/WS2wrDK+Axo+S8uyiCrVtkvyLoNMLXTG7IdEWXp6OigTZs20eWXXy4mnJigovDAAuzLly93fYGE0zsEMgK7etGwE8HY\/QCobr64LBNAxyQL5+EcTETd3B5dGJyOYz+5ojhfTPTKOpvp0JeupAmFH1NpZ7NwNcpONlNZ50FxOj435g0Tf+VyoPWk+G95cZ5wNbY1Hafy4nyRI45S39pJB9q6jsF3+D0OBaFIZD86FRMpJ2mx7OqOYGEngnHkBFEW3QJXBgWhw6Cui+onV5aXC3ABN8r3Sw4nxLnwxL8Sn99oHyL8Yi4M7bamE4nvyovyejWFga4uLTh9XDf01aWnX+mDgYAC6FE3BgPK9qYTkRoITncL3tEsNrCbEFQX2rAmlW5hOJ7wVRTlUdnJg\/TdYy8Lazyl+WlhbVEYZvz98MCBtFlcvj6eJPqaNCCqy\/qKp4vkQbCtscv6R20AmJz3hW7ZH3\/457R40WLa\/Pc3dXlN6bhk0RW3itUwnOony2E4+MoAG39hna\/u9x\/xl4tsPQEOP4KWUqNCOplhv2bUmeLuc805XXnkfGfZ8F6HcIc2vtcRkgTe1cYK9g2Tz6J+H\/07ocCh3SuNdS2nd\/b94JNh9MEnwxMtP9ixX6xw6hbEyeVVPKxmJnNh4BsXVI4XfjNAlid6uCYmeyhy5AKW8OLCFrq48GPhQ8NNEC5B9\/OUcDkyCYWurryOwwAA\/Cjy43Z4lC4Tlj9WsD9c8xjltx6kzk\/PoNbPLuqhayzywEcGnMc6\/yCAhYVhn5J9Xfkk\/AaLmawgiiLnvcByA+ouyLvizRy9SBZP5o7HbZ\/h5snexveORcrf9YI46O\/QAdqOOQPgxwDHd9ADdBD2AI8V7Mtm\/paeWL6+Rw6JnD\/i1AmqrwnoWel8PEcZYFHliAO7JAz3medPERYaBQsmgPuzd\/7k2ve4zt3jB3R1cPdkDx2bLZY7KPR8nmwAoHe4PgAeRggPUZsuGYddt0EmBZWVzZGGmjGV9INhh6kxv0REPd4qOE\/8q+17HjXmlYjJIEAH3MnizdyB08\/pn7BamLBFbbKmq\/d0HSeDj2vylhorsHOAIT\/fJEO+Jqjqi1fTvdcjh\/Y4h4P97cG71glLPLn5afEXRb7dcudzZ8DP5mPkEB0mZBbwYEOF77zXnHOm6AO29qnqMyOwq6umXi8igMpSEZT9bNRz5nlTRD4H\/GzO6\/jsnT\/SiQ939srbwPFQvBxag7\/JMWl85rkAbr34fPdrR4L1sD3LUQMMPtxBLkEjO6kwpAqnZdk50xE5MXKCvKkdwdhiQzhYbbbY7GdzVARwh7X8bbkNRwPs6siRnel\/Oqw9uU877G7vQXX7ntXmJqgcHUHYD3DLKaQAGmAL2H0mKYXTZbZWUxrAhJbXInTqTDvsbolfXq4MBP35I6voyXXr6F8f96GWgjIqHzOeWvp2pZgCcERIODpiwdbp\/uw9xildwG1HsCBa0HJj3GBn96a2ttb1BWKjblstQEcZ1p+opPCUgPuF1V1ZbbZYDbAGIpEIlgrssx74Na1ZvijxpgU0LOxEMItPPDUQiUSwVNwYPDaVzkSweHazldpNA2n32U1PUG3XWg3oaiDtsLv55qZCj7oNt8flngbSDjtUzJtJ8oZIXpEYnGNS0NzrZtti0wxpRWNY7ZlOF7Ddn3saMGkwfcHuV9UmBfV7bXt8dmjAJEOBYI\/jjmDZ0fW514qMwh7HHcFyD5HsaXHGYI\/jjmDZ0+252ZKMwB7HHcFyE4\/sanVGYJdVGIcdwbKry3OjNZFIBFNV7Rd29fywdwTLDTSyr5WRSARLFfZ07wiWfRjkRovSmgjGq6LYDJ6L03Omfi27TQTLDVjDaGXsfHYLexgY5EadFvbc6GfbSsP5VYFWUK0bYzlMlwZiZdmf\/80U8WQSXg77jzdbqL4Br\/I+SvxK73QpzV4nnhrIOOy6aoOgT6+9g175y+M0sqyQKkYMoPqGo1Rz8dBEFVvfONSjuv3d77XHlxgYcqkYUZgYLCNHnN6TBIMHBd\/h89Y3W3RFtMdFXAMZgR1vJsObzcrKunYGQAm6I9jIEYVUUVYo4EQBxPhOLhgcKICfBwr+v\/XNQ1RzUddg4XP2dw8K\/v\/Dq+po6aq6iHejFU9HA2mHnZO\/jhw5kng\/vM4r3k0K6qUYgA7of3h1Jf109lia87O3rIX3UloMfjfJkNYE1W1CGtXH8t7+63eEu\/S9Ga\/HoDutiMk0kHbY3YQB7NXV1a4vXjUpqB8kai4aQpuf+iZ978d\/t9bdj+IieKxJhrQsu5sOvF6+yoJm4tWQcGXmzx5rgY8gwG4iRTIRDMLKKb8LFixwlJ9hV39MVyLY5vXfEBGgcd\/+c4y6PHdFjWQiGE9OW1paEhNWpy5i2DOZCAbgUaz\/Hv1BFLlEMF3QoVqT\/lbQrkKU5sX139SasOJYuD8c2kT4EqFODm0GlcGeF1wDJhny5bOz69LY2JjUonPTTAoaXF1EPGFNFn\/nY373woe09Y0WEfu\/5KIhYgEMsAN8u2CVSi8EO9ckQ9qwM+h79uzp8UKCdIWNgqnq9FmA+bElFwpoAbRcYNEfe7DrLXqqu4Pfrru6Ukx2ZQuP0CYveCWTDXOGXC6prnekHXadBaRkPntUUnx5wUkF\/rElVcJ1STaRdVr1RZvllV95MKgrwjquENIjcEfJlgIdQK+pAJ922BFiHD16tGsf8JZ46gEmBTUFAID\/4dUj6Xcv7BcWni1+Kh1iSrZsrAcLfJj3zPlZbaDmmWRI240JIqlJQYNc3+0cWBx0AltbQG9zaUxq+HRdMC64cwZd4DPJkC\/YVQvvZtGjNkF16kb2xZF1abMkwwGda00l\/JsR2NXVUj+LSlHx2cPtUlu7mwZSse5ph93tZQQYAMXFxa5hSJOCWpTirYGg1t0kQ77cGFXdFvZ4A5hO6YMm50UCdvXlBMlCj5lIBEtnR9pr6WnAy7pHLhFM3tzUa4Ep04lgel1gj0qXBrx890gmgkE5\/JjewIEDaeHChbRjx45eOotCIli6OtJeR08DyeLukUsEk5vk9V4lk\/6WnirtUVHXAHx3FN1wr0mGUpqgWtijjlb85Us77G7PoLL\/Hqd0gfh3f261IO2ws38ONc+bN4\/27dtHOnntJgXNrS62rWUNmGTIlxujpgsE3TfGdqXVgK4GMga7roBhjEq\/17bHZ4cGIgG7154xULVJQZ26DqGqSZMm0ZYtW8R+kqZL3OuHPuLeBpMM+XJjGCb2148fP+4aY08H7CYV4TRQ4l6\/7YOeveobdnnPx7a2Ngt7CreTsAeThT1F2OG+jBo1ihoaGmjs2LEWdgu72BEurDRukwbBl2WX4+0QoqqqSgt2NREsBT56nAp\/FKkKtn53jWaLjkwMJm3YeSff2tpawg5gOhNUVjQGhi1WA0E1sHPnTlI32gpSlzbsau66DuwcDQD0tlgNBNUAIm0mom09YHd7NST882nTpvXYrVcX9qANtOdZDZjWgJZl99pKw2sl1bTQtj6rgSAa0ILdqWJr2YOo256TSQ1Y2DOpfXvttGrAwp5WdduLZVIDgWHPpND22lYDQTRgYQ+iNXtOLDVgYY9lt1mhg2jAwh5Ea\/acWGogFNjVt2F7ZUe6aU6O7yOdGAlHGzZsSKpoeV8bHOh17SDXkAVQ0yhU4YLU7\/Q2cbfnfIPUzzIXFRUJcb105EU2ZECZMWOG16Guv3OCIT\/2mawiv33MdRmHnTuqtbU10XivV0g6NcwpPSHZO1dRh9MuZagHMjkNlCDXcIPZaWEtSP1ObxN3230tlfqxKs5wBukf1gPL5rVhlg68Oq8v8tvH8nWNw+70ImAv66cqwmnXAqdBJJ\/n9rvbtYNcw01OfK\/CHrR+p8U6p7alUr9qNLy2RHECVb37BIVdvjN5we63j1W5\/w\/yjEF9WHq0ZQAAAABJRU5ErkJggg==","height":99,"width":164}}
%---
%[output:96a38533]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAALsAAABwCAYAAABGvwEdAAAAAXNSR0IArs4c6QAAFAVJREFUeF7tnQtwFtUVx08gQAjhYXgHQqQSGUB0NPKIT3B0KnSGAgMq1BmY0SoMwjgdVEpRqkVQoOCAEQo4g+OUykOpaKmltIrKIwgBWhXCQyRPRRDMk1dI53\/D2d5sdr99fHeT73F3hvnI99179+7\/\/vbu2XPPnk1ITU2tfeWVV+jee+8lbCUlJTRz5kw6efKk+BvbkCFDaN68eZSSkmJ8x\/+xKm8u9NBDD9G0adOoZcuWjvWnT59OEydOpEuXLlFOTg5t2LChQR35i1Btc7kdO3bQrFmzjGqh6uTn59PkyZNFWbncunXraPny5Q30kNteu3Yt9e3bt0F\/5WORtTT3K5TO5rGx0inUd1Yi2vXLXNY8xjIvVu26YYL7aje4Vm3Y6Ys25PHB31Z9TADsoQZA7ox5ZzIYIYkkot69e9PixYspLS3NKGpV3yvs3JjVwYUS3ao\/oU4KN7CjL+ZBrKiooDlz5lBubm7IE4WPw3wiok\/FxcUNJgC3sNtNVuZ+8f7djrH5ONHPbt26iZPdDezYn9UYWIErs2Xeb6hJ0aylgN0JVP27ViAWFDBgl89ou7PF6myWzYNYEEQfQ+wqIGAH6G3btjVsdZgE2dnZ9WxmgD5u3DjjO75E7N69u549HLtS6SOLdgUS5s6dW4sbQtkmZVuqvLxc3KyxTQ\/bkW\/e+CYgKyurnk0a7YLo\/seuAgmrVq2qtQIWszt\/n5GRIbwpmzZtMjwSkIRnd\/P3sSuXPrJoViBhy5YttbIJI3s32JTp2rVrPRPG7DkIZcp0796d8E9vWgG\/CpSWlhL+hbvZwi7b6Haws3mzf\/9+S7sdkMPtdtttt4XbT10\/jhXIy8sT6zzhAh8o7IAcC0MqOmo11rfeeis9\/vjjuv0QJ0K0aNSvXz967LHHaOfOnbR582bjiLj\/MKMBfThboGYMw66io1YHqdt3HnrVGpnNUr56r1mzhg4cOODcIZsSMuxvvPGGMYur7H+gN6gqO6ph98eRyjFoLLNUNltU9l+4HmX\/OSS1cz2abXPZY8PL4fKQqOyohr3pYVdplvJM3rp1a3FgJ06cEKEkx48fp4EDB9Lly5fpm2++oWXLlglTePv27XT\/\/fc3iIHxokpCVlZWLWJW2rVrZ\/jLrRaVOPaE\/fFuFpWChh0zzciRI2nr1q1h37xYiRbt7eOYVB6DPJ4HT5RS8\/b\/j3NyA13NTyV09afSBpOpHCPDUJs\/q6urae\/evWEtYAYaLhA07G4E1mXUKSCP55E2WZR89xOeGq\/6bBVVfbZaBMuNGjWq3mLkM888Q2PGjKGCggLidZ0ZM2aIkxUT8ZkzZwh\/y9G4nnZORIEGgmnYvQ5HZJdXNbPDSsjMzKwXSj5p0iR68skn68HeokULWrBgAcHU2bhxIy1ZsiQsgTTsYckXX5VVTV5uYe\/Tpw899dRTBOhh1jz\/\/PNhCa5hD0u++KqsCnaYMWPHjqXa2loxa2ND3BVuUNmMuXLlClVWVoob1U6dOolPfLdy5UrHB3rsRkXAbvfkiflhBq8hvqrEaQykBqTeQT9UF9Lp6sLG2F1U7kMez9PHDlF6SnNPx1FYUUNF5TV011130fz586mmpoaGDx8u\/gaDzZs3p7KyMmGj4zds69evpwkTJoi\/mzVrRs899xx9\/vnnjvtFJO93331X74bWMsTXqiU\/Ib7RBPtLQzYL2Jf\/Z4ajkPFaQB7PeymffpPV8DHNUNos2V9BS\/MqxQ3q6NGj6cKFC5SamiqqnD17ljp27GjAfvXqVTHzr1q1iqZMmSJmdZwMcH\/jRjXUxs9mmCdrw\/V47NgxW7eO3xBfDXtsnRaqZnY3Nvvp06eF+bJt2zZ68MEHBewwa6ZOnWrrkWFO8awzHkAyrwsljBgxohaxK1u2bKkXvisPk10or1OIr4Y9dmEPJ04FsGPhKCkpiZKTk4VIWECCv51tdlYOMzw2wAsTp6qqiq6\/\/nrxndWD4HgO9sUXXxTPO5sncGMFFY3J2QPkhznMJgx3xGlhiWE3x02oCtlUiZI2Y5zVVDV5IRIWszU4GD9+vIB8xYoV1L59ezp16pTws3\/77bfCxw648T1A79ChA7FpYl7ll3vPvzWAHQ9vII2GGW48vcQN28HuFOLL4phlBPxvvvmms7qNWELD7iy2DPuZ749QelrdrOx2KyypooLiKrGYNGLECDEzM+xgArM836BiRk9ISBBN4xPQt2rVihYtWkTvv\/+++N5sDpmzCWDVFZGUvBBl63qEkd+jRw\/RscGDB1s+vOEWdnOIr57Z3eIRWeVk2O\/PrqbnpvXz1MFXcw7TwpzDAlLExmDr0qWL+ITnBCYIw45HQuGWvHjxovhE6g98zp492\/DGyLAPGjTIeJruww8\/FGYMvDpoj\/Mg2cIux8f4fVJJ1WXPk6I+C+uZ3Vk4VTO7mxvU8+fPC7MaN6V4kg7QtmnTxhZ2rL7yqiyOBLDjaoATaPXq1cI37wp2VPbzDKqG3RmgaCqhajzdLCqZdQH8gB1eGlgc2GCm4Arw9NNP1\/PQsM3eAHY8g8rmihymK6fXwKUGpkikhfiqBEXP7M5qyrAXHf6BOrdOd64kleBFO6tFJSwyISyAb1CxYgq\/+pEjR6h\/\/\/4E2HGDum\/fPuGnB9C470tMTCTE1cgBYgw72oPpwxnZEqZMmVKLWRuFOU2GOZyXbwbkG1knTwzqqJoJPCnqs7CG3Vk4eTwzy++jhzOfca4klVh\/bBGtP7bYclEJ4QKYdBl2xLfDxYhVU9ygAnbY4AsXLjRuUOEuxwkg2\/HYHWDHYhTMINnx0iDXIwqrygimYffEQsQXVjWzW9ns7OzYs2ePeEjj0KFD1LlzZzF7IwDs4MGD4mZWTrpr1Q5E5LCWBiuoQeZ61LBHPL+eOmi3buKpESJ64oknxCwO3zq8MNj4ySVAPnToUN+w24GOfeiox2sjpc0YZ2Qb8xlU2Ozp6emeZnYG3Zy+mo9Mw65hd6ZcKqEi6RXcgYhxgTcFpgj\/jVgYxK0juwCei8bTTHCUIKbdyYyR\/eycR998YPqxPA27J9hVFTbnZof5csMNNxgxWuYXMzjBPnfuXMsXQaC\/PNMHmsVX2+yq0IjfdlQyFGgWX5UdDXq4tc0etML+2lfJUFQnSfInn3UtDbtKNdW1pRT2ILP46hBfdYMeDy1Z3fyyB0hFCsVGSWxqHigd4hsP6Ho\/RoTjIlGt1RY1sOsQX+8DH481rGZ2ncU3ABK0zR6AqAqaVGqzB\/maGZUdVaBbyCY07EEr7K99lQxFdRZff\/Jpb4xK3YJuSyns0ZzFV6XQmNmxvZA7RmWzuq0wFVAKO0c9BvHSX5UdDVMzx+oadkeJmqSASoZ0INi1IdSwNwnLjjvVsDtK5L2Aht27Zo1RQ8MegMorh++j01WF2mYPQNtwmlQOu87iS6RhDwfJ4Ooqh13OJGD3Go9Yz+KrYQ8O2HBaVgo7ux7jPYuvhj0cJIOrqxR2ncW3bqA07MEBG07LSmHn96DGexZfDXs4SKqpG3iIr87iq2d2NaiG30rgIb52eWPiLYuvntnDhzXcFgIP8bWDPd6y+GrYw0U1mPpKbXY3sOMwoiWLb5fW6SLh5lc\/7vKkvobdk1yNVlgp7LGWxddvXLqGvdH49bQjpbDHWhZfwH5Txzto7NaunkTVsHuSq9EKK4UdZow5+1I0Z\/Fl2Kd8fLunF\/gC9qTrztLk937eaAOpd+SsgHLYnXfpr4TKjrrtgV\/Yt\/7qa3FyaNjdKu2uHO6hwnlruEqGYi6enWHHm6o\/LlpvOSIYAGw8CL16JNPKYfsoKfUs3bf8bnejqEs5KvBw5kwa3vMRemHPGN\/Aa9hDyMywA3S7V7MP7\/kwTb95mWHXPzutHw07\/66G3RFfbwWg8YCOd4hKyw\/N8OwhQ70mg53zX\/Mhm99sYJZCZUfdygzYuySnE2Zvu5tUhv3pQ71Fs6+\/nEWJ\/\/yTht2tyC7LAXZ+7xKcBvyaGZfVRTGVDLk2Y6IlxJddjwDaTlyG\/eWj94ib0i1v3UO5L8\/TsHuh0EVZhh0PsUNzfgeTl1m+0WFnbw1e8sQvGcOxYpU1KyvLeBtZpMzsdW9lKxD24sdF79CXZ3fVu4TCloTwgH3unC6U3qMNla5dImBfXfgo\/esfV8XvuARjYHClsLP\/XYx53BaRYYcIuNpC138XvePapGl02PnNeJs2bSL5rQZ23\/PoquyoW2LkRSV5NkF9Bh+XVIj+duUIWrPiesKbl3\/29dt0403VVJE1m0reWip299XZXeIyjPK4mcWVQkPvdiRI3BdBv3DSk6hkyJUZYzZh+HCdXg9pzuJ78dskQynMvryF45oyS2+3gsqzuVweYP9iwjn6ZFkeJRVvplapZyk5o4LOHcig7cV3UXrb5qI4ZqQ+7cdTSflU8Tf6u6t0CbVMLKn7u6qQWiWWUM+UZkbze0ovU8+2zamovIYKK2ooPaWuLfP\/8Ts2lMXGf7tHKnJLeoU98BBfN2\/Ls4OdzRvzy4Dlmf2Pj6ygsoM9qeZ8orCPL5zrSK2al9DFmjTjE\/D0SksWEAGsgpIq3yOI+sd+2kgtEmcbsDF0Zmi73b6bkm55jd5++Qvq236HABL9umXqUjr4t3fo1OEfqfAajAB\/8LDBVHZpkOjj6fxhoo84ng43HKWjX7YWx4CttuyigLpPr\/N08ceOvo4F9bGhT\/x\/Xw01YSXoBBOSZ\/bs7i3FmGw8Wm3ZqyYL8ZV7Ew7sr01ZTh+s20YjRpZSs3ZJYvasLbtAl67UvZIbQFWfakPJA69Q1X8TKT2tDRVdA+X8iUwqKr96beZrZsy0BcVV1PPaTImyYma89nfigC6U3n8dfbPpCG04Wi3q7C69TNndWxjgZnYYT92SXhUnH+z2nV+cEU8q8QLIlQeepAmjMwj7KSyuFO3fObiz+MR3O7\/4QXyifGFxFR39Mom6tO5lSFZQUil+w5Z03Y904Vyq8amKPb4ScHu4IgAm3vgKwie60Oja1UPWzqwj1+fv5f5Ca6u6Rh+k3\/EdxqLX0Hfp4sYvxRUL+99deomW7q8Un+atyUJ83cDu1oxRkVvbLIx5sPE7BM17dxjtyi+np2bvt+QKC0gDUu+gX6f\/WfwOXzxs85eGbhaAsgcHvneAfOfgTqLczr1nqKC4UpwYsbrJphT\/n68sbIbJJw0DzNqjDky5ooqr4vPR6TdTQrtWtOC3uYY559VMa3SbPZpuULesvVuYQYD99flZtDDnMN05qG5WnjC6l\/C8AOrc+fPEJRYbAP\/D0Lpcj15jamIVfBXHBf1h2o2a\/Jnv5hoddjvbPBJdj4AdG6BmEwTww+zA9s5fTwkPwe9u\/FR4VuCxweyOmylsXqMlfY9iHFSMStgxLpxIid8p6WTCoI7Ks9ItG4CdQbebUTCzw0Z\/fs8YMaNjZucFDw27W6WdywF2XFX5Hmfn3h88m4EqGXLleuTDioZwAcCOm8lRkz61FVaGnSFnXzrMGL2pUeDOQZ1owpgMYcrIN\/i\/nPypcaV12lOTwe7UMfPvKjvqdt8Me8f+79lWkWHH6iibMHB9atjdKu29HJwDj4zOIC8zvEqGPM3sXg9PZUfd7pvNmFsf+EjD7la0CC6nkqGYgx2zR3packjbUJ7Z8WC27GPXM3tkka8c9njM4ovwAXhjYLpwKIE2YyILdNVODjGzx3sWX57pj9d+QM\/+3fqls5GHQXz0SOnMrrP41kHz3sjvaXfVG7Tokxfjg6IoOUqlsOssvhr2SOZeKew6i6+GPVJgDzzEV2fx1bBHCuxNFuIbj1l8T9R+oG32JiS\/yUJ84zGLr4a9CUm32bVSmz3Wsvj6HS4sLGnY\/aoXXD2lsMdaFl+\/smvY\/SoXbD2lsMdaFl+\/0mvY\/SoXbD2lsMdaFl+\/0mvY\/SoXbD3lsAfVXZUdDaqP3K6GPWiF\/bWvkqGYi3r0J2nde1D1Dapf9YKrp2EPQNuPFvye9h3Ppzlv\/iWA1nWTfhUIBHYsIvXt21f0KVrevIFFiJEjR9LWrVuptLTUr56inpyVgBtS2b5V54JuH\/sMeh9Bt68cdnOIr7ygtGHDBjFOkZjFV6UQeOiDMxAwmCrbt4I96Paxz6D3EU3tJyAQbOLEicRZAyBQ7969afHixVReXi6y9kZqFt9oElrD7u\/Cq3KMExAIZpV2Ws4Jk5GREZHvQVUpRFPAGHT\/9cxef1QTsILatm1bmjlzJp08edL4VWVszJo1a+jAgQP+Tu0QtWAvzpkzh3T79iLFikYqUijawi7b6F27dqVx48ZRTk4OsQ0PeZ2y+LLQmMH0phXwq0BeXh7NmzcvbCdEoLCzNwDQ600r4FcBeNrC9bZh34GaMX4PTtfTCgShQKA3qEF0WLepFfCrgHA9mu1xO9ej+Q0bTll8\/XZK19MKBKFAAqfSaNeunfHWO6tFJT9ZfIPosG5TK+BXASMQLIhwAb+d0vW0AkEoEGjUYxAd1m1qBfwqoGH3q5yuF3UKaNijbsh0h\/0qoGH3q5yuF3UKBAI7uy7T0tKEIBUVFYanx4tCbm6aze2ZX4XjtG8\/+5D36RQy4ad9s37YnxyVKu\/fT\/vc55SUlLDGh\/uBPmBDhKzfDd6+zMzMBjFaVu15HWNuQznsZh89diRnF8vNzXWlh5sYe3NDZvco7xt9Msf18G9yEJyVy9Wpswzbjh07aNasWfWK+zkGBrGsrMwYeKvj8tt\/q3BtP+PDB8p9y8\/P9w07w1tSUuIIu9cxlgdEOexWwDjNfnazc6gYe3Mdq5MMZez2zQJ72Ueoq4gZdr\/tWy3UWR1bOO1nZ2fXO\/ndvPnQTm++evuFXb4yOcHudYzNff4f0XSLhsWCoYkAAAAASUVORK5CYII=","height":99,"width":164}}
%---
%[output:5158e8f4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM0AAAB8CAYAAADKKjS5AAAAAXNSR0IArs4c6QAAD4FJREFUeF7tXV9oVskVP+qquzYxbVDXhLrWhyilrFRT9cVFYQsWjCl2gw92RUHjLgRLH0RURBsR8SFIQYJ\/omBAfRBBifFBSkXxQWJJsroProRFtPSLa901ISHsJv4pZ+Kk97t+934z956Zb+6950Lw+\/zmzj3zm\/Obc+bM3DOTKisr3wBfjAAjoIzAJCaNMlZckBEQCDBpWBEYAU0EmDSagHFxRoBJwzrACGgiwKTRBIyLMwJMGtYBRkATASaNJmBcnBGwSpqqqirAP77sI\/DdCMDzadUw9aNa8fApFVXwarBffJ41loP\/POiBj2elc8muv78f8I\/qskYaJMu+fftg6dKlVLJzPUUQeDYC8PX3k+Cf\/54s\/sVrzow38OEH+Td6f\/v9L9\/Ap\/New5wZ6YG3p6cHDh06REYca6RBsrS2tpIKn55uBWGB586dC729vSTNmv5xHQzW1MGzkUnCoozcPgWvB\/oDrQlaoheza2HGJ9vHrc9oDsq6T8Hrt9aIRKgYlUTFZ8mSJbBt2zZoamoCJA\/FZZ00lMJTAJC2OiZXVEF53QGYOr8WfrzfCT\/d74SxJ91azZy+uE6QB104JNvI7Tat+10qLAdrSr0LJc2RI0dg1apVAoNbt27B7t27A\/HYsWMHbNy4Ufw+PDwsXLGurq6J8iaEd6lzXJAF5ysVn58QlmX4arM2WfxtmPFJoyAP1jd47gtnrI4O1ib0LpA0GzZsgMbGRmhrGx9l5OeLFy++IzOWRSZfunQJjh07Bki2mpoa2LlzJzx69EiUNyG8DnhpLysVnNoySMuF+A11NieOOCb0LpA0fsU\/e\/YsPH36tKC1wbLoj2\/ZsiVQN00In3YiqLQPlfr9xXUiKoZumClXqqzugHjO4LkvY1swlXZRlTGhd4GkQZLgJYng\/y4btWDBAmhpaYG+vr5Q982E8FTAmqzHdJh96kdLhQuFFmbsCc1ENwgPtGZIThctTlBY2YTehZLGa1mCrIkkzf3792HlypVQVlYWOqfB0N+1a9dM6qkzdXOY3V5XBIWVnSbNzJkzJyb\/aJXKy8sLzmlOnz4NZ86csYdmCZ\/EYXY74IeFlbdu3Wov5BzHPfMGEWTgwATj7XRJ9Kdksc3R0Yp+ZxjOa9euFYO5lZCz3x0LCwT4f0PSbN68GQ4ePDgRds6iAmWxzdFVP\/qdYTib6AOSkDOu0dTX1+e5Z94gQlZDziY6LLpqpfdOZ0iDEActbsrJ\/82bN8W6DF7exc1cLpc3n2HS0G3hSILqS10YHR0VW6cKre2FtWPFihWwf\/9+aG9vV7rXKdJQdlAWR90stlkOtvhv2A4SJo0Cu7KoQFlss9c7efjwoVjnwznvokWLhJZ4t2NJj6W6ulr8duHCBejs7BTrfvh\/qpaKLY0CAZNSJIukkZZG7hBBEtXW1or57vz58\/O2W3mXJurq6qChoUG4c48fP2b3jOc0+XMa3PoypWJ8dE3y9WowV3Avmjfy6o+syqWM5uZmYVHkvNi7s+Ty5ctMGiZNPmnk5sokEwZlD9ocKkkjieHdYiV\/O378uHi3qqOjYyKYJAnGpHmrGVl0VYLazJYGQNXS3LlzB4aGhmBgYADmzZs38dk74PCcJunDr0f+LA4U1HOayspKmDNnDhw+fBiOHj0KV65cgRs3buRpCZOGSZN4BArtJlGNnuG6n5zfzJo1S7yjtXDhQrh+\/bp4IXLPnj3w8uVLJk3itSSgAVm1NNT9iSSsqKiAe\/fuwYkTJ96pni0NNeIlrI9JQwM+btHatGmTiKg9ePCASUMDq5u1MGlo+uWzzz4D3P6Paz2FLrY0NDg7UQuTJn43rF69GtatWwfnz58PTMHEpImPszM1MGnsdAWTxg7OVp4iOxPfVqVKAmhF8IQ9RL5WXuhFMxMDFycLNKggnCPAILi+qp3IEUDdXBOMp5bRRH0q2WhkRhkXs7xQYWK6jU5ko6ECS9aTVdKo4Fjx5xMw+efV8KK1XqV4Ysv8oqkDxh53w3Bns7U2mNA7ds+sdV\/hB+E+tIrPT4rNj5h3Oc0X5ojGPNPPDy+z1kwmjTWo7T0oK1ZGImrb2jBp7OmylSehlals6hAZK9NuZSSg8rWIH1rrreSFtk4anVMDJCiFcp7hbyaEt6LZBh+C+ZHxSIy0z2X8EM7a+y9xDIiNuY0JvSNJ4SRB8e5O9WchMSG8QX02XrW0MraUx3iDNB4gk6nbmNuY0DuSUwMkXjL\/GX7HIzq8qXtMCK\/RT84VlZPipGXhpwISrY0Nt9SE3sU+NUCCiLmqdu3aJd75xl2pTJpw9cIAAF6D57+k0sNE1WPL2lgnjcqpAbKncP6DF67MFjoASgqfpVMDgrRYnliWVSuDuNgKgjhLGpz8r1+\/Hvbu3QvLli0LJU2WTg0IIk1WAwB+PGxYWydPDUAgMItId3e3yCrC0bNwL0mOsNTH\/CXKN3srrI15nZOnBuBcBl0uPMzJf2HGRJnr2YSZZEVJIgL5Mpte7DShd6QhZ4SDLQ0HAHSobDogYJU02HCdUwMkUEyaYJWxNfnVUVoXypoMP1snDSWgJoSnlM9GXbiFZPridZnbAVAMW5P770zoHe9yLtajhL\/biBYRimutKpMBASaNtW6kfxCvzYRjaiogwKSh12VrNbJrFg613P1MvR+NSWNNxekfhK7Zq8F+Kzt76aW3U6OJgACTxk7fkT+FXTM1SE3M+Zg0atg7V4pdM7UukQEByhfUmDRq2DtXChfwplRUZXZHs06HUL+gxqTRQd+RsnJBM8s7mnW6gnqHAJNGB31HyrJrptcRcv5H9YJa4knz459OwOuBfnjS1Qkjt9v00ExoaROT24RCoSw25ZpN4kmz8i\/Hof0fvSKZBF4Ygk1zvi9+DUCZJ3kFKfOjJZ40mGwDk1R\/9W0\/vL+4TuzDwgmyWL+42gxjT7qjoezoXTyfid4xVGs2qSENvhItL1QszLqI1idtmVlMbkSMro7JuJNqMTiVpJFdKCeAabI6lL55MlSdTkqqNZtUkwbh9lqdpIdo2TWLTyAKFy31pJEwy817VGHH+N2nXwOHmvUx899BsSicGdIgeNI8J5U4HGqOTxoKFy1TpEHIqRe64nejWg0calbDSaVUXBctc6TxWpwkzXF4V7MKHdTKxHXRrJNG9dQAfxqnXC4HO3fuhEePHk0gE0d4OcdJCnF4PqNGCJVScV20OHoXJF\/sFE7ypIC+vj7YvXs3yO9DQ0OwZcsWEtJgJXLNY\/DcF1bONVHp0KAyPJ+Jg96798Zx0aySBq1MTU3NhMXALJre3M5hsPjvxbIUwidBGXk+Q0sYrC2Oi0ahd\/4WkZ0a4K3YFGmSkDeM5zP0pInjolknjc6pARIqOb\/B3M7orslLCh\/31ADXlZLnM\/SkwRqjumjOk0bOZ7CRQYEAilMD0E3DvWrUmUsoujsJLiRFO23XEdVFc\/bUAAQwjDBUcxpvR+G+rtcDOedeIcYRkU8EoKdUVBfNyVMDvITxR8y80FGbSRfdNN5vRk8Wb41RXDRqvUN5YoecsRKMrJWXl7\/jkpkkjQxDu+SmUb48ZVb9kll7lNcFrJIGYVU5NeDu3bsFz6cZHh6Gffv2QVdXl+ghE8LLCaIr7+HwCWdmyRjFRTOhd4lPgG4yebauCnAQQBcx\/fK6LhqTJgBjF5Q1CWtI+irq3h26L\/YxaQL60IXd0C4GJtxT+fgS6SZKZ9KEYE6dZE63e3lRUxexaOVxcCpbd0A5ixGTJgTnUu\/5csFFjKaGybtLx0Vj0hTpX2m6KRNoq6hUqQmrImOayuh4FUwahZ6nTqCt8MiJN0yT8r6PSptcLqMTMWXSKPRklFi+QrWhRXg+ExdB\/ftVB0cmjSK2Oj6vYpWhxXg+Q4GiXh2qC8lMGkVcbVsb3qSp2DGExVT7mEmjAbota8ObNDU6hbioyu4AJo0G6KojkUaVBYvyfCYugtHvVxkYmTSa+KqAqlnlO8U5yXlcBKPfrxJ6ZtJo4mtj3QaJ+dP9q5k5pEqzC4wWVwk9M2kidIFqaDJC1SJhe2VTB\/D6TBT0aO4p5k0waSLgrGLCI1QrbtHdPBj1OXxfMALF+pdJE1F7VKIsUarm9ZkoqNHeUyzgw6SJiHex0ShKtbzfLApqZu4JGxSZNBExN\/G+Db8\/E7EzDNwWNq9h0sQAnNqVUt3GEUNkvlURgTBPwmnSFDthwITwipiKYirhSdX6pGvmSkIPVbnTWi6sb03oHUlijQ0bNkBjYyO0tbWJfpGfL168ONFPJoTXVYJi4UnV+igJqPpMLheOQNDSggm9IyGNygkDJoTXVSSqxU52zXSRN18e+wSv4c7mvIeZ0DsS0mCyQLzkeTT+7\/ibFB5zOff29ppHMeAJT\/5wEsq6T0Hlf7sjy\/Dtr7dDVVUVzLiR30GRK3x745IlS8RxJv39\/XGrSuX9Yfj8MLsWhmu3w6\/u\/C3v\/CLsJ8y\/19TUBD09PSS4kJGm2AkDUngkTymvv381Gb7+fhJ8+EGwFHNmvBE\/PhuZVLAQ3v\/X376CT+eNl+PLDQTWXX0PsO8K9e03LX8kG4yskQZhReLgXymv70YAXsyuhckV1XliTKn4v1yvBsdH+teDORh73COCCN5r7Ek3\/Oa9XCmbwc8ugAAGaL752e8KYuN32+IASEYaFCLMPYsjJN\/LCLiEAAlpMBAwd+7cPNKoHjXoEhgsCyOgggAJaVRCzirCcBlGIAkIkJAGG1pscTMJYLCMjIAKAmSkCXsYhqAXLVokily4cAGOHTumIlvqyqBFxtDntGnTIJfLhZ7n4x2EEAj\/0SWpA0ehQfK0vZs3b5ZUh4yTZseOHVBfXy9i5cuXL5\/4LM+tUcAqFUVkh\/f19cHJkyehpaUF8LP3MF9vQ3WOoE8FQEUaIfGrrq4u+cBrnDTehU5XRopSKJl33ofbi3AwWb16dUFrk2WcCvWNtNDPnz+HmTNnQkdHR3otjXd0xRHV\/70UyluqZ3otLlpZ\/3evXHis\/P79+6G9vR28+\/dKJXupn7tmzRohwsDAgDh1LxOk8fqgWXU7\/JYFR8\/NmzfDwYMHJ45YlMrpnfvI\/8vyXFBigIMJk6bUQ5jF5+uQBss2NDRAa2ursDT+7xbFdupRmSKNnPCyezYeECnmnvk1Ncu4+d3W1FsabLDXHcvyBNfvjoUFAoJIU+pQa6nNTiYsDYLMIedxVdMJOfvfT8LvtbW1eUfMl1qBS\/H8zJBGWhte3AQIW9z079\/zLm6Ojo5OzG9KoayuPDNTpHEFdJaDEaBAwPjiJoWQXAcj4BIC\/wNaUCsKk5zhrgAAAABJRU5ErkJggg==","height":99,"width":164}}
%---
%[output:06358434]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM0AAAB7CAYAAADXLwQBAAAAAXNSR0IArs4c6QAADftJREFUeF7tnUtoFVkax7\/4SDeNJo7OwmTANgNqb8ZBJcmq0VkFYjrMIrhwBiKoY7chLiRICCFiJjOIEzeGjG0bwUCbRXCViWE2SsRBiK+AvZIgQcFEMDOTizF0x0cP37lzrpVK3XtPVZ1zqu6tf4GYe+95\/r\/zq++8qk7Jxo0bfyZcUAAKKCtQAmiUtUJAKCAUADRoCFDApwKAxqdgCA4FlKE5cOAAtbS0UGlpqVDtyZMndOjQoYyCZ8+epb1794rPt2\/fpvb2dqgLBYpSASVoamtrqaenh0ZGRqivr4\/k54cPHwo4GKijR4\/S5cuXhUjy7+Hh4aIUDZVKtgJK0HhJdPXqVfE1exv2Mtu2baO2tjaanp4m\/u3ly5fwNsluW0Vbey3QOAFipdyfpXoVFRXE\/3BBAZsKzM7OEv\/TdQWCRo5vrl+\/Lrprbs\/Cnmfz5s3LxjwMS2dnJ+3evVtX2ZEOFFBS4NGjR2J4oQsc39DI8cyLFy8yUKhAw7D09\/drLbySYgUSaNeuXXTkyBHok8VeQfWR8XgSi+HRcfmCxgsYr+6YV\/dMQqOz8DoEiEsa7Inr6+tpbGxM2x0xLnXTUY6g+phod8rQuGfMnEK4u2NeEwEmCq\/DGEijuBUw0e6UoKmqqqLe3l56\/fr1snGKlFtlytlE4Yvb3KidDgVMtDslaNwLm7IyMzMzmWnmfIubJgqvQ9QgaZT\/4Vt6n5qlhdEzQaIjjkUFTLQ7JWh01NFE4XWUK0gaXtBgOj2IkvriZJtWNtHuAE0Au7mhwXR6ABE1R8k2rQxoNAsdNDk3NJhOD6qknni5ppUBjR6NQ6eSDRpMp4eWNlACucAANIEk1R8J0OjXNEyKgCaMepbiAhpLQitmA2gUhYoyGEPDV+ra1+J\/E12AKOsXJG\/n4yOjo6NiXa+ysnLFc1eqaXvtX8wWF9CoqhphOECzUnwnNPfu3aNTp07RuXPnaGJiIpClAE2R3Y0BTZoDuVOEPQovdJeVldH9+\/epurqa1q1bR0tLS2KTLn8nPQ\/HGxoaErvj+XIunC8sLIid8DU1NXTw4EHxu\/sJYS8C4WkC3ZfsRgI0ab15j+H69evFrpCGhgbR0BkI9jRdXV00ODhI\/PSuO1xTU5OA6dmzZ2JXt3wC2Bnu2LFjKx4vQffMbjvXmhug+ehlxsfHhdeQXoc\/O6GRXsYdbmpqSmzVb25upu7ubtGNc+5hZO\/hfiYL0GhtxnYTU4FmVXkFrS6vtFswA7m9T83Qh9TKpx7d743IBo30JvL9EtJD8ePw\/ExWY2Oj6JIxNJym9FCAJoFjms++PEqfffknA83YbpKLd76jxTvpF6Y4LyckJj0NP5i3ZcsWev78uche\/v3u3btMcTCmsdsmAuUGT5OWzflCFVNjGobm\/PnzdPPmTXr16pUYN7FnSqVSgCZQ640okgo0ERXNarbu2TOeLbt79+6KiQBnOC5gvtkz7qq1trYKQHj27NatW7R161Z68+YNLS4u0sWLF5fVE57GqtmDZQZogukWNNb27dvp5MmTVFJSIt6t9+DBA0ATVMyo4gEa+8rzDNuGDRvoxIkTKzKHp7FvD985AhrfkoWKsGbNGuro6KCnT5\/StWvXAE0oNSOKDGjsCn\/8+HHatGkTXbhwYdkEgCwFPI1dewTKDdAEks1YJEBjTFp9CQMafVrqSAnQ6FDRcBrZoBkYGKDJyUnDuSN5twLyHQ1eT86aeGzD94s13CvBsgJJe4UT11s+T4MXa0QPcmxfrOFcpHIvUOU7n8YE8VGZyu1puBx4hVNU1kjnG8tXOMnnHubm5sRzE84NeCrn0xQ7NNE2GeSeTQET7U65e1ZXVyfKNT8\/v+xUNP5O5XwaE4WPqql4eZqoyoJ8cytgot0pQyOL5t4SLqFxnnzm9aiq891gN27cKGhbA5rCMV9RQMMzTFeuXCkc1T1KCmgKx3yHDx8W5\/7ofCedNk\/DMsrTnov9fBpAUzjQ7N+\/XzxKEDtoknY+DaApHGhi2z1L2vk0gAbQ\/OxHAq+JAI6f5MVNP\/ohrF0FYuFpglbZROGDliVsPHiasArai2+i3fmeCAhaXROFD1qWsPEYmlUbKum\/\/Y1hk0J8wwqYaHeAJoDRAE0A0SKKAmgiEt6dLaCJiSEUigFoFESyEQTQ2FBZTx6ARo+OoVMBNKEltJYAoLEmde6MAE1MDKFQDECjIJKNIIDGhsp68gA0enQMncovWkZEGphyDi2l8QQAjXGJ1TIANGo6xSEUoImDFYgI0MTEEArFADQKItkIAmhsqKwnD0CjR8fQqQCa0BJaSwDQWJM6d0ZuaPjUM3mA04fUjOchSDEpeuKKAWhiYnInNGu37KHyP35L71Oz9GF+htZ+vkf8\/dPjfwCeGNgL0MTACFwECU3q+2O0sWWEnEfssdf5dGeD8Dxvnz3MvFAwJkVPXDEATUxMLqFhz5LtEQH2QOu+Oi28z48\/jNJPj0djUvpkFQPQxMTeDM3q8gpRmtT3X9Pb5w89S+b2Oq9Hz3ielByTahVlMQCNZbPKRv\/j49FljV1Cw2MXlV0B0uswaNyVc6dnuVqJyg7QWDa3HOT\/p78xA438jouS7bjwbMXkY9I\/2fmV8FKYLLBjTEBjR+dMLl7QfLKzgdY3nBZhnDD5KRqnwWnzhAFfEqC3zx5l7er5SR9hPyoAaCy3BgmNc9zihGbur9WhSsTdP56idgPEkwc8TgJEoeQVkQFNeA19pWAaGndh5BiK82WY5AVP5MtsywLHGppifO+ZbWj8QsTh4Y1yAxVbaIr1DZtRQ5MNIv5eTijIMRG6dN7wxBaaYj3UiWe7eGXf1JgmeKcjHZO7c6vLK2nt57vFuMirS5d0bxRbaPwc6tTT00NxP59GNkaGhhuiExoJEjfGsBMBYaHxii\/HRfBGaXViDU2+Q51+9cVu+qb77\/R84gZ9d\/7PJtqLUppyR7Kc7uX9YbwVhi+5yq+UUEyh8erSJcUbOWc2ZbdV2nTLP48RH2ar49Lyhk32NPmg4Tt07e+P0g\/\/LhHl5hkhbrC8ld7mYFaOU3hhksvA14f5WVq1Ib0thi\/ukjkB4hV851YZuU4TR0+j0iic3kh266QWHJ93aNu0iUqZVcL8suO+2G0hbcU2\/O2vK6j6d\/vpXxe+iR80XKl8hzqd\/ls\/tXb8heZK0w3Uqx9uGiSvBUu3QfiO9elvGoQHkltfFu9czgSTxlkYPaNiy9iHyTc2Mm0TXQKxXXh\/n3NzbGy7Z2EPdZKLfNxA3Xc+3bNC0oWrrObLsO7tMmwcv1todDUMW+l4eSOZt1w3Sv8fn93bBQWN7ilnk3c+CYJq18oLkCRA4zU24u94LBhFD0HlZlFQ0HCFTC9uqtz5uBzObpSX0Dqg4V3OeDIzPeXNs4vOHoLbG6nYRAUIlTAFB02+SunuWzq90aryyszmRy6HfPTYa\/8WoMlnqXC\/y5ubH5uEy\/FjbEATQEmVO5\/sWoTpnsHTqBtHxSacmo5njACNul1yhvTq1rGBVGe+vMYvgCaccZw2kW\/xyddDUMkR0KioFCAMG+zD\/9dnVKIDGhWVwoXJN\/HD40e+8q0dAZpwdtAWG9Bok9JXQkG2AwEaXxKbC+zVFUP3zJze2VLO5414AZanwwtmcVNFQt2zZyp56gjjBQhvCcrXVdCRN9LIrYB7vMqh3W\/8MdHutOw9UzGuicKr5Bs2DLxKWAWjjW+i3QGaPDYFNNE2+rC5A5qwCgaID2gCiBajKIAmAmMAmghE15gloNEoJpJKhgKAJhl2Ri01KgBoNIqJpJKhAKBJhp1RS40KABqNYiKpZCgAaJJhZ9RSowKARqOYSCoZCgCaZNgZtdSoAKDRKCaSSoYCgCYZdkYtNSoAaDSKiaSSoUAsoKmqqqLe3l4aHx+nvr6+jPKmX+GUDBOjlroViBwaCUxlZSUNDQ1loNH9skDdwiG95CoQKTQMRktLC83NzVFZWRmNjIxkoCnW82mS29SKp+aRQlNXVyeUnJ+fJz5jxgmNn\/NpGDxdRx4Uj2mJKioqqL6+nsbGxmh2Nn2aAa6PCgTVJ1JoZPFra2s9ocl31IYs\/MDAAE1OTqI9uBTgRtHZ2UnQx7tpBNVHxtN5s\/b9uHNQaGThGR5cUMCmAtyz4d6RLg\/uCQ13t3bs2CHqNTMzQ21tbTQ9PS0+Z4OGf8t1Pg3\/zuDwP1xQwKYCDIsuYLjcWjyNyvk0NkVCXlDApAJaoFGZcjZZCaQNBWwqoAUaLnC+xU2blUJeUMCkAr6hCVIY5xjJuSgaJK1CjiPXukpLS1eMFd31ct6E+LeFhQUxuzYxMVHIEoQqe7bdKKESDRDZODStra3U2NgoDF5TU5P5O2nGlwafmpqiS5cuia1I\/Hd7e7un2dwnZgewbVFFybYbJYpKGofGufAZlztFFEI7x33Dw8PEN5N9+\/Ytm5mU5UqyTl62ybUbJQpbGoXGeXflO6r7cxQVjipPp8dlL+v+7CwXT+t3dXXR4OAgMWBJv3LtRolCGyvQOHdEJ7Xb4fYsfPdsbm6m7u7uFeMU59hHNookjwWlBl5rhIAmCgUs5ekHGg7b1NRE\/f39wtO4P1sqcuyySRQ0csCL7ll6QiRf98zdWpOsm7vb6t4sHAXZRrtnXCFndyzJA1x3dyzXREA2aNwP\/kXRYKLMMxGehgXGlHO6mfmZcnY\/n8Sf9+zZk\/h1msRAI72N3ACa5AFtrsVN9\/495+Lm0tJSZnwT5Z0+6rwTBU3UYiN\/KKBTAeNjGp2FRVpQIA4K\/A+7dB\/UnCRzugAAAABJRU5ErkJggg==","height":99,"width":164}}
%---
%[output:42ae0281]
%   data: {"dataType":"text","outputData":{"text":"Elapsed time is 3.608997 seconds.\n","truncated":false}}
%---
%[output:1ef3f307]
%   data: {"dataType":"image","outputData":{"height":99,"width":164}}
%---

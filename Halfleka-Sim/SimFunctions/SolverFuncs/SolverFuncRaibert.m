function out = SolverFuncRaibert(time, q)
    x = q(1);
    y = q(2);
    th1 = q(3);
    th2 = q(4);
    dx = q(5);
    dy = q(6);
    dth1 = q(7);
    dth2 = q(8);

    ph1 = ph1_calc(th1, th2);
    ph2 = ph2_calc(th1, th2);
    dph1 = dph1_calc(th1, th2, dth1, dth2);
    dph2 = dph2_calc(th1, th2, dth1, dth2);

    q = [x; y; th1; th2; ph1; ph2];
    dq = [dx; dy; dth1; dth2; dph1;dph2];


    % Foot dynamics
    foot = foot_Func(x, y, th1, th2, ph1, dx, dy, dth1, dth2, dph1);
    foot = reshape(foot, [4,1]);

    % persistent th_old diff_t dth_c controller_t t
    % if time == 0
    %     th_old = [th1; th2];
    %     diff_t = 0;
    %     dth_c = [0;0];
    %     t = [0;0;0;0;0;0;0];
    %     controller_t = 0;
    % end
    % if (time-diff_t)>1/40000
    %     dth_c = [(th1 - th_old(1))*40000; (th2 - th_old(2))*40000];
    %     th_old = [th1; th2];
    %     diff_t = time;
    % end
    % if (time - controller_t)>=1/1000
    %     controller_t = time;
    % t = Raibert(th1, th2, dth_c(1), dth_c(2), foot(2), time, dx, dy);
    % end
    t = Raibert(th1, th2, dth1, dth2, foot(2), time, dx, dy);
    t1 = t(1);
    t2 = t(2);
    u = [t1;t2];
    % u = [0;0];
    lambda_contact = lambda_contact_calc_func(q, dq, u);
    mu = 0.6;
    contact = 0;
    if foot(2) <=0 %&& norm(foot(3:4)) < 1e-3 % in contact but not moving
        if lambda_contact(4) >= 0
            lambda_contact(3) = max(min(lambda_contact(3), lambda_contact(4)*mu), -lambda_contact(4)*mu);
            ddq = ddq_contact_calc_func(q, dq, u, lambda_contact);
            contact = 1;
        else 
            ddq = ddq_flight_calc_func(q, dq, u, lambda_flight_calc_func(q, dq, u));  
        end
    else 
        ddq = ddq_flight_calc_func(q, dq, u, lambda_flight_calc_func(q, dq, u)); 
    end
    % out.dth_c = dth_c;
    out.ddq = ddq;
    out.foot = foot;
    out.controller = t;
    out.lambda = lambda_contact;
    out.u = u;
    out.contact = contact;

end


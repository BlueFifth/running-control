function dq_impact = dq_f_impact_func(q, dq)

    x = q(1); y = q(2); th1 = q(3); th2 = q(4); th3 = q(5); th4 = q(6);

    dth1 = dq(3); dth2 = dq(4);dth3 = dq(5); dth4 = dq(6);

    ph1 = ph1_calc(th1, th2);
    ph2 = ph2_calc(th1, th2);
    ph3 = ph1_calc(th3, th4);
    ph4 = ph2_calc(th3, th4);

    dph1 = dph1_calc(th1, th2, dth1, dth2);
    dph2 = dph2_calc(th1, th2, dth1, dth2);
    dph3 = dph1_calc(th3, th4, dth3, dth4);
    dph4 = dph2_calc(th3, th4, dth3, dth4);

    
    M = M_calc_func(x, y, th1, th2, th3, th4, ph1, ph2, ph3, ph4);
    
    H = H_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4);

    J_f = J_f_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4);

    K = [H;J_f];

    dq = [dq,dph1,dph2, dph3, dph4];

    lambda = -(K*(M\K'))\K*dq';
    dq_impact = dq' + M\K'*lambda;
    dq_impact = dq_impact(1:6);
end

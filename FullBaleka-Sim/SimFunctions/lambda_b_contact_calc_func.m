function lambda = lambda_b_contact_calc_func(q, dq,u)
    % Calculate lambda with only back foot in contact
    x = q(1); y = q(2); th1 = q(3); th2 = q(4); th3 = q(5); th4 = q(6);
    ph1 = q(7); ph2 = q(8); ph3 = q(9); ph4 = q(10);

    dx = dq(1); dy = dq(2); dth1 = dq(3); dth2 = dq(4);dth3 = dq(5); dth4 = dq(6);
    dph1 = dq(7); dph2 = dq(8); dph3 = dq(9); dph4 = dq(10);

    t1 = u(1);t2 = u(2); t3 = u(3); t4 = u(4);
    
    M = M_calc_func(x, y, th1, th2, th3, th4, ph1, ph2, ph3, ph4);
    C = C_calc_func(x, y, th1, th2, th3, th4, ph1, ph2, ph3, ph4, dx, dy, dth1, dth2, dth3, dth4, dph1, dph2, dph3, dph4);
    G = G_calc_func(x, y, th1, th2, th3, th4, ph1, ph2, ph3, ph4);
    Q = Q_t_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4, dth1, dth2, dth3, dth4, dph1, dph2, dph3, dph4, t1, t2, t3, t4);
    
    H = H_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4);
    dH = dH_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4, dth1, dth2, dth3, dth4, dph1, dph2, dph3, dph4);

    J_b = J_b_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4);
    dJ_b =dJ_b_calc_func(th1, th2, th3, th4, ph1, ph2, ph3, ph4, dth1, dth2, dth3, dth4, dph1, dph2, dph3, dph4);

    K = [H;J_b];
    dK = [dH; dJ_b];
    lambda = (K/M*K')\(K/M*(C+G-Q) - dK*dq);
end
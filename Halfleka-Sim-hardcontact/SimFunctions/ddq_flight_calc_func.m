function ddq_calc = ddq_flight_calc_func(q, dq, u, lambda)

x = q(1); y = q(2); th1 = q(3); th2 = q(4); ph1 = q(5); ph2 = q(6);
dx = dq(1); dy = dq(2); dth1 = dq(3); dth2 = dq(4); dph1 = dq(5); dph2 = dq(6);
t1 = u(1);t2 = u(2);

M = M_calc_func(x, y, th1, th2, ph1, ph2);
C = C_calc_func(x, y, th1, th2, ph1, ph2, dx, dy, dth1, dth2, dph1, dph2);
G = G_calc_func(x, y, th1, th2, ph1, ph2);
Q = Q_t_calc_func(x, y, th1,th2,ph1,ph2,dx, dy, dth1,dth2,dph1,dph2,t1,t2);

H = H_calc_func(th1, th2, ph1, ph2);

ddq_calc = M\(-C-G+H'*lambda+Q);

end

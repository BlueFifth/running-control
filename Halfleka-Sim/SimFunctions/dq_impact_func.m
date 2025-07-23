function dq_impact = dq_impact_func(q, dq)

x = q(1); y = q(2); th1 = q(3); th2 = q(4);
ph1 = ph1_calc(th1, th2);
ph2 = ph2_calc(th1, th2);
dph1 = dph1_calc(th1, th2, dq(3), dq(4));
dph2 = dph2_calc(th1, th2, dq(3), dq(4));

M = M_calc_func(x, y, th1, th2, ph1, ph2);
J = J_calc_func(th1, th2, ph1, ph2);
H = H_calc_func(th1, th2, ph1, ph2);
K = [H;J];

dq = [dq,dph1,dph2];

lambda = -(K*(M\K'))\K*dq';
dq_impact = dq' + M\K'*lambda;
dq_impact = dq_impact(1:4);
end

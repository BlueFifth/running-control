function out = LinearizationCompliance(VLcmd, q,dq,ddq, F_c)
   th1 = q(3); th2 = q(4);
    dth1 = dq(3); dth2 = dq(4);
    % ff_torque_extra = ff_torque_func(q,dq,ddq, F_c);
    [R, Th] = BalekaKinematicsRth(th1, th2);
    [J11, J12, J21, J22]= BalekaJacobianRth(th1, th2);
    J = [J11, J12;J21, J22];
    dsmd = J*([dth1, dth2]');
    dR = dsmd(1);
    dTh = dsmd(2);

    Fr = VLcmd.KPr*(VLcmd.r_des - R) + VLcmd.KDr*(VLcmd.dr_des -dR) + VLcmd.Fff;
    Tth = VLcmd.KPth*(VLcmd.th_des - Th) + VLcmd.KDth*(VLcmd.dth_des - dTh) + VLcmd.Tff;
    JT = transpose(J);
    if (th1 < (pi - 35*pi/180)) % Apply electrical break if close to mechanical limits
        t1 = 500*((pi-33*pi/180)-th1) + 5*(0-dth1);
    elseif (th1 > 270*pi/180)
        t1 = 500*((265*pi/180)-th1) + 5*(0-dth1);
    else
        t1 = (JT(1,1)*Fr +JT(1,2)*Tth);
    end
    if  (th2 > 35*pi/180)
        t2 = 500*((33*pi/180)-th2) + 5*(0-dth2);
    elseif (th2 < -90*pi/180)
        t2 = 500*((-85*pi/180)-th2) + 5*(0-dth2);
    else
        t2 = (JT(2,1)*Fr + JT(2,2)*Tth);
    end
    %     t1 = max(min(SpeedTorque(t1 + ff_torque_extra(1), dth1), 53), -53); % set torque limits
    % t2 = max(min(SpeedTorque(t2 + ff_torque_extra(2), dth2), 53), -53);
    t1 = max(min(SpeedTorque(t1, dth1), 53), -53); % set torque limits
    t2 = max(min(SpeedTorque(t2, dth2), 53), -53);
    out = [t1, t2, R, Th, dR, dTh];%, ff_torque_extra(1), ff_torque_extra(2)];

    
end


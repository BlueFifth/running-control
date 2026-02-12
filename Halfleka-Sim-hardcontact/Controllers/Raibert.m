function out = Raibert(th1, th2, dth1, dth2, footHeight, time,dx, dy)
    % Raibert hopping state machine
    persistent state landTime dR VLcmd x_f_des
    % State variable
    % 0 = SETUP
    % 1 = FLIGHT
    % 2 = COMPRESSION
    % 3 = THRUST
    
    % Params:
    Ts = 0.15;
    Kdx = -0.1;
    % Edit to swap kdx negative
    dx_d = 2;


    x_f = dx*Ts/2 + Kdx*(dx - dx_d); % Foot placement
    % x_f = 0.1;
    [R, th] = BalekaKinematicsRth(th1, th2);
    
    if time<0.01
        state = 0;
        x_f_des = 0;
        landTime = 99;
        dR = 0;
        VLcmd.Tff = 0;
        VLcmd.dth_des =0;
        % VLcmd.th_des = -pi/2;
        % VLcmd.KPth = 600;
        % VLcmd.KDth = 10;
    end

switch state
    case 0 % SETUP
        % Specific commands for first landing
        VLcmd.r_des = 0.45;
        VLcmd.dr_des =0;
        VLcmd.KPr = 1000;
        VLcmd.KDr =40;
        VLcmd.Fff =0;

        VLcmd.th_des = -pi/2 +asin(x_f/R);
        VLcmd.KPth = 200;
        VLcmd.KDth = 20;
        if (footHeight <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
            x_f_des = x_f;

        end
    case 1 % FLIGHT
        % Brace for impact with high-ish spring
        VLcmd.r_des = 0.45;
        VLcmd.dr_des =0;
        VLcmd.KPr = 600;
        VLcmd.KDr =20;
        VLcmd.Fff = 0;
        % Position leg for landing
        VLcmd.th_des = -pi/2 +asin(x_f/R);
        VLcmd.dth_des = 0;
        VLcmd.KPth = 600;
        VLcmd.KDth = 20;

        if (footHeight <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
            x_f_des = x_f;
        end
    case 2 % COMPRESSION
        % Let spring compress
        VLcmd.r_des = 0.45;
        VLcmd.KPr = 900;
        VLcmd.KDr =10;

        % Servo hip
        VLcmd.th_des = -pi/2- asin(x_f_des/R); 
        VLcmd.dth_des = 0;%(dx*R - dR*R*sin(th))/(cos(th)*R^2);
        VLcmd.KPth = 800;
        VLcmd.KDth = 50;
        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR>0)) 
            state = 3;
        end
    case 3 % THRUST
        % Apply upwards thrust force
        VLcmd.r_des = 0.6;
        VLcmd.KPr = 5000;
        VLcmd.KDr =0;
        VLcmd.Fff = 0;% 2000;

        % Servo hip
        VLcmd.th_des = -pi/2 - asin(x_f_des/R); 
        VLcmd.dth_des = 0;%(dx*R - dR*R*sin(th))/(cos(th)*R^2);
        VLcmd.KPth = 800;
        VLcmd.KDth = 50;

        if (footHeight>0.01)%&&(dy>0))% ||((th1>3*pi/2)||(th2<-pi/2))
            state =1;
        end

end
    cmd = BasicCompliance(th1, th2, dth1, dth2, VLcmd);
    dR = cmd(5);
    % out = [cmd, state];
    out = [cmd, state];
end
  
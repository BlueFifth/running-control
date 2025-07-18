function out = Hopping(th1, th2, dth1, dth2, footHeight, time,dx, dy)
    persistent state landTime dR VLcmd
    % State variable
    % 1 = flight
    % 2 = contact
    % 3 = thrust

    if time == 0
        state = 0;
        landTime = 99;
        dR = 0;
        VLcmd.Tff = 0;
        VLcmd.dth_des =0;
        VLcmd.th_des = -pi/2;
        VLcmd.KPth = 600;
        VLcmd.KDth = 10;
    end

switch state
    case 0 % Setup
        VLcmd.r_des = 0.35;
        VLcmd.dr_des =0;
        VLcmd.KPr = 400;
        VLcmd.KDr =40;
        VLcmd.Fff =0;
        VLcmd.KPth = 100;
        VLcmd.KDth = 20;
        if (abs(dx)<0.01)&&(abs(dy)<0.01)
            state = 1;
        end
    case 1 % flight
        % Brace for impact
        VLcmd.r_des = 0.35;
        VLcmd.dr_des =0;
        VLcmd.KPr = 200;
        VLcmd.KDr =10;
        VLcmd.Fff =0;
        VLcmd.KPth = 600;
        VLcmd.KDth = 20;
        if (footHeight <=0)&&(dy<0)
            state = 2;
            landTime = time;
        end
    case 2 % Contact
        % Let spring compress
        VLcmd.r_des = 0.15;
        VLcmd.KPr = 500;
        VLcmd.KDr =50;
        VLcmd.KPth = 600;
        VLcmd.KDth = 60;
        if (((time-landTime)>0.05)&&(dR<0))
            state = 3;
        end
    case 3 % thrust
        % Apply upwards thrust force
        VLcmd.r_des = 0.7;
        VLcmd.KPr = 5000;
        VLcmd.KDr =0;
        VLcmd.KPth = 600;
        VLcmd.KDth = 60;

        % VLcmd.KPr = 0;
        % VLcmd.KDr =0;
        % VLcmd.Fff = 10000; %Max out upwards thrust
        if (footHeight>0.05)%&&(dy>0))% ||((th1>3*pi/2)||(th2<-pi/2))
            state =1;
        end
end
    cmd = BasicCompliance(th1, th2, dth1, dth2, VLcmd);
    dR = cmd(5);
    out = [cmd, state];
end
  
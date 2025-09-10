function out = HopTest(th1, th2,th3, th4, dth1, dth2,dth3, dth4, footHeight_f, footHeight_b, time, dy)
    % 2 legged Hopping state machine
    persistent state landTime dR_f dR_b VLcmd_f VLcmd_b
    % State variable
    % 0 = SETUP
    % 1 = FLIGHT (front)
    % 2 = COMPRESSION (both)
    % 3 = THRUST (both)
    % 4 = COMPRESSION (front)
    % 5 = THRUST(front)
    % 6 = FLIGHT (back)
    % 7 = COMPRESSION (back)
    % 8 = THRUST(back)
    % 9 = FLIGHT (both)
    
    % Params:
    %None

    if time<0.01
        state = 0;
        landTime = 99;
        dR_f = 0;
        dR_b = 0;
        VLcmd_f.Fff = 0;
        VLcmd_b.Fff = 0;
        VLcmd_f.Tff = 0;
        VLcmd_b.Tff = 0;

    end

switch state
    case 0 % SETUP
        % Specific commands for first landing
        VLcmd_f.r_des = 0.45;
        VLcmd_f.dr_des =0;
        VLcmd_f.KPr = 1000;
        VLcmd_f.KDr =40;
        VLcmd_f.th_des = -pi/2;
        VLcmd_f.KPth = 200;
        VLcmd_f.KDth = 20;

        VLcmd_b.r_des = 0.45;
        VLcmd_b.dr_des =0;
        VLcmd_b.KPr = 1000;
        VLcmd_b.KDr =40;
        VLcmd_b.th_des = -pi/2;
        VLcmd_b.KPth = 200;
        VLcmd_b.KDth = 20;

        if ((footHeight_f <=0)||(footHeight_b <=0)) &&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
        end
    case 1 % FLIGHT (front)
        % Brace for impact with high-ish spring
        VLcmd_f.r_des = 0.45;
        VLcmd_f.dr_des =0;
        VLcmd_f.KPr = 600;
        VLcmd_f.KDr =20;
        VLcmd_f.Fff = 0;
        % Position leg for landing
        VLcmd_f.th_des = -pi/2; 
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 600;
        VLcmd_f.KDth = 20;

        % Back leg retract to do nothing
        VLcmd_b.r_des = 0.25;
        VLcmd_b.dr_des =0;
        VLcmd_b.KPr = 600;
        VLcmd_b.KDr =20;
        VLcmd_b.Fff = 0;
        VLcmd_b.th_des = -pi/2; 
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 600;
        VLcmd_b.KDth = 20;

        if (footHeight_f <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
        end
    case 2 % COMPRESSION (both)
        % Let spring compress
        VLcmd_f.r_des = 0.45;
        VLcmd_f.KPr = 800;
        VLcmd_f.KDr =10;
        VLcmd_f.th_des = -pi/2;
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 800;
        VLcmd_f.KDth = 50;

        VLcmd_b.r_des = 0.45;
        VLcmd_b.KPr = 800;
        VLcmd_b.KDr =10;
        VLcmd_b.th_des = -pi/2;
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 800;
        VLcmd_b.KDth = 50;

        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR_f>0))&&(dR_b>0) 
            state = 3;
        end
    case 3 % THRUST (both)
        % Apply upwards thrust force
        VLcmd_f.r_des = 0.6;
        VLcmd_f.KPr = 5000;
        VLcmd_f.KDr =0;
        VLcmd_f.th_des = -pi/2;
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 800;
        VLcmd_f.KDth = 50;

        VLcmd_b.r_des = 0.6;
        VLcmd_b.KPr = 5000;
        VLcmd_b.KDr =0;
        VLcmd_b.th_des = -pi/2;
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 800;
        VLcmd_b.KDth = 50;

        if (footHeight_f>0.01)&&(footHeight_b>0.01)
            state =1;
        end
    case 4 % COMPRESSION (front)
        % Let spring compress
        VLcmd_f.r_des = 0.45;
        VLcmd_f.KPr = 800;
        VLcmd_f.KDr =10;
        VLcmd_f.th_des = -pi/2;
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 800;
        VLcmd_f.KDth = 50;


        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR_f>0))
            state = 5;
        end
    case 5 % THRUST (front)
        % Apply upwards thrust force
        VLcmd_f.r_des = 0.6;
        VLcmd_f.KPr = 5000;
        VLcmd_f.KDr =0;
        VLcmd_f.th_des = -pi/2;
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 800;
        VLcmd_f.KDth = 50;

        if (footHeight_f>0.01)
            state =6;
        end

    case 6 % Flight (back)
        % Brace for impact with high-ish spring
        VLcmd_b.r_des = 0.45;
        VLcmd_b.dr_des =0;
        VLcmd_b.KPr = 600;
        VLcmd_b.KDr =20;
        VLcmd_b.Fff = 0;
        % Position leg for landing
        VLcmd_b.th_des = -pi/2; 
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 600;
        VLcmd_b.KDth = 20;

        % Front leg retract to do nothing
        VLcmd_f.r_des = 0.25;
        VLcmd_f.dr_des =0;
        VLcmd_f.KPr = 600;
        VLcmd_f.KDr =20;
        VLcmd_f.Fff = 0;
        VLcmd_f.th_des = -pi/2; 
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 600;
        VLcmd_f.KDth = 20;

        if (footHeight_b <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 7;
            landTime = time;
        end

    case 7 % Compression (back foot)
        % Let spring compress
        VLcmd_b.r_des = 0.45;
        VLcmd_b.KPr = 800;
        VLcmd_b.KDr =10;
        VLcmd_b.th_des = -pi/2;
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 800;
        VLcmd_b.KDth = 50;


        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR_b>0))
            state = 8;
        end
    case 8 % THRUST (back)
        % Apply upwards thrust force
        VLcmd_b.r_des = 0.6;
        VLcmd_b.KPr = 5000;
        VLcmd_b.KDr =0;
        VLcmd_b.th_des = -pi/2;
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 800;
        VLcmd_b.KDth = 50;

        if (footHeight_b>0.01)
            state =9;
        end

    case 9 % FLIGHT (Both)
        % Brace for impact with high-ish spring
        VLcmd_f.r_des = 0.45;
        VLcmd_f.dr_des =0;
        VLcmd_f.KPr = 600;
        VLcmd_f.KDr =20;
        VLcmd_f.Fff = 0;
        % Position leg for landing
        VLcmd_f.th_des = -pi/2; 
        VLcmd_f.dth_des = 0;
        VLcmd_f.KPth = 600;
        VLcmd_f.KDth = 20;

        VLcmd_b.r_des = 0.45;
        VLcmd_b.dr_des =0;
        VLcmd_b.KPr = 600;
        VLcmd_b.KDr =20;
        VLcmd_b.Fff = 0;
        VLcmd_b.th_des = -pi/2; 
        VLcmd_b.dth_des = 0;
        VLcmd_b.KPth = 600;
        VLcmd_b.KDth = 20;

   

        if (footHeight_f <=0)&&(footHeight_b <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
        end
end
    cmd_f = BasicCompliance(th1, th2, dth1, dth2, VLcmd_f);
    dR_f = cmd_f(5);

    cmd_b = BasicCompliance(th3, th4, dth3, dth4, VLcmd_b);
    dR_b = cmd_b(5);
    out = [cmd_f, cmd_b, state];
end
  
function out = Raibert(th1, th2,th3, th4, dth1, dth2,dth3, dth4, footHeight_f, footHeight_b, time, dx, dy)
    % Raibert hopping state machine
    persistent state landTime dR_f dR_b VLcmd_f VLcmd_b x_f_des
    % State variable
    % 0 = SETUP
    % 1 = FLIGHT (front)
    % 2 = COMPRESSION (front)
    % 3 = THRUST (front)
    % 4 = FLIGHT (back)
    % 5 = COMPRESSION (back)
    % 6 = THRUST (back)
    
    % Params:
    Ts = 0.12;
    Kdx = 0.1;
    dx_d = 2;

    FloatLeg = 0.25;
    GroundLeg = 0.40;

    


    x_f = dx*Ts/2 + Kdx*(dx - dx_d); % Foot placement
    % x_f = 0.1;
    [R_f, th] = BalekaKinematicsRth(th1, th2);
    [R_b, th] = BalekaKinematicsRth(th3, th4);

    %% Set up reused constants for simple editing

    % Landing
    VL_landing.r_des = GroundLeg;
    VL_landing.KPr = 1000;
    VL_landing.KDr =40;
    VL_landing.KPth = 200;
    VL_landing.KDth = 20;

    VL_landing.dr_des = 0;
    VL_landing.dth_des = 0;
    VL_landing.Fff = 0;
    VL_landing.Tff = 0;
    VL_landing.th_des = 0;
    
    
    % Floating

    VL_float.r_des = FloatLeg;
    VL_float.KPr = 1000;
    VL_float.KDr =40;
    VL_float.KPth = 200;
    VL_float.KDth = 20;

  
    VL_float.dr_des = 0;
    VL_float.dth_des = 0;
    VL_float.Fff = 0;
    VL_float.Tff = 0;
    VL_float.th_des = 0;

    % Compress
    VL_compress.r_des = GroundLeg;
    VL_compress.KPr = 1500;
    VL_compress.KDr =50;
    VL_compress.KPth = 600;
    VL_compress.KDth = 20;

    VL_compress.dr_des = 0;
    VL_compress.dth_des = 0;
    VL_compress.Fff =0;
    VL_compress.Tff = 0;
    VL_compress.th_des = 0;

    % Thrust

    VL_thrust.r_des = 0.6;
    VL_thrust.KPr = 7000;
    VL_thrust.KDr =0;
    VL_thrust.KPth = 800;
    VL_thrust.KDth = 50;

    VL_thrust.dr_des = 0;
    VL_thrust.dth_des = 0;
    VL_thrust.Fff =0;
    VL_thrust.Tff = 0;
    VL_thrust.th_des = 0;


    if time<0.01
        state = 0;
        landTime = 99;
        dR_f = 0;
        dR_b = 0;
        x_f_des = 0;
    end

switch state
    case 0 % SETUP
        % Specific commands for first landing
        VLcmd_f = VL_landing;
        VLcmd_f.th_des = -pi/2 +asin(x_f/R_f);
        

        VLcmd_b = VL_float;
        VLcmd_b.th_des = -pi/2;
        
        if (footHeight_f <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
            x_f_des = x_f;

        end
    case 1 % FLIGHT (front)
        % Brace for impact with high-ish spring
        VLcmd_f = VL_landing;
        VLcmd_f.th_des = -pi/2 +asin(x_f/R_f);

        VLcmd_b = VL_float;
        VLcmd_b.th_des = -pi/2 + asin(x_f_des/R_b);


        if (footHeight_f <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 2;
            landTime = time;
            x_f_des = x_f;
        end
    case 2 % COMPRESSION (front)
        % Let spring compress
        VLcmd_f = VL_compress;
        VLcmd_f.th_des = -pi/2- asin(x_f_des/R_f); 

        VLcmd_b = VL_float;
        VLcmd_b.th_des = -pi/2 + asin(x_f_des/R_b);

        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR_f>0)) 
            state = 3;
        end
    case 3 % THRUST (front)
        % Apply upwards thrust force
        VLcmd_f = VL_thrust;
        VLcmd_f.th_des = -pi/2 - asin(x_f_des/R_f);

        VLcmd_b = VL_float;
        VLcmd_b.th_des = -pi/2 + asin(x_f_des/R_b);

        if (footHeight_f>0.01)%&&(dy>0))% ||((th1>3*pi/2)||(th2<-pi/2))
            state =4;
        end

    case 4 % FLIGHT (back)
        VLcmd_f = VL_float;
        VLcmd_f.th_des = -pi/2 + asin(x_f_des/R_f);

        VLcmd_b = VL_landing;
        VLcmd_b.th_des = -pi/2 + asin(x_f/R_b);

        if (footHeight_b <=0)&&(dy<0) % On foot touching ground go to COMPRESSION
            state = 5;
            landTime = time;
            x_f_des = x_f;
        end
    case 5 % COMPRESSION (back)
        VLcmd_f = VL_float;
        VLcmd_f.th_des = -pi/2 + asin(x_f_des/R_f);

        VLcmd_b = VL_compress;
        VLcmd_b.th_des = -pi/2- asin(x_f_des/R_b); 

        % When leg stops compressing (dR = 0) go to THRUST
        if (((time-landTime)>0.05)&&(dR_b>0)) 
            state = 6;
        end
    case 6 % THRUST (back)
        % Apply upwards thrust force
        VLcmd_f = VL_float;
        VLcmd_f.th_des = -pi/2 + asin(x_f_des/R_f);

        VLcmd_b = VL_thrust;
        VLcmd_b.th_des = -pi/2 - asin(x_f_des/R_b);

        if (footHeight_b>0.01)%&&(dy>0))% ||((th1>3*pi/2)||(th2<-pi/2))
            state =1;
        end

end
    cmd_f = BasicCompliance(th1, th2, dth1, dth2, VLcmd_f);
    dR_f = cmd_f(5);

    cmd_b = BasicCompliance(th3, th4, dth3, dth4, VLcmd_b);
    dR_b = cmd_b(5);
    out = [cmd_f, cmd_b, state];
end
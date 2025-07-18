function t_out = SpeedTorque(t_in, w)
    t_max = 53; %Nm
    t_rated = 18; %Nm
    w_max = 320/60*2*pi; %rad/s
    w_rated = 235*2*pi/60; %rad/s

    t_upper = min(((w-w_max)*t_rated)/(w_rated-w_max), t_max); % Torque upper bound
    t_lower = max(((w+w_max)*t_rated)/(w_rated-w_max), -t_max); % Torque lower bound

    t_out = max(t_lower, min(t_upper, t_in));
end
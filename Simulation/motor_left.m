function RPM_Motor_reach = motor_left(Voltage, time)

% K = 20.7; 
K = 2.93;
T = 0.0226;
RPM_Motor_reach = 6*K*Voltage*(1-exp(-time/T));
end

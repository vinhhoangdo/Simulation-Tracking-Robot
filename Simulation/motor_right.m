function RPM_Motor_reach = motor_right(Voltage, time)

% K = 20.7; 
K = 2.87;
% T = 0.0226;
T = 0.0226;
RPM_Motor_reach = 6*K*Voltage*(1-exp(-time/T));
end

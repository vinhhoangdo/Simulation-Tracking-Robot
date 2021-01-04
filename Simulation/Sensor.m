function Real_Error = Sensor(Estimate_Error)

% Real_Error = 0.9929*Estimate_Error - 0.5159/1000 +(2.1/1000)*rand*(-1).^(randi(10));
Real_Error = 0.9929*Estimate_Error - 0.5159/1000;
end

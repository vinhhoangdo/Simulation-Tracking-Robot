function PID_Result = PID_Controller(Error, Pre_Error, Pre_Pre_Error, kp, ki, kd, t_samp)
 
P_part = kp*(Error - Pre_Error);
I_part = ki*t_samp*Error;
D_part = (kd/t_samp)*(Error - 2*Pre_Error + Pre_Pre_Error);

PID_Result = P_part + I_part + D_part;
end
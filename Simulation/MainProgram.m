clf
clc
close all
clear all

%Parameters
t_samp_sim = 0.001;                                                         
t_samp_sys = 0.15;                                                             
t_samp_motor = 0.02;            
no_sample = 30;          
t_samp_sensor = t_samp_sys/no_sample;                                     
v_des = 0.4;                                                            

d = 0.15;    %Distance between A and C
dw = 0.066;%1/2 Width                                                          
b = 0.04;    %Radius wheel                                                                                                                                                                     

%Initial condition
Error_wl(1) = 0;                                                         
U_PID_L(1) = 0;                                                            
U_Voltage_L(1) = 0;                                               

Error_wr(1) = 0;                                               
U_PID_R(1) = 0;                                                  
U_Voltage_R(1) = 0;                                                     

% Parameters of PID Controller of Motor                                                              
% Left Wheel
% kp1 = 0.35731;
% ki1 = 16.0074;
% kd1 = 0.0013;

kp1 = 0.2744;
ki1 = 24.42;
kd1 = 0.0013;
% Right Wheel
% kp2 = 0.35048;
% ki2 = 19.1704;
% kd2 = 0.0013;
kp2 = 0.2744;
ki2 = 24.42;
kd2 = 0.0013;

[xr, yr, phr, wr] = Ref_Data(t_samp_sim, v_des);
n = length (xr);                                                            

% Parameters of PD Controller (System)
%Theo thiet ke: KP = 90; KD = 0.27
KP_sys = 106.204;
% KP_sys = 0.05204;
KD_sys = 1.904;
% Initial Condition
xc(1) = 0;      yc(1) = 0;       phc(1) = -175;       wc(1) = 0;           % Starting Position of the robot
vc(1) = 0;
wcl(1) = 0;
wcr(1) = 0;

e2_measured(1) = Sensor(-sin(phc(1))*(xr(1) - xc(1))+ cos(phc(1))*(yr(1) - yc(1)));  % e2
% e2_measured_dot(1) = e2_measured(1);
e2_measured_dot(1) = 0;

vc_ref(1) = v_des;

%PD contronller
wc_ref(1) =KP_sys *e2_measured(1) +KD_sys *e2_measured_dot(1);

vc_ref(1) = 0;
wc_ref(1) = 0;

wcl_ref(1) = 0;
wcr_ref(1) = 0;

%Van toc dai ban dau Diem C
x_dot(1) = cos(phc(1))*vc(1);
y_dot(1) = sin(phc(1))*vc(1);
ph_dot(1) = wc(1);

% Phuong trinh dong hoc cua C
phc_dot(1) = ph_dot(1);
xc_dot(1) = x_dot(1) - d*sin(phc(1))*wc(1);
yc_dot(1) = y_dot(1) + d*cos(phc(1))*wc(1);

t(1) = 0;

n_sys = (t_samp_sys/t_samp_sim);
n_motor = (t_samp_motor/t_samp_sim);
n_sensor = (t_samp_sensor/t_samp_sim);


for i=2:n
  
    if mod(i,n_sensor) ==2                                                 
        e2_measured(i) = Sensor(-sin(phc(i-1))*(xr(i) - xc(i-1))+ cos(phc(i-1))*(yr(i) - yc(i-1))); 
        e2_measured_dot(i) = e2_measured(i)- e2_measured(i-1);
    else
        e2_measured(i) = e2_measured(i-1);
        e2_measured_dot(i) = e2_measured_dot(i-1);
    end
        
    
    %% PD Controller
    if mod(i,n_sys)==2                                                     
   
        vc_ref(i) = v_des;
%         wc_ref(i) = wr(i) + k2*e2_measured(i)*v_des;
        wc_ref(i) = KP_sys*e2_measured(i) + KD_sys*e2_measured_dot(i);
      
        wcl_ref(i) = ((2*vc_ref(i) - 2*dw*wc_ref(i))/(2*b))*(30/pi);      
        wcr_ref(i) = ((2*vc_ref(i) + 2*dw*wc_ref(i))/(2*b))*(30/pi);       
    else
        vc_ref(i) = vc_ref(i-1);
        wc_ref(i) = wc_ref(i-1);
        
        wcl_ref(i) = wcl_ref(i-1);
        wcr_ref(i) = wcr_ref(i-1);
    end
    
    %% PID Controller
    if mod(i,n_motor)==2                                                  
        %% Left motor
        Error_wl(i) = wcl_ref(i) - wcl(i-1);
        if (i==2) 
            U_PID_L(i) = U_PID_L(i-1) + PID(Error_wl(i), Error_wl(i-1), Error_wl(i-1), kp1, ki1, kd1, t_samp_motor);
        else
            U_PID_L(i) = U_PID_L(i-1) + PID(Error_wl(i), Error_wl(i-1), Error_wl(i-n_motor-1), kp1, ki1, kd1, t_samp_motor);
        end
        
        %% Right motor
        Error_wr(i) = wcr_ref(i) - wcr(i-1);
        if (i==2) 
            U_PID_R(i) = U_PID_R(i-1) + PID(Error_wr(i), Error_wr(i-1), Error_wr(i-1), kp2, ki2, kd2, t_samp_motor);
        else
            U_PID_R(i) = U_PID_R(i-1) + PID(Error_wr(i), Error_wr(i-1), Error_wr(i-n_motor-1), kp2, ki2, kd2, t_samp_motor);
        end
        
    else 
        Error_wl(i) = Error_wl(i-1);        
        Error_wr(i) = Error_wr(i-1);
        
        U_PID_L(i) = U_PID_L(i-1);
        U_PID_R(i) = U_PID_R(i-1);
    end
    
    U_Voltage_L(i) = Voltage_calculation(U_PID_L(i));
    if (U_Voltage_L(i) >= 12) U_Voltage_L(i) = 12; end
    if ((U_Voltage_L(i) <= 0.6)&&(U_Voltage_L(i) >=0)) U_Voltage_L(i) = 0.6; end
    if (U_Voltage_L(i) <= -12) U_Voltage_L(i) = -12; end
    if ((U_Voltage_L(i) <= 0)&&(U_Voltage_L(i) >=-0.6)) U_Voltage_L(i) = -0.6; end
    
    U_Voltage_R(i) = Voltage_calculation(U_PID_R(i));  
    if (U_Voltage_R(i) >= 12) U_Voltage_R(i) = 12; end
    if ((U_Voltage_R(i) <= 0.6)&&(U_Voltage_R(i) >=0)) U_Voltage_R(i) = 0.6; end
    if (U_Voltage_R(i) <= -12) U_Voltage_R(i) = -12; end
    if ((U_Voltage_R(i) <= 0)&&(U_Voltage_R(i) >=-0.6)) U_Voltage_R(i) = -0.6; end
    
    %% Calculate actual parameters of the robot    
    wcl(i) = motor_left(U_Voltage_L(i), i*t_samp_sim);
    wcr(i) = motor_right(U_Voltage_R(i), i*t_samp_sim);
    vc(i) = ((wcl(i) + wcr(i))*b*(pi/30))/2;
    wc(i) = (wcr(i) - wcl(i))*(pi/30)*(b/(2*dw));
    ph_dot(i) = wc(i);
    phc_dot(i) = ph_dot(i);
    
    %% Calculate actual position of the robot
    x_dot(i) = cos(phc(i-1))*vc(i);
    y_dot(i) = sin(phc(i-1))*vc(i);
    xc_dot(i) = x_dot(i) - d*sin(phc(i-1))*ph_dot(i);
    yc_dot(i) = y_dot(i) + d*cos(phc(i-1))*ph_dot(i);
    
    xc(i) = xc_dot(i)*t_samp_sim + xc(i-1);
    yc(i) = yc_dot(i)*t_samp_sim + yc(i-1);
    phc(i) = phc_dot(i)*t_samp_sim + phc(i-1);
    
    %% Time
    t(i) = t(i-1) + t_samp_sim;      
end;



%% Draw the result
plot(xr,yr,'b','linewidth',1.5);                                            % Draw the line
xlabel('x(m)');    ylabel('y(m)');
grid
axis equal
hold on


%Dimensions of line robot
dai=0.29;
rong=0.150;
goctinh=atan(rong/dai);
canhtinh=((dai*dai+rong*rong)^(1/2))/2;

for i=1:n
    ax(i)=xc(i)+cos(phc(i)-goctinh)*canhtinh;
    cx(i)=2*xc(i)-ax(i);
    bx(i)=cx(i)+dai*cos(phc(i));
    dx(i)=2*xc(i)-bx(i);
    ay(i)=yc(i)+sin(phc(i)-goctinh)*canhtinh;
    cy(i)=2*yc(i)-ay(i);
    by(i)=cy(i)+dai*sin(phc(i));
    dy(i)=2*yc(i)-by(i);
%     Xxe = [ax(i) bx(i) cx(i) dx(i) ax(i)]; 
%     Yxe = [ay(i) by(i) cy(i) dy(i) ay(i)];
%     plot(Xxe,Yxe,'r','LineWidth',0.2);
end
for i = 1:n
    if mod(i,50)==0   
    car_1 = line([ax(i), bx(i)], [ay(i), by(i)], 'Color', 'b', 'LineWidth', 1);
    car_2 = line([bx(i), cx(i)], [by(i), cy(i)], 'Color', 'b', 'LineWidth', 1);
    car_3 = line([cx(i), dx(i)], [cy(i), dy(i)], 'Color', 'b', 'LineWidth', 1);
    car_4 = line([dx(i), ax(i)], [dy(i), ay(i)], 'Color', 'b', 'LineWidth', 1);
%     plot(xs, ys, 'y');

    tracking_point = viscircles([xc(i) yc(i)], 0.01, 'EdgeColor', 'r');
    pause(0.05);
    delete(car_1);
    delete(car_2);
    delete(car_3);
    delete(car_4);
    end
end

hold on
% Draw the Tracking point of the robot
plot(xc,yc,'k','linewidth',1);
title('Tracking map')
legend('Reference','Car Tracking');
% draw phi
figure; plot(phr*180/pi,'b-','linewidth',0.5); xlabel('s(m)'); ylabel('angle of reference section (deg)'); grid
        hold on
        plot(phc*180/pi,'k-','linewidth',0.5);
        legend('phr', 'phc');

% % draw omega
figure; plot(wr,'b-','linewidth',0.5);         xlabel('s(m)'); ylabel('angular velocity of reference section (rad/s)'); grid
        hold on
        plot(wc,'k-','linewidth',0.5);
        legend('wr','wc');

% % draw motor speed
figure; plot(t,wcl,'b','linewidth',0.5); xlabel('t(s)'); ylabel('wheel angular velocity(RPM)'); grid
        hold on
        plot(t,wcr,'r','linewidth',0.5); 
        legend('Left Motor','Right Motor');
        
% % draw error
figure; plot(t, (e2_measured)*(10.^3),'b','linewidth',0.5);   xlabel('t(s)'); ylabel('Tracking Error(mm)'); grid   
        hold on
       % plot(t, (e2_filter)*(10.^3),'r','linewidth',3);
        legend('Measured error');

% % % draw Voltage supply for motors
figure; plot(t, U_Voltage_L,'b','linewidth',0.5);   xlabel('t(s)');     ylabel('U(V)');
        hold on
        plot(t, U_Voltage_R,'r','linewidth',0.5);
        grid
        legend('Left Motor','Right Motor');
function [x, y, ph, w] = Ref_Data(t_samp_sym, v_des)
xr1(1) = 0;        yr1(1) = 0;
phr1(1) = 0;        
wr1(1) = 0;
n1 = round(1.5/(v_des*t_samp_sym)) + 1;
for i=2:n1
    xr1(i) = xr1(i-1) - v_des*t_samp_sym;
    yr1(i) = yr1(i-1);
    phr1(i) = phr1(i-1);
    wr1(i) = wr1(i-1);      
end;
xr2(1) = xr1(n1);   yr2(1) = yr1(n1);
phr2(1) = phr1(n1);
wr2(1) = wr1(n1);
n2 = round((sqrt(5)/2)/(v_des*t_samp_sym)) + 1;
for i=2:n2
    xr2(i) = xr2(i-1) - v_des*t_samp_sym*2/sqrt(5);
    yr2(i) = yr2(i-1)+ v_des*t_samp_sym*1/sqrt(5);
    phr2(i) = phr2(i-1);
    wr2(i) = wr2(i-1);      
end;

Rr = 0.5;
xr3(1) = xr2(n2);   yr3(1) = yr2(n2);
phr3(1) = phr2(n2);
wr3(1) = wr2(n2);
n3 = round(0.5*pi*Rr/(v_des*t_samp_sym)) + 1;
for i=2:n3
    if i==2
        j = 0;
    end;
    xr3(i) = xr3(1) - Rr*sin(v_des*j*t_samp_sym/Rr);
    yr3(i) = yr3(1) - Rr*(1 - cos(v_des*j*t_samp_sym/Rr));
    phr3(i) = phr3(i-1) + v_des*t_samp_sym/Rr;
    wr3(i) = wr3(i-1);
    j = j + 1;
end
Rr = 0.5;
xr4(1) = xr3(n3);   yr4(1) = yr3(n3);
phr4(1) = phr3(n3);
wr4(1) = v_des/Rr;
n4 = round(0.5*pi*Rr/(v_des*t_samp_sym)) + 1;
for i=2:n4
    if i==2
        j = 0;
    end;
    xr4(i) = xr4(1) + Rr*(1-cos(v_des*j*t_samp_sym/Rr));
    yr4(i) = yr4(1) - Rr*sin(v_des*j*t_samp_sym/Rr);
    phr4(i) = phr4(i-1) + v_des*t_samp_sym/Rr;
    wr4(i) = wr4(i-1);
    j = j + 1;
end

xr5(1) = xr4(n4);   yr5(1) = yr4(n4);
phr5(1) = phr4(n4);
wr5(1) = wr4(n4);
n5 = round((sqrt(5))/(v_des*t_samp_sym)) + 1;
for i=2:n5
    xr5(i) = xr5(i-1) + v_des*t_samp_sym*2/sqrt(5);
    yr5(i) = yr5(i-1)+ v_des*t_samp_sym*1/sqrt(5);
    phr5(i) = phr5(i-1);
    wr5(i) = wr5(i-1);      
end;


Rr = 0.5;
xr6(1) = xr5(n5);   yr6(1) = yr5(n5);
phr6(1) = phr5(n5);
wr6(1) = v_des/Rr;
n6 = round(0.5*pi*Rr/(v_des*t_samp_sym)) + 1;
for i=2:n6
    if i==2
        j = 0;
    end;
    xr6(i) = xr6(1) + Rr*sin(v_des*j*t_samp_sym/Rr);
    yr6(i) = yr6(1) - Rr*(1 - cos(v_des*j*t_samp_sym/Rr));
    phr6(i) = phr6(i-1) - v_des*t_samp_sym/Rr;
    wr6(i) = wr6(i-1);
    j = j + 1;
end

Rr = 0.5;
xr7(1) = xr6(n6);   yr7(1) = yr6(n6);
phr7(1) = phr6(n6);
wr7(1) = v_des/Rr;
n7 = round(0.5*pi*Rr/(v_des*t_samp_sym)) + 1;
for i=2:n7
    if i==2
        j = 0;
    end;
    xr7(i) = xr7(1) - Rr*(1-cos(v_des*j*t_samp_sym/Rr));
    yr7(i) = yr7(1) - Rr*sin(v_des*j*t_samp_sym/Rr);
    phr7(i) = phr7(i-1) - v_des*t_samp_sym/Rr;
    wr7(i) = wr7(i-1);
    j = j + 1;
end

xr8(1) = xr7(n7);   yr8(1) = yr7(n7);
phr8(1) = phr7(n7);
wr8(1) = wr7(n7);
n8 = round((sqrt(5)/2)/(v_des*t_samp_sym)) + 1;
for i=2:n8
    xr8(i) = xr8(i-1) - v_des*t_samp_sym*2/sqrt(5);
    yr8(i) = yr8(i-1)+ v_des*t_samp_sym*1/sqrt(5);
    phr8(i) = phr8(i-1);
    wr8(i) = wr8(i-1);      
end;

xr9(1) = xr8(n8);   yr9(1) = yr8(n8);
phr9(1) = phr8(n8);
wr9(1) = wr8(n8);
n9 = round(1.5/(v_des*t_samp_sym)) + 1;
for i=2:n9
    xr9(i) = xr9(i-1) - v_des*t_samp_sym;
    yr9(i) = yr9(i-1);
    phr9(i) = phr9(i-1);
    wr9(i) = wr9(i-1);      
end;

%% Reference create
xr = [xr1,xr2,xr3,xr4,xr5,xr6,xr7,xr8,xr9];
yr = [yr1,yr2,yr3,yr4,yr5,yr6,yr7,yr8,yr9];
phr = [phr1,phr2,phr3,phr4,phr5,phr6,phr7,phr8,phr9];
wr = [wr1,wr2,wr3,wr4,wr5,wr6,wr7,wr8,wr9];
%% Result
x = xr;
y = yr;
ph = phr;
w = wr;

end

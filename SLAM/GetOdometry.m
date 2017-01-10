function [xnow]=GetOdometry(k)
persistent LastOdom;
global UTrue;
if(isempty(LastOdom))
    
    global xVehicleTrue;
    LastOdom=xVehicleTrue;
end
u=GetRobotControl(k);
xnow=tcomp(LastOdom,u);
uNoise=sqrt(UTrue)*randn(3,1);
xnow=tcomp(xnow,uNoise);
LastOdom=xnow;
end

    
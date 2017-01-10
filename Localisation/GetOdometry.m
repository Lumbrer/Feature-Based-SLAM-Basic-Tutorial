function [xnow]=GetOdometry(k)
persistent LastOdom; %internal to robot low level controller
global UTrue
if(isempty(LastOdom))
    global xTrue
    LastOdom=xTrue;
end
u=GetRobotControl(k);
xnow=tcomp(LastOdom,u);
uNoise=sqrt(UTrue)*randn(3,1);
xnow=tcomp(xnow,uNoise);
LastOdom=xnow;
end
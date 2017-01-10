function SimulateWorld(k)
global xTrue
u=GetRobotControl(k);
xTrue=tcomp(xTrue,u);
xTrue(3)=AngleWrap(xTrue(3));
end

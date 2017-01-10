function SimulateWorld(k)

global xVehicleTrue
u=GetRobotControl(k);
xVehicleTrue=tcomp(xVehicleTrue,u);
end
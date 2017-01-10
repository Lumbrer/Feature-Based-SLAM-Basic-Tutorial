function u =GetRobotControl(k)
global nSteps
u=[0;0.025;0.1*pi/180*sin(3*pi*k/nSteps)];
%u=[0;0.15;0.3*pi/180];
end

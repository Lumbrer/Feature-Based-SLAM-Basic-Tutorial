function [z,iFeature]=GetObservation(k)
global Map xTrue RTrue nSteps
%fake sensor failure
if(abs(k-nSteps/2)<0.1*nSteps)
    z=[];
    iFeature=-1;
   % disp('Sensor Failure!')
else
    iFeature=ceil(size(Map,2)*rand(1));
    z=DoObservationModel(xTrue,iFeature,Map)+sqrt(RTrue)*randn(2,1);
    %just added noise to sensor
    z(2)=AngleWrap(z(2));
end
end

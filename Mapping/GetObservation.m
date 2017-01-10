function [z,iFeature]=GetObservation(k)

global Map xVehicleTrue RTrue nSteps

iFeature=ceil(size(Map,2)*rand(1));
z=DoObservationModel(xVehicleTrue,Map(:,iFeature))+sqrt(RTrue)*randn(2,1);
z(2)=AngleWrap(z(2));
end
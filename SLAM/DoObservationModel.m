function [z] =DoObservationModel(xVeh,xFeature)
Delta=xFeature-xVeh(1:2);
z=[norm(Delta);atan2(Delta(2),Delta(1))-xVeh(3)];
z(2)=AngleWrap(z(2));
end
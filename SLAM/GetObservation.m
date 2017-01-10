function [z,iFeature]=GetObservation(k)
global Map xVehicleTrue RTrue nSteps SensorSettings
done=0;
Trys=1;
z=[]; iFeature=-1;
while(~done&&Trys<0.5*size(Map,2))
    
    %choose random feature
    iFeature=ceil(size(Map,2)*rand(1));
    z=DoObservationModel(xVehicleTrue,Map(:,iFeature))+sqrt(RTrue)*randn(2,1);
    z(2)=AngleWrap(z(2));
    if(abs(pi/2-z(2))<SensorSettings.FieldOfView*pi/180&&z(1)<SensorSettings.Range)%el sensor gira 90 deg en cada direccion!!
        done=1;
    else
        Trys=Trys+1;
        z=[];iFeature=-1;
    end
end
end
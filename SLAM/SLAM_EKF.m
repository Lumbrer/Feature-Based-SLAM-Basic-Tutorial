%EKF SLAM

close all; clear all; clc;
global xVehicleTrue Map RTrue UTrue nSteps
global SensorSettings

%change here sensor behaviour
SensorSettings.FieldOfView=45;
SensorSettings.Range=100;

DrawEveryNFrames=50;

nSteps=8000;

%when to take pictures
SnapShots=ceil(linspace(2,nSteps,25));

%size of problem

nFeatures=40;
MapSize=200;
Map=MapSize*rand(2,nFeatures)-MapSize/2;

UTrue=diag([0.01 0.01 1.5*pi/180]).^2;
RTrue=diag([1.1 5*pi/180]).^2;

UEst=2*UTrue;
REst=2*RTrue;
xVehicleTrue=[0;0;-pi/2];

xEst=xVehicleTrue;
PEst=diag([1 1 0.01]);
MappedFeatures=NaN*zeros(nFeatures,2);

%Store

PStore=NaN*zeros(nFeatures,nSteps);
xErrStore=NaN*zeros(nFeatures,nSteps);

%initial graphs
figure(1);
set(gcf,'units','normalized','outerposition',[0 0 1 1]);hold on; grid off; axis equal;
plot(Map(1,:),Map(2,:),'g*');hold on;
set(gcf,'doublebuffer','on');
hObsLine=line([0 0],[0 0]);
set(hObsLine,'linestyle',':');
a=axis; axis(a*1.1);

xOdomLast=GetOdometry(1);

for k=2:nSteps
    
    %Simulate world
    SimulateWorld(k);
    
    %get control
    xOdomNow=GetOdometry(k);
    u=tcomp(tinv(xOdomLast),xOdomNow);
    xOdomLast=xOdomNow;
    
    %we need this
    xVehicle=xEst(1:3);
    xMap=xEst(4:end);
    
    %prediction
    xVehiclePred=tcomp(xVehicle,u);
    PPredvv=J1(xVehicle,u)*PEst(1:3,1:3)*J1(xVehicle,u)'+J2(xVehicle,u)*UEst*J2(xVehicle,u)';
    PPredvm=J1(xVehicle,u)*PEst(1:3,4:end);
    PPredmm=PEst(4:end,4:end);
    
    xPred=[xVehiclePred;xMap];
    PPred=[PPredvv PPredvm;
        PPredvm' PPredmm];
    
    %observe a random feature
    [z,iFeature]=GetObservation(k);
    
    if (~isempty(z))
        %have we seen before
        if (~isnan(MappedFeatures(iFeature,1)))
            %predict observation
            FeatureIndex=MappedFeatures(iFeature,1);
            xFeature=xPred(FeatureIndex:FeatureIndex+1);
            
            zPred=DoObservationModel(xPred(1:3,1),xFeature); %Yo creo que en lugar de xVehicle es xPred(1:3)!!!
            
            [jHxv,jHxf]=GetObsJacs(xPred(1:3,1),xFeature);%same as above
            jH=zeros(2,length(xEst));
            jH(:,FeatureIndex:FeatureIndex+1)=jHxf;
            jH(:,1:3)=jHxv;
            
            Innov=z-zPred;
            Innov(2)=AngleWrap(Innov(2));
            
            S=jH*PPred*jH'+REst;
            W=PPred*jH'*inv(S);
            xEst=xPred+W*Innov;
            PEst=PPred-W*S*W';
            
            PEst=0.5*(PEst+PEst');
            
        else
            %new feature
            nStates=length(xEst);
            xFeature=xPred(1:2,1)+[z(1)*cos(z(2)+xVehicle(3));z(1)*sin(z(2)+xVehicle(3))];
            xEst=[xEst;xFeature];
            MappedFeatures(iFeature,:)=[length(xEst)-1 length(xEst)];
            
            [jGxv, jGz]=GetNewFeatureJacs(xVehicle,z);
            
            M=[eye(nStates), zeros(nStates,2);jGxv, zeros(2,nStates-3),jGz];
            PEst=M*blkdiag(PEst,REst)*M';
            
        end
        
    else
        xEst=xPred;
        PEst=PPred;
        
    end
    
    
    if(mod(k-2,DrawEveryNFrames)==0)
        a=axis;
        clf;
        axis(a);hold on;
        n=length(xEst);
        nF=(n-3)/2;
        DoVehicleGraphics(xEst(1:3),PEst(1:3,1:3),3,[0 1]);
        
        if (~isnan(z))
            h=line([xEst(1), xFeature(1)],[xEst(2),xFeature(2)]);
            set(h,'linestyle',':');
        end
        for (i=1:nF)
            iF=3+2*i-1;
            plot(xEst(iF),xEst(iF+1),'b*');
            PlotEllipse(xEst(iF:iF+1),PEst(iF:iF+1,iF:iF+1),3);
        end
        fprintf('k_=_%d\n',k);
        xlabel('X Coordinate'); ylabel('Y Coordinate'); title('Feature Based SLAM - EKF');
        drawnow;
        %pause(0.1)
    end
    
    if(ismember(k,SnapShots))
        iPic=find(SnapShots==k);
        print(gcf,'-depsc',sprintf('EKFSLAM%d.eps',iPic));
    end
    
end
            
            
    
    
%%EKFMapping
clear all;
close all;
clc;
global xVehicleTrue Map RTrue UTrue nSteps

nSteps=600;
nFeatures=6;
Mapsize=200;
Map=Mapsize*rand(2,nFeatures)-Mapsize/2; %true map to be generated

UTrue=diag([0.01 0.01 pi/180]).^2;
RTrue=diag([8 7*pi/180]).^2;

UEst=1.0*UTrue;
REst=1.0*RTrue;

xVehicleTrue=[1; -40; -pi/2];

%initial conditions - no map
xEst=[];
PEst=[];
MappedFeatures=NaN*zeros(nFeatures,2);

%storage
PStore=NaN*zeros(nFeatures,nSteps);
XErrStore=NaN*zeros(nFeatures,nSteps);

%init graphs
figure(1); hold on; grid off; axis equal;
plot(Map(1,:),Map(2,:),'g*'); hold on; 
set(gcf,'doublebuffer','on');
hObsLine=line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

for k=2:nSteps
    
    %do world iteration
    SimulateWorld(k)
    
    %prediction model
    xPred=xEst;
    PPred=PEst;
    
    %observe a random feature
    [z,iFeature]=GetObservation(k);
    
    if (~isempty(z))
        
        %have we seen this feature before?
        if(~isnan(MappedFeatures(iFeature,1)))
            %retrieve the feature from already observed data and only
            %correct using measurement data 
            FeatureIndex=MappedFeatures(iFeature,1);
            xFeature=xPred(FeatureIndex:FeatureIndex+1);
            zPred=DoObservationModel(xVehicleTrue,xFeature);
            
            %get observation jacobian
            [jHxv,jHxf]=GetObsJacs(xVehicleTrue,xFeature);
            %fill in full jacobian
            jH=zeros(2,length(xEst));
            jH(:,FeatureIndex:FeatureIndex+1)=jHxf;
            
            %Kalman Update
            Innov=z-zPred;
            Innov(2)=AngleWrap(Innov(2));
            
            S=jH*PPred*jH'+REst;
            W=PPred*jH'*inv(S);
            xEst=xPred+W*Innov;
            PEst=PPred-W*S*W';
            
            %note use of Joseph form
%             I=eye(size(PEst));
%             PEst=(I-W*jH)*PPred*(I-W*jH)'+W*REst*W';
%            %Ensure P remains symmetric
            PEst=0.5*(PEst+PEst');
        else
            %new feature
            nStates=length(xEst);
            xFeature=xVehicleTrue(1:2)+[z(1)*cos(z(2)+xVehicleTrue(3));z(1)*sin(z(2)+xVehicleTrue(3))];
            xEst=[xEst;xFeature];
            [jGxv,jGz]=GetNewFeatureJacs(xVehicleTrue,z);
            
            M=[eye(nStates),zeros(nStates,2);zeros(2,nStates),jGz]; %we do not use jacobian wrt vehicle
            PEst=M*blkdiag(PEst,REst)*M';
            %now we store thye location of this feature
            MappedFeatures(iFeature,:)=[length(xEst)-1,length(xEst)];
            
        end
        
    else
        %no observation available
        
    end
    
    if mod(k-2,40)==0
        plot(xVehicleTrue(1),xVehicleTrue(2),'r*');
        %now draw all the estimated feature points
        DoMapGraphics(xEst,PEst,5);
        disp(strcat('K value equals:','',num2str(k)));
        drawnow;
    end
    
    %Storage
    for i=1:nFeatures
        if(~isnan(MappedFeatures(i,1)))
            iL=MappedFeatures(i,1);
            PStore(k,i)=det(PEst(iL:iL+1,iL:iL+1));
            XErrStore(k,i)=norm(xEst(iL:iL+1)-Map(:,i));
        end
    end
    
end

figure(2);
plot(PStore);


            
    
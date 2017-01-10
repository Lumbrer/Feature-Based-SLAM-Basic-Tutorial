%% EKF localisation

close all; clear all; clc;
global xTrue Map RTrue UTrue nSteps
nSteps= 6000;
Map=140*rand(2,30)-70;
UTrue=diag([0.01 0.01 1*pi/180]).^2;
RTrue= diag([2 3*pi/180]).^2;

UEst=1*UTrue;
REst=1*RTrue;

xTrue=[1;-40;-pi/2];
xOdomLast=GetOdometry(1);

%initial conditions
xEst=xTrue;
PEst=diag([1 1 pi/180]).^2;

%%%%%%%%%%%%%%Storage%%%%%%%%%%%%%%%

InnovStore=NaN*zeros(2,nSteps);
SStore=NaN*zeros(2,nSteps);
PStore=NaN*zeros(3, nSteps);
XStore=NaN*zeros(3,nSteps);
XErrStore=NaN*zeros(3,nSteps);

%init graphs
figure(1); hold on; grid off; axis equal;
plot(Map(1,:),Map(2,:),'g*'); hold on; 
set(gcf,'doublebuffer','on');
hObsLine=line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

for k=2:nSteps
    
    %do world iteration
    SimulateWorld(k);
    
    % figure out control
    xOdomNow=GetOdometry(k);
    u=tcomp(tinv(xOdomLast),xOdomNow);
    xOdomLast=xOdomNow;
    
    %prediction
    xPred=tcomp(xEst,u);
    xPred(3)=AngleWrap(xPred(3));
    
    PPred=J1(xEst,u)*PEst*J1(xEst,u)'+J2(xEst,u)*UEst*J2(xEst,u)';
    
    %observe a random feature
    [z,iFeature]=GetObservation(k);
    
    if ~isempty(z)
        %predict observation
        zPred=DoObservationModel(xPred,iFeature,Map);
        
        %get observation jacobian
        jH=GetObsJac(xPred,iFeature,Map);
        
        %do Kalman update
        Innov=z-zPred;
        Innov(2)=AngleWrap(Innov(2));
        S=jH*PPred*jH'+REst;
        W=PPred*jH'*inv(S);
        xEst=xEst+W*Innov;
        xEst(3)=AngleWrap(xEst(3));
        
        %note use of Joseph form as nummerically stable
        
        PEst=(eye(3)-W*jH)*PPred*(eye(3)-W*jH)'+W*REst*W';
        PEst=0.5*(PEst+PEst');
        
    else
        %no data from sensor
        xEst=xPred;
        PEst=PPred;
        Innov=[NaN;NaN];
        S=NaN*eye(2);
    end
    
    if mod(k-2,300)==0
        DoVehicleGraphics(xEst,PEst(1:2,1:2),8,[0,1]);
        if(~isempty(z))
            set(hObsLine,'XData',[xEst(1),Map(1,iFeature)]);
            set(hObsLine,'YData',[xEst(2),Map(2,iFeature)]);
        end
        drawnow;
    end
    
    %store results
    InnovStore(:,k)=Innov;
    SStore(:,k)=sqrt(diag(S));
    PStore(:,k)=sqrt(diag(PEst));
    XStore(:,k)=xEst;
    XErrStore(:,k)=xTrue-xEst;
end
DoGraphs(InnovStore,PStore,SStore,XStore,XErrStore);


        
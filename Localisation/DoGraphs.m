function DoGraphs(InnovStore,PStore,SStore,XStore,XErrStore)
figure(1); print -depsc 'EKFLocation.eps'

figure(2);
subplot(2,1,1); plot(InnovStore(1,:));hold on;plot(SStore(1,:),'r');plot(-SStore(1,:),'r'); title('Innovation'); ylabel('range');
subplot(2,1,2); plot(180/pi*InnovStore(2,:));hold on;plot(180/pi*SStore(2,:),'r');plot(-180/pi*SStore(2,:),'r'); xlabel('Time'); ylabel('Bearing_deg');
print -depsc 'EKFLocationInnov.eps'

figure(3);
subplot(3,1,1); plot(XErrStore(1,:));hold on;plot(3*PStore(1,:),'r');plot(-3*PStore(1,:),'r');
title('Covariance and Error');ylabel('x');
subplot(3,1,2); plot(XErrStore(2,:));hold on;plot(3*PStore(2,:),'r');plot(-3*PStore(2,:),'r');
ylabel('y');
subplot(3,1,3); plot(180/pi*XErrStore(3,:));hold on;plot(3*180/pi*PStore(3,:),'r');plot(-3*180/pi*PStore(3,:),'r');
ylabel('Theta');xlabel('time')
print -depsc 'EKFLocationErr.eps'
end

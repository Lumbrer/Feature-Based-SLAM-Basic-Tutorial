function DoMapGraphics(xMap,PMap,nSigma)
persistent k;

if(isempty(k))
    k = 0;
end;
k = k+1;

colors = 'kkkk';
for(i = 1:length(xMap)/2)
    iL = 2*i-1; iH = 2*i;
    x = xMap(iL:iH);
    P = PMap(iL:iH,iL:iH);    
    h = PlotEllipse(x,P,nSigma,k);  
    c = colors(mod(i,4)+1);
    set(h,'color',char(c));
   % plot3(x(1),x(2),k,'r+');
end;
end

%% Function to handle the transformation of angle
function angle = AngleWrap(angle)

if(angle>pi)
    angle=angle-2*pi;
elseif(angle<-pi)
    angle = angle+2*pi;
end;
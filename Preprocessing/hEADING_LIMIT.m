
H=ans_EKF_noAd.Output.ESKF.O_corrected(:,9)*180/pi;
d=length(H);
% limit(-180,180)
for i=1:d
    if H(i)>=180
        h(i)=H(i)-360;
    elseif H(i)<=-180
        h(i)=H(i)+360;
    else
        h(i)=H(i);
    end
end

H=ans_AHRS112509_LargeOutlier_rejected_2.Output.ESKF.O_corrected(:,9);
H=ans_AHRS112509_LargOutlier_noReject_2.Output.ESKF.O_corrected(:,9)*180/pi;
d=length(H);
%limit(0,360)
for i=1:d
    if H(i)<-pi
        h2(i)=H(i)+2*pi;
    elseif H(i)>pi
        h2(i)=H(i)-2*pi;
    else 
        h2(i)=abs(H(i));
    end
end
for i=1:d
if h1(i)==0
    h1(i)=360;
end
end

for i=1:d
    if H(i)<0
        h2(i)=H(i)+360;
    elseif H(i)>5
        h2(i)=H(i);
    elseif H(i)<=5 && H(i)>=0
        h2(i)=360-H(i);
    end
end

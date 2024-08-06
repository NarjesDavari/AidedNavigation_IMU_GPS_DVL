%Direction cosine matrix expressed in terms of Euler angles.
%Reference : Strapdown inertial navigation system (Chapter 3, page 41)
%            My thesis page 48
function CBN = InCBN( euler )
    
for i=1:length(euler(:,1))
        roll  = euler(i,1);
        pitch = euler(i,2);
        yaw   = euler(i,3);
       
        CBN(1,1,i) =  cos(pitch)*cos(yaw);
        CBN(1,2,i) = -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
        CBN(1,3,i) =  sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
        CBN(2,1,i) =  cos(pitch)*sin(yaw);
        CBN(2,2,i) =  cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
        CBN(2,3,i) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
        CBN(3,1,i) = -sin(pitch);
        CBN(3,2,i) =  sin(roll)*cos(pitch);
        CBN(3,3,i) =  cos(roll)*cos(pitch);
end
end
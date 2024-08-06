%Direction cosine matrix expressed in terms of Euler angles.
%Reference : Strapdown inertial navigation system (Chapter 3, page 41)
%            My thesis page 48
function CBN = InCBN( euler )
    
        roll  = euler(1);
        pitch = euler(2);
        yaw   = euler(3);
       
        CBN(1,1) =  cos(pitch)*cos(yaw);
        CBN(1,2) = -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
        CBN(1,3) =  sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
        CBN(2,1) =  cos(pitch)*sin(yaw);
        CBN(2,2) =  cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
        CBN(2,3) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
        CBN(3,1) = -sin(pitch);
        CBN(3,2) =  sin(roll)*cos(pitch);
        CBN(3,3) =  cos(roll)*cos(pitch);
end
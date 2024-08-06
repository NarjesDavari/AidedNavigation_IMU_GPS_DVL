%Direction cosine matrix expressed in terms of Euler angles.
%Reference : Strapdown inertial navigation system (Chapter 3, page 41)
%            My thesis page 48
function C = Gamma(euler)

    
        roll  = euler(1);
        pitch = euler(2);
        yaw   = euler(3);
       
        C =  [cos(pitch)*cos(yaw), -sin(yaw), 0
              cos(pitch)*sin(yaw), cos(yaw) , 0
              -sin(pitch)        , 0        , 1];

end
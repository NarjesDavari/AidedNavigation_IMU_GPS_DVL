function  CBN = InCBN_quater (Quaternion)

q1 = Quaternion (1);
q2 = Quaternion (2);
q3 = Quaternion (3);
q4 = Quaternion (4);

        CBN(1,1) =  q1^2 + q4^2 - q2^2 - q3^2;
        CBN(1,2) = 2*(q1*q2 - q3*q4);
        CBN(1,3) = 2*(q4*q2 + q3*q1);
        CBN(2,1) = 2*(q1*q2 + q3*q4);
        CBN(2,2) = q2^2 + q4^2 - q1^2 - q3^2;
        CBN(2,3) = 2*(q3*q2 - q1*q4);
        CBN(3,1) = 2*(q3*q1 - q2*q4);
        CBN(3,2) =  2*(q1*q4 + q2*q3);
        CBN(3,3) =  q4^2 + q3^2 - q1^2 - q2^2;
end
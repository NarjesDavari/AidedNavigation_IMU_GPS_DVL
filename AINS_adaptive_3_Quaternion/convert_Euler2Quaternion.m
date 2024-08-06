function q = convert_Euler2Quaternion (Euler)

SPhi      = sin(Euler(1)/2);
CPhi      = cos(Euler(1)/2);
STheta      = sin(Euler(2)/2);
CTheta      = cos(Euler(2)/2);
SPsi      = sin(Euler(3)/2);
CPsi      = cos(Euler(3)/2);

q1 = SPhi *  CTheta * CPsi + SPsi * CPhi * STheta;
q2 = CPhi * STheta  * CPsi - SPhi * CTheta * SPsi;
q3 = CPhi * CTheta  * SPsi + SPhi * STheta * CPsi;
q4 = CPhi * CTheta *  CPsi - SPhi * STheta * SPsi;

q = [q1,q2,q3,q4];
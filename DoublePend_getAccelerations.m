function [Accl,alpha] = DoublePend_getAccelerations(d2ang,Pos,omega)

d2angA = d2ang(1,1);
d2angB = d2ang(2,1);

alpha.A = [0;0;d2angA];
alpha.B = [0;0;d2angB];

Accl.AO = cross(alpha.A,Pos.AO) + cross(omega.A,cross(omega.A,Pos.AO));
Accl.AB = cross(alpha.A,Pos.AB) + cross(omega.A,cross(omega.A,Pos.AB));

Accl.BO = Accl.AB + cross(alpha.B,Pos.BO-Pos.AB) + cross(omega.B,cross(omega.B,Pos.BO-Pos.AB));

end

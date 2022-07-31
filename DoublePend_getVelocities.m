function [Vel,omega] = DoublePend_getVelocities(dang,Pos)

dangA = dang(1,1);
dangB = dang(2,1);

omega.A = [0;0;dangA];
omega.B = [0;0;dangB];
Vel.AO = cross(omega.A,Pos.AO-Pos.NA);
Vel.AB = cross(omega.A,Pos.AB-Pos.NA);
Vel.BO = Vel.AB + cross(omega.B,Pos.BO-Pos.AB);

end

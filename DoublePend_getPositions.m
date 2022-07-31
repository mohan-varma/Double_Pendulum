function Pos = DoublePend_getPositions(ang)
global localvec

angA = ang(1,1);
angB = ang(2,1);

    N_A = [cos(angA) -sin(angA) 0;
           sin(angA)  cos(angA) 0;
           0 0 1];

    N_B = [cos(angB) -sin(angB) 0;
           sin(angB)  cos(angB) 0;
           0 0 1];

    Pos.NA = [0;0;0];
    Pos.AO = -N_A*localvec.AO_AN;
    Pos.AB = Pos.AO + N_A*localvec.AO_AB;
    Pos.BO = Pos.AB + N_B*(-localvec.BO_AB);
    Pos.BC = Pos.BO + N_B*(localvec.BO_BC);

end

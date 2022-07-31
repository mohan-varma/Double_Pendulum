function dyvar = DoublePend_Kane(~,yvar)

global Iner mass FG Torq

angA = yvar(1,1);
angB = yvar(2,1);
dangA = yvar(3,1);
dangB = yvar(4,1);

dyvar(1,1) = dangA;
dyvar(2,1) = dangB;

ang = [angA;angB];
Pos = DoublePend_getPositions(ang);

% Generalized coordinate 1: dangA
for ii = 1:2
    dang_temp = [0;0];
    dang_temp(ii,1) = 1;
    [CVel,Comg] = DoublePend_getVelocities(dang_temp,Pos);

    for fn = fieldnames(Comg)'
        omega.(fn{1}) = dangA*Comg.(fn{1});
    end

    d2ang_temp = [0;0];
    [Accl_k,~] = DoublePend_getAccelerations(d2ang_temp,Pos,omega);

    LHS = Torq.A'*Comg.A + FG.A'*CVel.AO + (-mass.A*Accl_k.AO)'*CVel.AO;
    LHS = LHS + Torq.B'*Comg.B + FG.B'*CVel.BO + (-mass.B*Accl_k.BO)'*CVel.BO;
    CoeffRHS = (Iner.A*Comg.A)'*Comg.A + mass.A*CVel.AO'*CVel.AO;
    CoeffRHS = CoeffRHS + (Iner.B*Comg.B)'*Comg.B + mass.B*CVel.BO'*CVel.BO;

    d2ang = LHS/CoeffRHS;

    dyvar(ii+2,1) = d2ang;

end

end

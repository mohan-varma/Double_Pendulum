% Forward Dynamics using Kane's Equation

clear all
clc
close all
global localvec Iner mass FG Torq


mass.A = 5;
mass.B = 10;
lengthA = 5;
lengthB = 5;

Iner.A(3,3) = (mass.A*lengthA^2)/12;
Iner.B(3,3) = (mass.B*lengthB^2)/12;

localvec.AO_AN = [0;0.5*lengthA;0];
localvec.AO_AB = [0;-0.5*lengthA;0];
localvec.BO_AB = [0;0.5*lengthB;0];
localvec.BO_BC = [0;-0.5*lengthB;0];

FG.A = [0;-9.81;0]*mass.A;
FG.B = [0;-9.81;0]*mass.B;
T1 = -50;
T2 = -25;
T1 = 0;
T2 = 0;
Torq.A = [0;0;T1-T2];
Torq.B = [0;0;T2];

% Initial Values
angA0 = deg2rad(45); %rad
dangA0 = 0; % rad/sec
angB0 = deg2rad(70); %rad
dangB0 = 0; % rad/sec

initvalues = [angA0;angB0;dangA0;dangB0];
[T,Y] = ode45(@DoublePend_Kane,[0 10],initvalues);

% Inverse dynamics to find reaction forces
[lenT,~] = size(T);
for ii = 1:length(T)

    angA = Y(ii,1);
    angB = Y(ii,2);
    dangA = Y(ii,3);
    dangB = Y(ii,4);
    DY = DoublePend_Kane(T(ii),Y(ii,:)');
    d2angA = DY(3,1);
    d2angB = DY(4,1);
    Y(ii,5) = d2angA;
    Y(ii,6) = d2angB;
    Pos = DoublePend_getPositions([angA;angB]);
    [Vel,omega] = DoublePend_getVelocities([dangA;dangB],Pos);
    [Accl,alpha] = DoublePend_getAccelerations([d2angA;d2angB],Pos,omega);

    FAB(:,ii) = mass.B*Accl.BO - FG.B;
    FAN(:,ii) = FAB(:,ii) + mass.A*Accl.AO - FG.A;

    linexy = [[0;0;0] Pos.AB Pos.BC];

    figure(1)
    plot(linexy(1,:),linexy(2,:),'*')
    hold on
    plot(linexy(1,:),linexy(2,:),'-')
    hold off
    axis([-10 10 -10 10])
    pause(0.01)

end

%
figure(2)
subplot(311)
plot(T,Y(:,1))
subplot(312)
plot(T,Y(:,2))
subplot(313)
plot(T,Y(:,3))

% figure(3)
% plot(T,Tor(3,:))
save('DoublePend_FwdDyn_Data.mat','T','Y')

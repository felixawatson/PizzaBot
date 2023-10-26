clc
close all
hold on;

PlaceObject('environment.ply');

steps = 100;
scoopoffset = [0,-0.3,0.05];
eescoopoffset = [-0.05,0,0.3];
I = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];

% Movement Variables
pizbasepickup = [1,-1,0.5] + scoopoffset;
basepickup = [1,0,0,1; 0,1,0,-1.3; 0,0,1,0.55; 0,0,0,1];
disp1 = [0.5,-1.35,0.5] + scoopoffset; 
disp2 = [0.17,-1.35,0.5] + scoopoffset;
disp3 = [-0.21,-1.35,0.5] + scoopoffset;
disp4 = [-0.57,-1.35,0.5] + scoopoffset;
cutpos = [-0.9,-1,0.5] + scoopoffset; 
oven = [1.3,0.1,0.7] + scoopoffset;
conveyor = [-1.17,-0.5,0.51]  + scoopoffset;

ur3base = [-1.15,-1.33,0.5]; % approx
ur3home = deg2rad([-120,-90,60,-60,-90,0]);
IRBhome = [-pi/2 0 pi/2 0 0 0];

% IRB init
IRB = IRB1200H;
IRB.model.base = [0,-0.4,0.2];
% IRB.model.teach();
IRB.model.animate(IRBhome);
CHEFee = IRB.model.fkine(IRB.model.getpos);
gripper = scoop;
gripper.model.base = CHEFee;
gripper.model.animate(0);

% UR3 init
ur3 = UR3;
ur3.model.base = ur3base;
ur3.model.animate(ur3home);
SLICEee = ur3.model.fkine(ur3.model.getpos);
% cut = cutter;
% cut.model.base = SLICEee;
% cut.model.plot(); %gripper not working for some reason??

%% DEMONSTRATION
one = Pizza(pizbasepickup-scoopoffset,'Base');

q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(basepickup); %make it a TR
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    drawnow()
end

%pickup base, go to disp1
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(disp3); 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    drawnow()
end
one.model.base = CHEFee + eescoopoffset;


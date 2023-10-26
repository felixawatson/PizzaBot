clc
close all
hold on;

IRB = IRB1200H;
home = [0 0 pi/2 0 0 0];
IRB.model.teach();
IRB.model.animate(home);

ee = IRB.model.fkine(IRB.model.getpos);
gripper = scoop;
gripper.model.base = ee;
gripper.model.animate(0);
%PlaceObject('scoop.ply',ee);

<<<<<<< HEAD
clc
close all
hold on;

IRB = UR3;
home = [0 0 pi/2 0 0 0];
IRB.model.teach();
IRB.model.animate(home);

ee = IRB.model.fkine(IRB.model.getpos);
ee.t = ee.t - [0,0,-0.01]';
gripper = cutter;
gripper.model.base = ee;
gripper.model.animate(0);
%PlaceObject('scoop.ply',ee);

=======
p = Pizza;
>>>>>>> 9d5d34337baef52656b906a9c2b6b3b9d48c1159

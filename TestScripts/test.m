clc
close all
hold on;

PlaceObject('environment.ply');

steps = 100;
scoopoffset = [0,0.35,0.05];
eescoopoffset = transl([-0.05,0,0.35]);

% Movement Variables
basepickup = transl([0.45,-0.45,0.4] + [-0.24,0.24,0.05]) * trotz(pi/4); % coming at the base at a corner
basepos = transl(0.6,-0.7,0.4);
disp1 = transl([0.3,-1,0.4] + scoopoffset); 
disp2 = transl([0,-1,0.4] + scoopoffset);
disp3 = transl([-0.3,-1,0.4] + scoopoffset);
disp4 = transl([-0.6,-1,0.4] + scoopoffset);
cutpos = transl([-0.8,-0.7,0.4] + [0.24,0.24,0.05]) * trotz(-pi/4); 
oven = transl([0.8,0,0.55] + [-0.35,0,0.05])*trotz(pi/2);
conveyor = transl([-1.1,-0.3,0.41]  + [0.35,0,0.05])*trotz(-pi/2);

ur3base = [-1,-1,0.4]; % approx
ur3home = deg2rad([-120,-90,60,-60,-90,0]);
IRBhome = [-pi/2 -pi/4 3*pi/4 0 0 0];

% IRB init
IRB = IRB1200H;
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
cut = cutter;
cut.model.base = SLICEee;
cut.model.animate(0); %gripper not working for some reason??

%% DEMONSTRATION
makeflat = trotx(pi/2)*troty(0)*trotz(pi/2);

one = Pizza(basepos,'Base');

% home pos
q1 = IRB.model.getpos;
q2 = IRBhome; %make it a TR
qMatrix = jtraj(q1,q2,steps);    
for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    drawnow()
end

% goes to way point for base pickup
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(basepickup*makeflat,q1); %make it a TR
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    drawnow()
end

% cart move to pickup base
q1 = IRB.model.getpos;
tf1 = IRB.model.fkine(q1).T;
basecartmove = tf1;
basecartmove(1:3,4) = (basepos(1:3,4) + [-0.24,0.24,0.051]');

tfMatrix = ctraj(tf1,basecartmove,steps);   

for i = 1:steps
    if i == 1
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos).T;  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        drawnow()
    else 
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos).T;  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        drawnow()
    end
end

% pickup base, go to disp1
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(disp3*makeflat,q1); 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    one.model.base = CHEFee * transl(eescoopoffset);
    one.model.animate(0)
    drawnow()
end

% add toppings, way point 1
basetr = one.model.base.T;
pizza = Pizza(basetr,'Four Cheeses');
delete(one)

q1 = IRB.model.getpos;
tf1 = IRB.model.fkine(q1).T;
transform = tf1;
transform(1:3,4) = (transform(1:3,4) + [0,0.3,0]');

tfMatrix = ctraj(tf1,transform,steps);   

for i = 1:steps
    if i == 1
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos);  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        pizza.model.base = CHEFee * transl(eescoopoffset);
        pizza.model.animate(0)
        drawnow()
    else 
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos);  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        pizza.model.base = CHEFee * transl(eescoopoffset);
        pizza.model.animate(0)
        drawnow()
    end
end

% way point 2
q1 = IRB.model.getpos;
q2 = [0 -pi/4 3*pi/4 0 0 0];  
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% way point 3
q1 = IRB.model.getpos;
q2 = [pi/2 -pi/4 3*pi/4 0 0 0]; 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% move into oven
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(oven*makeflat ,q1); 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% LET HIMMM COOK
for k = 1:3
    q1 = IRB.model.getpos;
    tf1 = IRB.model.fkine(q1).T;
    cartmove = tf1;
    if k == 2
        cartmove(1:3,4) = (cartmove(1:3,4) + [0.3,0,0]');
    else
        cartmove(1:3,4) = (cartmove(1:3,4) + [-0.3,0,0]');
    end

    tfMatrix = ctraj(tf1,cartmove,steps);   
    
    for i = 1:steps
        if i == 1
            qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
            lastpos = qMatrix(i,:);
            IRB.model.animate(qMatrix(i,:))
            CHEFee = IRB.model.fkine(IRB.model.getpos);  
            gripper.model.base = CHEFee;
            gripper.model.animate(0);
            if k == 3
                pizza.model.base = CHEFee * transl(eescoopoffset);
                pizza.model.animate(0)
            end
            drawnow()
        else 
            qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
            lastpos = qMatrix(i,:);
            IRB.model.animate(qMatrix(i,:))
            CHEFee = IRB.model.fkine(IRB.model.getpos);  
            gripper.model.base = CHEFee;
            gripper.model.animate(0);
            if k == 3
                pizza.model.base = CHEFee * transl(eescoopoffset);
                pizza.model.animate(0)
            end
            drawnow()
        end
    end
    if k == 1
        pause(5);
    end
end

% way point
q1 = IRB.model.getpos;
q2 = [0 -pi/4 3*pi/4 0 0 0];  
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% go to cutpos
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(cutpos*makeflat,q1); 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% cut
pause(5)

% got to conveyor 
q1 = IRB.model.getpos;
q2 = IRB.model.ikcon(conveyor*makeflat,q1); 
qMatrix = jtraj(q1,q2,steps);    

for i = 1:steps
    IRB.model.animate(qMatrix(i,:));
    %animate gripper
    CHEFee = IRB.model.fkine(IRB.model.getpos);  
    gripper.model.base = CHEFee;
    gripper.model.animate(0);
    pizza.model.base = CHEFee * transl(eescoopoffset);
    pizza.model.animate(0)
    drawnow()
end

% place on conveyor
q1 = IRB.model.getpos;
tf1 = IRB.model.fkine(q1).T;
transform = tf1;
transform(1:3,4) = (transform(1:3,4) + [0.5,0,0]');

tfMatrix = ctraj(tf1,transform,steps);   

for i = 1:steps
    if i == 1
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos);  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        drawnow()
    else 
        qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
        lastpos = qMatrix(i,:);
        IRB.model.animate(qMatrix(i,:))
        CHEFee = IRB.model.fkine(IRB.model.getpos);  
        gripper.model.base = CHEFee;
        gripper.model.animate(0);
        drawnow()
    end
end

% move pizza along and delete, print 'pizza delivered' or some shit
q1 = pizza.model.getpos;
tf1 = pizza.model.fkine(q1).T;
transform = tf1;
transform(1:3,4) = (transform(1:3,4) + [0,2.2,0]');

tfMatrix = ctraj(tf1,transform,30);   

for i = 1:30
        pizza.model.base = tfMatrix(:,:,i);
        pizza.model.animate(0)
        drawnow()
end
delete(pizza);
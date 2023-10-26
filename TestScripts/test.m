clc
close all
hold on;
% IRB = IRB1200H;
% home = [0 0 pi/2 0 0 0];
% IRB.model.teach();
% IRB.model.animate(home);


PizzaBot('base',1,[0.5,0.5,1]);

% PizzaBot('fourcheeses',1,[0.65,0.5,1]);
pos = [0,0,0];
for i = 1:100

pizza = PlaceObject('fourcheeses.ply',pos);
pos = pos + [0.1,0,0];
pause(0.1)
delete(pizza);

end
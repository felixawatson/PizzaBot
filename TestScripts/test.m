clc
close all
hold on;
IRB = IRB1200H;
home = [0 0 pi/2 0 0 0];
IRB.model.teach();
IRB.model.animate(home);

PlaceObject('fourcheeses.ply',[0.5,0.5,0])



% %%
% step = 50;
% distance = 0.1;
% 
% q1 = IRB.model.getpos;
% tf = IRB.model.fkine(q1).T;
% jog = tf;
% 
% direction = '+x';
% 
% switch direction
%     case '-x'
%         jog(1,4) = jog(1,4) - distance;
%     case '+x'
%         jog(1,4) = jog(1,4) + distance;
%     case '-y'
%         jog(2,4) = jog(2,4) - distance;
%     case '+y'
%         jog(2,4) = jog(2,4) + distance;
%     case '-z'
%         jog(3,4) = jog(3,4) - distance;
%     case '+z'
%         jog(3,4) = jog(3,4) + distance;
%     otherwise
%         return
% end
% 
% tfMatrix = ctraj(tf,jog,step);    
% for i = 1:step
%     if i == 1
%         qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
%         lastpos = qMatrix(i,:);
%         IRB.model.animate(qMatrix(i,:))
%         drawnow()
%     else 
%         qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
%         lastpos = qMatrix(i,:);
%         IRB.model.animate(qMatrix(i,:))
%         drawnow()
%     end
% end
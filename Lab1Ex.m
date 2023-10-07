close all;
clc

imshow('Lab1CircularRaceTrack.jpg');
axis on
hold on;

BLUE = SE2(300, 550, 0).T;
BLUE_h = trplot2(BLUE, 'frame', '1', 'color', 'b', 'length', 50);

RED = SE2(300, 125, 0).T;
RED_h = trplot2(RED, 'frame', '2', 'color', 'r', 'length', 50);

%Track is dimensionally 480 pixels and 360 degrees of rotation
totalSteps = 360;

BLUE_MOVE = SE2((pi * 484)/totalSteps, 0, 0).T;
BLUE_TURN = SE2(0, 0, -2*pi/totalSteps).T;

RED_MOVE = SE2((pi * 375)/totalSteps, 0, 0).T;
RED_TURN = SE2(0, 0, 2*pi/totalSteps).T;

for i = 1:360
    BLUE = BLUE * BLUE_MOVE * BLUE_TURN;
    try delete(BLUE_h);end
    BLUE_h = trplot2(BLUE, 'frame', '1', 'color', 'b', 'length', 50);
    drawnow();

    RED = RED * RED_MOVE * RED_TURN;
    try delete(RED_h);end
    RED_h = trplot2(RED, 'frame', '2', 'color', 'r', 'length', 50);
    drawnow();
end
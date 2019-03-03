clear all;
close all;

[Z1, refvec1, UHL1, DSI1, ACC1] = dted('n39_w122_1arc_v3.dt2');
[Z2, refvec2, UHL2, DSI2, ACC2] = dted('n39_w121_1arc_v3.dt2');
[Z3, refvec3, UHL3, DSI3, ACC3] = dted('n40_w122_1arc_v3.dt2');
[Z4, refvec4, UHL4, DSI4, ACC4] = dted('n40_w121_1arc_v3.dt2');


Z = [Z1(1:end-1,1:end-1) Z2(1:end-1,:); Z4(:,1:end-1) Z3];

csvwrite('altidata.csv',Z)
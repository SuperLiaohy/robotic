clc;
L(1)=Link([0 0 0 0],'modified');
L(2)=Link([0 0 0 -pi/2],'modified');
L(2).offset = -pi/2;
L(3)=Link([0 0 320 0],'modified');
L(3).offset = pi/2;
L(4)=Link([0 325.5 0 pi/2],'modified');
L(5)=Link([0 0 0 -pi/2],'modified');
L(6)=Link([0 122 0 pi/2],'modified');
Robot1 = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);
Robot1.display();
f1 = figure(1);
Robot1.teach('eul');

L(6)=Link([0 0 0 pi/2],'modified');
Robot2 = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);
f2 = figure(2);
Robot2.teach('eul');

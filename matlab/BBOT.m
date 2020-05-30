%% BBOT FK Model and IK Test
clear all;
close all;
clc;

syms pi
syms t1 t2 t3 t4 t5 t6 a2 a3 d4 d6 real

%%%%%%% Run for numeric values
clear pi
a2 = .105;
a3 = .134;
d4 = .0625;
%%%%%%%

q = [t1, t2, t3, t4, t5, t6];
% q = [0,0,0,0,0,0];
L1 = Link('revolute','a',0,'alpha',0,'d',0,'modified');
L2 = Link('revolute','a',0,'alpha',pi/2,'d',0,'modified');
L3 = Link('revolute','a',a2,'alpha',0,'d',0,'modified');
L4 = Link('revolute','a',a3,'alpha',0,'d',d4,'modified');
L5 = Link('revolute','a',0,'alpha',pi/2,'d',0,'modified');
L6 = Link('revolute','a',0,'alpha',pi/2,'d',0,'modified');

bbot = SerialLink([L1, L2, L3, L4, L5, L6],'name','BBOT 2.0');

T01 = simplify(L1.A(t1));
T12 = simplify(L2.A(t2+(pi/2)));
T23 = simplify(L3.A(t3));
T34 = simplify(L4.A(t4));
T45 = simplify(L5.A(t5+(pi/2)));
T56 = simplify(L6.A(t6));

T06 = simplify(T01*T12*T23*T34*T45*T56);

T10 = inv(T01);
T16 = simplify(T12*T23*T34*T45*T56);

T20 = simplify(inv(T12)*inv(T01));
T26 = simplify(T23*T34*T45*T56);

T30 = simplify(inv(T23)*inv(T12)*inv(T01));
T36 = simplify(T34*T45*T56);

%% Test FK
ang1 = 0;
ang2 = -pi/4;
ang3 = -pi/2; 
ang4 = pi/4;
ang5 = 0;
ang6 = 0;
T02 = simplify(L1.A(ang1)*L2.A(ang2+(pi/2)));
T03 = simplify(T02*L3.A(ang3));
T04 = simplify(T03*L4.A(ang4));
T05 = simplify(T04*L5.A(ang5+(pi/2)));
T06 = simplify(T05*L6.A(ang6))

aa2 = .105;
aa3 = .134;
dd4 = 0.0625;

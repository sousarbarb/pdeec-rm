close all
clear all
clc

%% INITIALIZATION
% Joints
syms th1 th2 th3 th4 th5 th6
% Lengths
syms l1 l2 l3 lt

%% FORWARD KINEMATICS: DH CONVENTION

H10 = DH(th1     ,-l1,0 ,pi/2)
H21 = DH(th2     , 0 ,l2,0   )
H32 = DH(th3+pi/2, 0 ,0 ,pi/2)
H43 = DH(th4+pi/2, l3,0 ,pi/2)
H54 = DH(th5+pi  , 0 ,0 ,pi/2)
H65 = DH(th6     , lt,0 ,0   )

H60 = H10 * H21 * H32 * H43 * H54 * H65

%% TEST

% Test 1
Htest = double(                                                         ...
  subs(H60                                                            , ...
       [th1   th2   th3   th4   th5   th6   l1    l2    l3    lt    ] , ...
       [0     0     0     0     0     0     0.1   0.2   0.3   0.05  ])  ...
)
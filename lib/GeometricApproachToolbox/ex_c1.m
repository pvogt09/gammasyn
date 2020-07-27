%EX_C1    System matrices (continuous-time)

A=[-.4 -4.04 0; 1 0 0; 0 0 -1];
B=[2 0; .4 1; 0 1.5];
C=[0 1 0; 0 0 1];

% Eigenvalues to be assigned (plant and observer)

P1=[-1-j;-1+j;-2;-2.5;-3.5];
P2=[-1-2*j;-1+2*j;-.8;-1.5;-3;-6;-7;-8;-10;-12;-20;-25;-30;-35];

% Continuous/discrete time

dis=0;
%EX_D1    System matrices (discrete-time)

A=[-.4 -4.04 0; 1 0 0; 0 0 -1];
B=[2 0; .4 1; 0 1.5];
C=[0 1 0; 0 0 1];
[A,B]=c2d(A,B,.2);

% Eigenvalues to be assigned (plant and observer)

P1=[-1-j;-1+j;-2;-2.5;-3.5];
P2=[-1-2*j;-1+2*j;-.8;-1.5;-3;-6;-7;-8;-10;-12];
P1=exp(.2*P1); P2=exp(.2*P2);

% Continuous/discrete time

dis=1;
% problems.m
% ---------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [] = problems(nprob)                                             %
% This function lists the respective information associated with the test   %
% function number NPROB. The files could be ascessed by just typing the     %
% filename with the extension "m", e.g. "rose.m". The numbers in parenthesis%
% mean that the files include parameters for varying the dimension of the   %
% problem.                                                                  %
%                                                                           %   
% Created by Madhu Lamba on 12/19/94                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

function [] = problems(nprob)

disp('No.    file      n    m   Name'); 
disp('---    ----      -    -   ----');

if (nprob==1)
disp('#  1.  ROSEN     2    2   Rosenbrock')
elseif (nprob==2)
 	disp('#  2.  FROTH     2    2   Freudenstein and Roth')
elseif (nprob==3)
	disp('#  3.  BADSCP    2    2   Powell Badly Scaled')
elseif (nprob==4)
	disp('#  4.  BADSCB    2    3   Brown Badly Scaled')
elseif (nprob==5)
	disp('#  5.  BEALE     2    3   Beale')
elseif (nprob==6)
	disp('#  6.  JENSAM    2   10   Jennrich and Sampson')
elseif (nprob==7)
	disp('#  7.  HELIX     3    3   Helical Valley')
elseif (nprob==8)
	disp('#  8.  BARD      3   15   Bard')
elseif (nprob==9)
	disp('#  9.  GAUSS     3   15   Gaussian')
elseif (nprob==10)
	disp('# 10.  MEYER     3   16   Meyer')
elseif (nprob==11)
	disp('# 11.  GULF      3   10   Gulf Research and Development')
elseif (nprob==12)
	disp('# 12.  BOX       3   10   Box 3-Dimensional')
elseif (nprob==13)
	disp('# 13.  SING      4    4   Powell Singular')
elseif (nprob==14)
	disp('# 14.  WOOD      4    6   Wood')
elseif (nprob==15)
	disp('# 15.  KOWOSB    4   11   Kowalik and Osborne')
elseif (nprob==16)
	disp('# 16.  BD        4   20   Brown and Dennis')
elseif (nprob==17)
	disp('# 17.  OSB1      5   33   Osborne 1')
elseif (nprob==18)
	disp('# 18.  BIGGS     6   13   Biggs EXP6')
elseif (nprob==19)
	disp('# 19.  OSB2     11   65   Osborne 2')
elseif (nprob==20)
	disp('# 20.  WATSON  (20)  31   Watson')
elseif (nprob==21)
	disp('# 21.  ROSEX   (10) (10)  Extended Rosenbrock')
elseif (nprob==22)
	disp('# 22.  SINGX   (10) (10)  Extended Powell Singular')
elseif (nprob==23)
	disp('# 23.  PEN1    ( 4) ( 5)  Penalty I')
elseif (nprob==24)
	disp('# 24.  PEN2    ( 4) ( 8)  Penalty II')
elseif (nprob==25)
	disp('# 25.  VARDIM  (10) (12)  Variably Dimensioned')
elseif (nprob==26)
	disp('# 26.  TRIG    (10) (10)  Trigonometric')
elseif (nprob==27)
	disp('# 27.  ALMOST  (10) (10)  Brown Almost Linear')
elseif (nprob==28)
	disp('# 28.  BV      (10) (10)  Discrete Boundary Value')
elseif (nprob==29)
	disp('# 29.  IE      (10) (10)  Discrete Integral Equation')
elseif (nprob==30)
	disp('# 30.  TRID    (10) (10)  Broyden Tridiagonal')
elseif (nprob==31)
	disp('# 31.  BAND    (10) (10)  Broyden Banded')
elseif (nprob==32)
	disp('# 32.  LIN     (10) (20)  Linear --- Full Rank')
elseif (nprob==33)
        disp('# 33.  LIN1    (10) (20)  Linear --- Rank 1')
elseif (nprob==34)
      disp('# 34.  LIN0    (10) (20)  Linear - Rank 1 with Zero Cols. & Rows')
elseif (nprob==35)
        disp('# 35. CHEB     (10) (10)  Chebyquad')
else    disp('The nprob asked is not in this list. (only from 1 to 35).')
end;

% 

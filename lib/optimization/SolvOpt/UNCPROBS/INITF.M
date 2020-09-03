% initf 
% ----- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [n,m,x0]=initf(nprob)
% This function sets n,m, and the standard starting    
% point based on the nprob and returns it to initpt     
% function.                                             
%                                                       
% Created on 10/30/94 by Madhu Lamba                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [n,m,x0] = initf(nprob)
global FIRSTIME;

% ROSE 
if nprob==1
        n=2;
        m=2;
        x0=[-1.2,1]';

% FROTH
elseif nprob==2
        n=2;
        m=2;
        x0=[0.5,-2]';

% BADSCP
elseif nprob==3
        n=2;
        m=2;
        x0=[0,1]';

% BADSCB
elseif nprob==4
        n=2;
        m=3;
        x0=[1,1]';

% BEALE
elseif nprob==5
        n=2;
        m=3;
        x0=[1,1]';

% JENSAM
elseif nprob==6
        n=2;
%       m=input('Enter a number for m (>=2) :')
        m=10;
        x0=[0.3,0.4]';
   
% HELIX
elseif nprob==7
        n=3;
        m=3;
        x0=[-1,0,0]';
   
% BARD
elseif nprob ==8
        n=3;
        m=15;
        x0=[1,1,1]';
   
% GAUSS
elseif nprob ==9
        n=3;
        m=15;
        x0=[0.4,1,0]';
   
% MEYER
elseif nprob == 10
        n=3;
        m=16;
        x0=[0.02,4000,250]';
 
% GULF
elseif nprob ==11
        n=3;
        m=10;
        x0=[5,2.5,0.15]';
   
% BOX
elseif nprob ==12
        n=3;
        m=10;
        x0=[0,10,20]';
    
% SING
elseif nprob == 13
        n=4;
        m=4;
        x0=[3,-1,0,1]';
   
% WOOD
elseif nprob == 14
        n=4;
        m=6;
        x0=[-3,-1,-3,-1]';
   
% KOWOSB
elseif nprob == 15
        n=4;
        m=11;
        x0=[0.25,0.39,0.415,0.39]';
  
% BD   
elseif nprob == 16
        n=4;
        m=20;
        x0=[25,5,-5,-1]';
   
% OSB1
elseif nprob == 17
        n=5;
        m=33;
	FIRSTIME=1;
        x0=[0.5,1.5,-1,0.01,0.02]';
  
% BIGGS
elseif nprob == 18
        n=6;
        m=13;
        x0=[1,2,1,1,1,1]';
  
% OSB2
elseif nprob == 19
        n=11;
        m=65;
	FIRSTIME=1;
        x0=[1.3,0.65,0.65,0.7,0.6,3,5,7,2,4.5,5.5]';
  
% WATSON
elseif nprob == 20
%  n =input('Enter n (>=2 and <=31) :');
        n=9;
        m=31;
        x0=zeros(n,1);

% ROSEX
elseif nprob==21
%        n=input('Enter an even number for n ?')
        n=10;
        m=n; 
        for j=1:n/2
            x0(2*j-1)=-1.2;
            x0(2*j)=1;
        end;
	x0=x0';

% SINGX
elseif nprob==22
%        n=input('Enter a multiple of 4 for n ?')
        n=4;
        m=n;
        for j=1:n/4
            x0(4*j-3)=3;
            x0(4*j-2)=-1;
            x0(4*j-1)=0;
            x0(4*j)=1;
        end;
        x0=x0';

% PEN1
elseif nprob==23
%        n=input('Enter a number for n ?')
        n=4;
        m=n;
        for j=1:n
            x0(j)=j;
        end;
        x0=x0';

% PEN2
elseif nprob==24
%        n=input('Enter a number for n ?')
        n=4;
        m=2*n;
        x0=(1/2)*(ones(n,1));

% VARDIM
elseif nprob==25
%        n=input('Enter a number for n ?')
        n=10;
        m=n+2;
        for j=1:n
            x0(j)=1-(j/n);
        end;
        x0=x0';

% TRIG
elseif nprob==26
%        n=input('Enter a number for n ?')
        n=10;
        m=n;
        x0=(1/n)*(ones(n,1));

% ALMOST
elseif nprob==27
%        n=input('Enter a number for n ?')
        n=10;
        m=n;
        x0=(1/2)*(ones(n,1));

% BV
elseif nprob==28
%        n=input('Enter a number for n ?')
        n=10;
        m=n;
        h=1/(n+1);
        for j=1:n
            t(j)=j*h;
            x0(j)=t(j)*(t(j)-1);
        end;
        x0=x0'; 

% IE
elseif nprob==29
%        n=input('Enter a number for n ?')
        n=10;
        m=n;
        h=1/(n+1);
        for j=1:n
            t(j)=j*h;
            x0(j)=t(j)*(t(j)-1);
        end;
        x0=x0';

% TRID
elseif nprob==30
%        n=input('Enter a number for n ?')
        n=10;
        m=n;
        x0=(-1)*(ones(n,1));

% BAND   
elseif nprob ==31
%        n=input('Enter n :');
        n=10;
        m=n;
        x0=(-1)*(ones(n,1));

% LIN   
elseif nprob == 32
%        n=input('Enter n: ');
        n=10;
%        m=input('Enter m ( >= n): ');
        m=20;
        x0=ones(n,1);

% LIN1   
elseif nprob ==33
%  n=input('Enter n: ');
        n=10;
%        m=input('Enter m ( >= n): ');
        m=20;
        x0=ones(n,1);

% LIN0   
elseif nprob==34
%  n=input('Enter n: ');
        n=10;
%        m=input('Enter m ( >= n): ');
        m=20;
        x0=ones(n,1);

end

%

% func 
% ---- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=func(n,m,x,nprob,option)           
% Selects the appropriate test function based on nprob.
%                                                      
% Created on 10/22/94 by Madhu Lamba                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [fvec,J] = func(n,m,x,nprob,opt)

if (nprob==1)
  funcname = 'rosen';
elseif (nprob==2)
        funcname = 'froth';
elseif (nprob==3)
        funcname = 'badscp';
elseif (nprob==4)
        funcname = 'badscb';
elseif (nprob==5)
        funcname = 'beale';
elseif (nprob==6)
        funcname = 'jensam';
elseif (nprob==7)
        funcname = 'helix';
elseif (nprob==8)
        funcname = 'bard';
elseif (nprob==9)
        funcname = 'gauss';
elseif (nprob==10)
        funcname = 'meyer';
elseif (nprob==11)        
        funcname = 'gulf';
elseif (nprob==12)   
        funcname = 'box';
elseif (nprob==13)
        funcname = 'sing';
elseif (nprob==14)
        funcname = 'wood';
elseif (nprob==15)
        funcname = 'kowosb';
elseif (nprob==16)       
        funcname = 'bd';
elseif (nprob==17)
        funcname = 'osb1';
elseif (nprob==18)
        funcname = 'biggs';
elseif (nprob==19)
        funcname = 'osb2';
elseif (nprob==20)
        funcname = 'watson';
elseif (nprob==21)
        funcname = 'rosex';
elseif (nprob==22)
        funcname = 'singx';
elseif (nprob==23)
        funcname = 'pen1';
elseif (nprob==24)
        funcname = 'pen2';
elseif (nprob==25)
        funcname = 'vardim';
elseif (nprob==26)
        funcname = 'trig';
elseif (nprob==27)
        funcname = 'almost';
elseif (nprob==28)
        funcname = 'bv';
elseif (nprob==29)
        funcname = 'ie';
elseif (nprob==30)
        funcname = 'trid';
elseif (nprob==31)
        funcname = 'band';
elseif (nprob==32)        
        funcname = 'lin';
elseif (nprob==33)        
        funcname = 'lin1';
elseif (nprob==34)
        funcname = 'lin0';
end;
[fvec,J] = feval(funcname,n,m,x,opt);
       
%

% Trigonometric function
% ---------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=trig(n,m,x,option)
% Problem no. 26
% Dimensions -> n=variable, m=n
% Standard starting point -> x=(1/n,..,1/n)
% Minima -> f=0 
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = trig(n,m,x,option)  

  zero=0.d0;
  one=1.d0;

  if option==1
      sum1 = zero;
      for i=1:n
        xi   = x(i);
        cxi  = cos(xi);
        sum1  = sum1 + cxi;
        fvec(i) = n + (i)*(one - cxi) - sin(xi);
      end
      fvec=fvec';
      fvec=fvec-sum1;
   
  elseif option==2
      for j=1:n
        xj  = x(j);
        sxj = sin(xj);
        for i=1:n
          J( i, j) = sxj;
        end
        J(j, j) =  (j+1)*sxj - cos(xj);
      end

  elseif option==3
      sum1 = zero;
      for i=1:n
        xi   = x(i);
        cxi  = cos(xi);
        sum1  = sum1 + cxi;
        fvec(i) = n + (i)*(one - cxi) - sin(xi);
      end
      fvec=fvec';
      fvec=fvec-sum1;

      for j=1:n
        xj  = x(j);
        sxj = sin(xj);
        for i=1:n
          J( i, j) = sxj;
        end
        J(j, j) =  (j+1)*sxj - cos(xj);
      end

  else error('Error: trig.m : invalid option')
  end

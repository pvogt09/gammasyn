function [ew, P, index] = sortewpv(ew,P,npq)
% [ew,P,index] = sortewpv(ew,P,npq) sorts the predefined eigenvalues 
% and parametervectors.
%
% OUTPUT:
% ew,P   - sorted eigenvalues and parametervectors
% index  - sorting order
%
% INPUT:
% ew     - vector with predefined eigenvalues
% P      - matrix of parametervectors 
% npq    - dimensions

[ew, index] = sortrows([ew,P',[1:npq(1)]'],1);
P = ew(:,2:npq(2)+2)';
ew = ew(:,1);
j=npq(6)+1;
n1=npq(2)+1;
n2 = 1;
n3 = j;
while n2<=npq(1) && j<=npq(1)
   if P(n1,n2)>=n3
      if isreal(ew(n2))
         P(n1,n2) = j;
         j = j+1;
      else
         P(:,n2) = rand(n1,1)-1i*rand(n1,1);
         P(:,n2+1) = conj(P(:,n2));
         P(n1,n2) = j;
         j = j+1;
         n2 = n2+1;
         P(n1,n2) = j;
         j = j+1;
      end
   end
   n2 = n2+1;
end

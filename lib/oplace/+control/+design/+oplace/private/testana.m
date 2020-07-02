function [anaok,ew,P,H,npq] = testana(c1,c2,ew,P,H,npq)
% anaok = testana(c1,c2,npq) checks if the computation of the feedback K
% can be made analytically.
%
% OUTPUT:
% W4     - ggf. veränderte Matrix mit den Gewichtungsfaktoren 
%          für das Gütekriterium der minimalen Reglernorm
%
% INPUT:
% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium 
%          der minimalen Reglernorm
% S      - Matrix für eine mögliche Strukturbeschränkung
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  

anaok = 0;		% no analytical solution
d = npq(3)-1;

if npq(2) == 1 & c1 & c2
   anaok = 4;	%output or state feedback with one input;
   return;
end

if npq(2)+npq(3)>npq(1) & c1 & c2 & npq(4)>=d
   anaok = 1;	% output or state feedback with exactly d parameter vectors
   				% and at least d corresponding eigenvalues predefined
else
   return;		% no analytical solution
end

if npq(3)==npq(1) & npq(4)==npq(1)
   anaok = 2;	%state feedback
   return;
end

if npq(6)==d
   return;
end

if npq(4)==d & npq(6)<d
   anaok = 3;	% output feedback with less than d parameter vectors
   				% and exactly d suitable eigenvalues predefined
   return;
end

if npq(6)>d
   anaok = 0;
   return;		% no analytical solution
end
               
d1 = d - npq(6);
if rem(d1,2)==0
   deven = 1;
else
   deven = 0;
end

n = 0;
for i=npq(6)+1:npq(4)
   if isreal(ew(i))
      n=n+1;
   end
end

ew1 = []; H1 = []; P1 = [];
ew2 = []; H2 = []; P2 = [];
j = 0;
if n>0 & ~deven
   re = 1;
   j = 1;
   for i=npq(6)+1:npq(4)
      if isreal(ew(i)) & re
         ew1(1,1) = ew(i);
         H1(:,1) = H(:,i);
         P1(:,1) = P(:,i);
         re = 0;
         d1 = d1-1;
         n = n-1;
         deven = 1;
         npq(6) = npq(6)+1;
      else
         ew2(j,1) = ew(i);
         H2(:,j) = H(:,i);
         P2(:,j) = P(:,i);
         j = j+1;
      end
   end
   j = 1;
   if d1==0
      i = npq(3)-2;
      ew1 = [ew(1:i);ew1;ew2];
      H = [H(:,1:i),H1,H2];
      P1 = [P(:,1:i),P1,P2];
      i = npq(4)+1;
      if i > npq(1)
         ew = ew1;
         P = P1;
         return;
      end
      ew = [ew1;ew(i:npq(1))];
      P = [P1,P(:,i:npq(1))];
      return;
   end
else
   ew2 = ew;
   P2 = P;
   H2 = H;
end

ew3 = []; H3 = []; P3 = [];
ew4 = []; H4 = []; P4 = [];
dh = d - n;
if deven
   while d1~=0
      i = npq(6)+1;
      j = j+1;
      jj = 1;
      ii = 1;
      while i<=npq(4) & dh>0 & d1~=0
         if ~isreal(ew2(jj))
            ew1(j,1) = ew2(jj);
            H1(:,j) = H2(:,jj);
            P1(:,j) = P2(:,jj);
            j = j+1;
            i = i+1;
            jj = jj+1;
            ew1(j,1) = ew2(jj);
            H1(:,j) = H2(:,jj);
            P1(:,j) = P2(:,jj);
            j = j+1;
            i = i+1;
            jj = jj+1;
            d1 = d1-2;
            dh = dh - 2;
         else
            ew3(ii,1) = ew2(jj);
            H3(:,ii) = H2(:,jj);
            P3(:,ii) = P2(:,jj);
            ii = ii+1;
            i = i+1;
            jj = jj+1;
         end
      end
      ew2_d = length(ew2);
      dimH2 = size(H2,2);
      if jj<=ew2_d
         for i=jj:ew2_d
            ew3(ii,1) = ew2(i);
            if i <= dimH2
               H3(:,ii) = H2(:,i);
            end
            P3(:,ii) = P2(:,i);
            ii = ii+1;
         end
      end
      
      jj = 1;
      ii = 1;
      while d1~=0
         if isreal(ew3(jj))
            ew1(j,1) = ew3(jj);
            H1(:,j) = H3(:,jj);
            P1(:,j) = P3(:,jj);
            j = j+1;
            jj = jj+1;
            d1 = d1-1;
         else
            ew4(ii,1) = ew3(jj);
            H4(:,ii) = H3(:,jj);
            P4(:,ii) = P3(:,jj);
            ii = ii+1;
            jj = jj+1;
         end
      end
      ew3_d = length(ew3);
      dimH3 = size(H3,2);
      if jj<=ew3_d
         for i=jj:ew3_d
            ew4(ii,1) = ew3(i);
            if i <= dimH3
               H4(:,ii) = H3(:,i);
            end
            P4(:,ii) = P3(:,i);
            ii = ii+1;
         end
      end
   end
   i = npq(3)-1-length(ew1);
   npq(6) = npq(3)-1;
   ew = [ew(1:i);ew1;ew4];
   P = [P(:,1:i),P1,P4];
   H = [H(:,1:i),H1,H4];
else
   anaok = 0;
   return;
end

function dK = gradient_1(K,S,npq,w1,G,ew)
% dK = GRADIENT_1(K,S,npq,w1,G) berechnet den Gradienten
% f�r das G�tekriterium J1
%
% OUTPUT:
% dK     - Gradient der zu optimierenden Matrix f�r J1
%
% INPUT:
% K      - die zu optimierenden Matrix
% S      - Matrix f�r eine m�gliche Strukturbeschr�nkung
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  
% w1     - Gewichtungsfaktoren f�r J1
% G      - 3-dim. �bertragungsmatrix der Regelstrecke ("G_lam") 

dK = zeros(npq(2),npq(3));
Ip = eye(npq(2));
if isempty(S)
   S = dK;
end

f1 = zeros(npq(4),1);
i = 1;
while i<=npq(4)
   if isreal(ew(i))
      Hr(:,:,i) = Ip+K*G(:,:,i);
      f1(i) = det(Hr(:,:,i));
      i = i + 1;
   else
      R = Ip+K*real(G(:,:,i));
      I = K*imag(G(:,:,i));
      Hc(:,:,i) = [R	-I;
                   I  R];
      i = i + 2;
   end
end
   
for l=1:npq(2)
   for m=1:npq(3)
      if ~S(l,m)
         i = 1;
       	while i<=npq(4)
             if isreal(ew(i))
                H1 = Hr(:,:,i);
                H1(l,:) = G(m,:,i);
                dK(l,m) = dK(l,m) + w1(i)*det(H1)*f1(i);
                i = i + 1;
             else
                H2 = Hc(:,:,i);
                H3 = H2;
                H2(l,:) = [real(G(m,:,i)) -imag(G(m,:,i))];
                H3(l+npq(2),:) = [imag(G(m,:,i))	real(G(m,:,i))];
                dK(l,m) = dK(l,m) + w1(i)*(det(H2)+det(H3));
                i = i + 2;
         	 end
         end
      end
   end
end
dK=real(dK);

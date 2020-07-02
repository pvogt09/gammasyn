function [Ko,T1] = para_exact(P,H,npq)
% [Ko,T1] = PARA_FEST(P,h,npq) bestimmt bei Erf�llung der 
% nachfolgenden Bedingung ein Teil der gesuchte Ausgangsr�ck-
% f�hrungsmatrix K analytisch. Hierdurch wird gew�hrleistet, da�
% die vorgegebenen Parameter exakt ber�cksichtigt werden.
% Bed.:
%    r <= (p*q-n) / (p-1)  mit  r - Anz. der Parametervektoren [npq(5)],
%                               p - Anz. der Eingangsgr��en [npq(2)],    
%                               q - Anz. der Ausgangsgr��en [npq(3)],
%                               n - Systemordnung [npq(1)].
%
% OUTPUT:
% Ko     - Matrix, durch die s�mtliche Parametervektor und die dazugeh�rigen 
%          EW festlegt werden
% T1     - Matrix f�r die Berechnung des neuen Systems
% 
% INPUT:
% P      - Matrix mit den Parametervektoren 
% H      - aus den Parametervektoren berechnete Vektoren
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  

r = npq(6);
d = npq(3) - r;
In=eye(npq(1));
Iq=eye(npq(3));
Hi = pinv(H(:,1:r));
Ko = real(P(1:npq(2),1:r) * Hi);
N = real(Iq - H(:,1:r) * Hi);
[U,S,V] = svd(N);
T1(:,1:d)=V(:,1:d);

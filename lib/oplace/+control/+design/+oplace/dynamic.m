function [Ag,Bg,Cg] = dynamic(A,B,C,r)
% [Ag,Bg,Cg] = dynamic(A,B,C,r) erzeugt bei einer dynamischen
% Ausgangsrückführung die erweiterten Matrizen Ag, Bg und Cg.
%
% OUTPUT:
% Ag - Systemmatrix des erweiterten Systems
% Bg - Eingangsmatrix des erweiterten Systems
% Cg - Ausgangsmatrix des erweiterten Systems
%
% INPUT:
% A - Systemmatrix
% B - Eingangsmatrix
% C - Ausgangsmatrix
% r - Ordnung der dynamische Ausgangsrückführung

if nargin < 4, error('dynamic benötigt 4 Eingangsgrößen'); end

a = size(A);
b = size(B);
c = size(C);

if a(1) == a(2) & b(1) == a(1) & c(2) == a(1)
	n = a(1);
	p = b(2);
	q = c(1);
else
	error('Kontrollieren Sie bitte die Dimension Ihrer Matrizen')
end

Ag = zeros(n+r);
Ag(1:n,1:n) = A;

Bg = zeros(n+r,p+r);
Bg(1:n,1:p) = B;
Bg(n+1:n+r,p+1:p+r) = eye(r);

Cg = zeros(q+r,n+r);
Cg(1:q,1:n) = C;
Cg(q+1:q+r,n+1:n+r) = eye(r);
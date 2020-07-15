function [npq,ew,w3,K0] = dimensions(sys,ew,K0,S,P,w3,W4)
	% npq = DIMENSIONEN(A,B,C,ew,K0,S,P,w3,W4) kontrolliert
	% die Dimensionen der vorgegebenen Matrizen und legt in
	% npq die für die Berechnung und Optimierung benötigten
	% Größen ab.
	%
	% OUTPUT:
	% npq(1) = n - Systemordnung
	% npq(2) = p - Anz. der Eingangsgrößen
	% npq(3) = q - Anz. der Ausgangsgrößen
	% npq(4) = Anz. der für die Optimierung verwendeten EW
	% npq(5) = Anz. der Stützstellen
	% npq(6) = Anz. der Parametervektoren
	%
	% INPUT:
	% A      - Systemmatrix
	% B      - Eingangsmatrix
	% C      - Ausgangsmatrix
	% ew     - Vektor mit den vorgegebenen Eigenwerten
	% K0     - Startrückführungsmatrix für die Optimierung
	% S      - Matrix für eine mögliche Strukturbeschränkung
	% P      - Matrix mit den Parametervektoren
	% w3     - Gewichtungsfaktoren für die einzelnen Parametervektoren
	% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium
	%          minimale Reglernorm

	e = size(ew);
	if e(1)~=1 && e(2)~=1
		usage;
		error('oplace:arguments', 'ew must be a vector!');
	end
	ew = ew(:);
	e = size(ew);

	a = size(sys.a);
	b = size(sys.b);
	c = size(sys.c);
	if isempty(K0), K0 = zeros(b(2),c(1)); end
	k = size(K0);
	s = size(S);
	p = size(P);
	w_4 = size(W4);

	if k(1)==b(2) && k(2)==c(1)
		npq = [a(1) b(2) c(1) a(1) 0 0];
	else
		usage;
		error('oplace:arguments', 'Wrong or incompatible dimensions of K0!');
	end

	if e(1)~=a(1)
		usage;
		error('oplace:arguments', 'There must be exactly %d eigenvalues in ew!', a(1));
	end

	if ~isempty(S) && (k(1)~=s(1) || k(2)~=s(2))
		usage;
		error('oplace:arguments', 'The dimensions of S and K0 are not identical!');
	end

	if ~isempty(P)
		if p(1)~=b(2)
			usage;
			error('oplace:arguments', 'Wrong dimension (number of rows) of the parametervectors in P!');
		end
		if p(2)>a(1)
			usage;
			error('oplace:arguments', 'The number (column dimension) of parametervectors in P exceeds system order!');
		end
		if isempty(w3)
			w3=ones(p(2),1);
			fprintf('\nAll %d parametervectors have a weight of 1!', p(2));
		end
		w_3 = size(w3);
		if w_3(1)~=1 && w_3(2)~=1
			usage;
			error('oplace:arguments', 'w3 must be a vector!');
		end
		w3=w3(:);
		w_3=size(w3);
		if p(2)~=w_3(1)
			usage;
			error('oplace:arguments', 'The number (column dimension) of parametervectors in P doesn`t match the dimension of w3!');
		end
		npq(6) = p(2);
	end

	if ~isempty(W4)
		if k(1)~=w_4(1) || k(2)~=w_4(2)
			usage;
			error('oplace:arguments', 'The dimensions of W4 and K0 are not identical!');
		end
	end
end

function [] = usage()
	disp(' ');
	disp('------------------------------------------------');
	disp('Usage: K = OPLACE(sys,ew,[K0],[S],[P],[w3],[W4])');
	disp('------------------------------------------------');
	disp(' ');
end
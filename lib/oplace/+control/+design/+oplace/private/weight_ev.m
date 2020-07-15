function [ew_opt, P_opt, index] = weight_ev(ew,merker,P,npq)
	% ew_opt = GEWICHTETE_EW(ew,merker,npq) eliminiert aus
	% den vorgegebenen EW diejenigen, die mit 0 gewichtet
	% sind, und liefert einen Vektor zurück, der nur die
	% gewichteten EW beinhaltet.
	%
	% OUTPUT:
	% ew_opt - für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
	%
	% INPUT:
	% ew     - Vektor mit den vorgegebenen Eigenwerten
	% merker - Vektor, der Aufschluß gibt, welcher EW mit 0 gewichtet ist
	%          (1: soll gewichtet werden, 0: wird mit 0 gewichtet)
	% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

	d = npq(1) - npq(4);
	if npq(4)==0 | ~d
		ew_opt = ew;
		P_opt = P;
		index = (1:size(ew_opt)).';
		return;
	end
	p = npq(2);
	ew_opt = zeros(npq(4),1);
	P_opt = zeros(p+1,npq(4));
	ewh = zeros(d,1);
	Ph = zeros(p+1,d);
	j = 1;
	n = 1;
	idx_opt = zeros(npq(4), 1);
	idx_h = zeros(d, 1);
	for i=1:npq(1)
		if merker(i)
			idx_opt(j) = i;
			ew_opt(j) = ew(i);
			P_opt(:,j) = P(:,i);
			j = j+1;
		else
			ewh(n) = ew(i);
			idx_h(n) = i;
			Ph(:,n) = P(:,i);
			n = n+1;
		end
	end
	ew_opt = [ew_opt;ewh];
	P_opt = [P_opt, Ph];
	index = [
		idx_opt;
		idx_h
	];
end
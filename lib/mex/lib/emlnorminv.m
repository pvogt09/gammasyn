function [x] = emlnorminv(p, mu, sigma)
	%EMLNORMINV Wrapper Funktion zur Codegenerierung für die Inverse kumulierte Wahrscheinlichkeitsfunktion der Standardnormalverteilung N(mu, sigma^2)
	%	Input:
	%		p:		Wahrscheinlichkeit
	%		mu:		Erwatungswert
	%		sigma:	Varianz
	%	Output:
	%		x:		Wert für den p = P(x, N(mu, sigma^2)) gilt
	%#codegen
	x = sigma.*(-sqrt(2)*erfcinv(2*p)) + mu;
end
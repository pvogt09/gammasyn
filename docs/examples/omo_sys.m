function sys = omo_sys(m, d, c)

	A = [0, 1; -c/m, -d/m];
	B = [0; 1/m];
	C = [1, 0];

	sys = struct('A', A, 'B', B, 'C', C);

end
Lambda_derv_expected = diag([
	0;
	-8*pi;
	0;
	0;
	0
]);
V_derv_expected = [
	-2i,	0,			0,		1 - 1i,		-1;
	-2,		0,			0,		0,			1;
	-2,		-1i,		-2i,	0,			0;
	-2,		-1 - 2i,	0,		-1 - 1i,	0;
	0,		0,			0,		0,			-1
];


p = sym('p', 'real');
Lambda_sym = [
	1;
	-1 + 11i + sin(2*pi*p) + sin(6*pi*p);
	(1 - 11i)*cos(6*pi*p) - 3*sin(2*pi*p) + sin(6*pi*p);
	7i + (1 - 4i)*cos(2*pi*p) - 3*sin(2*pi*p) + sin(6*pi*p);
	7i + (6 - 4i)*cos(2*pi*p) + 8*cos(4*pi*p) + 3*cos(6*pi*p)
];
W_sym = [% W in van der Aa equals V in notation used here
	1i,		0,		0,		1,		2*p;
	1,		0,		1i,		2*p,	0;
	1,		1i,		2*p,	0,		0;
	1,		2*p,	0,		0,		1;
	2*p,	0,		0,		1,		0
];
A_sym = W_sym*diag(Lambda_sym)/W_sym;
x = 0.5;
A_diff_sym = diff(A_sym, p);
A_diff2_sym = diff(A_diff_sym, p);
A_diff3_sym = diff(A_diff2_sym, p);
A_diff4_sym = diff(A_diff3_sym, p);
A_diff5_sym = diff(A_diff4_sym, p);
A_diff6_sym = diff(A_diff5_sym, p);
A_diff7_sym = diff(A_diff6_sym, p);
A = double(cat(3, subs(A_sym, p, x), subs(A_diff_sym, p, x), subs(A_diff2_sym, p, x), subs(A_diff3_sym, p, x), subs(A_diff4_sym, p, x), subs(A_diff5_sym, p, x), subs(A_diff6_sym, p, x), subs(A_diff7_sym, p, x)));
eigen = double(subs(Lambda_sym, p, x));
V_tilde = double(subs(W_sym, p, x));
W_tilde = double(subs(inv(W_sym), p, x))';
[mult, map] = eigenvalue_multiplicity(eig(A(:, :, 1)), sqrt(eps))
[V, D, W, V_derv, D_derv, W_derv] = control.design.gamma.eigenvector_derivative(A, [], struct(), V_tilde, eigen, W_tilde)

V_derv_difference = V_derv - V_derv_expected
Lambda_derv_difference = D_derv - Lambda_derv_expected
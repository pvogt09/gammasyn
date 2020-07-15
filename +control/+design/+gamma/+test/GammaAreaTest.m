function [pass] = GammaAreaTest(~)
	%GAMMASYNTEST test cases for checking gammasyn for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;

	R = 50;
	a = 0.65;
	b = 0.5;
	poleareatypes = {
		control.design.gamma.area.Circle(R);
		control.design.gamma.area.Circlesquare(R);
		control.design.gamma.area.CircleDiscrete(R);
		control.design.gamma.area.Ellipse(1, 2);
		control.design.gamma.area.Ellipsesquare(1, 2);
		control.design.gamma.area.Hyperbola(a, b);
		control.design.gamma.area.Hyperbolasquare(a, b);
		control.design.gamma.area.Imag(1, 0);
		control.design.gamma.area.Line(1, 1);
		control.design.gamma.area.LogSpiral(R, 1);
		control.design.gamma.area.None();
		control.design.gamma.area.PolyEllipse([1 + 1i, 1 - 1i, -2], [1, 1, 3]);
		control.design.gamma.area.PolyEllipsesquare([1 + 1i, 1 - 1i, -2], [1, 1, 3]);
		control.design.gamma.area.Custom(@(re, im) control.design.gamma.area.Circlesquare_border(re, im, struct('circle_R', 3, 'reshift', 0, 'imshift', 0)));
	};
	for ii = 1:size(poleareatypes, 1)
		area = poleareatypes{ii, 1};
		testpoint = 1 + 1i;
		test.TestSuite.assertNoException('functionstruct = area.tofunctionstruct();', 'control:gammasyn:area:test', 'tofunctionstruct must not throw an exception.');
		functionstruct = area.tofunctionstruct();
		test.TestSuite.assertFieldnames(functionstruct, control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY, 'control:gammasyn:area:test');
		test.TestSuite.assertNoException('f = area.border(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'border must not throw an exception.');
		f = area.border(real(testpoint), imag(testpoint), functionstruct);
		test.TestSuite.assertSameSize(f, 1, 'control:gammasyn:area:test', 'Border must be scalar.');
		test.TestSuite.assert(imag(f) == 0, 'control:gammasyn:area:test', 'Border must be real.');

		test.TestSuite.assertNoException('[dfdre] = area.gradborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'gradborder must not throw an exception.');
		test.TestSuite.assertNoException('[dfdre, dfdim] = area.gradborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'gradborder must not throw an exception.');
		[dfdre, dfdim] = area.gradborder(real(testpoint), imag(testpoint), functionstruct);
		test.TestSuite.assertSameSize(dfdre, 1, 'control:gammasyn:area:test', 'gradborder must be scalar.');
		test.TestSuite.assertSameSize(dfdim, 1, 'control:gammasyn:area:test', 'gradborder must be scalar.');
		test.TestSuite.assert(imag(dfdre) == 0, 'control:gammasyn:area:test', 'gradborder must be real.');
		test.TestSuite.assert(imag(dfdim) == 0, 'control:gammasyn:area:test', 'gradborder must be real.');

		test.TestSuite.assertNoException('[d2fdredre] = area.hessborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'hessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre] = area.hessborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'hessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre, d2fdredim] = area.hessborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'hessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = area.hessborder(real(testpoint), imag(testpoint), functionstruct);', 'control:gammasyn:area:test', 'hessborder must not throw an exception.');
		[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = area.hessborder(real(testpoint), imag(testpoint), functionstruct);
		test.TestSuite.assertSameSize(d2fdredre, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdimdre, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdredim, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdimdim, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assert(imag(d2fdredre) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdimdre) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdredim) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdimdim) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');


		test.TestSuite.assertNoException('f = area.getborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'getborder must not throw an exception.');
		f = area.getborder(real(testpoint), imag(testpoint));
		test.TestSuite.assertSameSize(f, 1, 'control:gammasyn:area:test', 'Border must be scalar.');
		test.TestSuite.assert(imag(f) == 0, 'control:gammasyn:area:test', 'Border must be real.');

		test.TestSuite.assertNoException('[dfdre] = area.getgradborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'getgradborder must not throw an exception.');
		test.TestSuite.assertNoException('[dfdre, dfdim] = area.getgradborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'getgradborder must not throw an exception.');
		[dfdre, dfdim] = area.getgradborder(real(testpoint), imag(testpoint));
		test.TestSuite.assertSameSize(dfdre, 1, 'control:gammasyn:area:test', 'gradborder must be scalar.');
		test.TestSuite.assertSameSize(dfdim, 1, 'control:gammasyn:area:test', 'gradborder must be scalar.');
		test.TestSuite.assert(imag(dfdre) == 0, 'control:gammasyn:area:test', 'gradborder must be real.');
		test.TestSuite.assert(imag(dfdim) == 0, 'control:gammasyn:area:test', 'gradborder must be real.');

		test.TestSuite.assertNoException('[d2fdredre] = area.gethessborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'gethessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre] = area.gethessborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'gethessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre, d2fdredim] = area.gethessborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'gethessborder must not throw an exception.');
		test.TestSuite.assertNoException('[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = area.gethessborder(real(testpoint), imag(testpoint));', 'control:gammasyn:area:test', 'gethessborder must not throw an exception.');
		[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = area.gethessborder(real(testpoint), imag(testpoint));
		test.TestSuite.assertSameSize(d2fdredre, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdimdre, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdredim, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assertSameSize(d2fdimdim, 1, 'control:gammasyn:area:test', 'hessborder must be scalar.');
		test.TestSuite.assert(imag(d2fdredre) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdimdre) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdredim) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');
		test.TestSuite.assert(imag(d2fdimdim) == 0, 'control:gammasyn:area:test', 'hessborder must be real.');

		test.TestSuite.assertNoException('str = area.getstring();', 'control:gammasyn:area:test', 'getstring must not throw an exception.');
		str = area.getstring();
		test.TestSuite.assert(ischar(str), 1, 'control:gammasyn:area:test', 'getstring must be char.');

		test.TestSuite.assertNoException('[L, M, success] = area.toLMIregion();', 'control:gammasyn:area:test', 'toLMIregion must not throw an exception.');
		[L, M, success] = area.toLMIregion();
		test.TestSuite.assert(islogical(success), 'control:gammasyn:area:test', 'success must be of type logical.');
		if success && area.type ~= GammaArea.NONE
			test.TestSuite.assertEqual(sum(imag(L(:)) ~= 0), 1, 'control:gammasyn:area:test', 'L must only have one imaginary entry.');
			test.TestSuite.assertNotEqual(imag(L(1)), 0, 'control:gammasyn:area:test', 'first entry of L must have imaginary part.');
			test.TestSuite.assertEqual(size(L), size(M), 'control:gammasyn:area:test', 'size of L and M must be equal.');
		else
			test.TestSuite.assert(isempty(L), 'control:gammasyn:area:test', 'L must be empty.');
			test.TestSuite.assert(isempty(M), 'control:gammasyn:area:test', 'M must be empty.');
		end
	end
end
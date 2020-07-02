function [z, dzdre, dzdim] = mindistance_area(re, im, R)
	z = R^2 - re^2 - im^2;
	dzdre = -2 * re;
	dzdim = -2 * im;
end
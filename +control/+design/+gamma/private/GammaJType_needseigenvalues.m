function [isgain] = GammaJType_needseigenvalues(this)
	%GAMMAJTYPE_NEEDSEIGENVALUES return if a GammaJType is an objective type that needs eigenvalues
	%	Input:
	%		this:	instance (must be static to work with code generation)
	%	Output:
	%		isgain:	true, if the supplied type needs eigenvalues
	%	TODO:		does not work as static method in GammaJType
	if isempty(this)
		isgain = false(size(this));
	elseif isscalar(this)
		isgain = any([
			GammaJType.LINEAR;
			GammaJType.SQUARE;
			GammaJType.CUBIC;
			GammaJType.EXP;
			GammaJType.LOG;
			GammaJType.MAX;
			GammaJType.KREISSELMEIER;
			GammaJType.SQUAREPENALTY;
			GammaJType.EIGENVALUECONDITION
		] == this(1));% HINT: this(1) is needed for scalar expansion in code generation
	else
		sz = size(this);
		t = reshape(this, [], 1);
		gain = false(size(t, 1), 1);
		for ii = 1:size(t, 1)
			gain(ii, 1) = any([
				GammaJType.LINEAR;
				GammaJType.SQUARE;
				GammaJType.CUBIC;
				GammaJType.EXP;
				GammaJType.LOG;
				GammaJType.MAX;
				GammaJType.KREISSELMEIER;
				GammaJType.SQUAREPENALTY;
				GammaJType.EIGENVALUECONDITION
			] == t(ii, 1));
		end
		isgain = reshape(gain, sz);
	end
end
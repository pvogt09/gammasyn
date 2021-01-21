function [isgain] = GammaJType_isgainobjective(this)
	%GAMMAJTYPE_ISGAINOBJECTIVE return if a GammaJType is an objective type that only depends on gain coeffcients
	%	Input:
	%		this:	instance (must be static to work with code generation)
	%	Output:
	%		isgain:	true, if the supplied type only depends on gain coefficients
	%	TODO:		does not work as static method in GammaJType
	if isscalar(this)
		isgain = this == GammaJType.NORMGAIN || this == GammaJType.LYAPUNOV || this == GammaJType.DECOUPLING;
	else
		sz = size(this);
		t = reshape(this, [], 1);
		gain = false(size(t, 1), 1);
		for ii = 1:size(t, 1)
			gain(ii, 1) = t(ii) == GammaJType.NORMGAIN || t(ii) == GammaJType.LYAPUNOV || t(ii) == GammaJType.DECOUPLING;
		end
		isgain = reshape(gain, sz);
	end
end
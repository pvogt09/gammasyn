function [needscondition] = GammaCouplingStrategy_needscouplingconditions(this)
	%GAMMACOUPLINGSTRATEGY_NEEDSCOUPLINGCONDITIONS return if a GammaCouplingStrategy is an objective type that results in nonlinear coupling conditions
	%	Input:
	%		this:			instance (must be static to work with code generation)
	%	Output:
	%		needscondition:	true, if the supplied type results in nonlinear coupling conditions
	%	TODO:		does not work as static method in GammaCouplingStrategy
	if isempty(this)
		needscondition = false(size(this));
	elseif isscalar(this)
		needscondition = any([
			GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
			GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
		] == this(1));% HINT: this(1) is needed for scalar expansion in code generation
	else
		sz = size(this);
		t = reshape(this, [], 1);
		gain = false(size(t, 1), 1);
		for ii = 1:size(t, 1)
			gain(ii, 1) = any([
				GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
				GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			] == t(ii, 1));
		end
		needscondition = reshape(gain, sz);
	end
end
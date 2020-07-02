function [is] = issubclassof(subclass, parentclass)
	%ISSUBCLASSOF determine if a class is a subclass of some parentclass
	%	Input:
	%		subclass:		class to find parent class for
	%		parentclass:	class the subclass has to be derived from
	%	Output:
	%		is:				true, if the subclass is a subclass of parentclass, else false
	if isempty(subclass)
		is = false;
		return;
	end
	if ischar(subclass)
		subclass = meta.class.fromName(subclass);
	end
	if iscellstr(subclass)
		subclass = cellfun(@meta.class.fromName, subclass, 'UniformOutput', false);
		subclass = [subclass{:}];
		if size(subclass, 2) > size(subclass, 1)
			subclass = subclass';
		end
	end
	if ~isa(subclass, 'meta.class')
		error('meta:class:subclass', 'class must be of type ''meta.class''.');
	end
	if nargin <= 1
		is = subclass.HandleCompatible;
		return;
	else
		if ischar(parentclass)
			parentclass = meta.class.fromName(parentclass);
		end
		if ~isa(parentclass, 'meta.class')
			error('meta:class:subclass', 'class must be of type ''meta.class''.');
		end
	end
	ischild = subclass == parentclass;
	parfor ii = 1:size(subclass, 1)
		subclassestemp = subclass(ii);
		while (~ischild(ii))
			superclass = subclassestemp.SuperclassList;
			if isempty(superclass)
				ischild(ii) = false;
				break;
			end
			ischild(ii) = any(superclass == parentclass);
			subclassestemp = superclass;
		end
	end
	is = ischild;
end
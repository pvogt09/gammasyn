function [rootclass] = rootclass(metaclass)
	%ROOTCLASS determine root class of a given class
	%	Input:
	%		metaclass:	class to find root class for
	%	Output:
	%		rootclass:	root class in class hierarchy for given class as cell-array
	if ischar(metaclass)
		metaclass = meta.class.fromName(metaclass);
	end
	if iscellstr(metaclass)
		metaclass = cellfun(@meta.class.fromName, metaclass, 'UniformOutput', false);
		metaclass = [metaclass{:}];
		if size(metaclass, 2) > size(metaclass, 1)
			metaclass = metaclass';
		end
	end
	if ~isa(metaclass, 'meta.class')
		error('meta:class:root', 'class must be of type ''meta.class''.');
	end
	rootclass = cell(size(metaclass, 1), 1);
	parfor ii = 1:size(metaclass, 1)
		isroot = false;
		metaclasstemp = metaclass(ii);
		while (~isroot)
			superclass = metaclasstemp.SuperclassList;
			if isempty(superclass)
				isroot = true;
				break;
			end
			metaclasstemp = superclass;
		end
		if ~isroot
			error('meta:class:root', 'no root class found.');
		end
		rootclass{ii, 1} = metaclasstemp;
	end
end
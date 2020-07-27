function [structout] = mergestruct(structa, structb, usefirst)
	%MEGRESTRUCT merge two structures into one
	%	Input:
	%		structa:	first structure to merge
	%		structb:	second structure to merge
	%		usefirst:	indicator, if values of first structure (true) or second structure (false) should be used
	%	Output:
	%		structout:	structure with fileds merged from both input structures
	if nargin <= 2
		usefirst = true;
	end
	if ~isstruct(structa) || ~isstruct(structb)
		error('struct:merge', 'Undefined function or variable for arguments of type ''%s'' and ''%s''.', class(structa), class(structb));
	end
	if ~islogical(usefirst)
		error('struct:merge', 'Undefined function or variable for arguments of type ''%s''.', class(usefirst));
	end
	if usefirst
		structout = structa;
		structa = structb;
	else
		structout = structb;
	end
	fieldnamesout = fieldnames(structout);
	fieldnamesa = fieldnames(structa);
	allfieldnames = unique([
		fieldnamesout;
		fieldnamesa
	]);
	for ii = 1:size(allfieldnames, 1)
		hasfieldout = isfield(structout, allfieldnames{ii, 1});
		hasfielda = isfield(structa, allfieldnames{ii, 1});
		if hasfieldout && hasfielda
			isstructout = isstruct(structout.(allfieldnames{ii, 1}));
			isstructa = isstruct(structa.(allfieldnames{ii, 1}));
			if isstructout && isstructa
				structout.(allfieldnames{ii, 1}) = mergestruct(structout.(allfieldnames{ii, 1}), structa.(allfieldnames{ii, 1}), true);
			elseif xor(isstructout, isstructa)
				error('struct:merge', 'Structures do not match.');
			end
		elseif hasfielda
			structout.(allfieldnames{ii, 1}) = structa.(allfieldnames{ii, 1});
		end
	end
end
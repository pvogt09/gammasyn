function [str] = strjoin(C, delimiter)
	%STRJOIN Join strings in cell array into single string
	%	Input:
	%		C:			Input text, specified as a 1-by-n cell array of strings. Each element in the cell array must contain a single string in a single row.
	%		delimiter:	Delimiting characters, specified as a single string or a 1-by-n cell array of strings.
	%	Output:
	%		str:		joined string
	if ~iscellstr(C)
		if ischar(C) || isstring(C)
			str = C;
			return;
		else
			error('strjoin:input', 'input argument must be of type ''cell''.');
		end
	end
	if nargin <= 1
		delimiter = repmat({' '}, length(C) - 1, 1);
	end
% 	if isempty(C)
% 		str = '';
% 		return;
% 	end
	if ~ischar(delimiter)
		if iscellstr(delimiter)
			if length(delimiter) ~= length(C) - 1
				error('strjoin:input', 'delimiters must have length %d.', length(C) - 1);
			end
		else
			error('strjoin:input', 'input argument must be of type ''char''.');
		end
	else
		if size(delimiter, 1) == 1 && size(delimiter, 2) == 2
			switch delimiter
				case '\n'
					delimiter = char(10);
				case '\r'
					delimiter = char(13);
				case '\t'
					delimiter = char(9);
				case '\\'
					delimiter = char(92);
				case '\0'
					delimiter = char(0);
				case '\a'
					delimiter = char(7);
				case '\b'
					delimiter = char(8);
				case '\f'
					delimiter = char(12);
				case '\v'
					delimiter = char(11);
			end
		end
		delimiter = repmat({delimiter}, length(C) - 1, 1);
	end
	if isempty(C)
		str = '';
	else
		str = C{1};
		for i = 2:length(C)
			str = [str, delimiter{i-1}, C{i}];
		end
	end
end
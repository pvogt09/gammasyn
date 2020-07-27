function [N, modified] = makeValidName(S, varargin)
	%MATLAB.LANG.MAKEVALIDNAME constructs valid MATLAB identifiers from input S
	%   N = MATLAB.LANG.MAKEVALIDNAME(S) returns valid identifiers, N,
	%   constructed from the input S. S is specified as a string or a cell
	%   array of strings. The strings in N are NOT guaranteed to be unique.
	%
	%   A valid MATLAB identifier is a character string of alphanumerics and
	%   underscores, such that the first character is a letter and the length
	%   of the string is <= NAMELENGTHMAX.
	%
	%   MATLAB.LANG.MAKEVALIDNAME deletes whitespace characters prior to replacing
	%   any characters that are not alphnumerics or underscores. If a whitespace
	%   character is followed by a lowercase letter, MATLAB.LANG.MAKEVALIDNAME
	%   converts the letter to the corresponding uppercase character.
	%
	%   N = MATLAB.LANG.MAKEVALIDNAME(___, PARAM1, VAL1, PARAM2, VAL2, ...)
	%   constructs valid identifiers using additional options specified by one
	%   or more Name, Value pair arguments.
	%
	%   Parameters include:
	%
	%   'ReplacementStyle'         Controls how non-alphanumeric characters
	%                              are replaced. Valid values are 'underscore',
	%                              'hex' and 'delete'.
	%
	%                              'underscore' indicates non-alphanumeric
	%                              characters are replaced with underscores.
	%
	%                              'hex' indicates each non-alphanumeric
	%                              character is replaced with a corresponding
	%                              hexadecimal representation.
	%
	%                              'delete' indicates all non-alphanumeric
	%                              characters are deleted.
	%
	%                              The default 'ReplacementStyle' is
	%                              'underscore'.
	%
	%   'Prefix'                   Prepends the name when the first character
	%                              is not alphabetical. A valid prefix must
	%                              start with a letter and only contain
	%                              alphanumeric characters and underscores.
	%
	%                              The default 'Prefix' is 'x'.
	%
	%   [N, MODIFIED] = MATLAB.LANG.MAKEVALIDNAME(S, ___) also returns a
	%   logical array the same size as S, MODIFIED, that denotes modified
	%   strings.
	%
	%	Examples
	%   --------
	%   Make valid MATLAB identifiers from input strings
	%
	%		S = {'Item_#','Price/Unit','1st order','Contact'};
	%       N = MATLAB.LANG.MAKEVALIDNAME(S)
	%
	%   returns the cell array {'Item__' 'Price_Unit' 'x1stOrder' 'Contact'}
	%
	%   Make valid MATLAB identifiers using specified replacement style
	%
	%		S = {'Item_#','Price/Unit','1st order','Contact'};
	%       N = MATLAB.LANG.MAKEVALIDNAME(S, 'ReplacementStyle', 'delete')
	%
	%   returns the cell array {'Item_' 'PriceUnit' 'x1stOrder' 'Contact'}
	%
	%   See also MATLAB.LANG.MAKEUNIQUESTRINGS, ISVARNAME, ISKEYWORD,
	%            ISLETTER, NAMELENGTHMAX, WHO, STRREP, REGEXP, REGEXPREP
	persistent parser;
	notAlphanumericRegex = '[^a-zA-Z_0-9]';
	if mod(nargin, 2) == 0
		error('makevalidname:input', 'key-value pairs have to be supplied.');
	end
	if nargin > 1
		if isempty(parser)
			parser = inputParser();
			parser.CaseSensitive = false;
			if ~verLessThan('matlab', '8.2')
				parser.PartialMatching = false;
			end
			parser.StructExpand = false;
			addOptional(parser, 'ReplacementStyle',	'underscore',	@(x) any(validatestring(x, {'underscore', 'hex', 'delete'})));
			addOptional(parser, 'Prefix',			'x',			@isvarname);
		end
		parse(parser, varargin{:});
		ReplacementStyle = parser.Results.ReplacementStyle;
		Prefix = parser.Results.Prefix;
	else
		ReplacementStyle = 'underscore';
		Prefix = 'x';
	end
	if ~isvarname(Prefix)
		error('makevalidname:input', 'prefix must result in a valid variable name.');
	end
	if iscell(S)
		if nargout >= 2
			modified = ~cellfun(@isvarname, S);
		end
		S = regexprep(S, '(?<=\S\s+)[a-z]', '${upper($0)}');
		S = regexprep(S, '\s*', '');
		if strcmpi(ReplacementStyle, 'underscore')
			S = regexprep(S, notAlphanumericRegex, '_');
		elseif strcmpi(ReplacementStyle, 'hex')
			%S = regexprep(S, notAlphanumericRegex, '${stringtohex($&)}');
			S = cellfun(@stringtohex, S, 'UniformOutput', false);
		else
			S = regexprep(S, notAlphanumericRegex, '');
		end
		useprefix = cellfun(@(x) ~isvarname(x(1)), S, 'UniformOutput', true);
		useprefixforkeyword = cellfun(@(x) iskeyword(x), S, 'UniformOutput', true);
		if any(useprefix)
			S(useprefix) = cellfun(@(x) [Prefix, x], S(useprefix), 'UniformOutput', false);
		end
		if any(useprefixforkeyword)
			S(useprefixforkeyword) = cellfun(@(x) [Prefix, upper(x(1)), x(2:end)], S(useprefixforkeyword), 'UniformOutput', false);
		end
		S = cellfun(@(x) x(1:min(length(x), namelengthmax)), S, 'UniformOutput', false);
	else
		if nargout >= 2
			modified = isvarname(S);
		end
		S = char(S);
		S = regexprep(S, '(?<=\S\s+)[a-z]', '${upper($0)}');
		S = regexprep(S, '\s*', '');
		if strcmpi(ReplacementStyle, 'underscore')
			S = regexprep(S, notAlphanumericRegex, '_');
		elseif strcmpi(ReplacementStyle, 'hex')
			%S = regexprep(S, notAlphanumericRegex, '${stringtohex($&)}');
			S = stringtohex(S);
		else
			S = regexprep(S, notAlphanumericRegex, '');
		end
		if ~isvarname(S(1))
			S = [Prefix, S];
		end
		if iskeyword(S)
			S = [Prefix, upper(S(1)), S(2:end)];
		end
		S = S(1:min(length(S), namelengthmax));
	end
	N = S;
end

function [hex] = stringtohex(string)
	%STRINGTOHEX convert non-alphanumeric string to hex encoded string
	%	Input:
	%		string:	string to convert
	%	Output:
	%		hex:	converted string
	illegalChars = unique(string(regexp(string, '[^a-zA-Z_0-9]')));
	hex = string;
	for illegalChar = illegalChars
		if illegalChar <= intmax('uint8')
			width = 2;
		else
			width = 4;
		end
		replace = ['0x' dec2hex(illegalChar, width)];
		hex = strrep(hex, illegalChar, replace);
	end
end
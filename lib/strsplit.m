function terms = strsplit(s, delimiter, varargin)
	%STRSPLIT Splits a string into multiple terms
	%   terms = strsplit(s)
	%       splits the string s into multiple terms that are separated by
	%       white spaces (white spaces also include tab and newline).
	%       The extracted terms are returned in form of a cell array of
	%       strings.
	%   terms = strsplit(s, delimiter)
	%       splits the string s into multiple terms that are separated by
	%       the specified delimiter.
	%   Remarks
	%   -------
	%       - Note that the spaces surrounding the delimiter are considered
	%         part of the delimiter, and thus removed from the extracted
	%         terms.
	%       - If there are two consecutive non-whitespace delimiters, it is
	%         regarded that there is an empty-string term between them.         
	%   History
	%   -------
	%       - Created by Dahua Lin, on Oct 9, 2008
	%	https://www.mathworks.com/matlabcentral/fileexchange/21710-string-toolkits/content/strings/strsplit.m
	if nargin > 2
		% compatibility with MATLAB 2013A
		terms = strsplit_builtin(s, delimiter, varargin{:});
		return;
	end
	%% parse and verify input arguments
	if ~(ischar(s) && ismatrix(s) && size(s, 1) <= 1)
		error('strsplit:invalidarg', 'The first input argument should be a char string.');
	end
	if nargin < 2
		by_space = true;
	else
		d = delimiter;
		if ~(ischar(d) && ismatrix(d) && size(d, 1) == 1 && ~isempty(d))
			error('strsplit:invalidarg', 'The delimiter should be a non-empty char string.');
		end
		%d = strtrim(d);
		by_space = isempty(d);
	end
	%% main
	if by_space
		w = isspace(s);            
		if any(w)
			% decide the positions of terms        
			dw = diff(w);
			if w(1)
				sp = find(dw == -1) + 1;     % start positions of terms
			else
				sp = [1, find(dw == -1) + 1];     % start positions of terms
			end
			ep = [find(dw == 1), length(s)];  % end positions of terms
			% extract the terms        
			nt = numel(sp);
			terms = cell(1, nt);
			for i = 1 : nt
				terms{i} = s(sp(i):ep(i));
			end                
		else
			terms = {s};
		end
	else    
		p = strfind(s, d);
		if ~isempty(p)        
			% extract the terms        
			nt = numel(p) + 1;
			terms = cell(1, nt);
			sp = 1;
			dl = length(delimiter);
			for i = 1 : nt-1
				terms{i} = s(sp:p(i)-1);
				sp = p(i) + dl;
			end         
			terms{nt} = s(sp:end);
		else
			terms = {s};
		end        
	end
end
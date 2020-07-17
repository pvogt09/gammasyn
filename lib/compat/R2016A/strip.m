function [s] = strip(str, side, pad_character)
	%STRIP remove leading and trailing characters from a string
	%	Input:
	%		str:			string to remove characters from
	%		side:			side to remove characters from, possible values are 'both', 'left' and 'right'
	%		pad_character:	character to remove, default is all whitespace (space, newline and tab)
	%	Output:
	%		s:				string with removed padding characters
	if nargin <= 2
		if nargin == 2
			if isscalar(side)
				pad_character = side;
				side = 'both';
			else
				pad_character = [
					' ';
					char(7);%\a
					char(8);%\b
					char(9);%\t
					char(10);%\n
					char(11);%\v
					char(12);%\f
					char(13)%\r
				];
			end
		else
			pad_character = [
				' ';
				char(7);%\a
				char(8);%\b
				char(9);%\t
				char(10);%\n
				char(11);%\v
				char(12);%\f
				char(13)%\r
			];
		end
	end
	if nargin <= 1
		side = 'both';
	end
	if ischar(str)
		s = stripchars(str, side, pad_character);
	elseif iscellstr(str)
		s = cellfun(@(x) stripchars(x, side, pad_character), str, 'UniformOutput', false);
	else
		error('string:strip', 'undefined function or variable ''strip'' for input arguments of type ''%s''.', class(str));
	end
end

function [s] = stripchars(str, side, pad_character)
	%STRIPCHARS remove leading and trailing characters from a char array
	%	Input:
	%		str:			string to remove characters from
	%		side:			side to remove characters from, possible values are 'both', 'left' and 'right'
	%		pad_character:	character to remove, default is all whitespace (space, newline and tab)
	%	Output:
	%		s:				string with removed padding characters
	if ~ischar(str)
		error('string:strip', 'undefined function or variable ''strip'' for input arguments of type ''%s''.', class(str));
	end
	switch lower(side)
		case 'both'
			snew = str;
			removeleft = true;
			removeright = true;
			while ~isempty(snew) && (removeleft || removeright)
				removeleft = any(snew(1) == pad_character);
				removeright = any(snew(end) == pad_character);
				if removeleft
					snew = snew(2:end);
				end
				if removeright
					snew = snew(1:end - 1);
				end
			end
			s = snew;
		case 'right'
			snew = str;
			removeright = true;
			while ~isempty(snew) && removeright
				removeright = any(snew(end) == pad_character);
				if removeright
					snew = snew(1:end - 1);
				end
			end
			s = snew;
		case 'left'
			snew = str;
			removeleft = true;
			while ~isempty(snew) && removeleft
				removeleft = any(snew(1) == pad_character);
				if removeleft
					snew = snew(2:end);
				end
			end
			s = snew;
		otherwise
			error('RBSABS:string:strip:side', 'Side must be one of ''both'', ''right'' or ''left''.');
	end
end
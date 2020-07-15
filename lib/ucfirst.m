function [str] = ucfirst(str)
	%UCFIRST convert first character to upper case
	%	Input:
	%		str:	string to convert
	%	Output:
	%		str:	string first character converted to upper case
	if ~isempty(str)
		if ischar(str)
			str = [upper(str(1)), str(2:end)];
		elseif iscellstr(str)
			for ii = 1:length(str)
				if ~isempty(str{ii})
					str{ii} = [upper(str{ii}(1)), str{ii}(2:end)];
				end
			end
		else
			error('ucfirst', 'undefined function or variable ''ucfirst'' for input arguments of type ''%s''.', class(str));
		end
	end
end
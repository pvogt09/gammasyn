function [string, positions, newlinechars] = normalizenewline(string)
	%NORMALIZENEWLINE normalize newline symbols in a string to output of newline function
	%	Input:
	%		string:	string to normalize
	%	Output:
	%		string:	string with normalized newline symbols
	%		positions:		positions of newline characters (for multi-char newlines, position of the first char is true)
	%		newlinechars:	newline chars in the same order as indicated in positions
	newlines = char([
		13, 10;%\r\n
		10,	0;%\n
		13,	0%\r
	]);
	newlinesymbol = newline();
	returnposition = nargout >= 2;
	returnchars = nargout >= 3;
	if returnposition
		positions = false(3, size(string, 2));
		if nargout >= 3
			newlinechars = cell(size(newlines, 1), 1);
		end
		originalstring = string;
		parfor ii = 1:size(newlines, 1)
			currentnewline = newlines(ii, :);
			if currentnewline(1, 2) == char(0)
				if returnposition
					positions(ii, :) = originalstring == currentnewline(1, 1);
					if returnchars
						newlinechars{ii, 1} = currentnewline(1, 1);
					end
				end
			else
				if returnposition
					if size(string, 2) > 2
						r = originalstring == currentnewline(1, 1);
						n = originalstring == currentnewline(1, 2);
						positions(ii, :) = [r(1:end - 1) & n(2:end), false];
						if returnchars
							newlinechars{ii, 1} = currentnewline(1, :);
						end
					end
				end
			end
		end
		positions(2, [false, positions(1, 1:end - 1)]) = false;
		positions(3, positions(1, :)) = false;
	end
	% using position information takes 25 times longer (with parfor) and 2 times longer (without parfor) than strrep
	%string(any(positions, 1)) = newlinesymbol;
	%string(positions(1, :)) = [];
	for ii = 1:size(newlines, 1)
		if newlines(ii, 2) == char(0)
			string = strrep(string, newlines(ii, 1), newlinesymbol);
		else
			string = strrep(string, newlines(ii, :), newlinesymbol);
		end
	end
end
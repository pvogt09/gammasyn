function [is] = ischarstring(string)
	%ISCHARSTRING check for string (= char array or cellstring)
	%	Input:
	%		string:	string to check
	%	Output:
	%		is:		true if string is a string else false
	is = ischar(string) || iscellstr(string);
end
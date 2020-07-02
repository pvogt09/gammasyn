function [nl] = newline()
% NEWLINE Create newline character 
%   C = NEWLINE creates the character representing a newline.
%   NEWLINE is equivalent to char(10) or sprintf('\n').
% 
%   Example:
%      lines = ['Four score and', newline, 'seven years ago']
%
%      returns
%
%      Four score and
%      seven years ago
%
%   See also SPRINTF, CHAR, STRING, COMPOSE, SPLIT, JOIN
	nl = char(10);
end
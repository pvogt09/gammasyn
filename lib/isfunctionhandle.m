function [is] = isfunctionhandle(handle)
	%ISFUNCTIONHANDLE check for function handle
	%	Input:
	%		handle:	handle to check
	%	Output:
	%		is:		true if handle is a function handle else false
	is = isa(handle, 'function_handle');
end
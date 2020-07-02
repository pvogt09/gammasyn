function [varargout] = flexargout(varargin)
	%FLEXARGOUT alternative form of deal that works if number of output arguments is less than number of input arguments, mainly useful for anonymous functions with multiple return values
	%	Input:
	%		varargin:	input arguments to use as output arguments
	%	Output
	%		varargout:	output arguments
	varargout = varargin;
end
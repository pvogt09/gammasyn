function [has] = hasYALMIP()
	%HASYALMIP return if YALMIP toolbox is available
	%	Output:
	%		has:		true, if YALMIP toolbox is installed, else false
	persistent hasYALMIP;
	if isempty(hasYALMIP)
		yalmipfun = exist('yalmiptest', 'file');
		sdpvarclass = exist('sdpvar', 'class');
		sdpvarfile = ~isempty(which('sdpvar', '-all'));
		optimizerfun = exist('optimize', 'file');
		hasYALMIP = yalmipfun && (sdpvarclass || sdpvarfile) && optimizerfun;
	end
	has = hasYALMIP;
end
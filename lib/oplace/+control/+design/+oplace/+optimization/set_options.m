function options = set_options()
% Set options for the optimization function fminunc called from oplace

op_dat = read_optim_dat;
if ~isempty(op_dat)
	default = optimset('fminunc');
	options = optimset(default,op_dat);
else
	options = optimset('GradObj','on','Display','iter','HessUpdate','bfgs',...
				'LineSearchType','quadcubic','LargeScale','off','MaxIter',500,...
				'TolFun',1d-14,'TolX',1d-14);

end
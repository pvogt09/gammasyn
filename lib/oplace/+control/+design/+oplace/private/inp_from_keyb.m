function inp = inp_from_keyb(string, flag)
	inp = [];
	if nargin <= 0
		return;
	end

	if nargin == 1
		flag = 1;
	end
	com = 'input';
	if (flag == 1)
		inp = feval(com, string);
	end

	if (flag == 2)
		inp = feval(com, string, 's');
	end
end
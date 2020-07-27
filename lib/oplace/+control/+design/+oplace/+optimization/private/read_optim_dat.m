function sf = read_optim_dat()

if exist('optim.dat','file') == 2
	dat_file = textread(which('optim.dat'),'%s','delimiter','\n','commentstyle','matlab');
else
	sf = '';
	return;
end

str = '';
n = 1;
index = strmatch('',dat_file,'exact');
for i=1:length(dat_file)
	okay = 0;
	for j=1:length(index)
		if index(j) == i
			okay = 1;
			break;
		end
	end
	if ~okay
		str = [str,dat_file(i)];
		%str(n) = dat_file(i);
		n = n + 1;
	end
end
j = 1;
for i=1:length(str)/2
	s(i) = str(j);
	j = j+1;
	value = double(str2double((char(str(j)))));
	if ~isempty(value)
		sh(i)={value};
	else
		sh(i) = str(j);
	end
	if strcmp(str(j),'[]')
		sh(i) = [];
	end
	j = j+1;
end
sf = cell2struct(sh,s,2);
function output_w(npq,value,w,flag)

if (flag == 1)
	% Ausgabe von w1
	disp(sprintf('\n\nDefault nonzero weighting factors of the eigenvalues:\n'));
	disp(sprintf('#\tEigenvalue\t\tw1'));
	count=npq(4);
elseif (flag == 2)
	% Ausgabe von w2
	disp(sprintf('\n\nDefault weighting factors of the (real) data points:\n'));
	disp(sprintf('#\tdata point\t\tw2'));
	count=npq(5);
end

sb = '-----------------------------------------------------';
disp(sb);
for i=1:count
	s = num2str(value(i));
	if length(s)>=8
		disp(sprintf('%d.\t%s\t\t%s',i,s,num2str(w(i))));
	else
		disp(sprintf('%d.\t%s\t\t\t%s',i,s,num2str(w(i))));
	end
end
disp(sb);
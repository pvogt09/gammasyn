% Ausgabe der vorgegebenen Eigenwerte
function evalue_output(ew,ews,flag,npq)
sb = '--------------------------------------------------------------------------------------------';
disp(sprintf('\nOpen-loop eigenvalues:'));
disp(sb);
for i=1:npq(1)
	disp(sprintf('%d.\t%s',i,num2str(ews(i))));
end
disp(sb);
disp(sprintf('\nPredefined closed-loop eigenvalues:'));
disp(sb);
for i=1:npq(1)
	s = num2str(ew(i));
	switch flag(i)
	case 0
		disp(sprintf('%d.\t%s',i,s));
	case 1
		if length(s)>=8
			disp(sprintf('%d.\t%s\t<-- weighting factor=0 (too close to an open-loop eigenvalue)',i,s));
		else
			disp(sprintf('%d.\t%s\t\t<-- weighting factor=0 (too close to an open-loop eigenvalue)',i,s));
		end
	case 2
		if length(s)>=8
			disp(sprintf('%d.\t%s\t<-- weighting factor=0 (too close to another closed-loop eigenvalue)',i,s));
		else
			disp(sprintf('%d.\t%s\t\t<-- weighting factor=0 (too close to another closed-loop eigenvalue)',i,s));
		end
	end
end
disp(sb);
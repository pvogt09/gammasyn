% Textdatei: Ausgabe von w1

disp(sprintf('\n\nDefault nonzero weighting factors of the eigenvalues:\n'));
disp(sprintf('#\tEigenvalue\t\tw1'));
sb = '-----------------------------------------------------';
disp(sb);
for i=1:npq(4)
   s = num2str(ew_opt(i));
   if length(s)>=8
      disp(sprintf('%d.\t%s\t\t%s',i,s,num2str(w1(i))));
   else
      disp(sprintf('%d.\t%s\t\t\t%s',i,s,num2str(w1(i))));
   end
end
disp(sb);

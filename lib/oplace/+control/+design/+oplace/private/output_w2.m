% Textdatei: Ausgabe von w2

disp(sprintf('\n\nDefault weighting factors of the (real) data points:\n'));
disp(sprintf('#\tdata point\t\tw2'));
sb = '----------------------------------------------------';
disp(sb);
for i=1:npq(5)
   s = num2str(st_opt(i));
   if length(s)>=8
      disp(sprintf('%d.\t%s\t\t%s',i,s,num2str(w2(i))));
   else
      disp(sprintf('%d.\t%s\t\t\t%s',i,s,num2str(w2(i))));
   end
end
disp(sb);

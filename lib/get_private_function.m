function [fun] = get_private_function(path, name)
	%GET_PRIVATE_FUNCTION Funktionszeiger auf eine Funktion in einem private-Ordner erzeugen
	%	Input:
	%		path:	Pfad zu der privaten Funktion
	%		name:	Name der privaten Funktion
	%	Output:
	%		fun:	Funktionszeiger auf die private Funktion
	if ~ischar(path) || ~ischar(name)
		error('getfun:path', 'Der Dateipfad und der Funktionsname müssen vom Typ ''char'' sein.');
	end
	oldDir = cd(path);
	fun = str2func(name);
	cd(oldDir);
end
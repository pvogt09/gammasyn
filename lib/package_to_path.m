function [path] = package_to_path(package)
	%PACKAGE_TO_PATH Paketnamen in Pfadnamen konvertieren
	%	Input:
	%		package:	Paketname
	%	Output:
	%		path:		Pfadname
	if ~ischar(package)
		error('package2path:input', 'Der Paketname muss vom Typ ''char'' sein.');
	end
	path = strrep(['.', package], '.', [filesep, '+']);
end
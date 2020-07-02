function [content] = fgetcontent(filename, binary, encoding)
	%FGETCONTENT read file contents into variable
	%	Input:
	%		filename:	name of file to read from
	%		binary:		indicator, if file should be read in binary mode
	%		encoding:	encoding to use for reading file
	%	Output:
	%		content:	content of the specified file as char
	if nargin <= 1
		binary = false;
	end
	if nargin <= 2
		encoding = '';
		if ~islogical(binary) && ischar(binary)
			encoding = binary;
			binary = false;
		end
	end
	if ~isempty(encoding)
		if ~ischar(encoding)
			error('matlab:file:open', 'Encoding must be of type ''char'', not ''%s''.', class(encoding));
		end
	end
	if exist(filename, 'file')
		if binary
			if isempty(encoding)
				file = fopen(filename, 'r');
			else
				file = fopen(filename, 'r', 'native', encoding);
			end
			if file >= 0
				content = fread(file, '*char')';
				fclose(file);
			else
				error('matlab:file:open', 'File %s can not be opened.', filename);
			end
		else
			if isempty(encoding)
				content = fileread(filename);
			else
				file = fopen(filename, 'r', 'native', encoding);
				if file >= 0
					content = fread(file, '*char')';
					fclose(file);
				else
					error('matlab:file:open', 'File %s can not be opened.', filename);
				end
			end
			content = normalizenewline(content);
		end
	else
		error('matlab:file:exist', 'File %s does not exist.', filename);
	end
end
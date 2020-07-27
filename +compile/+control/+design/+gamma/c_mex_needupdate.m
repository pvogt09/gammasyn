function [needsupdate, changedfiles] = c_mex_needupdate(report)
	%NEEDUPDATE return if gamma area control constraint functions neeed to be recompiled
	%	Input:
	%		report:			indicator, if a report should be generated
	%	Output:
	%		needsupdate:	true, if gamma area control functions need to be recompiled
	%		changedfiles:	changed files that caused the recompilation
	if nargin < 1
		report = false;
	end
	[~, files] = compile.control.design.gamma.c_mex(false, true);
	needsupdate = false(size(files, 1), 1);
	changedfiles = cell(size(files, 1), 3);
	parfor ii = 1:size(files, 1)
		b = files(ii, :);
		if ~isempty(b{1, 2})
			build = b{1, 2};
			[upd, updfiles] = build(report);
			needsupdate(ii) = any(upd);
			changedfiles(ii, :) = {[], updfiles, []};
		else
			build = b{1, 1};
			[~, fileinfo] = build(false, true);
			if exist(fileinfo.mex, 'file') ~= 3
				needsupdate(ii) = true;
				changedfiles(ii, :) = {fileinfo, {fileinfo.mex}, b{1, 1}};
				continue;
			end
			if ~exist(fileinfo.m, 'file')
				error('compile:dependency', 'The file to compile does not exist.');
			end
			if ~exist(fileinfo.build, 'file')
				error('compile:dependency', 'The file to compile does not exist.');
			end
			mexfileinfo = dir(fileinfo.mex);
			mextime = mexfileinfo.datenum;
			buildfileinfo = dir(fileinfo.build);
			buildtime = buildfileinfo.datenum;
			if mextime < buildtime
				needsupdate(ii) = true;
				changedfiles(ii, :) = {fileinfo, {fileinfo.build}, b{1, 1}};
				continue;
			end
			dependencies = compile.dependencies(fileinfo.m);
			filechanges = cell(size(dependencies, 1), 1);
			for j = 1:size(dependencies, 1)
				dependentinfo = dir(dependencies{j});
				dependenttime = dependentinfo.datenum;
				if mextime < dependenttime
					needsupdate(ii) = true;
					filechanges{j, 1} = dependencies{j};
				end
			end
			if ~any(cellfun(@isempty, filechanges, 'UniformOutput', true))
				changedfiles(ii, :) = {fileinfo, filechanges, b{1, 1}};
			else
				changedfiles(ii, :) = {fileinfo, cell(0, 1), b{1, 1}};
			end
		end
	end
	if report
		if any(needsupdate)
			changes = changedfiles(needsupdate, :);
			changes = changes(~cellfun(@isempty, changes(:, 1), 'UniformOutput', true), :);
			if ~isempty(changes)
				cprintf('red', 'Gamma Area Control contraint functions must be recompiled.\n');
				for ii = 1:size(changes, 1) %#ok<FORPF> no parfor for command line output
					if ~isempty(changes{ii, 1})
						fprintf('\t%s\n', strrep(changes{ii, 1}.m, compile.destpath(), ''));
					end
				end
			else
				cprintf('green', 'Gamma Area Control contraint functions do not have to be recompiled.\n');
			end
		else
			cprintf('green', 'Gamma Area Control contraint functions do not have to be recompiled.\n');
		end
	end
end
classdef TestResult < uint8
	%TESTRESULT Aufzählung für Testergebnisse
	
	enumeration
		% misslungener Test
		FAILED(255);
		% bestandener Test
		PASSED(0);
		% übersprungener Test
		SKIPPED(1)
	end
	
	methods(Static=true)
		function [testresult] = getresult(res)
			%GETRESULT Ergebnis eines Tests in ein TestResult umwandeln
			%	Input:
			%		res:		Testergebnis
			%	Output:
			%		testresult:	Testergebnis als TestResult
			if isa(res, 'identifier.TestResult')
				testresult = res;
				return;
			end
			if isnumeric(res)
				if res == 0
					testresult = identifier.TestResult.PASSED;
				elseif res == 1
					testresult = identifier.TestResult.SKIPPED;
				else
					testresult = identifier.TestResult.FAILED;
				end
			elseif ischar(res)
				switch lower(res)
					case {'passed', 'pass'}
						testresult = identifier.TestResult.PASSED;
					case {'skipped', 'skip'}
						testresult = identifier.TestResult.SKIPPED;
					case {'failed', 'fail'}
						testresult = identifier.TestResult.FAILED;
					otherwise
						testresult = identifier.TestResult.FAILED;
				end
			else
				testresult = identifier.TestResult.FAILED;
			end
		end
	end
	
	methods
		function [passed] = ispassed(this)
			%ISPASSED zurückgeben, ob das Resultat als Bestanden gilt
			%	Input:
			%		this:	Instanz
			%	Output:
			%		passed:	Indikator, ob der Test als bestanden gilt
			passed = false(size(this));
			for i = 1:length(this) %#ok<FORPF> Initialisierung von parfor dauert länger als der Vergleich
				passed(i) = this(i) == identifier.TestResult.PASSED || this(i) == identifier.TestResult.SKIPPED;
			end
		end
		
		function [string] = tostring(this)
			%TOSTRING Testergebnis in einen String umwandeln
			%	Input:
			%		this:	Instanz
			%	Output:
			%		string:	Stringrepräsentation des Testergebnisses
			string = char(this);
		end
	end
end
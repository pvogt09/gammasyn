	Comments on test results
	------------------------
These functions were tested by running MATLAB 4.2c with
	1) the MATLAB Optimization Toolbox (on a 486 PC)
	2) the shareware UMSOLVE package developed by Rick Behrens (on a SUN Sparc10)
In both cases, the functions were tested both with user-supplied gradients and 
without.  Default settings were used, if possible.  In some cases, the default 
settings had to be changed in order to reach the solution.


1) ROSE		all test runs successful.
2) FROTH	all test runs successful.
3) BADSCP	using default settings, maximum iterations exceeded in all tests.
			eventual convergence obtained with increased iterations.
4) BADSCB	all tests successful. 
5) BEALE	all test runs successful.
6) JENSAM	runs successfully if x0(2)<= .365; if bigger it loops.         
7) HELIX	all test runs successful.
8) BARD		all test runs successful.
9) GAUSS	all test runs successful.
10)MEYER	using default settings, all runs fail.  With scaling and gradients, 
			UMSOLVE gets close to solution; Optimization toolbox, using
			gradients, is successful with increased number of iterations
			and linesearch options set to 1 (in place of default).
11)GULF		all test runs successful, although x* from UMSOLVE, and from 
			Optimization toolbox run without gradients not as accurate.
12)BOX		all test runs successful.
13)SING		all test runs successful; default termination criteria set 
			smaller than default for optimization toolbox runs.
14)WOOD 	all test runs successful; default termination criteria set 
			smaller than default for optimization toolbox runs;
			increased default number of iterations for opt toolbox run
			without gradients.
15)KOWOSB	all test runs successful.
16)BD		all test runs successful.
17)OSB1		UMSOLVE runs fail (loop) using default settings.  Using second linesearch
			option, and gradients, UMSOLVE works.  Optimization Toolbox runs
			require termination criteria smaller than default; more accurate
			solution found when using gradients.
18)BIGGS	all test runs successful.
19)OSB2		UMSOLVE run without gradients fails (loops), succeeds with gradients.
			Optimization toolbox runs succeed, using default settings.
20)WATSON 	UMSOLVE run without gradients fails (loops), succeeds with gradients.
			Optimization toolbox runs succeed, using default settings.
21)ROSEX	all test runs successful.
22)SINGX	all test runs successful, x* from UMSOLVE runs less accurate 
			than from opt toolbox
23)PEN1		all test runs successful.
24)PEN2		all test runs successful.
25)VARDIM	all test runs successful.
26)TRIG		for optimization toolbox tests, factor must be < 1.
27)ALMOST 	all test runs successful.
28)BV 		all test runs successful.
29)IE 		all test runs successful.
30)TRID 	all test runs successful.
31)BAND		all test runs successful.
32)LIN		all test runs successful.
33)LIN1		all test runs successful.
34)LIN0		all test runs successful.


% GA - Geometric Approach Toolbox.
%
% Version 5.1  20-Jan-2010
%
% NOTE: the GA Toolbox works with Matlab 5,6,7 and requires the Control System Toolbox.
%
% BASIC SUBSPACE OPERATIONS
%
%    ima : Orthogonalization.
%    ortco : Complementary orthogonalization.
%    ker : Kernel of a matrix.
%    sums : Sum of subspaces.
%    ints : Intersection of subspaces.
%    invt :  Inverse transform of a subspace.
%    subsplit : Invariant subspaces of stable/unstable/boundary modes of a matrix.
%    fattmat : Positive semi-definite matrix factorization.
%
% BASIC ROUTINES FOR SYSTEM ANALYSIS
%
%    vstar : Maximum output-nulling controlled invariant.
%    sstar : Minimum input-containing conditioned invariant.
%    rvstar : Maximum output-nulling reachable subspace.
%    effe : State feedback matrix for a controlled invariant.
%    effesta : Assigns all the assignable eigenvalues of a controlled invariant.
%    reldeg : Global relative degree.
%    gazero : Invariant zeros and invariant zero structure.
%    vstarg : Maximum internally stabilizable output nulling controlled invariant.
%    vstargh2 : Maximum internally stabilizable H2-norm-minimizing controlled invariant.
%    reldegv : Vector relative degree.
%    rhomin : Minimum response delay.
%    rhominv : Vector minimum response delay.
%    sstaru : Control sequence on Sstar.
%    sysinfo : Detects and displays the main properties of an LTI system.
%    minrega : Minimal realization with the geometric approach routines.
%
% BASIC ROUTINES FOR SYSTEM SYNTHESIS
%
%    hud : Synthesis of a feedforward decoupling compensator.
%    stoor : Output matrices for the Stoorvogel equivalence.
%    modfol : H2-optimal feedforward or feedback model following.
%    regtr : Francis regulator design with the geometric approach tools.
%    follobs : Full-order observer.
%    redobs : Reduced-order observer.
%    singcare : Regular or singular or cheap continuous-time LQR problem.
%
% AUXILIARY ROUTINES
%
%    mininv : Minimum A-invariant containig imB.
%    maxinv : Maximum A-invariant contained in imX.
%    stabi : Internal and external stability of an invariant.
%    compls : Complementation of an A-invariant.
%    miinco : Minimum (A,imC)-conditioned invariant containing imX.
%    mainco : Maximum (A,imB)-controlled invariant contained in imX.
%    robcoin : Maximal robust controlled invariant.
%    kalmcd : Kalman canonical decomposition.
%    maxpcs : Maximal perfect controllability subspace.
%    extendf : Squaring down for non left-invertible systems.
%    dualsys : Dual system.
%    adjsys : Adjoint system.
%    revsys : Reverse-time system.
%    can1 : Controllability canonical realization.
%    can2 : Controller canonical realization.
%    can3 : Observability canonical realization.
%    can4 ; Observer canonical realization.
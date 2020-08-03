# gammasyn


## Purpose

Pole placement or pole assignment is a major control design method for linear time-invariant systems.
*gammasyn* is a toolbox for Matlab which enables an easy (robust) pole region assignment by
* offering an easy description of the (robust) pole region assignment and
* providing a unified interface for a large number of optimizers.

As this control design method relies on "heavy" optimization, this toolbox offers functionality to create and use compiled and paralleled versions of crucial functions, provided the necessary Matlab toolboxes are available.

It works with continuous time and discrete time systems.

This documentation assumes a basic knowledge of control systems and the concept of pole placement.



### Pole region assignment

With static and/or structured output feedback it is in general not possible to assign the poles of the closed loop system freely.
In this case the control objective may be softened in the sense that no specific values for the closed loop poles are demanded, but a region is specified, in which the poles of the closed loop have to lie.

Another application is robust control.
Here the aim is to design a single controller which has to work for a whole set of plants with different parameters.
In this case it is not possible to achieve the exact same pole locations for all admissible plants, even with full state feedback, and one has to soften the objectives to a region as well.

Especially in the robust controller design with rather large parameter variation it often proves to be a hard task to place all poles in the desired region.
With this in mind, this toolbox allows to define two fundamental regions:
* A "hard" region which translates to an optimization constraint. This region should assure stability and a minimal performance.
* A "soft" region which translates to an optimization objective. The optimization tries to place all poles within or as near as possible to this region, which may be motivated by some more eager performance goals.

In addition, individual pole regions can be defined for the different plants within a robust design.
This allows to define a good control performance for plants near the rated plant and only some basic behavior for plants under "extreme" working conditions.




## Installation and setup


### Prerequisities

* Matlab R2015B (might also work with older versions but is not tested) (Octave is not supported because it does not support packages)
* Toolboxes:
	* Matlab Coder (optional, if objective and constraint functions should be compiled. If the eigenvalue derivative calculation method of van der Aa is to be used, at least Matlab R2016A is needed to support runtime recursion in generated code)
	* Control Systems Toolbox (optional, if `tf` and `ss` system descriptions should be used)
	* Robust Control Toolbox (optional, if uncertain or parametric system descriptions should be used)
	* Symbolic Toolbox (optional, if symbolic constraints should be used)

If optimizers of the Optimization Toolbox or Global Optimization Toolbox of Matlab are used, the corresponding toolbox is necessary.


### Installation
This repository has to be cloned or copied to a location accessible for the local Matlab installation.
For the ease of installation and usage, it contains all necessary code including the open source optimizers.


### Setup
To set up *gammasyn* within Matlab the script *startup.m* has to be executed.
The effects of this script are only temporary, therefore it has to be executed anew after a restart of Matlab.

This script works directly in the base workspace.
It cleans up behind itself, i.e. it clears all variables used by it, but it does not check beforehand if the variables were in use.
Therefore, it is recommended to execute this script with a cleared base workspace to avoid an inconsistent state.


## Minimal examples

### Test system

For the minimal examples the test system is a simple one-mass-oscillator described by

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c49ba4306d29141bff2884630138be34.svg?invert_in_darkmode" align=middle width=272.43645pt height=95.16181784999999pt/></p>
```
The nominal parameters are <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/943d82f958eef033f361fb6b7386d70a.svg?invert_in_darkmode" align=middle width=53.70238829999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/ff152ba203143cd6f6f7d125c0f36c9f.svg?invert_in_darkmode" align=middle width=56.04446099999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f5afd9293d92b9f847f43089fcf1e781.svg?invert_in_darkmode" align=middle width=71.04072029999999pt height=22.831056599999986pt/>.
For the robust design the parameters <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e7edab300a93d640814d40ac9152468e.svg?invert_in_darkmode" align=middle width=23.565549149999992pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/c29664102abae7738be73af4aca73735.svg?invert_in_darkmode" align=middle width=17.68841084999999pt height=22.831056599999986pt/> are assumed to be uncertain, given by

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c2e3d8f3ee369745e6bac24d5170b081.svg?invert_in_darkmode" align=middle width=98.4512991pt height=41.09589pt/></p>
```

The following function is used in the examples to construct the system for given parameter values <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e7edab300a93d640814d40ac9152468e.svg?invert_in_darkmode" align=middle width=23.565549149999992pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/c29664102abae7738be73af4aca73735.svg?invert_in_darkmode" align=middle width=17.68841084999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/da26587c8582f87667b0c05aee338b31.svg?invert_in_darkmode" align=middle width=16.24625309999999pt height=22.831056599999986pt/>:
```matlab
function [sys] = omo_sys(m, d, c)

	A = [0, 1; -c/m, -d/m];
	B = [0; 1/m];
	C = [1, 0];

	sys = struct('A', A, 'B', B, 'C', C);

end
```


### Pole region assignment

As simplest example static proportional output feedback is applied to the rated system

```matlab
sys = omo_sys(1, 10, 1000);
```

As only the position is measured, the poles cannot be placed arbitrarily.

#### Region

The target region is the sector shown in the following image.

<img src="docs/images/tex/tikz_ext/ex-omo-area.png" width=25%/>

It can be defined by
```matlab
polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];
```

For this example the solution can be determined analytically:

```math
	R \in [-991,\ -950]
```
(<img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> is negative, which means that it is actually positive feedback.
This is correct, as with this feedback structure the only possibility to dampen the system is to partly "compensate" the spring <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/da26587c8582f87667b0c05aee338b31.svg?invert_in_darkmode" align=middle width=16.24625309999999pt height=22.831056599999986pt/>.)

#### Pole region assignment

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions();

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```


#### Additional objective term

With the code above, all solutions for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> within the interval <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/18d0d2c1f9a644ae84ac09fbb73b0e26.svg?invert_in_darkmode" align=middle width=105.93634589999999pt height=24.65753399999998pt/> are "equally good" solution of the feasibility problem.
The exact value found depends on the initial value and the optimizer used.
(You could change the start value to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/71a21aee671591207ba93e51f462f4cb.svg?invert_in_darkmode" align=middle width=62.10047744999999pt height=22.831056599999986pt/> to observe a difference.)

This means there exists a certain degree of freedom which can be used for other purposes.
The following code finds the controller among the feasible ones with the smallest norm of the feedback matrix.
In this case, it is just the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> with the smallest absolute value, i.e. the unique solution now is <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/2d0877be8d7875b4ac8d2e03f5bc5a29.svg?invert_in_darkmode" align=middle width=100.74639464999998pt height=22.831056599999986pt/>.

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions(...
	'type', GammaJType.NORMGAIN,...
	'objective.normgain.R', (1)...
);

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```


### Robust pole region assignment

#### Systems

Now it is assumed that the mass and the damping coefficient are not exactly known but only intervals are given.
To apply the multiple model approach in a first step the "corner" models as well as the rated model are build and stored in one vector.

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)
];
```

#### Region

The target pole region is the same as in the example above.

```matlab
polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];
```

#### Robust pole region assignment

Also the call to *gammasyn* is the same as above, the only difference is that `sys` now contains five instead of one system.

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions();

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

This task proves to be feasible, i.e. there exists a controller gain for which the poles of all five closed loops lie within the specified region.



The following image shows the poles of all five closed loop systems

<img src="docs/images/tex/tikz_ext/ex-omo-area-probust-sol.png" width=25%/>

This multiple model approach is heuristic and makes no guarantees about the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8436263fb7e9bd7a5e2a43bbf948f9be.svg?invert_in_darkmode" align=middle width=19.40645189999999pt height=22.831056599999986pt/>-stability of the other systems described by the given parameter ranges.
It is advisable to check the properties for a larger subset of the admissible systems.
This is made in the following image, where the poles of 100 additional closed loop systems are shown in gray.

<img src="docs/images/tex/tikz_ext/ex-omo-area-probust-sol2.png" width=25%/>

## Method/Theory

### System and Controller

This framework considers static, possibly structured output feedback only.
As will be discussed later, this is actually not a restriction, as any dynamic feedback can be cast into this form.

In the simplest form for this framework a system is given by the three matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/73b6888541fd61edff8be10b90799836.svg?invert_in_darkmode" align=middle width=21.46124639999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/173837d7aca5fffcacc62295b7bf910b.svg?invert_in_darkmode" align=middle width=22.42585124999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/dcd597547a375785ec8589f3b8f30f5d.svg?invert_in_darkmode" align=middle width=22.05708614999999pt height=22.831056599999986pt/> of the state space representation

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dc79e7c316e6be7ff32bf486f6d2494a.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>
```
and the control loop is to be closed with the controller

```math
	u = -R y + F r
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c760dcb89001e7e63d44cd4e872d48d3.svg?invert_in_darkmode" align=middle width=17.00540324999999pt height=22.831056599999986pt/> is the reference value.
This leads to a closed loop

```math
	\dot x = (A - B R C) \cdot x + B F r
```
whose poles <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/755185fdc9bddd3acb135005c7eb4738.svg?invert_in_darkmode" align=middle width=26.900801399999988pt height=22.831056599999986pt/> are the solutions of the eigenvalue problem

```math
	\det(I \lambda_\nu - (A - B R C)) = 0
```


#### Mass matrix

As a small notational convenience in some cases, the model can be described as

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/86c3c1299a761865a5b29c693fc023d6.svg?invert_in_darkmode" align=middle width=108.91340955pt height=39.086746049999995pt/></p>
```
with the *invertible* mass matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/>.

The feedback has the same form as above which leads to the associated eigenvalue problem

```math
	\det(E \lambda_\nu - (A - B R C)) = 0
```
to determine the eigenvalues or poles of the closed loop system.


#### Differential feedback

To allow modeling true differential feedback, the model can be extended to

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/2370ad6488cd7df83298120c2b7a9603.svg?invert_in_darkmode" align=middle width=99.6973593pt height=63.744281699999995pt/></p>
```
for which the controller has the structure

```math
	u = - R y + K y' + F r
```

* The prime-notation <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/eaa5d1dc1ac16e43babbd082522328cc.svg?invert_in_darkmode" align=middle width=22.393529399999988pt height=24.7161288pt/> is not the same as <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/80ebc002da084114a605586296f32059.svg?invert_in_darkmode" align=middle width=17.78164739999999pt height=22.831056599999986pt/> but allows that not all or others outputs are used for the differential feedback than for the "normal" feedback. If all outputs should be used for the differential feedback, i.e. <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/ede960c4d58fa452c52f9e36f2d3b6e8.svg?invert_in_darkmode" align=middle width=52.96036019999999pt height=24.7161288pt/>, then <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/6e2e3534920142241655bd2794f24810.svg?invert_in_darkmode" align=middle width=61.51122779999999pt height=24.7161288pt/> can be chosen.
* The differential feedback is defined as positive feedback whereas the normal feedback is defined as negative feedback. This is a deliberate choice which leads to a more symmetric generalized eigenvalue problem

```math
	\det((I - B K C') \cdot \lambda_\nu - (A - B R C)) = 0
```



#### Process variables

As the model used here is an augmented system, as discussed below, the output <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c7dea865aff30fe079cee413fde80793.svg?invert_in_darkmode" align=middle width=17.78165399999999pt height=22.831056599999986pt/> doesn't generally reflect the actual process variables.
Therefore, the process variables for which sensible reference values (or set points) exist are described by an additional output equation:

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8b433ca5e00ee665ef532538f8b75dab.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=63.744281699999995pt/></p>
```



#### Full continuous time model

Combining all extensions, the most general system description used by this toolbox is

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c8ebec63975502639ce8a7df339af0fa.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=88.4018157pt/></p>
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/> must be an invertible matrix and the controller is given by

```math
	u = - R y + K y' + F r
```

The eigenvalues or poles of the closed loop are the solution of the generalized eigenvalue problem

```math
	\det((E - B K C') \cdot \lambda_\nu - (A - B R C)) = 0
```

The structure is depicted here:

<img src="docs/images/tex/tikz_ext/augsys-cl-full.png" width=75%/>




#### Discrete time model
The discrete time model is defined analogously to the continuous time case as

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dfddb00d931bdc6437d431201de77f74.svg?invert_in_darkmode" align=middle width=182.33157195pt height=89.9086386pt/></p>
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/> must be an invertible matrix and the controller is given by

```math
	u_k = - R y_k + K y'_k + F r_k
```

The discrete time analogous "derivative" output <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/a0ed369069bcc5f493c3d69a606a672b.svg?invert_in_darkmode" align=middle width=25.279823249999993pt height=24.7161288pt/> is only defined for accordance with the continuous time system matrices and serves no engineering purpose because it results in a non causal system.

The structure is depicted here:

<img src="docs/images/tex/tikz_ext/augsys-cl-full-discrete.png" width=75%/>



### Augmented System

The system given by the structure described above is an augmented system in the sense that it may contain parts of the controller.

The approach applied in this toolbox relies on static structured output feedback.
However, this is a very general approach, as all the controller dynamics can be added to the system, resulting in the "augmented system".

If for example the system

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dc79e7c316e6be7ff32bf486f6d2494a.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>
```
is to be controlled by a PI-controller

```math
	u = K_\mathrm{P} e + K_\mathrm{I} \int e \mathrm{d} \tau
```
with <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ba91339813ac55b903219e3d70e0b671.svg?invert_in_darkmode" align=middle width=75.31756814999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/c760dcb89001e7e63d44cd4e872d48d3.svg?invert_in_darkmode" align=middle width=17.00540324999999pt height=22.831056599999986pt/> being the reference value, which can be written in the state space representation

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/7f4eae2db028308f1bda4689bd86c082.svg?invert_in_darkmode" align=middle width=175.32334545pt height=39.086746049999995pt/></p>
```
the resulting augmented system is

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/f1914aa8ec2ac01062aa4a732c5c3bed.svg?invert_in_darkmode" align=middle width=238.0232877pt height=85.48022999999999pt/></p>
```
to which the static output feedback

```math
	u_\mathrm{a}
		=
			- \underbrace{<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d8391c69bfcba1709d7b82fc88e155bd.svg?invert_in_darkmode" align=middle width=89.72622615pt height=39.452455349999994pt/></p>}_{K} y_\mathrm{a}
			+ <p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/b7de5b2d7c9c67e4b856ecc84862f895.svg?invert_in_darkmode" align=middle width=40.970432249999995pt height=39.452455349999994pt/></p> r
```
is applied.
This is a *structured* feedback, as the second row of the feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> doesn't contain any free parameter but values which must not be altered by the optimizer.

More generally, if the given system is controlled with a general dynamic controller

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/cf49255b3c8608824934e820bdb04fb6.svg?invert_in_darkmode" align=middle width=183.59271149999998pt height=39.086746049999995pt/></p>
```
(where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/1ffcd284453ce446756898febd59e893.svg?invert_in_darkmode" align=middle width=32.18046644999999pt height=22.831056599999986pt/> to <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/cbbf25b739f309e298ec0f9b1d28e933.svg?invert_in_darkmode" align=middle width=33.46126409999999pt height=22.831056599999986pt/> may be structured) the augmented system is

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c75b6f37f28f18273f45c61d6f055cb9.svg?invert_in_darkmode" align=middle width=248.32008464999996pt height=85.48022999999999pt/></p>
```
which is closed by

```math
	u_\mathrm{a} = - \underbrace{<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5f62941933e29d07ef62da9822c4af51.svg?invert_in_darkmode" align=middle width=106.73770304999998pt height=39.452455349999994pt/></p>}_{K} y_\mathrm{a} + \underbrace{<p align="center"><img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/247a2a45afb83b997b692325258f2821.svg?invert_in_darkmode" align=middle width=35.296922099999996pt height=39.452455349999994pt/></p>}_{F} r
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> (and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/>) are generally structured corresponding to the structure of <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/1ffcd284453ce446756898febd59e893.svg?invert_in_darkmode" align=middle width=32.18046644999999pt height=22.831056599999986pt/> to <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/cbbf25b739f309e298ec0f9b1d28e933.svg?invert_in_darkmode" align=middle width=33.46126409999999pt height=22.831056599999986pt/> (and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/39c5e9ca979b2a643a9526ff951a82f6.svg?invert_in_darkmode" align=middle width=27.07770614999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/d427c052d18f55771936a702e62a9119.svg?invert_in_darkmode" align=middle width=27.07770614999999pt height=22.831056599999986pt/>).

As such structure is mandatory to achieve given controller structures as for example PI or PID controllers, this toolbox provides the possibility to define such structures.




### Pole region

The basic aim of this control design procedure is to determine <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> such that all poles <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/755185fdc9bddd3acb135005c7eb4738.svg?invert_in_darkmode" align=middle width=26.900801399999988pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/2701a8831eba9a316bda93bd233e5471.svg?invert_in_darkmode" align=middle width=105.79115085pt height=22.831056599999986pt/>, of the closed loop system lie within a certain region <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/8436263fb7e9bd7a5e2a43bbf948f9be.svg?invert_in_darkmode" align=middle width=19.40645189999999pt height=22.831056599999986pt/> of the complex plane.

This toolbox distinguishes between two regions:
* <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/36cb13ae33627acd528ae02168b32d0e.svg?invert_in_darkmode" align=middle width=46.43853884999999pt height=22.831056599999986pt/>: All poles <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/755185fdc9bddd3acb135005c7eb4738.svg?invert_in_darkmode" align=middle width=26.900801399999988pt height=22.831056599999986pt/> must lie within this region to consider the problem solved.
* <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dfe2859bce17e960d93ffcc75663c341.svg?invert_in_darkmode" align=middle width=41.20109399999999pt height=22.831056599999986pt/>: All poles should lie within or as near as possible to this region.



For a compact notation, the real part of a complex value is written as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/7ef9c6ad75ebe921d5bf5ae0bf1202ec.svg?invert_in_darkmode" align=middle width=19.115329199999987pt height=22.831056599999986pt/> and the imaginary part as <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/d2442540953667226e65f04c65f6deab.svg?invert_in_darkmode" align=middle width=19.95435419999999pt height=22.831056599999986pt/>, i.e. for example

```math
	\lambda_\nu = \sigma_\nu + \mathrm{j} \omega_\nu
```


#### Mathematical description of pole regions

A region is defined by one or the intersection of more areas.
Here, "area" refers to the "left side" of a curve in the complex plane.

```math
	z_\rho(\sigma,\ \omega)
		<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5ab607a84ec3481ad62a1c3f282b2682.svg?invert_in_darkmode" align=middle width=261.62698155pt height=69.0417981pt/></p>
```

Depending on the optimizer, a function <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4de785608ddea1555c130ed2ebc3bbc0.svg?invert_in_darkmode" align=middle width=69.8815755pt height=24.65753399999998pt/> should be differentiable twice after each argument.

A region is defined as a set of areas, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4eb4ef773e9577c24064e489f743496a.svg?invert_in_darkmode" align=middle width=135.19389179999996pt height=24.65753399999998pt/>.
The condition that all poles lie within this area translates to

```math
	z_\rho(\sigma_\nu,\ \omega_\nu) \leq 0 \forall \rho = 1,\ \ldots,\ r, \ \forall \nu = 1,\ \ldots, n
```

For the robust case, where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e7edab300a93d640814d40ac9152468e.svg?invert_in_darkmode" align=middle width=23.565549149999992pt height=22.831056599999986pt/> models are to be considered, the condition is

```math
	z_{\mu \rho}(\sigma_{\mu \nu},\ \omega_{\mu \nu}) \leq 0 \forall \rho = 1,\ \ldots,\ r_\mu, \ \forall \nu = 1,\ \ldots, n_\mu, \ \forall \mu = 1,\ \ldots,\ m
```
* The region may depend on the model <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/f52ae2404372e97b59c9eb89084d4fcd.svg?invert_in_darkmode" align=middle width=19.03737164999999pt height=22.831056599999986pt/>. This can be important from a practical point of view. If the uncertainty is rather large one may have to loosen the performance goals, described by the region, for corner case models.
* The system order <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/9c1d214bdbbfbbb1b7c68988915d4982.svg?invert_in_darkmode" align=middle width=27.81409784999999pt height=22.831056599999986pt/> may depend on the model <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f52ae2404372e97b59c9eb89084d4fcd.svg?invert_in_darkmode" align=middle width=19.03737164999999pt height=22.831056599999986pt/> as well.

As there are two pole regions, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/36cb13ae33627acd528ae02168b32d0e.svg?invert_in_darkmode" align=middle width=46.43853884999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/dfe2859bce17e960d93ffcc75663c341.svg?invert_in_darkmode" align=middle width=41.20109399999999pt height=22.831056599999986pt/>, there are also two sets of functions <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/c510280481df699418095e09a741dd09.svg?invert_in_darkmode" align=middle width=32.41212644999999pt height=22.831056599999986pt/>: <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/904b5ca3320f10fe8cb99890ea2716e5.svg?invert_in_darkmode" align=middle width=138.1885428pt height=24.65753399999998pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/4ac3f97061f7a5aae313696ec54c760a.svg?invert_in_darkmode" align=middle width=127.71365639999999pt height=24.65753399999998pt/>.




### Problem formulation

The aim is to determine the matrices of the controller

```math
	u = - R y + K y' + F r
```


#### Controller structure

Generally it is structured feedback, that is, the matrices cannot be chosen freely but certain entries are fixed and there may be additional conditions to be respected.
Mathematically fixed entries and linear dependencies between different entries can be expressed in the form
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8491ebd57c48fe73211deeea5aad6e6c.svg?invert_in_darkmode" align=middle width=149.0489649pt height=66.34700985pt/></p>
```
which allows dependecies of entries of the same matrix only or the more general form
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/42354ae19bf33315daa86f291ff6a80a.svg?invert_in_darkmode" align=middle width=207.01724174999998pt height=59.1786591pt/></p>
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/13dbd0b1769a1ce03b092176f89e0db5.svg?invert_in_darkmode" align=middle width=31.963517849999988pt height=22.831056599999986pt/> is the vectorization operator.
Mathematically the latter form comprises the precedent three equations, but this framework allows the specification in either form or both forms simultanously.

The notation used here is versatile.
Of course equality conditions of the form <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/719fcb6b8bed75e29a14b9c0e5c418e3.svg?invert_in_darkmode" align=middle width=82.49430254999999pt height=24.65753399999998pt/> actually simply reduce the effective number of optimization variables.
The same is valid for linear equation constraints between two and more optimization variables.
This is used by the toolbox when it constructs the problem, but for the sake of readability it is not denoted explicitly here.

The possibility to formulate linear equality conditions is necessary for the design of a structured controller.
Not necessary but possible are linear inequality conditions (aside from the ones resulting from the pole region constraints which are introduced below), which can be specified in the form

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/09aefdd06ad0a52f885432b46a5e2020.svg?invert_in_darkmode" align=middle width=149.96215245pt height=66.34700985pt/></p>
```
and
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/258cd22b7574404d589b7bc75909d0a8.svg?invert_in_darkmode" align=middle width=207.9304293pt height=59.1786591pt/></p>
```

To provide more flexibility, this toolbox allows also for nonlinear equality and inequality conditions,

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fa25999ffd4d3436835309d9ff4821a4.svg?invert_in_darkmode" align=middle width=88.86044475pt height=140.31961184999997pt/></p>
```

Instead of referring to these seven equations and seven inequalities in the feasibility and optimization problems that follow, it is used the shorter notation

```math
	(R,\ K,\ F) \in \mathcal{S}
```
For example

```math
	\min_{(R,\ K,\ F) \in \mathcal{S}} J
```






#### Pole regions

These are translated into constraints or into an objective function, depending on the type of region (hard or soft) and the capabilities of the optimizer.


##### Hard pole region - Constrained optimization

If the optimizer supports inequality constraints directly,

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81a05acc3b38a9a4007669e22df25d9c.svg?invert_in_darkmode" align=middle width=598.3804348499999pt height=56.5021809pt/></p>
```

If no additional objective function is given, i.e. <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/01eb1509890c54fdde448ee1d585c93d.svg?invert_in_darkmode" align=middle width=49.96563824999999pt height=22.831056599999986pt/>, this is a feasibility problem.

The weights <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d7e5b844958d67af4f5b2fffc16024bf.svg?invert_in_darkmode" align=middle width=36.53598134999999pt height=22.831056599999986pt/> are not necessary from a theoretical - and mostly practical - point of view.
Generally they should be set to 1.

* The weights can be used to reverse the left and right side of an area.
* They could be used to "help" the optimizer to find a solution.


##### Hard pole region - Unconstrained optimization using a loss function

If the optimizer doesn't support inequality constraints (or for the soft pole region <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dfe2859bce17e960d93ffcc75663c341.svg?invert_in_darkmode" align=middle width=41.20109399999999pt height=22.831056599999986pt/>) the inequality constraints have to be transformed into an objective function using loss functions.

In most cases the resulting objective function has the form

```math
	J_\Gamma = \sum_{\mu=1}^m \sum_{\nu=1}^{n_\mu} \sum_{\rho=1}^{r_\mu} j(z_{\mu \rho}(\sigma_{\mu \nu},\ \omega_{\mu \nu}))
```
i.e. for each combination of model, pole and area the value of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ce521aa40a0f103efbda012ca0aa06dd.svg?invert_in_darkmode" align=middle width=109.95237659999998pt height=24.65753399999998pt/> is assessed by some loss function <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/8bfa4e773baaafee75f9b9e78a22a43a.svg?invert_in_darkmode" align=middle width=16.84286504999999pt height=22.831056599999986pt/> and the sum is used as objective function.
The following table lists the most common choices for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8bfa4e773baaafee75f9b9e78a22a43a.svg?invert_in_darkmode" align=middle width=16.84286504999999pt height=22.831056599999986pt/>:

| loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/b3b00dd5c9e494ec6a38ee644803c938.svg?invert_in_darkmode" align=middle width=82.13174144999999pt height=24.65753399999998pt/> |
| --- | --- |
| Quadratic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5533ae840db83be5d6995f62fc9cbf74.svg?invert_in_darkmode" align=middle width=173.58299969999996pt height=26.76175259999998pt/> |
| <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/83771fef97f748cb0fed8d9e1d007c87.svg?invert_in_darkmode" align=middle width=21.41178269999999pt height=22.831056599999986pt/> loss function | <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/5aca9b1e154d3cff0896bb38a81718a5.svg?invert_in_darkmode" align=middle width=156.16276499999998pt height=24.65753399999998pt/> |
| Exponential loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8ef7fd695c3f0a0d83e7cacdaa98cbd6.svg?invert_in_darkmode" align=middle width=129.67873544999998pt height=24.65753399999998pt/> |
| Logarithmic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/95bfe004bb048e4146b3f3c917360dc3.svg?invert_in_darkmode" align=middle width=155.9343918pt height=24.65753399999998pt/> |

* The downside of the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/83771fef97f748cb0fed8d9e1d007c87.svg?invert_in_darkmode" align=middle width=21.41178269999999pt height=22.831056599999986pt/> loss function is that it is not differentiable on the border curves.
* The logarithmic loss function is an inner penalty function which is not defined for any pole not lying within the defined region. Therefore, it can only be used if the initial value for the optimization variables is feasible.
* The exponential loss function may lead to very high values if the poles are far out of the regions. This may results in problems if the initial value for the optimization variables are not chosen carefully. In this case, the quadratic loss function may be a better choice.

An alternative objective function is based on the Kreisselmeier-Steinhauser function,

```math
	J = f_\mathrm{max,KM} + \frac{1}{\rho_\mathrm{KM}} \cdot \ln\left( \sum_{\mu=1}^m \sum_{\nu=1}^{n_\mu} \sum_{\rho=1}^{r_\mu} \exp(\rho_\mathrm{KM} w_{\mu \rho} z_{\mu \rho}(\cdot,\cdot) - f_\mathrm{max,KM}) \right)
```
which is an (rough) approximation of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8eb0cf9d781959aad64df7ceddc2abd7.svg?invert_in_darkmode" align=middle width=135.15823035pt height=24.65753399999998pt/>.

The resulting optimization problem is

```math
	(R^\star,\ K^\star,\ F^\star) = \underset{(R,\ K,\ F) \in \mathcal{S}}{\arg \min} J_{\Gamma, \mathrm{hard}}
```

#### Soft pole region

If a constrained optimizer is used, a second pole region can be defined.
This soft region is treated in the same way as unconstrained optimizers treat the hard pole region, i.e.
* The soft pole region <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dfe2859bce17e960d93ffcc75663c341.svg?invert_in_darkmode" align=middle width=41.20109399999999pt height=22.831056599999986pt/>
* It makes only sense if the optimizer supports inequality constraints


```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/265bdbeb3e6c19285ed3623bf9caf5cf.svg?invert_in_darkmode" align=middle width=565.4974578pt height=56.5021809pt/></p>
```


#### Additional objective terms

Additional objective functions can be selected.


##### Controller norm
In order to get a small control action, the controller matrices can be minimized by the choice of `GammaJType.NORMGAIN` as objective type with the objective function
```math
	J_\mathrm{Ctrl} = \| W_\mathrm{R} \odot (R - S_\mathrm{R}) \|_\mathrm{F}^2 + \| W_\mathrm{K} \odot (K - S_\mathrm{K}) \|_\mathrm{F}^2 + \| W_\mathrm{F} \odot (F - S_\mathrm{F}) \|_\mathrm{F}^2
```
where the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e509759439a4ef1b95bc06ed8208b1d5.svg?invert_in_darkmode" align=middle width=26.94070664999999pt height=22.831056599999986pt/> of appropriate dimension are chosen for weighting.


##### Condition of the eigenvector matrix
For greater robustness of the closed loop, the condition number of the eigenvector matrix can be minimized by the choice of `GammaJType.EIGENVALUECONDITION` with the objective function
```math
	J_\mathrm{EV} = \mathrm{cond}(V)
```

##### Norm of the Lyapunov matrix
Another possibility for achieving greater robustness against time varying unstructured uncertainty in the system matrix of the closed loop, is the minimization of the norm of the Lyapunov matrix of the closed loop system, which can be achieved by the choice of `GammaJType.LYAPUNOV`.
The objective function in this case has the form
```math
	J_\mathrm{Lyap} = -\frac{1}{\|\tilde{P}_{11}\|_{F}^2} + \frac{1}{\|\tilde{P}_{22}\|_{F}^2}
```
where the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4c28a537aacd5fac8f4df776ace4ccfb.svg?invert_in_darkmode" align=middle width=22.12787279999999pt height=22.831056599999986pt/> in the Lyapunov equation can be chosen independently for every multiple model.
The matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dbd9e92afae6d85716b521a8b16d61c5.svg?invert_in_darkmode" align=middle width=33.61309709999999pt height=30.267491100000004pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/8e0de3a3fde533378f202abc749498b6.svg?invert_in_darkmode" align=middle width=33.61309709999999pt height=30.267491100000004pt/> which correspond to the unstable and stable part of the system respectively stem from a Schur decomposition of the closed loop system matrix where the unstable system matrix is replaced by <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/2a1d0d29316f1346b3d73abf6df529fc.svg?invert_in_darkmode" align=middle width=41.55243839999999pt height=22.831056599999986pt/> in the continuous time case and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/2801e4248c173ffc825d75a59ea47cc3.svg?invert_in_darkmode" align=middle width=39.10970414999999pt height=26.76175259999998pt/> in the discrete time case.


#### Complete optimization problem

##### Constrained optimizers

For constrained optimizers the "full" optimization problem is
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5d0ae9573889fc992940426d025b18f1.svg?invert_in_darkmode" align=middle width=623.0152697999999pt height=56.5021809pt/></p>
```

For unconstrained optimizers the "full" optimization problem is
```math
	(R^\star,\ K^\star,\ F^\star) = \underset{(R,\ K,\ F) \in \mathcal{S}}{\arg \min} w_\Gamma J_\mathrm{\Gamma,hard} + w_\mathrm{Ctrl} J_\mathrm{Ctrl} + w_\mathrm{EV} J_\mathrm{EV}
```
In this case only "simple" linear equality conditions can be imposed for the entries of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/86cac5c1ecaa751197c77cbfb01c0dc7.svg?invert_in_darkmode" align=middle width=26.59824584999999pt height=24.7161288pt/> which can be incorporated directly by reducing the number of optimization variables.


## Usage


```matlab
[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts,...
	[Rbounds [, Rnonlin]]...
);
```

### Return values

* `R_opt`: Found solution, the format depends on relevant
	* If `sys` defines neither <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/2a400357c40ac1a7a9c87a249c6796a6.svg?invert_in_darkmode" align=middle width=26.66895989999999pt height=24.7161288pt/> nor <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/7c35e2cd0b773a91d1eb4c717108d4df.svg?invert_in_darkmode" align=middle width=37.67934389999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/5b753e899e8d08ad505db14679284ed3.svg?invert_in_darkmode" align=middle width=39.54009014999999pt height=22.831056599999986pt/>, then `R_opt`  is simply a numerical matrix corresponding to <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/bae6ff213faeb39b314e9b7bb2202984.svg?invert_in_darkmode" align=middle width=29.29800719999999pt height=22.831056599999986pt/>
	* If `sys` defines all matrixes, then `R_opt` is a cell array with three numerical entries corresponding to the solution <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/47a2f14955944778512cb25f1035e7d0.svg?invert_in_darkmode" align=middle width=110.75919359999999pt height=24.65753399999998pt/>
* `J_opt`: value of the objective function at `R_opt`
* `info`: structure with additional information about the result


PLEASE NOTE: The current version of the toolbox "ignores" the prefilter <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/>.
Current work aims to extend the toolbox for the design of coupling and decoupling controllers.
In theses cases the manipulation of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> is necessary.
Therefore, it is included in the API.
But in the current release version, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> will always be returned as the initial value or a zero matrix.


### System `sys`

The argument `sys` describes one or more systems.
A system is described by a `struct` whose fields correspond to the matrices of a state space realization.
If more than one system is given, `sys` is a vector of structs.

These systems are always the augmented systems which may include (parts of) the controller.

In the simplest form a system is given by the three matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/73b6888541fd61edff8be10b90799836.svg?invert_in_darkmode" align=middle width=21.46124639999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/173837d7aca5fffcacc62295b7bf910b.svg?invert_in_darkmode" align=middle width=22.42585124999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/dcd597547a375785ec8589f3b8f30f5d.svg?invert_in_darkmode" align=middle width=22.05708614999999pt height=22.831056599999986pt/> of the state space representation

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dc79e7c316e6be7ff32bf486f6d2494a.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>
```

Optionally, this toolbox allows to specify a mass matrix and to design ideal differential feedback as well as it can design a prefilter.
The "full" system form is given by

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c8ebec63975502639ce8a7df339af0fa.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=88.4018157pt/></p>
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/> must be an invertible matrix.

| Fields | Remark |
| --- | --- |
| A, B, C | Minimum form |
| C_dot | Optional |
| E  | Optional |
| C_ref, D_ref | Optional |


PLEASE NOTE: A field D may be given (and is also returned by functions of this toolbox), but it must be a zero matrix of compatible size! (Future versions may allow systems with feed through but in the current version a non-zero D results in undefined behavior.)

If the additional flexibility of differential feedback or reference outputs is not needed, it is also possible to supply system descriptions that are included in Matlab's Control System toolbox as `ss`, `tf`, `uss` or `dss`.


### Controller structure `Rfixed`

The controller structure is given by the mandatory argument `Rfixed` as well as the optional arguments `Rbounds` and `Rnonlin`.

In the following, the structure of one of the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> are explained.
(It is the same for each one.)
They are combined by forming a cell array, i.e. if all three matrices are used:
```matlab
Rfixed = {Ra_fixed, Ka_fixed, Fa_fixed, RKFa_fixed}
```
If no dependencies between different gain matrices are needed, this can be reduced to
```matlab
Rfixed = {Ra_fixed, Ka_fixed, Fa_fixed}
```
If only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> is used,
```matlab
Rfixed = {Ra_fixed, Ka_fixed}
```
and if only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> is used,
```matlab
Rfixed = {Ra_fixed}
```

* Fixed values

	`Ra_fixed = {Rfix, Rval}`

	`Rfix` is a logical matrix with `true`-entries marking the fixed entries.
	`Rval` is a numerical matrix where the fixed values are given.
	The non-fixed values are marked as `NaN` in this matrix.

	(This is redundant, as `Rfix = ~isnan(Rval)` but is needed to distinguish the format.)

	For example, if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dd92860b95894e6ee841c5f145daf6f8.svg?invert_in_darkmode" align=middle width=123.03464414999999pt height=47.6716218pt/> with the parameters <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/30c3d28d77c09e9badacba39ddbed7c0.svg?invert_in_darkmode" align=middle width=26.20632794999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/9828cb0c9bfc81bafba8a244a6d5eb5a.svg?invert_in_darkmode" align=middle width=22.11958979999999pt height=22.831056599999986pt/> being free, the definition of the structure would be
	```matlab
		{[false, false; true, true], [NaN, NaN; 1, 0]}
	```

* Linear dependencies between controller parameters

	`Ra_fixed = {Zlhs, Zrhs}`

	Linear dependencies are given by linear equations of the form

	```math
	\sum_{i,j} (Z_k \odot R) = z_k
	```
	where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d1866f3bd35e683d8cd331a170875548.svg?invert_in_darkmode" align=middle width=29.223638399999988pt height=22.831056599999986pt/> means element-wise multiplication (Hadamard product).
	If there is more than one equation <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fbdb696db6a1b0322aa20999d63696f2.svg?invert_in_darkmode" align=middle width=18.207811049999993pt height=22.831056599999986pt/>, the matrices <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f7cfe92bfcf30ad9f3819deb842583ce.svg?invert_in_darkmode" align=middle width=28.44187334999999pt height=22.831056599999986pt/> are stacked along the third dimension in `Zlhs`.
	I.e, if `Nz` linear dependencies are specified, the dimensions of `Zlhs` and `zrhs` are `size(Zlhs): [size(R, 1), size(R, 2), Nz]` (for the combined constraints `size(Zlhs): [size(R, 1), size(R, 2) + size(K, 2) + size(F, 2), Nz]`) and `size(Zrhs): [Nz, 1]`, resp.

	For example, if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e7c635b38c4d368b3e2276b5ffb04124.svg?invert_in_darkmode" align=middle width=140.99887065pt height=47.6716218pt/> with <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/1004f031aecbdc845e115b29a111a469.svg?invert_in_darkmode" align=middle width=22.02156494999999pt height=22.831056599999986pt/> being free but subject to the constraints <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/791f7e515eb6072bfc082980f3829fe8.svg?invert_in_darkmode" align=middle width=97.34004884999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/eb64dfe5317e3c242ddef997f1ef45a6.svg?invert_in_darkmode" align=middle width=60.63163919999999pt height=22.831056599999986pt/>, the definition of the structure would be
	```matlab
		{cat(3,...
			[1, -1, 0; 0, 0, 0],... % lhs of r_1 - r_2 = 0
			[1, 0, -1; 0, 0, 0],... % lhs of r_1 - r_3 = 0
			[0, 0, 0; 1, -1, 0],... % lhs of r_4 - r_5 = 0
			[0, 0, 0; 0, 0, 1]...   % lhs of r_6 = 1
			),...
		[
			0;    % rhs of r_1 - r_2 = 0
			0;    % rhs of r_1 - r_3 = 0
			0;    % rhs of r_4 - r_5 = 0
			1     % rhs of r_6 = 1
		]
		}
	```
	* As said above, linear dependencies for `K` and `F` can be specified in the same way in `Ka_fixed` and `Fa_fixed`. For dependecies involving all matrices `RKFa_fixed` can be used, specifying `{Zlhs, Zrhs}` corresponding to
	```math
		\sum_{i,j} (Z_k \odot <p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d292f58aeafdaf08bc59e1c2ca6d3cb2.svg?invert_in_darkmode" align=middle width=87.1747602pt height=19.726228499999998pt/></p>) = z_k
	```
* Symbolic dependencies between controller parameters
If the Symbolic Math Toolbox is available, it is also possible, to formulate the controller coefficient constraints as symbolic expressions.
This can be achieved by specifying the symbolic gain matrix in the first element of a cell array and the symbolic equation system in the second element of the cell array for every gain matrix like in
```matlab
	R = sym('R', [2, 2]);
	K_I = sym('K_I');
	K_P = sym('K_P');
	R(1, 1) = K_P;
	R(1, 2) = K_I;
	Ka_fixed = {R, [
		K_I == K_P;
		R(2, 1) == 1;
		R(2, 2) == 0
	]};
```
for the proportional part of a PID controller where integral and proportional action should have the same coefficient value.


As it may be cumbersome to construct the augmented system as well as to specify the structure of the controller matrices manually for the desired controller structure, this toolbox provides helper functions for common controller structures.

Alternatively the capabilities of the parameterized and uncertain system objects of the Control System Toolbox and the Robust Control Toolbox can be used.


#### Predefined controller setups

| Class | Parameters | Description |
| --- | --- | --- |
| ConstantOutputFeedback | |
| DynamicOutputFeedback | nD | nD: order of the dynamic controller |
| ObserverDirectOutputFeedback | |
| ObserverOutputFeedback | |
| ObserverStateFeedback | |
| PDOutputFeedback | |
| PIDOutputFeedback | |
| StateFeedback | |


```matlab
sys = omo_sys(1, 10, 1000);

controller = control.design.outputfeedback.PIDOutputFeedback();

% construct augmented system
sys_augmented = controller.amend(sys);

% get structure description
[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```
This works as well if `sys` contains several systems:
```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)
];

controller = control.design.outputfeedback.PIDOutputFeedback();

sys_augmented = controller.amend(sys);

[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```


#### Using Matlab system definitions
Instead of supplying the system descripton as a structure with the neccessary fields, it is also possible to use a `tf` or `ss` object (or array in case of multiple models) as system.
```matlab
sys = omo_sys(1, 10, 1000);
sys = ss(sys.A, sys.B, sys.C);

controller = control.design.outputfeedback.PIDOutputFeedback();

% construct augmented system
sys_augmented = controller.amend(sys);

% get structure description
[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```



### Initial value `R0`

An initial value for the feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> must be given.

#### Numerical value
It must have the same dimension as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/>, i.e. `size(R0) : [size(sys.B, 1), size(sys.C, 2)]`.
If <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> contains fixed values, the corresponding entries of `R0` are ignored.

An inital value for the differential feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> may be given.
In this case, `R0` is a cell array containing two matrices,
```matlab
R0 = {Ra0, Ka0}
```
If the structure employs differential feedback but no initial value for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> is given, it is set to <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/1e6574b129f717202571fb320ccf3996.svg?invert_in_darkmode" align=middle width=17.35165739999999pt height=22.831056599999986pt/>.
Supplying an initial value for the prefilter gain `<img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/>` is possbile with three matrices
```matlab
R0 = {Ra0, Ka0, Fa0}
```


#### Multiple numerical values

It is possible to provide more than one initial value.
In this case the optimization problem is solved for each (set of) initial value(s) and the best solution is returned.

To provide multiple initial values, instead a two-dimensional matrix `R0`, a cell array of two two-dimensional matrices `{Ra0, Ka0}` or a cell array of three two-dimensional matrices `{Ra0, Ka0, Fa0}` three-dimensional values have to be provided.


#### InitialValue
If the user supplied initial value for optimization is to be changed by a further processing step to make it more suitable for the pole region assignment problem by taking the multiple models into account, this is possible by the use of the `InitialValue` class.
This class is a container for different `InitialValueElement` objects that may implement this processing, e.g. using the `oplace` toolbox for design of structurally constrained output feedback controllers with pole assignment developed at the institute of automatic control at the TU Darmstadt.
Currently this task is left to the user and there are only two simple `InitialValueElement` classes defined, `GammasynInitialValueElement` which is basically only a wrapper for numeric initial values described above and `RandomInitialValue` which returns random initial values of appropriate size.
An initial value consisting of a random value and a numeric value can be constructed by
```matlab
R0 = control.design.gamma.InitialValue(...
	control.design.gamma.RandomInitialValue(2),...
	control.design.gamma.GammasynInitialValueElement(Ra0)...
)
```
which creates two random initial values and the numeric initial value `Ra0`.



### Region definition

A region is defined by one or the intersection of more areas.
Here, "area" means a function which maps any point of the complex plane, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/10725299aee15314a4e0e1094cd54a1a.svg?invert_in_darkmode" align=middle width=55.05128639999998pt height=22.831056599999986pt/>, to a real number, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/4de785608ddea1555c130ed2ebc3bbc0.svg?invert_in_darkmode" align=middle width=69.8815755pt height=24.65753399999998pt/>.
It is zero on the border of the area, has negative values on the side at which poles must or should lie and positive values on the other side.
It should be differentiable twice.

These are translated into constraints or into an objective function, depending on the type of region (hard or soft) and the capabilities of the optimizer.
If it is translated to an objective function, the area functions are applied to a loss function defined with `GammaJType`.

To allow for an efficient optimization, an area should not only provide its value when called for a certain point of the complex plane, but also returns its first (and second) derivatives.

For the common areas these functions are predefined, such that there should be seldom the need to define custom area functions.


Generally, a region (hard or soft) is defined by a row vector of `control.design.gamma.area`-Objects, for example
```matlab
region = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
]
```
If a robust design is performed, i.e. there is more than one model defined, *and* different regions should be given for different models, then the regions are defined by a matrix, where every row describes the region for the corresponding model in the `sys`-vector.
(If the same region is used for all models, only one has to be defined.)
For three systems the regions may be defined for example by
```matlab
region = [
	control.design.gamma.area.Line(1, 0), control.design.gamma.area.Imag(1, 1);
	control.design.gamma.area.Line(0.7, 0), control.design.gamma.area.Imag(1, 1);
	control.design.gamma.area.Line(0.5, 0), control.design.gamma.area.None()
]
```
If the number of areas differs for the different regions, the `None`-area can be used to fill the rows of matrices.

If both a hard and a soft region are refined, `polearea` is a cell array with two elements, each containing a vector of areas:
```matlab
polearea = {hardregion, softregion}
```
If only a hard region is defined, the soft region can be set to an empty matrix
```matlab
polearea = {hardregion, []}
```
or equivalently, `polearea` can be set directly as the hard region vector or matrix
```matlab
polearea = hardregion
```

NOTE: If an unconstrained optimizer is used, only the hard region is used.
I.e. if a soft region is defined, it is simply ignored.


If a constrained optimizer is used, and the user wants to specify a soft region only, that could be achieved with
```matlab
polearea = {control.design.gamma.area.None(), softregion}
```
explicitly or by specifying `[]` implicitly.
However, it is advisable to specify at least the "minimal" hard region, i.e. the left half plane for continuous time systems or the unit circle for discrete time systems.






#### Predefined areas

| Area | Parameters | Description | |
| ----------- | --- | --- | --- |
| Circle <br/> Circlesquare | R | R: Radius | <img src="docs/images/tex/tikz_ext/area-circle.png" width=50%/>
| Ellipse <br/> Ellipsesquare | a, b | a: semi axis along real axis <br/> b: semi axis along imaginary axis |  <img src="docs/images/tex/tikz_ext/area-ellipse.png" width=50%/>
| Hyperbola <br/> Hyperbolasquare | a, b | a: semi axis along real axis <br/> b: semi axis along imaginary axis | <img src="docs/images/tex/tikz_ext/area-hyperbola.png" width=50%/>
| Imag | a, b | Vertical line <br/> For left part: a: 1, b: neg. position of line on real axis <br/> For right part: a: -1, b: position of line on real axis | <img src="docs/images/tex/tikz_ext/area-imag-left.png" width=50%/> <img src="docs/images/tex/tikz_ext/area-imag-right.png" width=50%/>
| Line | a, b | a: slope <br/> b: intercept <br/> The "left" area is always the part above the line! | <img src="docs/images/tex/tikz_ext/area-line.png" width=50%/>
| LogSpiral | r, k | r: start point on real axis <br/> k: polar slope | <img src="docs/images/tex/tikz_ext/area-logspiral.png" width=50%/>
| None |
| PolyEllipse <br/> PolyEllipsesquare |

All areas (except None, where it is ignored) accept an additional optional parameter *shift*.
This parameter is a complex number which allows to shift the origin of the "base" coordinate system defining the area.
I.e. for circles, *shift* defines the center point.
If *shift* is not specified, it is set to zero.


##### "square" versions


Some of the areas are more naturally defined by a squared expression.
Taking the square root results in a function that may not be differentiable, if the argument of the root can attain zero.
Therefore, it may be a good choice to work with the square form directly.

However, depending on the usage this may lead to other problems if squared and non-squared forms are mixed, as in the first case the "distance" is measured squared and in the latter it is measured "normally".
Therefore, the non-squared versions are provided as well.

And an additional definition has to be added, if the shape doesn't distinguish between the left and right half plane.


#### Custom areas

```matlab
z = areafun(re, im)
```

```matlab
[z, dzdre, dzdim] = areafun(re, im)
```

```matlab
[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = areafun(re, im)
```


##### Example

In this simple example, the _exterior_ of a circle with radius <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> is chosen as desired area.
In square form, the area is defined by

```math
	z(\sigma,\ \omega) = R^2 - \underbrace{(\sigma^2 + \omega^2)}_{\textnormal{distance}^2}
```
which gives the derivatives

```math
	\frac{\mathrm{d}z}{\mathrm{d}\sigma} = -2 \sigma
	\qquad
	\frac{\mathrm{d}z}{\mathrm{d}\omega} = - 2 \omega
```

Accordingly, the function could be implemented as
```matlab
function [z, dzdre, dzdim] = mindistance_area(re, im, R)
	z = R^2 - re^2 - im^2;
	dzdre = -2 * re;
	dzdim = -2 * im;
end
```
and used as
```matlab
polearea = @(re, im) mindistance_area(re, im, 5)
```
Multiple regions can be defined by simply letting the function return row vectors of function and gradient values instead of scalars.




### `weight`: Weighting of areas

As described above, each area and model is associated with a weight <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d7e5b844958d67af4f5b2fffc16024bf.svg?invert_in_darkmode" align=middle width=36.53598134999999pt height=22.831056599999986pt/> which appears within the inequality constraints or the derived loss functions.

`weight` is structured like `polearea`.
If both a hard and a soft region is defined, `weight` is a cell array with two numeric elements (scalar, vector or matrix)
```matlab
weight = {hardweight, softweight}
```
If only a hard region is given, `weight` might be set directly to corresponding numeric element
```matlab
weight = hardweight
```

`hardweight` and `softweight` can be defined in one of the following forms:

* Scalar value: All weights (for all areas and all models) are set to this value. This is the standard choice for the hard region, where all weights are normally set to one.
* Row vector: Each entry is the weight of the corresponding area. If more than one model is used, the weights may differ for the different areas, but they are the same for all models.
* Column vector: Each entry is the weight for all areas for the corresponding model.
* Matrix: Each entry is the weight for the corresponding model (row) and area (column).


**Remarks:**

* If the hard region is translated to inequality constraints, the weights for the hard region are more academic (a weight might be set to a negative value to inverse the area). With some experience the weights may be used to "help" the optimizer to find a feasible solution.
* If negative weights are used, the option `allownegativeweight` must be set to true.



### `gammaopts`: Objective function and associated options

The options, which are described below, can be set using the function
```matlab
gammaopts = control.design.gamma.GammasynOptions(parameter, value, ...)
```
The parameter name may contain dots, for example `'objective.normgain.K`.


| Option | Remark |
| --- | --- |
| `type` | objective functions, element of `GammaJType` |
| `objective.normgain.{R, K, F}`| Weights for minimization of controller norm |
| `objective.normgain.{R_shift, K_shift, F_shift}`| Shift for minimization of controller norm |
| `objective.kreisselmeier.{rho, max}`| Parameters for Kreisselmeier objective function |
| `objective.lyapunov.Q`| right hand side matrix for Lyapunov equation |
| `weight` | weight for the used objective functions
| `allowvarorder` | true, {false}: indicator if systems with different numbers of states are allowed |
| `allownegativeweight` | true, {false}: indicator if negative area weights are allowed |
| `usecompiled` | true, {false}: indicator if generated code should be used |
| `numthreads` | number of parallel threads to use
| `eigenvaluederivative` | method for calculation of eigenvalue derivatives to use
| `eigenvaluefilter` | filter for eigenvalues to use
| `strategy` | strategy for solution to use
| `errorhandler` | type of error handler to use
| `errorhandler_function` | error handler function to use in case of `GammaErrorHandler.USER`
| `system.usereferences` | indicator if output matrix `C` should be used as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/7c35e2cd0b773a91d1eb4c717108d4df.svg?invert_in_darkmode" align=middle width=37.67934389999999pt height=22.831056599999986pt/> for matlab system descriptions
| `system.usemeasurements_xdot` | indicator if output matrix `C_dot` should be used as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/2a400357c40ac1a7a9c87a249c6796a6.svg?invert_in_darkmode" align=middle width=26.66895989999999pt height=24.7161288pt/> for matlab system descriptions
| `system.samples` | structure with fields equal to the names of uncertain blocks in the `uss` system description to indicate the number of multiple models to create from the corresponding uncertain parameter
| `system.Blocks` | structure with fields equal to the names of uncertain blocks in the `uss` system description to indicate the number of multiple models to create from the corresponding uncertain parameter


#### Objective functions `type`

With this option the type of loss function used for the soft region (constrained optimizers) or the hard region (unconstrained optimizers) is selected.
Also, additional objective function terms can be selected.
`type` is a scalar or a vector of elements of the enumeration `GammaJType`.
The elements are listed in the following table and some examples are given below.

| GammaJType | Remark | Loss function
| --- | --- | --- |
| ZERO | no objective function (pure feasibility problem) | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/01eb1509890c54fdde448ee1d585c93d.svg?invert_in_darkmode" align=middle width=49.96563824999999pt height=22.831056599999986pt/> |
| MAX | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/83771fef97f748cb0fed8d9e1d007c87.svg?invert_in_darkmode" align=middle width=21.41178269999999pt height=22.831056599999986pt/> loss function | <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/bd9e679fbdea3041b2a9b5553be90432.svg?invert_in_darkmode" align=middle width=183.05115509999996pt height=24.65753399999998pt/> |
| SQUAREPENALTY | Quadratic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/9a7c7fb4f6f4435ea790f8e15a05ede0.svg?invert_in_darkmode" align=middle width=203.21104815pt height=26.76175259999998pt/> |
| EXP | Exponential loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/130afde698621f42bd151157080a3c5f.svg?invert_in_darkmode" align=middle width=156.56712555pt height=24.65753399999998pt/> |
| LINEAR | (*) linear weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/31b35fb65ff05ba0f31b9e0cbafa10d4.svg?invert_in_darkmode" align=middle width=118.66747199999999pt height=24.65753399999998pt/> |
| SQUARE | (*) *signed* quadratic weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/121ce94b2164ec14bbabf435ace4fe49.svg?invert_in_darkmode" align=middle width=271.7936462999999pt height=26.76175259999998pt/> |
| CUBIC | (*) cubic weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/69db3b15e730043270c993c4e9ca1c46.svg?invert_in_darkmode" align=middle width=138.82736504999997pt height=26.76175259999998pt/> |
| LOG | (*) Logarithmic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/0dd673d3135574a815a11bfe17322942.svg?invert_in_darkmode" align=middle width=180.99634079999998pt height=24.65753399999998pt/> |
| KREISSELMEIER | vector performance index weighting according to Kreisselmeier | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4332e56ccc38d7e00fd5719ad10179ae.svg?invert_in_darkmode" align=middle width=472.4203869pt height=37.80850590000001pt/> |
| EIGENVALUECONDITION | (**) eigenvector matrix condition objective function ||
| NORMGAIN | (**) norm of gain matrices | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/7de3dded5b15d9464a9d240657cfb862.svg?invert_in_darkmode" align=middle width=337.1515884pt height=26.76175259999998pt/> |
| LYAPUNOV | (**) norm of Lyapunov matrix of closed loop ||

(*) These loss functions are unbounded below.
This may lead to unexpected results if the closed loop possesses conjugate complex poles.
They are provided mainly for experimental and academic reasons.

(**) These loss functions don't assess the poles at all but are motivated by the aim to get a robust or economic controller.


For example, if `type` is set to
```matlab
	GammaJType.SQUAREPENALTY
```
the quadratic loss function is used and none of the additional objective functions <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/54fab8c1beec1c6440e1029278fd7a23.svg?invert_in_darkmode" align=middle width=42.44876954999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/497e7dba7d286918a8185ecee89f5d27.svg?invert_in_darkmode" align=middle width=37.78547519999999pt height=22.831056599999986pt/> is added.

If `type` is set to
```matlab
	[GammaJType.SQUAREPENALTY; GammaJType.NORMGAIN]
```
the quadratic loss function is used and the controller norm is added.
To weight these two terms, the option `weight`can be set appropriately.

If `type` is set to
```matlab
	[GammaJType.ZERO; GammaJType.NORMGAIN]
```
only the norm of the controller is used in the objective function.
This can be a sensible choice when constrained optimizers are used, as the hard region is still respected.
When an unconstrained optimizer is used, this is not a sensible choice, as the pole region is ignored completely and therefore the optimum is that all controller matrices are zero.


The function accepts multiple types of loss functions, but there should be no use for.
However, it is important to notice that
```matlab
	[GammaJType.SQUAREPENALTY; GammaJType.SQUAREPENALTY]
```
does *not* mean that the hard and the soft region are both translated to an objective function term using the quadratic loss function but the following!

* In the case of a constrained optimizer, the soft region appears twice in the objective function.
* In the case of an unconstrained optimizer, the hard region appears twice in the objective function. The soft region is still ignored.



##### NORMGAIN

If `NORMGAIN` is used as objective function, the weighting matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4cce0316187ef5316734515fe9c3cc67.svg?invert_in_darkmode" align=middle width=35.02290659999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/c2652e379b5063fcafc5643c92d0091c.svg?invert_in_darkmode" align=middle width=35.53665554999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/5f29510662c3f8494fa81846f7b84da4.svg?invert_in_darkmode" align=middle width=33.96127019999999pt height=22.831056599999986pt/> have to be specified using the following parameters of `objoptions`:

| Parameter | Description |
| --- | --- |
| `objective.normgain.R` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4cce0316187ef5316734515fe9c3cc67.svg?invert_in_darkmode" align=middle width=35.02290659999999pt height=22.831056599999986pt/> |
| `objective.normgain.K` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/c2652e379b5063fcafc5643c92d0091c.svg?invert_in_darkmode" align=middle width=35.53665554999999pt height=22.831056599999986pt/> (only necessary if <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> is used in the structure) |
| `objective.normgain.F` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5f29510662c3f8494fa81846f7b84da4.svg?invert_in_darkmode" align=middle width=33.96127019999999pt height=22.831056599999986pt/> (only necessary if <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> is used in the structure) |
| `objective.normgain.R_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/010535dc9b457be63091f94c31124f8c.svg?invert_in_darkmode" align=middle width=29.577697049999987pt height=22.831056599999986pt/> |
| `objective.normgain.K_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81b2f56f8d8efcb815fbabf58ff1c13a.svg?invert_in_darkmode" align=middle width=30.091445999999987pt height=22.831056599999986pt/> |
| `objective.normgain.F_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fb5f4c889d388e55ae8b2ed8eced2a29.svg?invert_in_darkmode" align=middle width=28.51606229999999pt height=22.831056599999986pt/> |

The weighting matrices have to be of the same dimension as the corresponding controller matrix.
It is not sufficient to use a scalar value, even if the weight should be the same for all entries.
The shifting matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/a7497f8350e9923cbd5ee9132dbbed9e.svg?invert_in_darkmode" align=middle width=20.159830349999993pt height=22.831056599999986pt/> are optional.


##### KREISSELMEIER

| Parameter | Description |
| --- | --- |
| `objective.kreisselmeier.rho` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/3f0bcdb0b50dea0c8f1b76d1c1157ec0.svg?invert_in_darkmode" align=middle width=40.33696259999999pt height=22.831056599999986pt/> |
| `objective.kreisselmeier.max` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/433d7f35ff78e09fc4d480720831e341.svg?invert_in_darkmode" align=middle width=68.04828855pt height=22.831056599999986pt/> |

##### LYAPUNOV
If `LYAPUNOV` is used as objective function, the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4c28a537aacd5fac8f4df776ace4ccfb.svg?invert_in_darkmode" align=middle width=22.12787279999999pt height=22.831056599999986pt/> have to be specified using the following parameters of `objoptions`:

| Parameter | Description |
| --- | --- |
| `objective.lyapunov.Q` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4c28a537aacd5fac8f4df776ace4ccfb.svg?invert_in_darkmode" align=middle width=22.12787279999999pt height=22.831056599999986pt/> |

If the same matrix is to be used for all multiple model, it is sufficient to supply a single matrix.
In case a specific matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4c28a537aacd5fac8f4df776ace4ccfb.svg?invert_in_darkmode" align=middle width=22.12787279999999pt height=22.831056599999986pt/> for every multiple model should be used, the matrices have to be concatenated in the third dimension.
When nothing is specified, the identity matrix is used.
If the discrete time Lyapunov equation is to be solved in case of discrete time systems, it is vital to add a field `T` with the sampling time to the system description in order to signal this.
When the option `allowvarorder` is set to `true` and therefore systems with different state dimension are allowed, the remaining elements of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4c28a537aacd5fac8f4df776ace4ccfb.svg?invert_in_darkmode" align=middle width=22.12787279999999pt height=22.831056599999986pt/> must be filled with `NaN` to match the dimension of the largest system in use.

#### Weighting of the objective function terms

If more than one objective function term is selected by `type`, their weighting can be specified by `weight` which is a numeric vector of the same dimension as `type` with the corresponding non-negative weights.

If for example the objective function
```math
	J = 1 \cdot J_\mathrm{\Gamma,soft} + 10^{-5} \cdot J_\mathrm{Ctrl}
```
is to be used, the following options are to be set:
```matlab
	'type'   : [GammaJType.SQUAREPENALTY; GammaJType.NORMGAIN]
	'weight' : [1; 1e-5]
```


#### Algorithm for eigenvalue derivation


| GammaEigenvalueDerivativeType | Remark |
| --- | --- |
| DEFAULT | calculation of eigenvalue derivatives by Rayleigh coefficient
| VANDERAA | calculation of eigenvalue and eigenvector derivatives with method of van der Aa
| RUDISILLCHU | calculation of eigenvalue and eigenvector derivatives with Rayleigh coefficient and pseudo inverse


#### Filtering

| GammaEigenvalueFilterType | Remark |
| --- | --- |
| NONE | no filtering of eigenvalues
| NEGATIVEIMAG | remove eigenvalues with negative imaginary part from calculation by taking negative imaginary part instead
| POSITIVEIMAG | remove eigenvalues with positive imaginary part from calculation by taking negative imaginary part instead



#### Strategy


| GammaSolutionStrategy | Remark |
| --- | --- |
| SINGLESHOT | solve pole region assignment problem "as is" |
| FEASIBILITYITERATION | solve a feasibility problem before solving the actual problem and use the solution of the feasibility problem as initial value |
| FEASIBILITYITERATION_COUPLING | solve a feasibility problem with only coupling conditions before solving the actual problem and use the solution of the feasibility problem as initial value |


#### Error handling

| GammaErrorHandler | Remark |
| --- | --- |
| WARNING | convert errors to warnings
| ERROR | leave errors as is
| USER | use user defined error handler

If `USER` is chosen, a handler to an error handler function has to be provided in the option `errorhandler_function`.



### Optimizer

The optimizers marked with (*) are included in this repository.


| Optimizer | Constr.  | Obj. | Origin           | Licence | Remark  |
| ----------- | --- | --- |-------------| ----- | ----|
| FMINCON        | * |   | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINCONGLOBAL  | * |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| IPOPT          | * |   | https://github.com/coin-or/Ipopt                    | Eclipse Public License 1.0 |  |
| FMINIMAX       | * | * | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINUNC        |   |   | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINUNCGLOBAL  |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| FMINSEARCH     |   |   | Matlab                                              | prop.      |  |
| GA             | * | * | Matlab, Global Optimization Toolbox                 | prop.      |  |
| KSOPT          | * | * | https://github.com/madebr/pyOpt                     | LGPL       |  |
| NLOPTUNC       |   |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTCON       | * |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTUNCGLOBAL |   |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTCONGLOBAL | * |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| PARTICLESWARM  |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| PATTERNSEARCH  | * |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| SIMULANNEAL    |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| PPPBOX         |   | * | ?                                                   |            |  |
| SCBFGS         |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |
| SLPGS          |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |
| SNOPT          | * |   | http://ccom.ucsd.edu/~optimizers/solvers/snopt/     | prop.      |  |
| SQPGS          |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |

Optimizers which are marked with an asterisk in the column "Constr." support constraint optimization directly.
For these optimizers, the hard pole region is by default expressed as a constraint.
Optimizers which are marked with an asterisk in the column "Obj." support multi objective optimization directly.

For the other optimizers the hard pole region is transformed into a soft pole region by an outer penalty function specified with `GammaJType`.
These optimiziers are only applicable to pole region assignment problems with one hard or one soft region and no other objective terms (e.g. norm of controller).


To specify one of the available optimizers with their default options, the argument `optimizer` can be set to
```matlab
optimizer = optimization.solver.Optimizer.IPOPT;
```
or any other optimizer than IPOPT.

To change the options of the optimizer, the following syntax can be used.
This is an example for IPOPT, other optimizers may provide different options:
```matlab
options = optimization.options.OptionFactory.instance.options(...
	optimization.solver.Optimizer.IPOPT, ...
	'ProblemType',                  optimization.options.ProblemType.CONSTRAINED,...
	'Retries',                      1,...
	'Algorithm',                    solver.defaultalgorithm,...
	'FunctionTolerance',            1E-8,...
	'StepTolerance',                1E-10,...
	'ConstraintTolerance',          1E-7,...
	'MaxFunctionEvaluations',       25E3,...
	'MaxIterations',                25E3,...
	'MaxSQPIter',                   25E3,...
	'SpecifyObjectiveGradient',     true,...
	'SpecifyObjectiveHessian',      false,...
	'SpecifyConstraintGradient',    true,...
	'SpecifyConstraintHessian',     false,...
	'CheckGradients',               false,...
	'FunValCheck',                  false,...
	'FiniteDifferenceType',         'forward',...
	'Diagnostics',                  false,...
	'Display',                      'iter-detailed'...
);
```


The most important options are

| Option | Description |
| --- | --- |
| `ProblemType` | type of problem formulation (constrained, unconstrained, multiobjective)
| `Algorithm` | algorithm to use for optimization (if the solver supports different algorithms)
| `SpecifyObjectiveGradient` | indicator if gradient information for the objective function should be used
| `SpecifyObjectiveHessian` | indicator if hessian information for the objective function should be used
| `SpecifyConstraintGradient` | indicator if gradient information for the constraint functions should be used
| `SpecifyConstraintHessian` | indicator if hessian information for the constraint functions should be used
| `Display` | level of verbosity of the displayed information

In general the class `Options` has most the fields that the `optimoptions` class of the Optimization Toolbox of Matlab R2016B and newer has with the same meaning and additionally the fields `ProblemType` and `Retries`.
Which of these settings are taken into account by a certain solver depends on the solvers interface.
Generally speaking, the solvers from the Opimization Toolbox support the same settings as can be set by `optimoptions` while the external solvers with MEX interfaces only support a limited subset of all options.

### Bounds and nonlinear constraints

#### `Rbounds`: Bounds (linear inequality constraints)

Bounds can be imposed for single entries of the controller matrices as well as bounds on linear combinations of parameters of the same matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> or <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> (i.e. linear inequality constraints) can be imposed.

They are defined similarly to the equality constraints in `Rfixed`.

Combined constraints on all gain coefficients can be formed as a cell array containing matrices for constraints of the single matrices and another one for the combined constraints as follows:
```matlab
Rbounds = {Ra_bounds, Ka_bounds, Fa_bounds, RKFa_bounds}
```
The definitions of the bounds for the single matrices (which are explained below) are combined by forming a cell array, i.e. if all three matrices are used:
```matlab
Rbounds = {Ra_bounds, Ka_bounds, Fa_bounds}
```
If only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> is used,
```matlab
Rbounds = {Ra_bounds, Ka_bounds}
```
and if only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> is used,
```matlab
Rbounds = {Ra_bounds}
```

The bounds of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/> (<img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/> analogously) are defined by
```matlab
Ka_bounds = {Zlhs, Zrhs}
```
where `Zlhs` and `Zrhs` correspond to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/bdd12812852764fdf0a0441c3eaac725.svg?invert_in_darkmode" align=middle width=48.795826199999986pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/73b5748e1a609785e1fca7bc7a8e83f0.svg?invert_in_darkmode" align=middle width=35.26465964999999pt height=14.15524440000002pt/>, resp., in
```math
\sum_{i,j} (Z_{\mathrm{Bd},k} \odot R) \leq z_{\mathrm{Bd},k}
```
where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/d1866f3bd35e683d8cd331a170875548.svg?invert_in_darkmode" align=middle width=29.223638399999988pt height=22.831056599999986pt/> means element-wise multiplication (Hadamard product).
If there is more than one inequality <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fbdb696db6a1b0322aa20999d63696f2.svg?invert_in_darkmode" align=middle width=18.207811049999993pt height=22.831056599999986pt/>, the matrices <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f7cfe92bfcf30ad9f3819deb842583ce.svg?invert_in_darkmode" align=middle width=28.44187334999999pt height=22.831056599999986pt/> are stacked along the third dimension in `Zlhs`.
I.e, if `Nz` linear inequalities are specified, the dimensions of `Zlhs` and `zrhs` are `size(Zlhs): [size(R, 1), size(R, 2), Nz]` (`size(Zlhs): [size(R, 1), size(R, 2) + size(K, 2) + size(F, 2), Nz]` for combined constraints) and `size(Zrhs): [Nz, 1]`, resp.

For an example refer to the section about the parameter `Rfixed`.



#### `Rnonlin`: Nonlinear inequality and equality constraints

It is possible to impose nonlinear equality and inequality constraints on the parameters of the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/81134b13734180a1832d4760261e96e4.svg?invert_in_darkmode" align=middle width=21.74091809999999pt height=22.831056599999986pt/>, <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/0be7b0830bddac0f6300ac856ea79086.svg?invert_in_darkmode" align=middle width=24.26944739999999pt height=22.831056599999986pt/> or <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/f309af093099f0f9fe6b32d48b711ec0.svg?invert_in_darkmode" align=middle width=21.986370449999992pt height=22.831056599999986pt/>.
In contrast to the linear constraints, a single constraint can only be imposed on one or more parameter of the same matrix, i.e.

```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fa25999ffd4d3436835309d9ff4821a4.svg?invert_in_darkmode" align=middle width=88.86044475pt height=140.31961184999997pt/></p>
```

These functions are provided by a single function `Rnonlin_wrapper` which is to be passed as the argument `Rnonlin` and has the signature
```matlab
[cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, ceq_F] = Rnonlin_wrapper(R, K, F)
```
or
```matlab
[cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, geq_F, gineq_R, geq_R, gineq_K, geq_K, gineq_F, geq_F] = Rnonlin_wrapper(R, K, F)
```
The second variation returns the gradients as well.

* The return values `c...` are vectors of possibly different length corresponding to the functions above.
* If the gradients `g...` are returned, they are returned in three-dimensional matrices. For each entry in the corresponding `c...`vector `g...`contains a matrix with the derivations after each entry of the concerned controller matrix. These are stacked in the third dimension. I.e.
```matlab
size(gineq_R) : [size(R, 1), size(R, 2), length(cineq_R) ]
```

##### Example
If the controller matrix is
```math
R =
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/0ce11da9d3c771dc5cac564c8a7dd8ac.svg?invert_in_darkmode" align=middle width=94.60066935pt height=39.452455349999994pt/></p>
```
and the constraints
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/bc582530c7c77897031a2dbc3e1f4db7.svg?invert_in_darkmode" align=middle width=80.26617555pt height=44.96263035pt/></p>
```
are given (and ignoring that the first constraint can be expressed as two simpler linear constraints), the `Rnonlin_wrapper` would be
```matlab
function [cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, ceq_F] = Rnonlin_wrapper(R, K, F)

	ceq_R = []; cineq_K = []; ceq_K = []; cineq_F = []; ceq_F = [];

	cineq_R = [
		R(1, 2)^2 - 9;
		R(1, 2) + R(2, 3)^2 - 4
	];

end
```
or
```matlab
function [cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, geq_F, gineq_R, geq_R, gineq_K, geq_K, gineq_F, geq_F] = Rnonlin_wrapper(R, K, F)

	ceq_R = []; cineq_K = []; ceq_K = []; cineq_F = []; ceq_F = [];
	geq_R = []; gineq_K = []; geq_K = []; gineq_F = []; geq_F = [];

	cineq_R = [
		R(1, 2)^2 - 9;
		R(1, 2) + R(2, 3)^2 - 4
	];

	gineq_R = cat(3,...
		[0, 2 * R(1, 2), 0; 0, 0, 0],...
		[0, 1, 0; 0, 0, 2 * R(2, 3)]...
	);
end
```



### Saving results
For comparing and examining different calculated solutions the `SolutionSet` class can be used.
It is instantiated like
```matlab
controllerdata = control.design.gamma.SolutionSet(controller, system, polearea, weight, R_fixed, R_bounds, true);
controllerdata.add_solution(R_opt, J_opt, information, R0, options, gammaopts);
controllerdata.save();
```
after a call to `gammasyn` and expects the controller type used as `OutputFeedback` and the arguments passed to and returned by `gammasyn`.
It has the ability to plot the closed loop eigenvalues and pole regions with the `plot` method, plot step responses with the `step` method and solve the problem again with possibly different initial values or different optimizers with the `rerun` method.

## Robust Coupling Control
`gammasyn` is prepared for the synthesis of coupling controllers and will be extended to handle decoupling controllers as well.
For archieving this a specialized wrapper function named `gammasyn_couplingcontrol` is used that converts the supplied system to the needed description for coupling controller design.
The task of a coupling controller is to ensure
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/080a458ea7f49587d5ba79ee152a9ada.svg?invert_in_darkmode" align=middle width=206.13289124999997pt height=15.936036599999998pt/></p>
```
asymptotically and indpendently from any reference inputs <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/3a7ef255fdc6c23ddc9686f25569ab00.svg?invert_in_darkmode" align=middle width=23.92322789999999pt height=22.831056599999986pt/>.
The system must be given, such that the lower rows of
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/5a689c4e41591c781bed2cf5caf75c64.svg?invert_in_darkmode" align=middle width=106.81975424999999pt height=39.452455349999994pt/></p>
```
and
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/3565f2cf72a1a18f86ebd1f0f57e68e9.svg?invert_in_darkmode" align=middle width=110.54122860000001pt height=39.452455349999994pt/></p>
```
contain the parameters of a given number of coupling conditions in the form
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8c806f3dd66c31ff078fddaf55feec19.svg?invert_in_darkmode" align=middle width=158.94679349999998pt height=22.01250645pt/></p>
```
To achieve coupling, the transfer matrix of the closed-loop system
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/1e24b9cd3e4e9dd4b6a14c537b2cec54.svg?invert_in_darkmode" align=middle width=544.4439825pt height=85.48022999999999pt/></p>
```
is designed, such that <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/bbed6fe0fa386e053dce60c85bbc32db.svg?invert_in_darkmode" align=middle width=86.61184619999999pt height=24.65753399999998pt/> holds.
By writing
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/fcb45cb210935a07b307d3cd1e72a87e.svg?invert_in_darkmode" align=middle width=425.13309795pt height=44.89738935pt/></p>
```
with the closed-loop eigenvalues <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/4dbc9eca92668245d9328036dda93234.svg?invert_in_darkmode" align=middle width=33.737734799999984pt height=22.831056599999986pt/> and the right and left eigenvectors <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/1172410e50024277db5d6cd7afbd14e8.svg?invert_in_darkmode" align=middle width=32.11671869999999pt height=22.831056599999986pt/> and <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/e144bd133a34dbf5fd2d376fc4a46f7b.svg?invert_in_darkmode" align=middle width=35.917160399999986pt height=22.831056599999986pt/>, the non-linear output- and input-coupling conditions
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/deb3c9ec26e9faa96d0c78604eb2776a.svg?invert_in_darkmode" align=middle width=310.36146735pt height=43.53610965pt/></p>
```
as well as
```math
	<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e4ff8756abbf6eb2279222820a0a8e17.svg?invert_in_darkmode" align=middle width=88.94639159999998pt height=15.936036599999998pt/></p>
```
are obtained.
Here, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/e7edab300a93d640814d40ac9152468e.svg?invert_in_darkmode" align=middle width=23.565549149999992pt height=22.831056599999986pt/> denotes the dimension of the output nulling space of the system <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/644d98a3e0e0d7838b92f369af564cdd.svg?invert_in_darkmode" align=middle width=149.32557254999998pt height=24.65753399999998pt/>.
In case of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/8147c6f8fd95fa6b643b89b4af5d1a7f.svg?invert_in_darkmode" align=middle width=80.13358154999999pt height=22.831056599999986pt/>, this space is equivalent to the largest controlled invariant subspace within the kernel of <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/8fe58364f433e3ea467ebffa083ecb2a.svg?invert_in_darkmode" align=middle width=48.13599449999999pt height=22.831056599999986pt/>.

The conditions found can directly be included in the synthesis process using the built-in non-linear constraint function. Alternatively, using geometric concepts, the coupling conditions can be transformed to linear equality constraints which reduce the set of feasible controllers and prefilters.

The coupling control design implemented in `gammasyn` is only available for the design of a complete state feedback, i.e. <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/dcd597547a375785ec8589f3b8f30f5d.svg?invert_in_darkmode" align=middle width=22.05708614999999pt height=22.831056599999986pt/> must be chosen as identity matrix.

### Robust DAE synthesis
The methodology for designing robust coupling controllers using pole region assignment, can immediately be transferred to systems in differential-algebraic form (DAE systems, descriptor systems).

Therefore, the systems handed over are transformed using a singular value decomposition of the descriptor matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/> in order to obtain state space systems with feedthrough.
For these, a robust coupling control best possibly fulfilling the algebraic equations is calculated.

### Usage
To perform the coupling control synthesis or robust DAE synthesis, `gammasyn_couplingcontrol` has to be used.
Furthermore, the `objectiveoptions` structure has to be extended by the field `couplingcontrol` which in turn is a structure containing the following fields
* `couplingstrategy`: the coupling design method, an instance of `GammaCouplingStrategy`.
	* `GammaCouplingStrategy.EXACT`: Only allow <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/bbed6fe0fa386e053dce60c85bbc32db.svg?invert_in_darkmode" align=middle width=86.61184619999999pt height=24.65753399999998pt/> and use geometric methods.
	* `GammaCouplingStrategy.APPROXIMATE`: Use geometric method but also allow <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/302f35a74246b05f98ee241108b7b9d9.svg?invert_in_darkmode" align=middle width=86.61184619999999pt height=24.65753399999998pt/> if <img src="https://rawgit.com/pvogt09/gammasyn/feature/gitlabdoc/docs/svgs/bbed6fe0fa386e053dce60c85bbc32db.svg?invert_in_darkmode" align=middle width=86.61184619999999pt height=24.65753399999998pt/> is not solvable.
	* `GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY`: directly use coupling conditions as non-linear equality constraints of the form `ceq(x) = 0` with `x` denoting the vector of optimization variables
	* `GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY`: directly use coupling conditions as non-linear inequality constraints of the form `c(x) < tolerance_coupling` and `-c(x) < tolerance_coupling` with `x` denoting the vector of optimization variables
* `couplingconditions`: (`uint32`) the number of coupling conditions specified in <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/7c35e2cd0b773a91d1eb4c717108d4df.svg?invert_in_darkmode" align=middle width=37.67934389999999pt height=22.831056599999986pt/>
* `tolerance_coupling`: (`double`) the tolerance when using `GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY`
* `solvesymbolic`: (`logical`) only for `EXACT` and `APPROXIMATE`: use symbolic toolbox if available to increase precision of obtained equality constraints.
* `round_equations_to_digits`: (`double`, whole number) only for `EXACT` and `APPROXIMATE`: decimal places to which linear equality constraints are rounded in case of numerical precision difficulties. Use `NaN` if no rounding is desired.
* `weight_coupling`: (`double`, nonnegative) weighting factor for nonlinear coupling conditions to increase/decrease importance in comparison with pole region constraints
* `weight_prefilter`: (`double`, nonnegative) weighting factor for prefilter regularity condition to increase/decrease importance in comparison with pole region constraints

`gammasyn_couplingcontrol` checks the descriptor matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/development/docs/svgs/ad159e0c806e1c3479f7b557e7134490.svg?invert_in_darkmode" align=middle width=22.21462484999999pt height=22.831056599999986pt/> to choose between a regular coupling control design and DAE design.

## Examples

### Robust PID control

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)...
];

controller = control.design.outputfeedback.PIDOutputFeedback();

sys_augmented = controller.amend(sys);

[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);

Rfixed = {Ra_fixed, Ka_fixed};


polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

weight = 1;
R0 = ones(2, 2);


gammaopts = control.design.gamma.GammasynOptions();

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys_augmented,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

### Hard and soft regions

If a more ambitious region is targeted, as the one with the darker shade in the following image, and only a P-controller is to be used, there is no feasible solution.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft.png" width=25%/>

Therefore, this region can be defined as a soft region.

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)...
];

hardregion = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

softregion = [
	control.design.gamma.area.Line(0.5, 0),...
	control.design.gamma.area.Imag(1, 2)
];

polearea = {hardregion, softregion};
weight = {1, 1};
R0 = 1;
Rfixed = [];

gammaopts = control.design.gamma.GammasynOptions('type', GammaJType.SQUAREPENALTY);

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

This is a feasible problem.
The following image shows the poles of the closed loop systems.
(Not all poles are shown.
The poles which aren't shown lie to the left of the shown section and are less critical concerning the defined region.)
Clearly, some poles lie outside of the soft region, but all poles lie inside of the lighter shaded hard region.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft-sol1.png" width=25%/>

With the weights the designer can influence the result further.
If for the problem at hand the velocity of the closed loop system is more important than its oscillation, the weight of the vertical border at -2 could be increased by
```matlab
weight = {1, [1, 10]};
```
As a result, the poles respect the right border of the soft region more, while the distances to the upper and lower borders increase.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft-sol2.png" width=25%/>



<!--- ### Specifying parameter bounds --->


### Using compiled functions
In order to make the objective and constraint functions run a couple of orders faster it is possible to generate mex files for these functions with the help of the Matlab Coder toolbox.
The functions for compiling the functions are located in the `compile` package and the `all` function therein can be used to generate code and compile all the supported functions.
If it is intended to use the eigenvector derivative supplied by van der Aa's method it is necessary to use at least Matlab R2016B for code generation as this method relies on run time recursion which is not supported in earlier versions.
When the compiled functions should be used the option `usecompiled` in the `gammaopts` argument has to be set to true.
Since the generated code only support a limited set of Matlab's functionality, only builtin pole areas can be used and no custom defined areas as function handles.


### Using Matlab system definitions
Besides the structure description of the systems it is also possible to use the system description provided by the Control toolbox and Robust Control toolbox.
The system descriptions without tunable or uncertain parameters (i.e. `tf`, `ss` and `dss`) are internally converted to structure arrays of systems with the respective fields and whether the fields `C_dot` and `C_ref` should be filled by the output matrix of the system can be controlled by the options `usemeasurements_xdot` and `usereferences` in the `gammaopts` structure.
For systems with uncertain parameters (i.e. `uss`) it is possible to create multiple models from a single uncertain system.
This behavior can be controlled by the structures `Blocks` or `systems` which should contain the number of sampling points for the respective uncertain parameter.
For example an uncertain system with uncertain parameter `u` can be converted into 10 multiple models by setting
```matlab
gammaopts.Blocks.u = 10;
```
Currently it is only possible to handle real uncertainty this way while complex uncertainty results in an error.

If tunable coefficients should be contained in the system description (i.e. `genss`) a specialized wrapper function named `gammasyn_loopune` has to be used which converts the internal system description to the output feedback formulation used by this toolbox and splits the system in an uncertain part and the controller matrix.
Bounds on the coefficients and fixed elements are respected as well in the conversion steps.
For the creation of multiple models from the uncertain part of the resulting system the same rules as above apply.
The signature of the function is
```matlab
[system_cl, Jopt, information] = control.design.gamma.gammasyn_looptune(systems, areafun, weights, solveroptions, objectiveoptions)
```
and it expects the system to be of type `genss` while the rest of the arguments equals the arguments used for the normal call to `gammasyn`.
The output contains the optimal objective value as well as the structure with information about the optimization run while the controller is returned as part of the closed loop into which the found solution is inserted.


### Iterative calls to gammasyn

Especially if the constrained optimizers cannot be used, it is quite probable that a desired pole region cannot be reached directly if the initial value of the controller matrices aren't good as the objective functions generally possess many bad local minima.
Therefore, it is often necessary to begin with rather wide pole regions and a quadratic loss function and interactively narrowing the region and perhaps switching to an exponential loss function.
For the exponential loss function the weights might be increased over multiple steps as well.




## 3rd party components

* Optimizers -> see table above
* [ROLMIP](https://github.com/agulhari/ROLMIP)
* oplace
* [DataHash](https://de.mathworks.com/matlabcentral/fileexchange/31272-datahash)
* [Geometric Approach Toolbox](http://www3.deis.unibo.it/Staff/FullProf/GiovanniMarro/geometric.htm)


## Licence

GNU Lesser General Public License Version 3.0

Patrick Vogt
function [] = update_hascompiled()
	%UPDATE_HASCOMPILED reset persistent variable in configuration.control.design.gamma.hascompiled to reflect shanges in generated code
	clear +configuration/+control/+design/+gamma/hascompiled;
end
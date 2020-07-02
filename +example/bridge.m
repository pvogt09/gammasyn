function [system] = bridge(l, m_k, m_g)
	system = struct(...
		'A',	[[
				0,	1,	0,							0;
				0,	0,	m_g*9.81/m_k,				0;
				0,	0,	0,							1;
				0,	0,	-(m_g + m_k)*9.81/m_k/l,	0
			], zeros(4, 1);
			zeros(1, 4),	0
		],...
		'B',	[[
				0;
				1/m_k;
				0;
				-1/m_k/l
			],				zeros(4, 1);
			zeros(1, 1),	1
		],...
		'C',	[[
				1,	0,	0,	0;
				0,	1,	0,	0;
				0,	0,	1,	0
			],				zeros(3, 1);
			zeros(1, 4),	1
		],...
		'D',	zeros(4, 2)...
	);
end
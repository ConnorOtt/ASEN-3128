function f = trimOpt(trim)

ti = 0;
tf = 5; % s
initConds = zeros(1, 12);
initConds(12) = -3;
initConds(3) = 1;

options = odeset('Events', @termEvents, 'RelTol', 1e-8);
[t, F] = ode45(@(t, F)quadCopterODE(t, F, trim), ...
                                            [ti, tf], initConds, options);

f = F(end, 3);
end
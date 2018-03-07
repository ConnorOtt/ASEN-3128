function [value, isTerm, direction] = termEvents(t, F)
% Define events for quad copter ODE
value = F(12);
isTerm = 1;
direction = [];
end
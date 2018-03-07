function [value, isTerm, direction] = termEvents(t, F)
% Define events for quad copter ODE
value = [F(12), F(12) + 100]';
isTerm = [1,1]';
direction = [];
end
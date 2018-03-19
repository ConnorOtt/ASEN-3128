function dfdt = longSimODE(t, F, A)
dfdt = A*F(1:4);
end
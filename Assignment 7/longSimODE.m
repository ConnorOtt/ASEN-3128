function dfdt = longSimODE(t, F, A, B, K)
dfdt = A*F(1:4) - B*K*F(1:4);
end
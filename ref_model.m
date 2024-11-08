function xd_dot = ref_model(xd, psi_ref)
    w_ref = 0.03;
    zeta = 1;
    wn= w_ref/sqrt(1-2*zeta^2+sqrt(4*zeta^4 -4*zeta^2+2));
    A_m = [0,             1,              0;
           0,             0,              1;
          -wn^3, -(2*zeta+1)*wn^2, (-2*zeta+1)*wn];
    B_m = [0; 0; wn^3];
    xd_dot = A_m * xd + B_m * psi_ref;
end

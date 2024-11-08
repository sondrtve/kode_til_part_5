function nd = PI_speed(Ud)
    Dia = 3.3;           % Propeller diameter (m)
    rho = 1025;          % Density of water (kg/m^3)
    t_thr = 0.05;
    
    PD = 1.5;
    AEAO = 0.65;
    nBlades = 4;
    [KT, ~] = wageningen(0, PD, AEAO, nBlades);
    
    m = 17.0677e6;       % Mass (kg)
    Xudot = -8.9830e5;
    T1 = 20;
    Xu = -(m - Xudot) / T1;

    % Calculate the desired thrust and propeller speed
    Td = -Xu * Ud / (1 - t_thr);  % Desired thrust
    nd = sign(Td) * sqrt(Td / (rho * Dia^4 * KT));
end

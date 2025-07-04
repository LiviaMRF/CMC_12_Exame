function dx = servomotorStateFcn(x, u)
    % Estados
    theta = x(1);
    omega = x(2);
    i = x(3);

    % Entrada
    V = u;

    % Equações diferenciais
    dx = zeros(3,1);
    dx(1) = omega;
    dx(2) = (planta.Kt*planta.N*planta.eta*i - planta.Beq*omega)/planta.Jeq;
    dx(3) = (V - planta.R*i - planta.Kt*planta.N*omega)/planta.L;
end

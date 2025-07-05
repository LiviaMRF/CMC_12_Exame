function [Ga, Gf] = obterMalhaPosicao(controladorPosicao, controladorCorrente, planta)
% [Ga, Gf] = obterMalhaPosicao(controladorPosicao, controladorCorrente,
% planta) obtem as malhas aberta Ga e fechada Gf do servomotor de posicao.
% A struct controladorPosicao eh dada por:
% controladorPosicao.Kp: ganho proporcional do controlador de posicao.
% controladorPosicao.Kd: ganho derivativo do controlador de posicao.
% controladorPosicao.a: frequencia de corte do filtro do termo derivativo.
% controladorPosicao.T: periodo de amostragem do controlador de posicao.
% A struct controladorCorrente eh dada por:
% controlador.K: ganho proporcional do controlador de corrente.
% controlador.alpha: parametro alpha da compensacao lead.
% controlador.Tl: parametro Tl da compensacao lead.
% controlador.T: tempo de amostragem do controlador de corrente.
% A struct planta contem os parametros da planta e pode ser obtida atraves
% de planta = obterPlantaServoPosicao().

s = tf('s');

Kp = controladorPosicao.Kp;
Kd = controladorPosicao.Kd;
a = controladorPosicao.a;
Tp = controladorPosicao.T;

K = controladorCorrente.K;
alpha = controladorCorrente.alpha;
Tl = controladorCorrente.Tl;
Tc = controladorCorrente.T;

Kt = planta.Kt;
Jeq = planta.Jeq;
Beq = planta.Beq;
L = planta.L;
R = planta.R;
N = planta.N;
eta = planta.eta;

Ap = pade(exp(-s*Tp / 2), 2);
Cp = Kp + Kd*s*(a / (s+a));

[num, den] = pade(Tc/2, 2);
Nc = tf(num, 1);
Dc = tf(den, 1);
Cc = K*((Tl*s+1) / (alpha*Tl*s+1))*(1/s);

% Modelo do filtro de KalmanCont como sistema de estados
A = planta.KalmanCont.A;
C = planta.KalmanCont.C;
K = planta.KalmanCont.K;

Ak = A - K*C;
Bk = K;
Ck = [1 0];
Dk = 0;

Hk = ss(Ak, Bk, Ck, Dk);

% Malha aberta com o KalmanCont incluído
numGa = N * eta * Kt * Cp * Hk * Ap * Cc * Nc;
denGa = s*(Jeq*s + Beq)*(L*s + R)*Dc + N^2*eta*Kt^2*s*Dc + ...
         s*(Jeq*s + Beq)*Cc*Nc;

Ga = minreal(numGa / denGa);
Gf = minreal(feedback(Ga, 1));
end
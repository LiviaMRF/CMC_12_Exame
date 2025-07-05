function controlador = projetarControladorPosicaoOtimizacao(requisitos, controladorCorrente, planta)
% controlador = projetarControladorPosicaoOtimizacao(requisitos, controladorCorrente, planta)
%
% Ajusta Kp e Kd do controlador PD de posição por otimização (Nelder–Mead).
%
% ---------- Estruturas de entrada ----------
% requisitos            : struct com especificações de desempenho
%   .wb   – largura de banda desejada (rad/s)
%   .GM   – margem de ganho mínima (dB)   (opcional; use 0 se não exigir)
%   .PM   – margem de fase mínima (graus)
%   .fs   – frequência de amostragem do controlador de posição (Hz)
%
% controladorCorrente   : struct com parâmetros do loop de corrente
%   .K     – ganho proporcional
%   .alpha – parâmetro α da compensação lead
%   .Tl    – parâmetro Tl   da compensação lead
%   .T     – período de amostragem do controlador de corrente (s)
%
% planta                 : struct com parâmetros do servo-motor
%                          (obtido em planta = obterPlantaServoPosicao())
%
% ---------- Estrutura de saída ----------
% controlador           : struct com controlador de posição otimizado
%   .Kp  – ganho proporcional
%   .Kd  – ganho derivativo
%   .a   – frequência de corte do filtro derivativo (Inf ⇒ PD ideal)
%   .T   – período de amostragem de posição (= 1/requisitos.fs)
%   .J   – valor mínimo da função-custo (distância às especificações)

controlador = projetarControladorPosicaoAnalitico(requisitos, planta);
x0 = [controlador.Kp controlador.Kd];

opt = optimset( ...
               'Display',    'iter', ...
               'TolX',       1e-6, ...
               'TolFun',     1e-6, ...
               'MaxFunEvals',200);

cost = @(x) custoControladorPosicao(requisitos, controladorCorrente, planta, x);

[xOtimo, Jopt] = fminsearch(cost, x0, opt);

controlador.Kp = xOtimo(1);
controlador.Kd = xOtimo(2);
controlador.a  = Inf;
controlador.T  = 1/requisitos.fs;
controlador.J  = Jopt;
end

function J = custoControladorPosicao(requisitos, controladorCorrente, planta, parametros)
% J = custoControladorPosicao(requisitos, controladorCorrente, planta, parametros)
%
% Função-custo para a otimização de Kp e Kd.
% Penaliza diferenças quadráticas entre especificações (wb, PM, GM) e
% valores obtidos na malha projetada. Também pune ganhos negativos ou
% numéricos inválidos.

ctrlPos.Kp = parametros(1);
ctrlPos.Kd = parametros(2);
ctrlPos.a  = Inf;
ctrlPos.T  = 1/requisitos.fs;

try
    [Ga, Gf] = obterMalhaPosicao(ctrlPos, controladorCorrente, planta);
    wb       = bandwidth(Gf);
    [GM, PM] = margin(Ga);
catch
    J = 1e6;
    return
end

if any(parametros <= 0) || isnan(wb) || isnan(PM) || isinf(wb) || isinf(PM)
    J = 1e6;
    return
end

J = (requisitos.wb - wb)^2 + (requisitos.PM - PM)^2;

if isfield(requisitos,'GM') && requisitos.GM > 0
    J = J + (requisitos.GM - GM)^2;
end
end

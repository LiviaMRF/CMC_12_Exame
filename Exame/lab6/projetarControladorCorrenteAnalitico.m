function controlador = projetarControladorCorrenteAnalitico(requisitos, planta)
% controlador = projetarControladorCorrenteAnalitico(requisitos, planta)
% projeta o controlador de corrente atraves de um metodo analitico. A
% struct requisitos eh:
% requisitos.wb: requisito de banda passante.
% requisitos.GM: requisito de margem de ganho.
% requisitos.PM: requisito de margem de fase.
% requisitos.fs: requisito de taxa de amostragem.
% A struct planta contem os parametros da planta e pode ser obtida atraves
% de planta = obterPlantaServoPosicao().
% A saida da funcao eh a struct controlador:
% controlador.K: ganho proporcional do controlador de corrente.
% controlador.alpha: parametro alpha da compensacao lead.
% controlador.Tl: parametro Tl da compensacao lead.
% controlador.T: periodo de amostragem do controlador de corrente.

controlador.T = 1.0 / requisitos.fs;

controlador.K = requisitos.wb * (-requisitos.wb*planta.L + sqrt(2*requisitos.wb^2 * planta.L^2 + planta.R^2));

wcp = sqrt((-planta.R^2 + sqrt(planta.R^4+4*planta.L^2*controlador.K^2)) / (2*planta.L^2));
PM = 90 - atand(planta.L*wcp / planta.R);
phi = requisitos.PM - PM;

controlador.alpha = (1-sind(phi)) / (1+sind(phi));
controlador.Tl = 1/(wcp*sqrt(controlador.alpha));

end
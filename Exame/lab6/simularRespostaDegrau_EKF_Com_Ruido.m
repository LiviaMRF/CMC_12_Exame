function simularRespostaDegrau_EKF_Com_Ruido(controlador, planta)
% simularRespostaDegrau(controlador, planta) simula a resposta ao degrau
% unitario do servomotor de posicao. A struct controlador eh dada por:
% controlador.posicao.Kp: ganho proporcional do controlador de posicao.
% controlador.posicao.Kd: ganho derivativo do controlador de posicao.
% controlador.posicao.a: frequencia de corte do filtro do termo derivativo.
% controlador.posicao.T: periodo de amostragem do controlador de posicao.
% controlador.posicao.ftd: funcao de transferencia discreta do controlador
%                          de posicao.
% controlador.corrente.K: ganho proporcional do controlador de corrente.
% controlador.corrente.alpha: parametro alpha da compensacao lead.
% controlador.corrente.Tl: parametro Tl da compensacao lead.
% controlador.corrente.T: tempo de amostragem do controlador de corrente.
% controlador.corrente.ftd: funcao de transferencia discreta do controlador
%                          de corrente.
% A struct planta contem os parametros da planta e pode ser obtida atraves
% de planta = obterPlantaServoPosicao().

var_thetas = [0, 0.01, 0.1, 1];  % Variâncias do ruído
tf = 0.5;

% Referência degrau
thetar.time = [0; tf];
thetar.signals.values = [1; 1];
thetar.signals.dimensions = 1;

assignin('base', 'controlador', controlador);
assignin('base', 'planta', planta);
assignin('base', 'tf', tf);
assignin('base', 'thetar', thetar);

% Inicializa as células para cada sinal
signals = struct('thetal', [], 'thetam', [], 'wl', [], ...
                 'i', [], 'ic', [], 'V', [], 'Vc', []);
             
for i = 1:length(var_thetas)
    var_theta = var_thetas(i);
    assignin('base', 'var_theta', var_theta);
    
    out = sim('servomotor_posicao_EKF_Com_Ruido_Posicao.slx');
    
    signals.thetal{i} = out.thetal;
    signals.thetam{i} = out.thetam;
    signals.wl{i}     = out.wl;
    signals.i{i}      = out.i;
    signals.ic{i}     = out.ic;
    signals.V{i}      = out.V;
    signals.Vc{i}     = out.Vc;
end

% Legendas e cores
colors = get(0, 'DefaultAxesColorOrder');
legs = arrayfun(@(v) sprintf('\\sigma_{\\theta}^2 = %.3g', v), var_thetas, 'UniformOutput', false);

% Função auxiliar para plotar
plotSignalGroup(signals.thetal, 'Tempo (s)', '\theta_l (rad)', 'degrau_thetal_variancias', legs, colors);
plotSignalGroup(signals.thetam, 'Tempo (s)', '\theta_m (rad)', 'degrau_thetam_variancias', legs, colors);
plotSignalGroup(signals.wl, 'Tempo (s)', '\omega_l (rad/s)', 'degrau_wl_variancias', legs, colors);
plotSignalGroup(signals.i, 'Tempo (s)', 'i (A)', 'degrau_i_variancias', legs, colors);
plotSignalGroup(signals.ic, 'Tempo (s)', 'i_c (A)', 'degrau_ic_variancias', legs, colors);
plotSignalGroup(signals.V, 'Tempo (s)', 'V (V)', 'degrau_V_variancias', legs, colors);
plotSignalGroup(signals.Vc, 'Tempo (s)', 'V_c (V)', 'degrau_Vc_variancias', legs, colors);

end

function plotSignalGroup(signalCell, xlab, ylab, filename, legends, colors)
    figure;
    hold on; grid on;
    for i = 1:length(signalCell)
        plot(signalCell{i}.time, signalCell{i}.signals.values, 'LineWidth', 1.5, 'Color', colors(i,:));
    end
    xlabel(xlab, 'FontSize', 14);
    ylabel(ylab, 'FontSize', 14);
    legend(legends, 'FontSize', 12, 'Location', 'Southeast');
    set(gca, 'FontSize', 14);
    print(['-dpng'], ['-r400'], [filename, '.png']);
end
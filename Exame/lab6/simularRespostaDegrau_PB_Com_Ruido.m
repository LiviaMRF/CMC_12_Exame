function simularRespostaDegrau_PB_Com_Ruido(controlador, planta)
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

var_thetas = [0, 0.01, 0.1, 1];  % Variância do ruído da posição
tf = 0.5;

% Referência degrau
thetar.time = [0; tf];
thetar.signals.values = [1; 1];
thetar.signals.dimensions = 1;

assignin('base', 'controlador', controlador);
assignin('base', 'planta', planta);
assignin('base', 'tf', tf);
assignin('base', 'thetar', thetar);

thetal = cell(length(var_thetas),1);
thetam = cell(length(var_thetas),1);
wl = cell(length(var_thetas),1);

for i = 1:length(var_thetas)
    var_theta = var_thetas(i);
    assignin('base', 'var_theta', var_theta);
    
    out = sim('servomotor_posicao_PB_Com_Ruido_Posicao.slx'); 

    thetal{i} = out.thetal;
    thetam{i} = out.thetam;
    wl{i} = out.wl;
end

colors = get(0, 'DefaultAxesColorOrder');
legs = arrayfun(@(v) sprintf('\\sigma_{\\theta}^2 = %.3g', v), var_thetas, 'UniformOutput', false);

% Plot θ_l
figure;
hold on; grid on;
for i = 1:length(var_thetas)
    plot(thetal{i}.time, thetal{i}.signals.values, 'LineWidth', 1.5, 'Color', colors(i,:));
end
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('\theta_l (rad)', 'FontSize', 14);
legend(legs, 'FontSize', 12, 'Location', 'Southeast');
set(gca, 'FontSize', 14);
print -dpng -r400 degrau_thetal_variancias.png

% Plot θ_m
figure;
hold on; grid on;
for i = 1:length(var_thetas)
    plot(thetam{i}.time, thetam{i}.signals.values, 'LineWidth', 1.5, 'Color', colors(i,:));
end
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('\theta_m (rad)', 'FontSize', 14);
legend(legs, 'FontSize', 12, 'Location', 'Southeast');
set(gca, 'FontSize', 14);
print -dpng -r400 degrau_thetam_variancias.png

% Plot ω_l
figure;
hold on; grid on;
for i = 1:length(var_thetas)
    plot(wl{i}.time, wl{i}.signals.values, 'LineWidth', 1.5, 'Color', colors(i,:));
end
xlabel('Tempo (s)', 'FontSize', 14);
ylabel('\omega_l (rad/s)', 'FontSize', 14);
legend(legs, 'FontSize', 12, 'Location', 'Southeast');
set(gca, 'FontSize', 14);
print -dpng -r400 degrau_wl_variancias.png

end
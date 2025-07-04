function simularRespostaDegrau(controlador, planta)
% --------------------------------------------------------------
% Executa o modelo “servomotor_posicao” para quatro valores fixos
% de variância e gera **um único gráfico por sinal** contendo as
% quatro curvas sobrepostas.
%
% Entradas
%   controlador : struct com parâmetros do controlador
%   planta      : struct com parâmetros da planta
%
% O modelo Simulink deve ler a variável-workspace  var_posicao.
% --------------------------------------------------------------

    % ► Variâncias que serão testadas (fixas)
    vars_posicao = [0 10 100 1000];

    % ► Parâmetros comuns às simulações
    tf                  = 0.5;
    thetar.time         = [0; tf];
    thetar.signals.values = [1; 1];
    thetar.signals.dimensions = 1;

    % Envia para o “base workspace” (onde o Simulink acessa)
    assignin('base','tf',tf);
    assignin('base','thetar',thetar);
    assignin('base','controlador',controlador);
    assignin('base','planta',planta);

    % ► Sinais que queremos plotar
    sinais = {'thetal','thetam','wl','ic','i','Vc','V'};

    % ► Containers para guardar dados de todas as variâncias
    dados = struct();
    for s = 1:numel(sinais)
        dados.(sinais{s}).t = {};
        dados.(sinais{s}).y = {};
    end

    % ----------------------------------------------------------
    % 1.  Loop de simulação (uma por variância)
    % ----------------------------------------------------------
    for k = 1:numel(vars_posicao)
        var_posicao = vars_posicao(k);
        assignin('base','var_posicao',var_posicao);      % torna visível ao modelo

        out = sim('servomotor_posicao');                 
        % Guarda cada sinal
        for s = 1:numel(sinais)
            sig = sinais{s};
            dados.(sig).t{k} = out.(sig).time;
            dados.(sig).y{k} = out.(sig).signals.values;
        end
    end

    % Paleta de cores para as curvas (4 cores)
    cores = lines(numel(vars_posicao));

    % ----------------------------------------------------------
    % 2.  Um gráfico por sinal, com todas as variâncias
    % ----------------------------------------------------------
    for s = 1:numel(sinais)
        sig = sinais{s};

        figure; hold on; grid on;
        for k = 1:numel(vars_posicao)
            plot(dados.(sig).t{k}, dados.(sig).y{k}, ...
                 'LineWidth',1.8, 'Color',cores(k,:));
        end
        xlabel('Tempo (s)', 'FontSize',14);
        ylabel(labelEixo(sig), 'FontSize',14);
        title(sprintf('%s – comparação de variâncias',sig), 'FontSize',14);
        legend(arrayfun(@(v) sprintf('\\sigma^{2} = %g',v), ...
                        vars_posicao, 'UniformOutput',false), ...
               'Location','best','Interpreter','tex');
    end
end
%==================================================================
function ylbl = labelEixo(sig)
    switch sig
        case 'thetal', ylbl = '\theta_l (rad)';
        case 'thetam', ylbl = '\theta_m (rad)';
        case 'wl',     ylbl = '\omega_l (rad/s)';
        case 'ic',     ylbl = 'i_c (A)';
        case 'i',      ylbl = 'i (A)';
        case 'Vc',     ylbl = 'V_c (V)';
        case 'V',      ylbl = 'V (V)';
        otherwise,     ylbl = sig;
    end
end

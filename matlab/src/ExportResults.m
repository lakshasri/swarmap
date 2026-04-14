function ExportResults(results, out_dir, timestamp)
%EXPORTRESULTS Save benchmark sweep to CSV and a multi-page PDF report.
    if nargin < 3, timestamp = datestr(now, 'yyyymmdd_HHMMSS'); end %#ok<DATST>
    T = struct2table(results);
    csv_path = fullfile(out_dir, sprintf('benchmark_%s.csv', timestamp));
    writetable(T, csv_path);

    pdf_path = fullfile(out_dir, sprintf('benchmark_report_%s.pdf', timestamp));
    fig = figure('Visible', 'off', 'Position', [0 0 800 600]);

    % Page 1: cover
    axes('Parent', fig); axis off; %#ok<LAXES>
    text(0.1, 0.8, 'Swarmap Benchmark Report', 'FontSize', 22, 'FontWeight', 'bold');
    text(0.1, 0.7, sprintf('Generated: %s', timestamp));
    text(0.1, 0.6, sprintf('Trials: %d rows', height(T)));
    text(0.1, 0.5, sprintf('Swarm sizes: %s', mat2str(unique(T.num_robots)')));
    text(0.1, 0.45, sprintf('Failure rates: %s', mat2str(unique(T.failure_rate)')));
    text(0.1, 0.4, sprintf('Noise levels: %s', mat2str(unique(T.noise)')));
    exportgraphics(fig, pdf_path, 'Append', false);

    % Page 2: coverage vs swarm size (one line per failure rate, noise=0.05)
    clf(fig);
    nz_ref = 0.05;
    T2 = T(T.noise == nz_ref, :);
    frs = unique(T2.failure_rate);
    hold on;
    for k = 1:numel(frs)
        sub = T2(T2.failure_rate == frs(k), :);
        g = groupsummary(sub, 'num_robots', 'mean', 'final_coverage');
        plot(g.num_robots, g.mean_final_coverage, '-o', ...
            'DisplayName', sprintf('fail=%.2f', frs(k)));
    end
    xlabel('Swarm size'); ylabel('Final coverage');
    title('Coverage vs. swarm size'); legend('Location', 'southeast'); grid on;
    exportgraphics(fig, pdf_path, 'Append', true);

    % Page 3: time-to-80% vs failure rate (swarm=20, noise=0.05)
    clf(fig);
    sub = T(T.num_robots == 20 & T.noise == nz_ref, :);
    g = groupsummary(sub, 'failure_rate', 'mean', 'time_to_80');
    bar(g.failure_rate, g.mean_time_to_80);
    xlabel('Failure rate'); ylabel('Time to 80% coverage (s)');
    title('Time to 80% coverage (N=20)'); grid on;
    exportgraphics(fig, pdf_path, 'Append', true);

    % Page 4: accuracy vs noise
    clf(fig);
    scatter(T.noise, T.final_accuracy, 20, T.num_robots, 'filled');
    xlabel('Noise level'); ylabel('Map accuracy'); title('Accuracy vs. noise');
    colorbar; grid on;
    exportgraphics(fig, pdf_path, 'Append', true);

    % Page 5: heatmap swarm × failure rate -> coverage (noise=0.05)
    clf(fig);
    sub = T(T.noise == nz_ref, :);
    g = groupsummary(sub, {'num_robots', 'failure_rate'}, 'mean', 'final_coverage');
    Nvals = unique(g.num_robots); Fvals = unique(g.failure_rate);
    H = nan(numel(Nvals), numel(Fvals));
    for i = 1:numel(Nvals)
        for j = 1:numel(Fvals)
            m = g.num_robots == Nvals(i) & g.failure_rate == Fvals(j);
            if any(m), H(i, j) = g.mean_final_coverage(m); end
        end
    end
    imagesc(Fvals, Nvals, H); set(gca, 'YDir', 'normal');
    xlabel('Failure rate'); ylabel('Swarm size');
    title('Final coverage — swarm size × failure rate'); colorbar;
    exportgraphics(fig, pdf_path, 'Append', true);

    close(fig);
    fprintf('Saved CSV: %s\nSaved PDF: %s\n', csv_path, pdf_path);
end

function ExportResults(results, out_dir, timestamp)
%EXPORTRESULTS Write benchmark struct array to CSV + PNG plots.
%   Octave-compatible (no struct2table / writetable / exportgraphics /
%   groupsummary). MATLAB also runs this fine.
    if nargin < 3, timestamp = datestr(now, 'yyyymmdd_HHMMSS'); end %#ok<DATST>
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    csv_path = fullfile(out_dir, sprintf('benchmark_%s.csv', timestamp));
    fields = {'num_robots', 'failure_rate', 'noise', 'trial', ...
              'final_coverage', 'final_accuracy', 'time_to_80', 'alive_end'};
    fid = fopen(csv_path, 'w');
    fprintf(fid, '%s\n', strjoin(fields, ','));
    n = numel(results);
    rows = zeros(n, numel(fields));
    for r = 1:n
        for f = 1:numel(fields)
            v = results(r).(fields{f});
            if isempty(v) || (isnumeric(v) && isnan(v)), v = NaN; end
            rows(r, f) = double(v);
            if f > 1, fprintf(fid, ','); end
            if isnan(rows(r, f))
                fprintf(fid, 'NaN');
            else
                fprintf(fid, '%g', rows(r, f));
            end
        end
        fprintf(fid, '\n');
    end
    fclose(fid);

    Nvec = rows(:, 1); Fvec = rows(:, 2); Cvec = rows(:, 5);
    Avec = rows(:, 6); Tvec = rows(:, 7); Nz   = rows(:, 3);
    nz_ref = mode(Nz);

    fig = figure('Visible', 'off', 'Position', [0 0 800 600]);

    % 1. Coverage vs swarm size, one line per failure rate
    clf(fig); hold on;
    frs = unique(Fvec);
    for k = 1:numel(frs)
        mask = (Fvec == frs(k)) & (Nz == nz_ref);
        Ns_k = unique(Nvec(mask));
        means = arrayfun(@(n) mean(Cvec(mask & (Nvec == n))), Ns_k);
        plot(Ns_k, means, '-o', 'DisplayName', sprintf('fail=%.2f', frs(k)));
    end
    xlabel('Swarm size'); ylabel('Final coverage');
    title('Coverage vs. swarm size'); legend('Location', 'southeast'); grid on;
    p1 = fullfile(out_dir, sprintf('coverage_vs_size_%s.png', timestamp));
    print(fig, p1, '-dpng', '-r120');

    % 2. Accuracy vs noise (scatter, colour by swarm size)
    clf(fig);
    scatter(Nz, Avec, 30, Nvec, 'filled');
    xlabel('Noise level'); ylabel('Map accuracy');
    title('Accuracy vs. noise (color = swarm size)'); colorbar; grid on;
    p2 = fullfile(out_dir, sprintf('accuracy_vs_noise_%s.png', timestamp));
    print(fig, p2, '-dpng', '-r120');

    % 3. Heatmap swarm size x failure rate
    clf(fig);
    Ns = unique(Nvec); Fs = unique(Fvec);
    H = nan(numel(Ns), numel(Fs));
    for i = 1:numel(Ns)
        for j = 1:numel(Fs)
            m = (Nvec == Ns(i)) & (Fvec == Fs(j)) & (Nz == nz_ref);
            if any(m), H(i, j) = mean(Cvec(m)); end
        end
    end
    imagesc(Fs, Ns, H); set(gca, 'YDir', 'normal');
    xlabel('Failure rate'); ylabel('Swarm size');
    title(sprintf('Final coverage — swarm size x failure rate (noise=%.2f)', nz_ref));
    colorbar;
    p3 = fullfile(out_dir, sprintf('heatmap_%s.png', timestamp));
    print(fig, p3, '-dpng', '-r120');

    close(fig);
    fprintf('Saved CSV: %s\n', csv_path);
    fprintf('Saved plots: %s, %s, %s\n', p1, p2, p3);
end

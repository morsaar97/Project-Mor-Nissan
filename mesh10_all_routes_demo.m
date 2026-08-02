function mesh10_all_routes_demo()
% MESH10_ALL_ROUTES_DEMO
% 10-node mesh simulation + enumerate ALL simple paths from src->dst
% and compute metrics per path: Throughput, Latency, SNR, PDR, Capacity.
%
% No special toolboxes required (uses base MATLAB: graph/plot/table).
%
% You can tweak:
%   - src, dst
%   - connectivity probability / minimum degree
%   - PHY assumptions (Tx power, bandwidth, noise figure, etc.)

clc; close all;

rng(7); % reproducible

%% ----------------- User knobs -----------------
N   = 10;
src = 1;
dst = 10;

% Topology density controls
pEdge        = 0.30;   % initial edge probability
minDegree    = 2;      % enforce at least this degree per node
maxEnumPaths = 20000;  % safety cap for too many paths
maxHops      = 9;      % for simple paths on 10 nodes, max hops is 9

% PHY / MAC assumptions (simple, explanatory model)
B_Hz      = 20e6;      % bandwidth (Hz)
Tx_dBm    = 15;        % transmit power per node (dBm)
NF_dB     = 7;         % receiver noise figure (dB)
f_Hz      = 2.4e9;     % carrier frequency (Hz)
packet_B  = 1500;      % packet size (bytes)
macEff    = 0.65;      % MAC efficiency factor (overhead)
c_light   = 3e8;       % speed of light

% Queueing / processing delay model (very rough)
procDelay_s_mean = 0.3e-3;  % per-hop processing mean
procDelay_s_jit  = 0.2e-3;  % per-hop jitter

%% ----------------- Build a connected random mesh -----------------
[pos, A] = buildConnectedMesh(N, pEdge, minDegree);

G = graph(A);
fprintf("Topology built: %d nodes, %d undirected links\n", numnodes(G), numedges(G));

%% ----------------- Link metrics from geometry -----------------
% Compute per-link distance and SNR, capacity, etc.
[link] = computeLinkMetrics(G, pos, B_Hz, Tx_dBm, NF_dB, f_Hz);

%% ----------------- Enumerate all simple paths src->dst -----------------
paths = allSimplePaths(A, src, dst, maxHops, maxEnumPaths);

if isempty(paths)
    error("No path found from %d to %d (should not happen if connected).", src, dst);
end

fprintf("Enumerated %d simple routes from Node %d to Node %d\n", size(paths,1), src, dst);

%% ----------------- Compute per-path end-to-end metrics -----------------
results = computePathMetrics(paths, link, B_Hz, macEff, packet_B, c_light, ...
                            procDelay_s_mean, procDelay_s_jit, pos);

% Show best paths
showBestPaths(results);

%% ----------------- Display results table -----------------
T = results.table;
disp(T);

%% ----------------- Visualize topology + highlight best routes -----------------
visualizeTopology(G, pos, src, dst, results);

end

%% ====== Helper: build connected mesh with min degree ======
function [pos, A] = buildConnectedMesh(N, pEdge, minDegree)
% Random positions in 2D (meters)
pos = [80*rand(N,1), 60*rand(N,1)];

% Build random adjacency, ensure symmetric, no self loops
A = zeros(N,N);

% Try until connected and min degree satisfied
for attempt = 1:200
    A(:) = 0;
    for i=1:N
        for j=i+1:N
            if rand < pEdge
                A(i,j)=1; A(j,i)=1;
            end
        end
    end

    % Enforce minimum degree by adding nearest neighbors if needed
    for i=1:N
        deg = sum(A(i,:));
        if deg < minDegree
            % add edges to closest nodes not already connected
            d = hypot(pos(:,1)-pos(i,1), pos(:,2)-pos(i,2));
            [~, idx] = sort(d, "ascend");
            idx(idx==i) = [];
            k = 1;
            while sum(A(i,:)) < minDegree && k <= numel(idx)
                j = idx(k);
                A(i,j)=1; A(j,i)=1;
                k = k+1;
            end
        end
    end

    % Check connectivity
    G = graph(A);
    bins = conncomp(G);
    if numel(unique(bins))==1
        return;
    end
end

error("Failed to build a connected graph after many attempts. Increase pEdge.");
end

%% ====== Helper: compute per-link metrics ======
function link = computeLinkMetrics(G, pos, B_Hz, Tx_dBm, NF_dB, f_Hz)
% For each undirected edge, compute:
% - distance
% - path loss (free-space)
% - received power
% - noise power
% - SNR (linear & dB)
% - Shannon capacity (bps)
%
% Store in NxN matrices with NaN for non-edges.

N = numnodes(G);
A = adjacency(G);

dist_m = nan(N);
snrLin = nan(N);
snr_dB = nan(N);
cap_bps = nan(N);

% Noise power: -174 dBm/Hz + 10log10(B) + NF
noise_dBm = -174 + 10*log10(B_Hz) + NF_dB;

% FSPL(dB) = 20log10(d) + 20log10(f) - 147.55  (d in meters, f in Hz)
for i=1:N
    for j=i+1:N
        if A(i,j)==1
            d = hypot(pos(i,1)-pos(j,1), pos(i,2)-pos(j,2));
            d = max(d, 1.0); % avoid 0

            fspl_dB = 20*log10(d) + 20*log10(f_Hz) - 147.55;
            rx_dBm  = Tx_dBm - fspl_dB;

            snr_dB_ij  = rx_dBm - noise_dBm;
            snrLin_ij  = 10^(snr_dB_ij/10);

            cap_ij = B_Hz * log2(1 + snrLin_ij); % Shannon

            dist_m(i,j)=d; dist_m(j,i)=d;
            snr_dB(i,j)=snr_dB_ij; snr_dB(j,i)=snr_dB_ij;
            snrLin(i,j)=snrLin_ij; snrLin(j,i)=snrLin_ij;
            cap_bps(i,j)=cap_ij; cap_bps(j,i)=cap_ij;
        end
    end
end

link.A = A;
link.dist_m = dist_m;
link.snr_dB = snr_dB;
link.snrLin = snrLin;
link.cap_bps = cap_bps;
end

%% ====== Helper: enumerate all simple paths (DFS) ======
function paths = allSimplePaths(A, src, dst, maxHops, maxEnumPaths)
N = size(A,1);

paths = zeros(0, N);   % store as fixed-width rows, 0 padding
pathLens = zeros(0,1);

stack = src;
visited = false(1,N);
visited(src) = true;

% manual recursion via nested function
count = 0;
dfs(src);

% trim to variable length cell -> then to padded numeric
if isempty(pathLens)
    paths = zeros(0,N);
    return;
end
paths = paths(:,1:max(pathLens)); %#ok<NASGU>
% We’ll return as cell array inside computePathMetrics; easier:
% But keep numeric padded for speed
% We kept numeric padded; computePathMetrics will read by lens.

% Instead of returning lens separately, encode length in last column? We'll return struct-ish:
% To keep interface simple: return a cell array of vectors.
pathsCell = cell(numel(pathLens),1);
for k=1:numel(pathLens)
    pathsCell{k} = paths(k,1:pathLens(k));
end
paths = pathsCell;

    function dfs(u)
        if count >= maxEnumPaths
            return;
        end
        if u == dst
            count = count + 1;
            % record current stack
            L = numel(stack);
            row = zeros(1,N);
            row(1:L) = stack;
            paths(end+1,:) = row; %#ok<AGROW>
            pathLens(end+1,1) = L; %#ok<AGROW>
            return;
        end
        if numel(stack)-1 >= maxHops
            return;
        end

        nbrs = find(A(u,:));
        for v = nbrs
            if ~visited(v)
                visited(v) = true;
                stack(end+1) = v; %#ok<AGROW>
                dfs(v);
                stack(end) = [];
                visited(v) = false;
                if count >= maxEnumPaths
                    return;
                end
            end
        end
    end
end

%% ====== Helper: compute per-path metrics ======
function results = computePathMetrics(pathsCell, link, B_Hz, macEff, packet_B, c_light, ...
                                     procDelay_mean, procDelay_jit, pos)

numPaths = numel(pathsCell);

routeStr  = strings(numPaths,1);
hops      = zeros(numPaths,1);

bottSNRdB = zeros(numPaths,1);
bottCapMbps = zeros(numPaths,1);
thrMbps   = zeros(numPaths,1);
latMs     = zeros(numPaths,1);
pdr       = zeros(numPaths,1);

bitsPerPkt = packet_B * 8;

for k=1:numPaths
    p = pathsCell{k};
    hops(k) = numel(p)-1;

    % per-hop metrics
    hopSNRlin = zeros(hops(k),1);
    hopSNRdB  = zeros(hops(k),1);
    hopCap    = zeros(hops(k),1);
    hopLat    = zeros(hops(k),1);
    hopPsucc  = zeros(hops(k),1);

    for i=1:hops(k)
        u = p(i); v = p(i+1);

        snrLin_uv = link.snrLin(u,v);
        snrDB_uv  = link.snr_dB(u,v);
        cap_uv    = link.cap_bps(u,v);

        % Propagation delay ~ d/c
        d = hypot(pos(u,1)-pos(v,1), pos(u,2)-pos(v,2));
        prop = d / c_light;

        % Processing/queue delay (simple jitter)
        proc = max(0, procDelay_mean + procDelay_jit*(2*rand-1));

        % Total hop latency
        hopLat(i) = prop + proc;

        % Simple BER model (BPSK in AWGN): BER = 0.5*erfc(sqrt(SNR))
        % Then packet success ~ (1-BER)^(bits)
        ber = 0.5 * erfc(sqrt(max(snrLin_uv,1e-12)));
        hopPsucc(i) = max(0, min(1, (1-ber)^bitsPerPkt));

        hopSNRlin(i) = snrLin_uv;
        hopSNRdB(i)  = snrDB_uv;
        hopCap(i)    = cap_uv;
    end

    % End-to-end path aggregation (typical for multi-hop):
    % Throughput and capacity dominated by bottleneck link:
    bottCap = min(hopCap);                 % bps
    bottSNR = min(hopSNRdB);               % dB (bottleneck)
    e2ePDR  = prod(hopPsucc);              % product
    e2eLat  = sum(hopLat);                 % sum seconds

    % Achievable throughput ~ MAC efficiency * bottleneck capacity
    thr = macEff * bottCap;

    bottSNRdB(k)   = bottSNR;
    bottCapMbps(k) = bottCap / 1e6;
    thrMbps(k)     = thr / 1e6;
    latMs(k)       = e2eLat * 1e3;
    pdr(k)         = e2ePDR;

    routeStr(k) = strjoin("N"+string(p), "→");
end

T = table(routeStr, hops, thrMbps, latMs, bottSNRdB, pdr, bottCapMbps, ...
    'VariableNames', ["Route","Hops","Throughput_Mbps","Latency_ms","BottleneckSNR_dB","PDR","BottleneckCapacity_Mbps"]);

results.table = T;

% Identify best per metric
[~, iThr] = max(T.Throughput_Mbps);
[~, iLat] = min(T.Latency_ms);
[~, iSNR] = max(T.BottleneckSNR_dB);
[~, iPDR] = max(T.PDR);
[~, iCap] = max(T.BottleneckCapacity_Mbps);

results.best.throughput = iThr;
results.best.latency    = iLat;
results.best.snr        = iSNR;
results.best.pdr        = iPDR;
results.best.capacity   = iCap;

end

%% ====== Helper: show best routes ======
function showBestPaths(results)
T = results.table;
b = results.best;

fprintf("\n===== BEST ROUTES (Node 1 -> Node 10) =====\n");

fprintf("Max Throughput:  %.3f Mbps | %s\n", T.Throughput_Mbps(b.throughput), T.Route(b.throughput));
fprintf("Min Latency:     %.3f ms   | %s\n", T.Latency_ms(b.latency), T.Route(b.latency));
fprintf("Max Bottleneck SNR: %.2f dB | %s\n", T.BottleneckSNR_dB(b.snr), T.Route(b.snr));
fprintf("Max PDR:         %.6f      | %s\n", T.PDR(b.pdr), T.Route(b.pdr));
fprintf("Max Capacity:    %.3f Mbps | %s\n", T.BottleneckCapacity_Mbps(b.capacity), T.Route(b.capacity));

fprintf("==========================================\n\n");
end

%% ====== Helper: visualize topology and highlight best routes ======
function visualizeTopology(G, pos, src, dst, results)
T = results.table;
b = results.best;

figure;
p = plot(G, 'XData', pos(:,1), 'YData', pos(:,2));
grid on;
title("10-Node Mesh Topology");
xlabel("X [m]"); ylabel("Y [m]");
labelnode(p, 1:numnodes(G), "N"+string(1:numnodes(G)));

% Mark src/dst
highlight(p, src, 'MarkerSize', 8);
highlight(p, dst, 'MarkerSize', 8);

% Helper to highlight a route
    function highlightRoute(routeStr, lineWidth)
        % Parse route like "N1→N3→N7→N10"
        parts = split(routeStr, "→");
        nodes = zeros(numel(parts),1);
        for i=1:numel(parts)
            nodes(i) = str2double(extractAfter(parts(i),"N"));
        end
        for i=1:numel(nodes)-1
            highlight(p, nodes(i), nodes(i+1), 'LineWidth', lineWidth);
        end
    end

% Show best routes in separate figures for clarity
bestNames = ["Throughput","Latency","SNR","PDR","Capacity"];
bestIdx   = [b.throughput, b.latency, b.snr, b.pdr, b.capacity];

for k=1:numel(bestIdx)
    figure;
    pp = plot(G, 'XData', pos(:,1), 'YData', pos(:,2));
    grid on; axis tight;
    title("Best route by " + bestNames(k));
    xlabel("X [m]"); ylabel("Y [m]");
    labelnode(pp, 1:numnodes(G), "N"+string(1:numnodes(G)));

    highlight(pp, src, 'MarkerSize', 8);
    highlight(pp, dst, 'MarkerSize', 8);

    % Highlight the winning path
    route = T.Route(bestIdx(k));
    parts = split(route, "→");
    nodes = zeros(numel(parts),1);
    for i=1:numel(parts)
        nodes(i) = str2double(extractAfter(parts(i),"N"));
    end
    for i=1:numel(nodes)-1
        highlight(pp, nodes(i), nodes(i+1), 'LineWidth', 3);
    end
end

% Metric comparison plots (per route)
figure; bar(T.Throughput_Mbps); grid on;
title("Throughput per route"); xlabel("Route #"); ylabel("Mbps");

figure; bar(T.Latency_ms); grid on;
title("Latency per route"); xlabel("Route #"); ylabel("ms");

figure; bar(T.BottleneckSNR_dB); grid on;
title("Bottleneck SNR per route"); xlabel("Route #"); ylabel("dB");

figure; bar(T.PDR); grid on;
title("PDR per route"); xlabel("Route #"); ylabel("Probability");

figure; bar(T.BottleneckCapacity_Mbps); grid on;
title("Bottleneck Capacity per route"); xlabel("Route #"); ylabel("Mbps");
end

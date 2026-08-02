function mesh10_mobility_connected_demo()
% 10-node mobile mesh; random positions; movement; always connected.
% Computes all simple paths from src->dst each time step and evaluates:
% Throughput, Latency, Bottleneck SNR, PDR, Capacity.

clc; close all;
rng(4);

%% ---------------- Settings you can change ----------------
N = 10;      src = 1;   dst = 10;

% Area size (meters): increase these to see the effect
areaW = 120;     % width  (x range)
areaH = 80;      % height (y range)

% Mobility
dt      = 0.5;   % seconds per step
T       = 40;    % number of steps
%so total simulated time=40x0.5=20 sec
vMin    = 0.5;   % m/s
vMax    = 2.5;   % m/s

% ---- Snapshots (10 frames) ----
numSnapshots = 10;
snapSteps = round(linspace(1, T, numSnapshots));   % which time steps to capture
snapPos   = cell(numSnapshots,1);
snapA     = cell(numSnapshots,1);
snapR     = nan(numSnapshots,1);
snapIdx   = 0;


% PHY assumptions
B_Hz   = 20e6;     % bandwidth (20 MHz)
f_Hz   = 2.4e9;    % 2.4 GHz carrier
NF_dB  = 7;        % noise figure
Tx_dBm = 15;       % transmit power per node (dBm)

% MAC + packet model
packet_B = 1500;
macEff   = 0.60;

% Safety cap for path enumeration
maxEnumPaths = 20000;

%% ---------------- Init node positions and waypoints ----------------
pos = [areaW*rand(N,1), areaH*rand(N,1)];
wp  = [areaW*rand(N,1), areaH*rand(N,1)];
spd = vMin + (vMax-vMin)*rand(N,1);

%% ---------------- Time series storage ----------------
bestRouteStr = strings(T,5); % [Thr Lat SNR PDR Cap]
bestVal      = nan(T,5);

meanThr = nan(T,1); meanLat = nan(T,1); meanSNR = nan(T,1); meanPDR = nan(T,1);

%% ---------------- Main loop ----------------
for t = 1:T
    % Move nodes toward waypoint
    [pos, wp, spd] = moveRandomWaypoint(pos, wp, spd, areaW, areaH, vMin, vMax, dt);

    % Ensure connected graph by adapting radio range (conceptual: keep mesh connected)
    % We find the minimum R such that the graph is connected given current positions.
    R = findMinConnectedRange(pos); %minimal range to keep connected

    % Build adjacency from range
    A = buildAdjFromRange(pos, R);
    G = graph(A);

    % Capture snapshot if this step is one of the chosen snapshot steps
if any(t == snapSteps)
    snapIdx = snapIdx + 1;
    snapPos{snapIdx} = pos;
    snapA{snapIdx}   = A;
    snapR(snapIdx)   = R;
end

    % Compute link metrics from geometry & PHY
    link = computeLinkMetrics_fromPos(A, pos, B_Hz, Tx_dBm, NF_dB, f_Hz);

    % Enumerate all simple paths src->dst (no repeated nodes)
    paths = allSimplePaths_cell(A, src, dst, N-1, maxEnumPaths);

    % Compute per-path metrics
    if isempty(paths)
        % Shouldn't happen since graph connected, but keep robust
        continue;
    end
    results = computePathMetrics(paths, link, B_Hz, macEff, packet_B, pos);

    % Aggregate for plots/intuition
    meanThr(t) = mean(results.table.Throughput_Mbps);
    meanLat(t) = mean(results.table.Latency_ms);
    meanSNR(t) = mean(results.table.BottleneckSNR_dB);
    meanPDR(t) = mean(results.table.PDR);

    % Best per metric this time step
    Ttbl = results.table;
    [bestVal(t,1), iThr] = max(Ttbl.Throughput_Mbps);
    [bestVal(t,2), iLat] = min(Ttbl.Latency_ms);
    [bestVal(t,3), iSNR] = max(Ttbl.BottleneckSNR_dB);
    [bestVal(t,4), iPDR] = max(Ttbl.PDR);
    [bestVal(t,5), iCap] = max(Ttbl.BottleneckCapacity_Mbps);

    bestRouteStr(t,1) = Ttbl.Route(iThr);
    bestRouteStr(t,2) = Ttbl.Route(iLat);
    bestRouteStr(t,3) = Ttbl.Route(iSNR);
    bestRouteStr(t,4) = Ttbl.Route(iPDR);
    bestRouteStr(t,5) = Ttbl.Route(iCap);

    % Quick visualization every few steps
    if mod(t,10)==1 || t==T
        figure(1); clf;
        p = plot(G,'XData',pos(:,1),'YData',pos(:,2));
        title(sprintf("Mobile mesh (t=%d/%d), forced connected. Range R=%.1f m", t, T, R));
        grid on; axis([0 areaW 0 areaH]); xlabel("x [m]"); ylabel("y [m]");
        labelnode(p,1:N,"N"+string(1:N));
        highlight(p,src,'MarkerSize',8);
        highlight(p,dst,'MarkerSize',8);
        drawnow;
    end
end

%% ---- 10-frame montage of topology over time ----
figure; 
tiledlayout(2,5, "Padding","compact","TileSpacing","compact");

for k = 1:numSnapshots
    nexttile;
    A_k = snapA{k};
    pos_k = snapPos{k};
    G_k = graph(A_k);

    p = plot(G_k, 'XData', pos_k(:,1), 'YData', pos_k(:,2));
    axis([0 areaW 0 areaH]); axis square; grid on;
    title(sprintf("t=%d (R=%.1fm)", snapSteps(k), snapR(k)), 'FontSize', 9);

    % Optional: show node labels (can clutter). Comment out if too busy.
    labelnode(p, 1:N, "N"+string(1:N));

    % Highlight src/dst
    highlight(p, src, 'MarkerSize', 7);
    highlight(p, dst, 'MarkerSize', 7);
end

%% ---------------- Report (last step snapshot) ----------------
fprintf("\n=== ASSUMPTIONS USED IN THIS SIM ===\n");
fprintf("Bandwidth B = %.1f MHz\n", B_Hz/1e6);
fprintf("Carrier f = %.2f GHz\n", f_Hz/1e9);
fprintf("Tx power = %.1f dBm\n", Tx_dBm);
fprintf("Noise figure = %.1f dB\n", NF_dB);
fprintf("Area = %.0fm x %.0fm\n", areaW, areaH);
fprintf("Mobility: Random waypoint, v in [%.1f, %.1f] m/s, dt=%.2fs, steps=%d\n", vMin, vMax, dt, T);

fprintf("\n=== BEST ROUTES OVER TIME (examples) ===\n");
disp(table((1:T)', bestRouteStr(:,1), bestVal(:,1), bestRouteStr(:,2), bestVal(:,2), ...
    'VariableNames', ["Step","BestThroughputRoute","Thr_Mbps","BestLatencyRoute","Lat_ms"]));

%% Plots of how the network metrics evolve over time
figure; plot(meanThr,'-'); grid on; xlabel("Step"); ylabel("Mbps"); title("Mean throughput across routes vs time");
figure; plot(meanLat,'-'); grid on; xlabel("Step"); ylabel("ms");  title("Mean latency across routes vs time");
figure; plot(meanSNR,'-'); grid on; xlabel("Step"); ylabel("dB");  title("Mean bottleneck SNR across routes vs time");
figure; plot(meanPDR,'-'); grid on; xlabel("Step"); ylabel("PDR"); title("Mean PDR across routes vs time");

end

%% ===== Random Waypoint movement =====
function [pos, wp, spd] = moveRandomWaypoint(pos, wp, spd, W, H, vMin, vMax, dt)
N = size(pos,1);
for i=1:N
    v = spd(i);
    dir = wp(i,:) - pos(i,:);
    d = norm(dir);
    if d < 1e-6
        wp(i,:)  = [W*rand, H*rand];
        spd(i)   = vMin + (vMax-vMin)*rand;
        continue;
    end
    step = min(d, v*dt);
    pos(i,:) = pos(i,:) + (dir/d)*step;

    % If reached waypoint, pick a new one
    if norm(wp(i,:) - pos(i,:)) < 0.5
        wp(i,:) = [W*rand, H*rand];
        spd(i)  = vMin + (vMax-vMin)*rand;
    end
end
end

%% ===== Build adjacency from range =====
function A = buildAdjFromRange(pos, R)
N = size(pos,1);
A = zeros(N);
for i=1:N
    for j=i+1:N
        d = hypot(pos(i,1)-pos(j,1), pos(i,2)-pos(j,2));
        if d <= R
            A(i,j)=1; A(j,i)=1;
        end
    end
end
end

%% ===== Find minimum range to make the graph connected =====
function R = findMinConnectedRange(pos)
% Binary search on R
N = size(pos,1);
% upper bound: max distance in area
Dmax = 0;
for i=1:N
    for j=i+1:N
        Dmax = max(Dmax, hypot(pos(i,1)-pos(j,1), pos(i,2)-pos(j,2)));
    end
end
lo = 0; hi = Dmax;
for it=1:25
    mid = (lo+hi)/2;
    A = buildAdjFromRange(pos, mid);
    G = graph(A);
    bins = conncomp(G);
    if numel(unique(bins))==1
        hi = mid;
    else
        lo = mid;
    end
end
R = hi;
end

%% ===== Link metrics from geometry + PHY =====
function link = computeLinkMetrics_fromPos(A, pos, B_Hz, Tx_dBm, NF_dB, f_Hz)
N = size(A,1);

snrLin = nan(N); snr_dB = nan(N);
cap_bps = nan(N);

noise_dBm = -174 + 10*log10(B_Hz) + NF_dB;

for i=1:N
    for j=i+1:N
        if A(i,j)==1
            d = hypot(pos(i,1)-pos(j,1), pos(i,2)-pos(j,2));
            d = max(d,1.0);
            fspl_dB = 20*log10(d) + 20*log10(f_Hz) - 147.55; % free-space
            rx_dBm = Tx_dBm - fspl_dB;
            snr_ij_dB = rx_dBm - noise_dBm;
            snr_ij_lin = 10^(snr_ij_dB/10);
            cap_ij = B_Hz*log2(1+snr_ij_lin);

            snr_dB(i,j)=snr_ij_dB; snr_dB(j,i)=snr_ij_dB;
            snrLin(i,j)=snr_ij_lin; snrLin(j,i)=snr_ij_lin;
            cap_bps(i,j)=cap_ij;    cap_bps(j,i)=cap_ij;
        end
    end
end

link.A = A;
link.snrLin = snrLin;
link.snr_dB = snr_dB;
link.cap_bps = cap_bps;
end

%% ===== Enumerate all simple paths (DFS) =====
function pathsCell = allSimplePaths_cell(A, src, dst, maxHops, maxEnumPaths)
N = size(A,1);
visited = false(1,N);
stack = src;
visited(src)=true;
pathsCell = {};

count = 0;
dfs(src);

    function dfs(u)
        if count >= maxEnumPaths
            return;
        end
        if u==dst
            count = count + 1;
            pathsCell{end+1,1} = stack; %#ok<AGROW>
            return;
        end
        if numel(stack)-1 >= maxHops
            return;
        end
        nbrs = find(A(u,:));
        for v = nbrs
            if ~visited(v)
                visited(v)=true;
                stack(end+1)=v; %#ok<AGROW>
                dfs(v);
                stack(end)=[];
                visited(v)=false;
                if count >= maxEnumPaths
                    return;
                end
            end
        end
    end
end

%% ===== Per-path metrics =====
function results = computePathMetrics(pathsCell, link, B_Hz, macEff, packet_B, pos)
numPaths = numel(pathsCell);
bitsPerPkt = packet_B*8;

routeStr = strings(numPaths,1);
hops = zeros(numPaths,1);
thrMbps = zeros(numPaths,1);
latMs = zeros(numPaths,1);
bottSNRdB = zeros(numPaths,1);
pdr = zeros(numPaths,1);
bottCapMbps = zeros(numPaths,1);

c = 3e8;

for k=1:numPaths
    p = pathsCell{k};
    H = numel(p)-1;
    hops(k)=H;

    hopCap = zeros(H,1);
    hopSNRdB = zeros(H,1);
    hopPsucc = zeros(H,1);
    hopLat = zeros(H,1);

    for i=1:H
        u=p(i); v=p(i+1);

        snrLin_uv = link.snrLin(u,v);
        snrDB_uv  = link.snr_dB(u,v);
        cap_uv    = link.cap_bps(u,v);

        % Prop delay
        d = hypot(pos(u,1)-pos(v,1), pos(u,2)-pos(v,2));
        prop = d/c;

        % Small processing delay (fixed simple) - fixed processing delay
        % per hop
        proc = 0.4e-3;

        hopLat(i) = prop + proc;

        % BER->PER approximation (BPSK-ish)
        ber = 0.5*erfc(sqrt(max(snrLin_uv,1e-12)));
        hopPsucc(i) = max(0,min(1,(1-ber)^bitsPerPkt));

        hopCap(i)=cap_uv;
        hopSNRdB(i)=snrDB_uv;
    end

    bottCap = min(hopCap);
    bottCapMbps(k)=bottCap/1e6;

    bottSNRdB(k)=min(hopSNRdB);
    pdr(k)=prod(hopPsucc);
    latMs(k)=sum(hopLat)*1e3;

    thrMbps(k)=(macEff*bottCap)/1e6;

    routeStr(k)=strjoin("N"+string(p),"→");
end

T = table(routeStr,hops,thrMbps,latMs,bottSNRdB,pdr,bottCapMbps, ...
    'VariableNames',["Route","Hops","Throughput_Mbps","Latency_ms","BottleneckSNR_dB","PDR","BottleneckCapacity_Mbps"]);

results.table = T;
end

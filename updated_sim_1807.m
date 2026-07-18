function tactical_mesh_sim_selected_metrics()
% =========================================================================
%  ESP32-S3FN8 + SX1262 Multi-Mode 100-Node Mobile Mesh Simulation
%  Area: 30 m x 200 m | Custom tactical RF / LoRa / Wi-Fi / BLE selectable PHY model
%
%  VISUALIZATIONS:
%   Fig 1  - Live animated topology (updates every step):
%              * Edge colors = SNR quality (red->yellow->green)
%              * Node trails showing recent movement
%              * Best throughput route highlighted in gold
%              * Weak links with SNR < 10 dB are excluded from routing
%              * SNR colorbar
%   Fig 2  - Live 4-panel dashboard (updates every step):
%              * Mean & best throughput vs time
%              * PDR area fill vs time
%              * Active SNR-valid connections area chart
%              * SNR + radio range dual-axis
%   Fig 3  - 10-frame topology montage (post-run)
%   Fig 4  - 6-panel time-series metric plots (post-run)
%
%  Mobility:   Random Waypoint
%  Link rule:  Links with SNR < 10 dB are excluded from the routing graph
%  PHY:        selectable LoRa / Wi-Fi / BLE Mesh model. LoRa uses SF, duty cycle and Time-on-Air.
% =========================================================================
clc; close all;
rng(42);

%% ==================== CONFIGURABLE PARAMETERS ==========================
N   = 50;   src = 1;   dst = N;

areaW = 30;  areaH = 200;      % 30 x 200 m tactical corridor area

% Mobility
dt   = 0.5;  T  = 60;     % 1 ms step, total sim time = 30 s CHANGED
vMin = 0.3;   vMax = 1.5;     % m/s

% Hardware / PHY mode selection: ESP32-S3FN8 + SX1262 board
% Supported board radios:
%   1) LoRa SX1262: 470-510 MHz / 863-928 MHz, Tx up to 21 dBm, sensitivity down to -139 dBm
%   2) Wi-Fi 802.11 b/g/n: up to 150 Mbps, short-range high-rate mode
%   3) Bluetooth LE / Bluetooth Mesh: low-rate short-range mesh mode
radioMode = "LoRa";          % choose: "Custom" , "LoRa", "WiFi", or "BLEMesh"

% LoRa scenario parameters. These are used only when radioMode = "LoRa".
loraSF = 10;                   % choose 7, 10, or 12 to show range/latency tradeoff
loraBW_Hz = 125e3;            % common LoRa bandwidth: 125 kHz
loraCR = 1;                   % coding rate index: 1 means 4/5
loraPreamble = 8;             % LoRa preamble symbols
dutyCycle = 0.01;             % 1% duty-cycle limitation, important for LoRa

snrFailThreshold_dB = 10;

if radioMode == "LoRa"

    switch loraSF
        case 7
            snrFailThreshold_dB = -7.5;
        case 10
            snrFailThreshold_dB = -15;
        case 12
            snrFailThreshold_dB = -20;
        otherwise
            error('Unsupported LoRa spreading factor');
    end
end

% Configure radio parameters according to selected mode
[B_Hz,f_Hz,Tx_dBm,NF_dB,rxSensitivity_dBm,phyDataRate_bps,perHopAirtime_s,dutyCycle] = ...
    configureRadioMode(radioMode,loraSF,loraBW_Hz,loraCR,loraPreamble,dutyCycle);

% Scenario control
% "NORMAL": chip-spec 20x20m network, usually very strong SNR.
% "SNR_FAILURE_DEMO": adds obstacle/body/urban loss so some nodes fail
% only when their strongest SNR is below 10 dB.
scenario = "NORMAL";
switch scenario
    case "NORMAL"
        extraLoss_dB = 5;
        shadowStd_dB = 4;
    case "SNR_FAILURE_DEMO"
        extraLoss_dB = 40;      
        shadowStd_dB = 8;
    otherwise
        extraLoss_dB = 0;
        shadowStd_dB = 4;
end

% MAC
packet_B = 127; % bytes
macEff   = 0.70; % MAC efficiency / duty-cycle / overhead factor

% Path search restrictions
maxEnumPaths = 100;
maxHops      = 5;

% Trail length for animated topology
trailLen = 6;

% Snapshots for montage
numSnapshots = 10;
snapSteps    = round(linspace(1, T, numSnapshots));
plotEvery = 1;

%% ==================== INITIALISE =======================================
pos   = [areaW*rand(N,1), areaH*rand(N,1)];
wp    = [areaW*rand(N,1), areaH*rand(N,1)];
spd   = vMin + (vMax-vMin)*rand(N,1);
alive = true(N,1);
activeLinks = zeros(T,1); 

% Trail ring-buffer  N x trailLen x 2
trail     = nan(N, trailLen, 2);
trailHead = 1;

% Snapshots
snapPos   = cell(numSnapshots,1);
snapA     = cell(numSnapshots,1);
snapAlive = cell(numSnapshots,1);
snapLink  = cell(numSnapshots,1);
snapR     = nan(numSnapshots,1);
snapIdx   = 0;

% Time-series storage
bestVal      = nan(T,5);
bestRouteStr = strings(T,5);
bestPath_t   = cell(T,1);

meanThr    = nan(T,1);  meanLat  = nan(T,1);
meanSNR    = nan(T,1);  meanPDR  = nan(T,1);
aliveCount = nan(T,1);  numPaths_t = nan(T,1);
rangeR_t   = nan(T,1);
selectedThr = nan(T,1);
selectedLat = nan(T,1);
selectedPDR = nan(T,1);
selectedSNR = nan(T,1);

% Collected across all steps for histograms/CDF
allHops = [];  allPDR = [];

% Store SNR-PDR samples for post-analysis
allLinkSNR = [];
allSNRonlyHopPDR = [];
allFinalHopPDR   = [];

% Failure log  T x N
failLog = false(T,N);

% Link-failure log
fallenTime     = [];
fallenNode1    = [];
fallenNode2    = [];
fallenDistance = [];
fallenSNR      = [];
fallenRxPower  = [];

previousValidA = false(N);

%% SNR-threshold sensitivity analysis

thresholdVec = 0:5:20;             % Tested SNR thresholds [dB]
numThresholds = numel(thresholdVec);

% Selected-route metrics for every time step and threshold
thresholdSelectedPDR = nan(T,numThresholds);
thresholdSelectedThr = nan(T,numThresholds);
thresholdSelectedLat = nan(T,numThresholds);

% Number of links that remain after applying each threshold
thresholdActiveLinks = zeros(T,numThresholds);

% Whether a route from SRC to DST exists
thresholdRouteExists = false(T,numThresholds);

fprintf("=== Multi-Mode Mesh Simulation ===\n");
fprintf("Area: %.0fm x %.0fm | N=%d | Steps=%d | dt=%.4fs\n",areaW,areaH,N,T,dt);
fprintf("Mode=%s | BW=%.2f MHz | f=%.0f MHz | Tx=%.0f dBm | NF=%.0f dB\n",radioMode,B_Hz/1e6,f_Hz/1e6,Tx_dBm,NF_dB);
fprintf("RX sensitivity=%.0f dBm | PHY rate=%.2f kbps | dutyCycle=%.2f%%\n",rxSensitivity_dBm,phyDataRate_bps/1e3,100*dutyCycle);
if radioMode == "LoRa"
    fprintf("LoRa SF=%d | Time-on-Air per %d-byte packet = %.1f ms\n",loraSF,packet_B,1000*perHopAirtime_s);
end
fprintf("Scenario=%s | extraLoss=%.1f dB | shadowStd=%.1f dB\n",scenario,extraLoss_dB,shadowStd_dB);
fprintf("SNR failure threshold = %.1f dB\n",snrFailThreshold_dB);
fprintf("\n");

%% ==================== FIGURE WINDOWS ===================================
% Use fixed positive figure numbers instead of figure handles.
% This avoids "Argument must be a Figure object or a positive integer"
% errors if a live figure is closed/recreated during the animation.
figure(1);
set(gcf,'Name','LoRa Mesh Live Topology','NumberTitle','off',...
    'Position',[10 430 700 580],'Color',[0.08 0.08 0.12]);

figure(2);
set(gcf,'Name','LoRa Mesh Live Dashboard','NumberTitle','off',...
    'Position',[720 430 680 560],'Color',[0.08 0.08 0.12]);

%% ==================== MAIN SIMULATION LOOP =============================
for t = 1:T

    %-- 1. Move alive nodes ------------------------------------------
    [pos,wp,spd] = moveRWP(pos,wp,spd,areaW,areaH,vMin,vMax,dt,alive);

    % Update trail ring-buffer
    trail(:,trailHead,:) = reshape(pos,N,1,2);
    trailHead = mod(trailHead,trailLen)+1;

    %-- 2. Link-based SNR failure model -------------------------------
    % SNR is a link property, not a node property.
    % Therefore all nodes stay physically alive, but links with SNR < threshold
    % are removed from the routing graph so the algorithm cannot choose them.
    alive = true(N,1);
    aliveCount(t) = sum(alive);
    failLog(t,:)  = false(1,N);

    %-- 3. Build radio graph and remove weak links ---------------------
    [A_all, link_all] = buildLoRaRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB, ...
    f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);
    
    %% Test network performance for different SNR thresholds

for s = 1:numThresholds

    currentThreshold_dB = thresholdVec(s);

    % Keep only physical links whose SNR meets this threshold
    A_threshold = A_all & ...
        (link_all.snr_dB >= currentThreshold_dB);

    % Make sure the adjacency matrix is symmetric
    A_threshold = double(A_threshold & A_threshold');

    % Count unique active links
    thresholdActiveLinks(t,s) = ...
        nnz(triu(A_threshold));

    % Create link structure for this threshold
    link_threshold = link_all;
    link_threshold.A = A_threshold;

    % Remove the metrics of excluded links
    excludedLinks = (A_threshold == 0);

    link_threshold.snr_dB(excludedLinks)  = NaN;
    link_threshold.snrLin(excludedLinks)  = NaN;
    link_threshold.cap_bps(excludedLinks) = NaN;
    link_threshold.rx_dBm(excludedLinks)  = NaN;

    % Create graph for this threshold
    G_threshold = graph(A_threshold);

    % Check whether SRC and DST are connected
    componentID = conncomp(G_threshold);

    thresholdReachable = ...
        (componentID(src) == componentID(dst));

    if thresholdReachable

        % Find candidate routes
        thresholdPaths = allSimplePaths(...
            A_threshold,...
            src,...
            dst,...
            maxHops,...
            maxEnumPaths);

        if ~isempty(thresholdPaths)

            thresholdResults = pathMetrics(...
                thresholdPaths,...
                link_threshold,...
                B_Hz,...
                macEff,...
                packet_B,...
                pos,...
                perHopAirtime_s,...
                dutyCycle,...
                radioMode,...
                loraSF);

            thresholdTable = thresholdResults.table;

            % Select the route with maximum throughput
            [~,thresholdRouteIndex] = ...
                max(thresholdTable.Throughput_Mbps);

            % Store the selected-route metrics
            thresholdSelectedPDR(t,s) = ...
                thresholdTable.PDR(thresholdRouteIndex);

            thresholdSelectedThr(t,s) = ...
                thresholdTable.Throughput_Mbps(...
                thresholdRouteIndex);

            thresholdSelectedLat(t,s) = ...
                thresholdTable.Latency_us(...
                thresholdRouteIndex);

            thresholdRouteExists(t,s) = true;
        end
    end
end

    % Links allowed for routing
    currentValidA = A_all & (link_all.snr_dB >= snrFailThreshold_dB);

    % Links that were valid previously and have now fallen
    newlyFallenLinks = previousValidA & ~currentValidA;

    [badI,badJ] = find(triu(newlyFallenLinks));

    for q = 1:numel(badI)
        u = badI(q);
        v = badJ(q);

        d_uv = hypot(pos(u,1)-pos(v,1),pos(u,2)-pos(v,2));

        fallenTime(end+1,1)     = t*dt;
        fallenNode1(end+1,1)    = u;
        fallenNode2(end+1,1)    = v;
        fallenDistance(end+1,1) = d_uv;
        fallenSNR(end+1,1)      = link_all.snr_dB(u,v);
        fallenRxPower(end+1,1)  = link_all.rx_dBm(u,v);
    end

    % Routing network
    A = double(currentValidA);

    link = link_all;
    link.A = A;

    link.snr_dB(A==0)  = NaN;
    link.snrLin(A==0)  = NaN;
    link.cap_bps(A==0) = NaN;
    link.rx_dBm(A==0)  = NaN;

    G = graph(A);
    activeLinks(t) = nnz(triu(A));

    previousValidA = currentValidA;

    % evaluation radio range estimate based on sensitivity (no shadowing)
    % gives the maximum radius (R) around any given node (t)
    R = estimateLoRaRange(Tx_dBm,rxSensitivity_dBm,f_Hz,extraLoss_dB);
    rangeR_t(t) = R;

    %-- 5. Snapshot capture ------------------------------------------
    if any(t==snapSteps)
        snapIdx            = snapIdx+1;
        snapPos{snapIdx}   = pos;
        snapA{snapIdx}     = A;
        snapAlive{snapIdx} = alive;
        snapLink{snapIdx}  = link;
        snapR(snapIdx)     = R;
    end

    %-- 6. Reachability + path search --------------------------------
    bins      = conncomp(G);
    reachable = (bins(src)==bins(dst));
    bestPathNow = [];

    if reachable
        paths = allSimplePaths(A,src,dst,maxHops,maxEnumPaths);
        numPaths_t(t) = numel(paths);

        if ~isempty(paths)
            results = pathMetrics(paths,link,B_Hz,macEff,packet_B,pos,perHopAirtime_s,dutyCycle,radioMode,loraSF);
            Ttbl    = results.table;
            
            % Save per-hop SNR and PDR samples for post-run analysis
            allLinkSNR = [allLinkSNR; results.linkSNR];
            allSNRonlyHopPDR = [allSNRonlyHopPDR;results.snrOnlyHopPDR];
            allFinalHopPDR = [allFinalHopPDR;results.finalHopPDR];
    
            % Mean values over all available routes
            meanThr(t) = mean(Ttbl.Throughput_Mbps);
            meanLat(t) = mean(Ttbl.Latency_us);
            meanSNR(t) = mean(Ttbl.BottleneckSNR_dB);
            meanPDR(t) = mean(Ttbl.PDR);

            % Best routes according to each metric
            [bestVal(t,1),iThr] = max(Ttbl.Throughput_Mbps);
            [bestVal(t,2),iLat] = min(Ttbl.Latency_us);
            [bestVal(t,3),iSNR] = max(Ttbl.BottleneckSNR_dB);
            [bestVal(t,4),iPDR] = max(Ttbl.PDR);
            [bestVal(t,5),iCap] = max(Ttbl.BottleneckCapacity_Mbps);
    
            % Metrics of the route actually selected by maximum throughput
            selectedThr(t) = Ttbl.Throughput_Mbps(iThr);   % Throughput of selected path
            selectedLat(t) = Ttbl.Latency_us(iThr);        % Latency of selected path
            selectedPDR(t) = Ttbl.PDR(iThr);               % PDR of selected path
            selectedSNR(t) = Ttbl.BottleneckSNR_dB(iThr);  % Bottleneck SNR of selected path
    
            % Save best route strings
            bestRouteStr(t,1) = Ttbl.Route(iThr);
            bestRouteStr(t,2) = Ttbl.Route(iLat);
            bestRouteStr(t,3) = Ttbl.Route(iSNR);
            bestRouteStr(t,4) = Ttbl.Route(iPDR);
            bestRouteStr(t,5) = Ttbl.Route(iCap);
    
            % Selected route is the maximum-throughput route
            bestPathNow   = paths{iThr}; 
            bestPath_t{t} = bestPathNow;

            allHops = [allHops; Ttbl.Hops]; %#ok<AGROW>
            allPDR  = [allPDR;  Ttbl.PDR];  %#ok<AGROW>
        end
    else
        numPaths_t(t) = 0;
    end

if mod(t,plotEvery)==0 || t==1 || t==T

    %================================================================
    %  FIG 1 — Animated topology
    %================================================================
    figure(1);
    clf(gcf);
    set(gcf,'Name','LoRa Mesh Live Topology',...
        'NumberTitle','off',...
        'Position',[10 430 700 580],...
        'Color',[0.08 0.08 0.12]);
    ax1 = axes('Parent',gcf,'Color',[0.08 0.08 0.12],...
        'XColor','w','YColor','w');
    hold(ax1,'on');

    % -- Movement trails --
    for i = 1:N
        if ~alive(i), continue; end
        trXY = squeeze(trail(i,:,:));     % trailLen x 2
        tx = trXY(:,1);  ty = trXY(:,2);
        ok = ~isnan(tx);
        if sum(ok)<2, continue; end
        tx=tx(ok); ty=ty(ok);
        alphas = linspace(0.04, 0.30, numel(tx));
        for seg=1:numel(tx)-1
            plot(ax1,tx(seg:seg+1),ty(seg:seg+1),'-',...
                'Color',[0.35 0.75 1.0],'LineWidth',0.9);
        end
    end

    % -- Edges colored by SNR quality (red=poor -> green=good) --
    [ei,ej] = find(triu(A));
    for e = 1:numel(ei)
        u=ei(e); v=ej(e);
        if ~alive(u) || ~alive(v)
            continue; 
        end
        
        tc = max(0,min(1,(link.snr_dB(u,v)-5)/20));
        eCol = [(1-tc), tc*0.85, tc*0.25];
        plot(ax1,[pos(u,1) pos(v,1)],[pos(u,2) pos(v,2)],'-',...
            'Color',eCol,'LineWidth',0.85);
    end

    % -- Best path highlighted in gold --
    if ~isempty(bestPathNow)
        for h=1:numel(bestPathNow)-1
            u=bestPathNow(h);
            v=bestPathNow(h+1);
            if ~alive(u) || ~alive(v)
                continue;
            end
            plot(ax1,[pos(u,1) pos(v,1)],[pos(u,2) pos(v,2)],'-',...
                'Color',[1.0 0.85 0.0],'LineWidth',3.5);
        end
        % Annotate best path nodes
        for h=1:numel(bestPathNow)
            nd=bestPathNow(h);
            if nd~=src && nd~=dst
                scatter(ax1,pos(nd,1),pos(nd,2),40,[1 0.85 0],...
                    'filled','MarkerEdgeColor','none');
            end
        end
    end

    % -- Alive nodes --
    aIdx = find(alive);
    dIdx = setdiff(find(~alive),[src dst]);
    if ~isempty(aIdx)
        scatter(ax1,pos(aIdx,1),pos(aIdx,2),25,[0.35 0.65 1.0],...
            'filled','MarkerEdgeColor','none','MarkerFaceAlpha',0.85);
    end
    if ~isempty(dIdx)
        scatter(ax1,pos(dIdx,1),pos(dIdx,2),30,[1 0.25 0.25],...
            'x','LineWidth',1.8);
    end

    % -- SRC / DST --
    scatter(ax1,pos(src,1),pos(src,2),110,[0.1 1.0 0.3],...
        'filled','MarkerEdgeColor','w','LineWidth',1.8);
    scatter(ax1,pos(dst,1),pos(dst,2),110,[1.0 0.2 1.0],...
        'filled','MarkerEdgeColor','w','LineWidth',1.8);
    text(ax1,pos(src,1)+0.25,pos(src,2)+0.4,'SRC',...
        'Color',[0.1 1 0.3],'FontSize',8,'FontWeight','bold');
    text(ax1,pos(dst,1)+0.25,pos(dst,2)+0.4,'DST',...
        'Color',[1 0.2 1],'FontSize',8,'FontWeight','bold');

    % -- Legend box --
    text(ax1,0.05,areaH-0.15,'● SRC','Color',[0.1 1 0.3],'FontSize',7,'FontWeight','bold');
    text(ax1,0.85,areaH-0.15,'● DST','Color',[1 0.2 1],'FontSize',7,'FontWeight','bold');
    text(ax1,1.65,areaH-0.15,'● Node','Color',[0.35 0.65 1],'FontSize',7);
    text(ax1,2.65,areaH-0.15,'— Valid link','Color',[1 0.3 0.3],'FontSize',7);
    text(ax1,3.65,areaH-0.15,'— Best path','Color',[1 0.85 0],'FontSize',7);

   
    axis(ax1,[0 areaW 0 areaH]); grid(ax1,'on');
    ax1.GridColor=[0.25 0.25 0.25]; ax1.GridAlpha=0.5;

    if reachable && ~isnan(bestVal(t,1))
        ttl = sprintf("t=%.1f s | R=%.2f m | active links=%d | paths=%d | best Thr=%.1f Mbps",...
            t*dt,R,activeLinks(t),numPaths_t(t),bestVal(t,1));
    else
        ttl = sprintf("t=%.1f s | R=%.2f m | active links=%d | *** NO PATH src->dst ***",...
            t*dt,R,activeLinks(t));
    end
    title(ax1,ttl,'Color','w','FontSize',10,'FontWeight','bold');
    xlabel(ax1,'x [m]','Color','w'); ylabel(ax1,'y [m]','Color','w');
    drawnow;

    %================================================================
    %  FIG 2 — Live dashboard
    %================================================================
    figure(2);
    clf(gcf);
    set(gcf,'Name','LoRa Mesh Live Dashboard',...
        'NumberTitle','off',...
        'Position',[720 430 680 560],...
        'Color',[0.08 0.08 0.12]);
    tV = (1:t)*dt;

    axD = @(r,c) subplot(2,2,(r-1)*2+c,'Parent',gcf);

    % Panel A: Throughput
    axA = axD(1,1);
    set(axA,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
        'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);
    hold(axA,'on'); grid(axA,'on');
    plot(axA,tV,meanThr(1:t),'Color',[0.2 0.8 1.0],'LineWidth',1.4,'DisplayName','Mean Network');
    plot(axA,tV,selectedThr(1:t),'c-','LineWidth',2.2,'DisplayName','Selected Route');
    plot(axA,tV,bestVal(1:t,1),'y--','LineWidth',1.2,'DisplayName','Best Possible');
    legend(axA,'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',7,'Location','northwest');
    ylabel(axA,'Mbps','Color','w');
    title(axA,'Throughput (Mbps)','Color','w','FontWeight','bold');
    xlim(axA,[0 T*dt]);

    % Panel B: PDR
    %FIX TO BEST ROUTE
    axB = axD(1,2);
    set(axB,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
        'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);
    hold(axB,'on'); grid(axB,'on');
    patch(axB,[tV fliplr(tV)],[meanPDR(1:t)' zeros(1,t)],...
        [0.2 0.7 0.4],'FaceAlpha',0.35,'EdgeColor','none');
    plot(axB,tV,meanPDR(1:t),'Color',[0.2 0.8 0.2],'LineWidth',1.2);
    plot(axB,tV,selectedPDR(1:t),'c-','LineWidth',2.2);
    plot(axB,tV,bestVal(1:t,4),'y--','LineWidth',1.0);
    legend(axB,{'Network Fill','Mean Network','Selected Route','Best Possible'},...
        'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',7,'Location','southwest');
    ylabel(axB,'PDR','Color','w'); ylim(axB,[0 1.05]);
    title(axB,'Packet Delivery Ratio','Color','w','FontWeight','bold');
    xlim(axB,[0 T*dt]);

    % Panel C: Active connections
    axC = axD(2,1);
    set(axC,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);
    hold(axC,'on'); grid(axC,'on');

    area(axC,tV,activeLinks(1:t),'FaceColor',[0.2 0.5 0.9],...
        'FaceAlpha',0.65,'EdgeColor','none');

    ylabel(axC,'# Links','Color','w');
    xlabel(axC,'Time (s)','Color','w');
    title(axC,'Active SNR-Valid Connections','Color','w','FontWeight','bold');
    xlim(axC,[0 T*dt]);

    % Panel D: SNR 
    axD2 = axD(2,2);
    set(axD2,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
        'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);
    hold(axD2,'on'); grid(axD2,'on');
    plot(axD2,tV,selectedSNR(1:t),'c-','LineWidth',2.2);
    yline(axD2,snrFailThreshold_dB,'--','Color',[1 0.6 0],...
        'LineWidth',1.2,'Label','10 dB threshold');
    ylabel(axD2,'Bottleneck SNR (dB)','Color','w');
    xlabel(axD2,'Time (s)','Color','w');
    title(axD2,'Selected Route Bottleneck SNR','Color','w','FontWeight','bold');
    legend(axD2,...
    {'Selected Route Bottleneck SNR','Failure Threshold'},'TextColor','w',...
    'Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
    xlim(axD2,[0 T*dt]);

    sgtitle(sprintf("RF Mesh Dashboard  |  t = %.1f / %.1f s",t*dt,T*dt),...
        'Color','w','FontSize',12,'FontWeight','bold');
    drawnow;
end 
end % main loop

%% ===========================================================
%% Threshold sensitivity analysis
%% ===========================================================

meanPDRvsThreshold = ...
    mean(thresholdSelectedPDR,1,'omitnan');

meanThrVsThreshold = ...
    mean(thresholdSelectedThr,1,'omitnan');

meanLatVsThreshold = ...
    mean(thresholdSelectedLat,1,'omitnan');

meanLinksVsThreshold = ...
    mean(thresholdActiveLinks,1,'omitnan');

routeAvailabilityPercent = ...
    100*sum(thresholdRouteExists,1)/T;

%% FIGURE: Network performance versus SNR routing threshold

figure(...
    'Name','Performance versus SNR Threshold',...
    'NumberTitle','off',...
    'Position',[100 80 1100 720],...
    'Color',[0.08 0.08 0.12]);

tlThreshold = tiledlayout(...
    2,2,...
    'Padding','compact',...
    'TileSpacing','compact');

% ---------------------------------------------------------------
% 1. Selected-route PDR versus threshold
% ---------------------------------------------------------------
ax1 = nexttile(tlThreshold);

set(ax1,...
    'Color',[0.12 0.12 0.18],...
    'XColor','w',...
    'YColor','w');

hold(ax1,'on');
grid(ax1,'on');

plot(ax1,...
    thresholdVec,...
    meanPDRvsThreshold,...
    'o-',...
    'LineWidth',2,...
    'MarkerSize',6);

xline(ax1,...
    snrFailThreshold_dB,...
    '--',...
    'Current threshold',...
    'LineWidth',1.3);

xlabel(ax1,'SNR Threshold (dB)');
ylabel(ax1,'Mean Selected-Route PDR');

title(ax1,...
    'PDR vs SNR Threshold',...
    'Color','w',...
    'FontWeight','bold');

ylim(ax1,[0 1.02]);


% ---------------------------------------------------------------
% 2. Selected-route throughput versus threshold
% ---------------------------------------------------------------
ax2 = nexttile(tlThreshold);

set(ax2,...
    'Color',[0.12 0.12 0.18],...
    'XColor','w',...
    'YColor','w');

hold(ax2,'on');
grid(ax2,'on');

plot(ax2,...
    thresholdVec,...
    meanThrVsThreshold,...
    'o-',...
    'LineWidth',2,...
    'MarkerSize',6);

xline(ax2,...
    snrFailThreshold_dB,...
    '--',...
    'Current threshold',...
    'LineWidth',1.3);

xlabel(ax2,'SNR Threshold (dB)');
ylabel(ax2,'Mean Selected Throughput (Mbps)');

title(ax2,...
    'Throughput vs SNR Threshold',...
    'Color','w',...
    'FontWeight','bold');


% ---------------------------------------------------------------
% 3. Selected-route latency versus threshold
% ---------------------------------------------------------------
ax3 = nexttile(tlThreshold);

set(ax3,...
    'Color',[0.12 0.12 0.18],...
    'XColor','w',...
    'YColor','w');

hold(ax3,'on');
grid(ax3,'on');

plot(ax3,...
    thresholdVec,...
    meanLatVsThreshold,...
    'o-',...
    'LineWidth',2,...
    'MarkerSize',6);

xline(ax3,...
    snrFailThreshold_dB,...
    '--',...
    'Current threshold',...
    'LineWidth',1.3);

xlabel(ax3,'SNR Threshold (dB)');
ylabel(ax3,'Mean Selected Latency (\mus)');

title(ax3,...
    'Latency vs SNR Threshold',...
    'Color','w',...
    'FontWeight','bold');


% ---------------------------------------------------------------
% 4. Active links versus threshold
% ---------------------------------------------------------------
ax4 = nexttile(tlThreshold);

set(ax4,...
    'Color',[0.12 0.12 0.18],...
    'XColor','w',...
    'YColor','w');

hold(ax4,'on');
grid(ax4,'on');

plot(ax4,...
    thresholdVec,...
    meanLinksVsThreshold,...
    'o-',...
    'LineWidth',2,...
    'MarkerSize',6);

xline(ax4,...
    snrFailThreshold_dB,...
    '--',...
    'Current threshold',...
    'LineWidth',1.3);

xlabel(ax4,'SNR Threshold (dB)');
ylabel(ax4,'Mean Number of Active Links');

title(ax4,...
    'Active Links vs SNR Threshold',...
    'Color','w',...
    'FontWeight','bold');

sgtitle(tlThreshold,...
    'SNR Routing-Threshold Sensitivity Analysis',...
    'Color','w',...
    'FontSize',13,...
    'FontWeight','bold');

thresholdResultsTable = table(...
    thresholdVec',...
    meanPDRvsThreshold',...
    meanThrVsThreshold',...
    meanLatVsThreshold',...
    meanLinksVsThreshold',...
    routeAvailabilityPercent',...
    'VariableNames',{...
        'SNR_Threshold_dB',...
        'Mean_Selected_PDR',...
        'Mean_Selected_Throughput_Mbps',...
        'Mean_Selected_Latency_us',...
        'Mean_Active_Links',...
        'Route_Availability_Percent'});

disp(' ');
disp('SNR threshold sensitivity results:');
disp(thresholdResultsTable);

%% =========================================================================
%%  POST-RUN FIGURES
%% =========================================================================

fallenLinksTable = table( ...
    fallenTime, ...
    fallenNode1, ...
    fallenNode2, ...
    fallenDistance, ...
    fallenSNR, ...
    fallenRxPower, ...
    'VariableNames', { ...
        'Time_s', ...
        'Node1', ...
        'Node2', ...
        'Distance_m', ...
        'SNR_dB', ...
        'RxPower_dBm'});

disp('Fallen links:');
disp(fallenLinksTable);

tVec = (1:T)*dt;

%----------------------------------------------------------------
%  FIG 3 — 10-frame topology montage
%----------------------------------------------------------------
figure('Name','RF Mesh Topology Montage','NumberTitle','off',...
    'Position',[10 40 1350 530],'Color',[0.08 0.08 0.12]);
tl3 = tiledlayout(2,5,'Padding','compact','TileSpacing','compact');

for k = 1:numSnapshots
    nexttile(tl3);
    axM = gca;
    set(axM,'Color',[0.10 0.10 0.15],'XColor','w','YColor','w');
    hold(axM,'on');

    A_k     = snapA{k};
    pos_k   = snapPos{k};
    alive_k = snapAlive{k};
    link_k = snapLink{k};
    
    % Edges colored by SNR
    [eki,ekj]=find(triu(A_k));
    for e=1:numel(eki)
        u=eki(e); v=ekj(e);
        tc=max(0,min(1,(link_k.snr_dB(u,v)-5)/20));
        eCol=[(1-tc) tc*0.85 tc*0.25];
        plot(axM,[pos_k(u,1) pos_k(v,1)],[pos_k(u,2) pos_k(v,2)],'-',...
            'Color',eCol,'LineWidth',0.7);
    end

    aIdx_k = find(alive_k);
    dIdx_k = setdiff(find(~alive_k),[src dst]);
    if ~isempty(aIdx_k)
        scatter(axM,pos_k(aIdx_k,1),pos_k(aIdx_k,2),16,...
            [0.35 0.65 1.0],'filled','MarkerEdgeColor','none');
    end
    if ~isempty(dIdx_k)
        scatter(axM,pos_k(dIdx_k,1),pos_k(dIdx_k,2),18,...
            [1 0.25 0.25],'x','LineWidth',1.1);
    end
    scatter(axM,pos_k(src,1),pos_k(src,2),55,[0.1 1 0.3],...
        'filled','MarkerEdgeColor','w');
    scatter(axM,pos_k(dst,1),pos_k(dst,2),55,[1 0.2 1],...
        'filled','MarkerEdgeColor','w');

    axis(axM,[0 areaW 0 areaH]); axis(axM,'square');
    grid(axM,'on'); axM.GridColor=[0.22 0.22 0.22]; axM.GridAlpha=0.55;
    title(axM,sprintf("t=%.1fs  R=%.2fm  active links=%d",...
        snapSteps(k)*dt,snapR(k),nnz(triu(A_k))),...
        'Color','w','FontSize',8);
end
sgtitle(tl3,'RF Mesh Topology Snapshots  (green=SRC, magenta=DST, weak links excluded)',...
    'Color','w','FontSize',11,'FontWeight','bold');

%----------------------------------------------------------------
%  FIG 4 — 6-panel time-series metrics
%----------------------------------------------------------------
figure('Name','Mesh Metrics Over Time','NumberTitle','off',...
    'Position',[10 40 1100 720],'Color',[0.08 0.08 0.12]);
tl4 = tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

darkAx = @(ax) set(ax,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
    'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
plot(ax,tVec,meanThr,'Color',[0.2 0.8 1.0],'LineWidth',1.4);
plot(ax,tVec,selectedThr,'c-','LineWidth',2.3);
plot(ax,tVec,bestVal(:,1),'y--','LineWidth',1.2);
legend(ax,{'Mean Network','Selected Route','Best Possible'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
ylabel(ax,'Mbps','Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,'Throughput (Mbps)','Color','w','FontWeight','bold'); xlim(ax,[0 T*dt]);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
plot(ax,tVec,meanLat,'r-','LineWidth',1.4);
plot(ax,tVec,selectedLat,'c-','LineWidth',2.3);
plot(ax,tVec,bestVal(:,2),'y--','LineWidth',1.2);
legend(ax,{'Mean Network','Selected Route','Best Possible Min Lat'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northeast');
ylabel(ax,'\mus','Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,'Latency (\mus)','Color','w','FontWeight','bold'); xlim(ax,[0 T*dt]);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
plot(ax,tVec,meanSNR,'m-','LineWidth',1.5);
plot(ax,tVec,selectedSNR,'c-','LineWidth',2.5);
plot(ax,tVec,bestVal(:,3),'y--','LineWidth',1.2);
yline(ax,10,'--','Color',[1 0.6 0],'LineWidth',1.2,'Label','10 dB');
legend(ax,{'Mean Network SNR','Selected Route SNR','Best Possible SNR'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
ylabel(ax,'dB','Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,'Bottleneck SNR (dB)','Color','w','FontWeight','bold'); xlim(ax,[0 T*dt]);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
patch(ax,[tVec fliplr(tVec)],[meanPDR' zeros(1,T)],[0.2 0.7 0.4],'FaceAlpha',0.15,'EdgeColor','none');
plot(ax,tVec,meanPDR,'Color',[0.2 0.8 0.2],'LineWidth',1.5);
plot(ax,tVec,selectedPDR,'Color',[0 1 1],'LineWidth',2.5);
plot(ax,tVec,bestVal(:,4),'y--','LineWidth',1.2);
legend(ax,{'Network Fill','Mean Network PDR','Selected Route PDR',...
    'Best Possible PDR'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','southwest');
ylabel(ax,'PDR','Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,'Packet Delivery Ratio','Color','w','FontWeight','bold');
ylim(ax,[0 1.05]); xlim(ax,[0 T*dt]);

sgtitle(tl4,sprintf('RF %d-Node: Performance Metrics Over Time',N),...
    'Color','w','FontSize',12,'FontWeight','bold');

%% Selected-route PDR versus bottleneck SNR

validSelected = ~isnan(selectedSNR) & ~isnan(selectedPDR);
figure('Name','Selected Route PDR versus SNR','NumberTitle','off');
scatter(selectedSNR(validSelected),selectedPDR(validSelected),45,...
    tVec(validSelected),'filled');
hold on;
xline(snrFailThreshold_dB,'r--','10 dB routing threshold','LineWidth',1.5);

grid on;
xlabel('Selected Route Bottleneck SNR (dB)');
ylabel('Selected Route End-to-End PDR');
title('Selected Route PDR versus Bottleneck SNR');
ylim([0 1.02]);
cb = colorbar;
cb.Label.String = 'Simulation Time (s)';

%% ==================== SUMMARY REPORT ===================================
fprintf("\n==========================================\n");
fprintf("         RF MESH SIMULATION SUMMARY\n");
fprintf("==========================================\n");
fprintf("Steps=%d | Sim time=%.1f s\n",T,T*dt);
fprintf("Mean active SNR-valid links: %.1f\n",mean(activeLinks,'omitnan'));
fprintf("Mean network throughput:    %.2f Mbps\n",mean(meanThr,'omitnan'));
fprintf("Mean selected throughput:   %.2f Mbps\n",mean(selectedThr,'omitnan'));
fprintf("Mean network latency:       %.2f us\n",mean(meanLat,'omitnan'));
fprintf("Mean selected latency:      %.2f us\n",mean(selectedLat,'omitnan'));
fprintf("Mean network SNR:           %.2f dB\n",mean(meanSNR,'omitnan'));
fprintf("Mean selected SNR:          %.2f dB\n",mean(selectedSNR,'omitnan'));
fprintf("Mean network PDR:           %.4f\n",mean(meanPDR,'omitnan'));
fprintf("Mean selected PDR:          %.4f\n",mean(selectedPDR,'omitnan'));
fprintf("Mean radio range:   %.2f m\n",mean(rangeR_t,'omitnan'));

fprintf("\n--- Best routes (last valid step) ---\n");
lastT = find(~isnan(bestVal(:,1)),1,'last');
if ~isempty(lastT)
    fprintf("Best Throughput: %s\n  -> %.2f Mbps\n",bestRouteStr(lastT,1),bestVal(lastT,1));
    fprintf("Best Latency:    %s\n  -> %.2f us\n",bestRouteStr(lastT,2),bestVal(lastT,2));
    fprintf("Best SNR:        %s\n  -> %.2f dB\n",bestRouteStr(lastT,3),bestVal(lastT,3));
    fprintf("Best PDR:        %s\n  -> %.4f\n",bestRouteStr(lastT,4),bestVal(lastT,4));
end
fprintf("\n--- PDR<1 scenario explanation ---\n");
fprintf("PDR can drop below 1 when links are near the 10 dB SNR threshold, when routes use many hops, or when nodes have many neighbors causing contention.\n");
fprintf(['Links with SNR below %.1f dB are excluded ',...
         'from routing; nodes remain physically active.\n'],...
         snrFailThreshold_dB);
fprintf("==========================================\n");

end % function

%% =========================================================================
%%  LOCAL HELPER FUNCTIONS
%% =========================================================================

function [B_Hz,f_Hz,Tx_dBm,NF_dB,rxSensitivity_dBm,phyDataRate_bps,perHopAirtime_s,dutyCycle] = ...
    configureRadioMode(radioMode,loraSF,loraBW_Hz,loraCR,loraPreamble,dutyCycle)
% Configure the simulation according to the selected radio on the board.
packet_B_default = 127;
NF_dB = 6;
switch radioMode
    case "LoRa"
        B_Hz = loraBW_Hz;
        f_Hz = 915e6;
        Tx_dBm = 10; 
    switch loraSF
        case 7
            rxSensitivity_dBm = -124;
        case 10
            rxSensitivity_dBm = -133;
        case 12
            rxSensitivity_dBm = -137;
        otherwise
            error('Unsupported LoRa spreading factor');
    end
        phyDataRate_bps = loraBitrate(loraSF,loraBW_Hz,loraCR);
        perHopAirtime_s = loraTimeOnAir(packet_B_default,loraSF,loraBW_Hz,loraCR,loraPreamble);
    case "WiFi"
        B_Hz = 20e6;
        f_Hz = 2.4e9;
        Tx_dBm = 18;
        rxSensitivity_dBm = -90;
        phyDataRate_bps = 150e6;
        perHopAirtime_s = (packet_B_default*8)/phyDataRate_bps + 300e-6;
        dutyCycle = 1.0;
    case "BLEMesh"
        B_Hz = 2e6;
        f_Hz = 2.4e9;
        Tx_dBm = 8;
        rxSensitivity_dBm = -96;
        phyDataRate_bps = 1e6;
        perHopAirtime_s = (packet_B_default*8)/phyDataRate_bps + 2e-3;
        dutyCycle = 1.0;
    case "Custom"
        B_Hz = 4e6;
        f_Hz = 915e6;
        Tx_dBm = 0;
        rxSensitivity_dBm = -105;
        phyDataRate_bps = 4e6;
        perHopAirtime_s = (packet_B_default*8)/phyDataRate_bps + 0.5e-3;
        dutyCycle = 1.0;
    otherwise
        error('Unknown radioMode.');
end
end

function rbps = loraBitrate(SF,BW,CR)
% Approximate LoRa PHY bitrate. CR=1 corresponds to coding rate 4/5.
rbps = SF * (BW / 2^SF) * (4/(4+CR));
end

function toa_s = loraTimeOnAir(payloadBytes,SF,BW,CR,preambleSymbols)
% LoRa explicit-header Time-on-Air approximation.
% CR=1 means 4/5. CRC enabled. Low-data-rate optimization enabled for SF>=11.
CRC = 1; IH = 0; DE = double(SF >= 11);
Tsym = 2^SF / BW;
Tpreamble = (preambleSymbols + 4.25) * Tsym;
payloadSymbNb = 8 + max(ceil((8*payloadBytes - 4*SF + 28 + 16*CRC - 20*IH) / ...
                     (4*(SF - 2*DE))) * (CR + 4), 0);
Tpayload = payloadSymbNb * Tsym;
toa_s = Tpreamble + Tpayload;
end

function [pos,wp,spd] = moveRWP(pos,wp,spd,W,H,vMin,vMax,dt,alive)
N=size(pos,1);
for i=1:N
    if ~alive(i), continue; end
    dir=wp(i,:)-pos(i,:); d=norm(dir);
    if d<1e-6, wp(i,:)=[W*rand H*rand]; spd(i)=vMin+(vMax-vMin)*rand; continue; end
    pos(i,:)=pos(i,:)+(dir/d)*min(d,spd(i)*dt);
    if norm(wp(i,:)-pos(i,:))<0.3
        wp(i,:)=[W*rand H*rand]; spd(i)=vMin+(vMax-vMin)*rand;
    end
end
end



function [A,link] = buildLoRaRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB)
% Build graph using the selected radio link budget.
% A link exists if received power is above the receiver sensitivity.
N=size(pos,1);
A=zeros(N);
rx_dBm  = nan(N);
snr_dB  = nan(N);
snrLin  = nan(N);
cap_bps = nan(N);
noise_dBm = -174 + 10*log10(B_Hz) + NF_dB; %receiver's noise baseline

for i = 1:N
    if ~alive(i)
        continue;
    end

    for j = i+1:N
        if ~alive(j)
            continue;
        end

        d = max(hypot(pos(i,1)-pos(j,1),pos(i,2)-pos(j,2)),0.1); %distance between Nodes i,j

        % One received-power realization per link and time step
        rx = calcRxPowerLoRa(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB); %final received power
        %snr calculations in db and linear
        sdb  = rx - noise_dBm; 
        slin = 10^(sdb/10);

        shannonCap = B_Hz*log2(1+slin); %bits per sec
        cap = min(shannonCap,phyDataRate_bps); %data-rate limitation

        rx_dBm(i,j) = rx;
        rx_dBm(j,i) = rx;

        snr_dB(i,j) = sdb;
        snr_dB(j,i) = sdb;

        snrLin(i,j) = slin;
        snrLin(j,i) = slin;

        cap_bps(i,j) = cap;
        cap_bps(j,i) = cap;

        % Initial physical-link condition
        if rx >= rxSensitivity_dBm
            A(i,j) = 1;
            A(j,i) = 1;
        end
    end
end

link.A = A;
link.rx_dBm = rx_dBm;
link.snr_dB = snr_dB;
link.snrLin = snrLin;
link.cap_bps = cap_bps;
link.rxSensitivity_dBm = rxSensitivity_dBm;

end


function rx_dBm = calcRxPowerLoRa(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB)
% Log-distance path loss model. PL exponent 2.7 is an indoor/urban assumption.
PL_exp=4; 
PL0_dB=20*log10(4*pi*1/3e8*f_Hz); %reference path loss at 1 meter.
fastFade =  0.8*randn;
shadow = shadowStd_dB*randn; %shadowing component.Uses a normal random distribution.Simulates signal blocking by moving obstacles or buildings
pl=PL0_dB+10*PL_exp*log10(d)+extraLoss_dB+shadow; %Total path loss calculation.Combines distance loss, extra static losses, and random fading.
rx_dBm=Tx_dBm-pl+ fastFade; %final recived power.
end

function R = estimateLoRaRange(Tx_dBm,rxSensitivity_dBm,f_Hz,extraLoss_dB)
% Approximate max range using sensitivity and no random shadowing.
PL_exp=2.7;
PL0_dB=20*log10(4*pi*1/3e8*f_Hz);
maxPL = Tx_dBm - rxSensitivity_dBm - extraLoss_dB;
R = 10^((maxPL-PL0_dB)/(10*PL_exp));
end

function pc=allSimplePaths(A,src,dst,maxH,maxE)
N=size(A,1); vis=false(1,N);
stk=src; vis(src)=true; pc={}; cnt=0;
dfs(src);
    function dfs(u)
        if cnt>=maxE, return; end
        if u==dst, cnt=cnt+1; pc{end+1,1}=stk; return; end
        if numel(stk)-1>=maxH, return; end
        for v=find(A(u,:))
            if ~vis(v)
                vis(v)=true; stk(end+1)=v; %#ok<AGROW>
                dfs(v); stk(end)=[]; vis(v)=false;
                if cnt>=maxE, return; end
            end
        end
    end
end

function results=pathMetrics(pc,link,B_Hz,macEff,pktB,pos,perHopAirtime_s,dutyCycle, radioMode,loraSF)
% Path metrics for the selected radio mode.
% Throughput is based on bottleneck PHY capacity, duty cycle, MAC efficiency and route-level PDR.
% PDR can be < 1 because every hop may loשלנאse packets due to weak SNR,
% longer hop distance, multi-hop accumulation, and local contention.
np=numel(pc); c=3e8;
rt=strings(np,1); hp=zeros(np,1);
thr=zeros(np,1); lat=zeros(np,1);
bSNR=zeros(np,1); pdr=zeros(np,1); bCap=zeros(np,1);
sampleSNR = [];
sampleSNRonlyPDR = [];
sampleFinalPDR   = [];
for k=1:np
    p=pc{k}; H=numel(p)-1; hp(k)=H;
    hC=zeros(H,1); hS=zeros(H,1); hP=zeros(H,1); hL=zeros(H,1);
    for i=1:H
        u=p(i); v=p(i+1);
        d=max(hypot(pos(u,1)-pos(v,1),pos(u,2)-pos(v,2)),0.1);

        % Realistic per-hop delay. For LoRa this is packet Time-on-Air;
        % for Wi-Fi/BLE this is a small packet-airtime/processing delay.
        propDelay_s = d / c;   % propagation delay
        if radioMode == "LoRa"
            txDelay_s = perHopAirtime_s;
        else
            txDelay_s = (pktB*8) / link.cap_bps(u,v);
        end
        procDelay_s = 0.5e-3;  % fixed 0.5 ms processing delay per hop
        nodeDegree = sum(link.A(u,:));
        queueDelay_s = 0.1e-3 * max(0,nodeDegree-5); % congestion delay. No queueing penalty up to 5 neighboring nodes. Every additional neighbor adds 0.1 ms of queueing delay.

        hL(i) = propDelay_s + txDelay_s + procDelay_s + queueDelay_s; %hop latency

        % Capacity and SNR are physics-based.
        hC(i)=link.cap_bps(u,v); 
        hS(i)=link.snr_dB(u,v);

        % Packet-success model that allows PDR < 1.
        % Links with SNR below the routing threshold are excluded.
        % Links close to the threshold may still experience packet errors.
        snrDB = hS(i);
        % At 8 dB, packet success is 50%.
        % At 10 dB, packet success is approximately 67%.
        if radioMode == "LoRa"

            switch loraSF
                case 7
                    snrMid_dB = -5.5;
                case 10
                    snrMid_dB = -8;
                case 12
                    snrMid_dB = -12;
            end

            snrSlope = 0.5;

        else
            snrMid_dB = 8;
            snrSlope = 0.35;
        end
        
        % PDR caused only by the physical SNR
        hopSucc = 1 ./ (1 + exp(-snrSlope*(snrDB-snrMid_dB)));
        hopSucc = max(0.001,min(0.9999,hopSucc)); % Limit probability to a valid numerical range

        % Distance penalty: longer hops are more fragile.
        %REMOVED:distPenalty = 0.0002*max(0,d-10); %For every meter beyond 10 meters,packet success probability drops by 0.02%

        % Additional MAC-layer congestion penalty: many neighbors around the transmitting
        % node increase collisions and medium access delay.
        % assumes the channel can handle up to 15 neighboring nodes before collisions become an issue.
        %For every neighbor above 15, a 0.02% penalty is deducted
        nodeDegree = sum(link.A(u,:));
        degreePenalty = 0.0002*max(0,nodeDegree-15);
        
        % Final per-hop PDR used by the simulation
        hP(i)=max(0.001, min(0.9999, hopSucc - degreePenalty));

        % Save samples for post-run analysis
        sampleSNR(end+1,1)        = snrDB;
        sampleSNRonlyPDR(end+1,1) = hopSucc;
        sampleFinalPDR(end+1,1)   = hP(i);
    end
    bc=min(hC); bCap(k)=bc/1e6;
    bSNR(k)=min(hS);

    % Route-level PDR is the mult product of hop success probabilities.
    % This naturally creates scenarios with PDR < 1, especially for weak or
    % multi-hop routes.
    pdr(k)=prod(hP);

    % Optional retransmission effect: lower PDR increases expected latency.
    expectedRetries = 1/max(pdr(k),0.05);
    lat(k)=sum(hL)*1e6*expectedRetries;
    thr(k)=(dutyCycle*macEff*bc*pdr(k))/1e6;  % effective goodput includes duty cycle and delivery success
    rt(k)=strjoin("N"+string(p),"->"); 
end
T2=table(rt,hp,thr,lat,bSNR,pdr,bCap,...
    'VariableNames',["Route","Hops","Throughput_Mbps","Latency_us",...
                     "BottleneckSNR_dB","PDR","BottleneckCapacity_Mbps"]);
results.table=T2;
results.linkSNR        = sampleSNR;
results.snrOnlyHopPDR  = sampleSNRonlyPDR;
results.finalHopPDR    = sampleFinalPDR;
end

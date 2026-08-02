function esp32_sx1262_mesh_sim_improved()
% =========================================================================
%  ESP32-S3FN8 + SX1262 Multi-Mode 100-Node Mobile Mesh Simulation
%  Area: 20 m x 20 m  |  LoRa / Wi-Fi / BLE Mesh selectable PHY model
%
%  VISUALIZATIONS:
%   Fig 1  - Live animated topology (updates every step):
%              * Edge colors = SNR quality (red->yellow->green)
%              * Node trails showing recent movement
%              * Best throughput route highlighted in gold
%              * Failed nodes marked with red X
%              * SNR colorbar
%   Fig 2  - Live 4-panel dashboard (updates every step):
%              * Mean & best throughput vs time
%              * PDR area fill vs time
%              * Alive nodes area chart
%              * SNR + radio range dual-axis
%   Fig 3  - 10-frame topology montage (post-run)
%   Fig 4  - 6-panel time-series metric plots (post-run)
%
%  Mobility:   Random Waypoint
%  Failures:   SNR-threshold based only: node fails when strongest link SNR < 10 dB
%  PHY:        selectable LoRa / Wi-Fi / BLE Mesh model. LoRa uses SF, duty cycle and Time-on-Air.
% =========================================================================
clc; close all;
rng(42);

%% ==================== CONFIGURABLE PARAMETERS ==========================
N   = 100;   src = 1;   dst = 100;

areaW = 30;  areaH = 200;      % 20 x 20 m area

% Mobility
dt   = 0.5;  T  = 60;     % 1 ms step, total sim time = 30 s CHANGED
vMin = 0.3;   vMax = 1.5;     % m/s

% Hardware / PHY mode selection: ESP32-S3FN8 + SX1262 board
% Supported board radios:
%   1) LoRa SX1262: 470-510 MHz / 863-928 MHz, Tx up to 21 dBm, sensitivity down to -139 dBm
%   2) Wi-Fi 802.11 b/g/n: up to 150 Mbps, short-range high-rate mode
%   3) Bluetooth LE / Bluetooth Mesh: low-rate short-range mesh mode
radioMode = "Custom";          % choose: "Custom" , "LoRa", "WiFi", or "BLEMesh"

% LoRa scenario parameters. These are used only when radioMode = "LoRa".
loraSF = 7;                   % choose 7, 10, or 12 to show range/latency tradeoff
loraBW_Hz = 125e3;            % common LoRa bandwidth: 125 kHz
loraCR = 1;                   % coding rate index: 1 means 4/5
loraPreamble = 8;             % LoRa preamble symbols
dutyCycle = 0.01;             % 1% duty-cycle limitation, important for LoRa

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
        extraLoss_dB = 0;
        shadowStd_dB = 4;
    case "SNR_FAILURE_DEMO"
        extraLoss_dB = 40;      
        shadowStd_dB = 8;
    otherwise
        extraLoss_dB = 0;
        shadowStd_dB = 4;
end

% MAC
packet_B = 127;               % bytes
macEff   = 0.70;                % MAC efficiency / duty-cycle / overhead factor

% Failure model
snrFailThreshold_dB = 10;      % node fails only when strongest available link SNR < 10 dB

% Path search
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
snapR     = nan(numSnapshots,1);
snapIdx   = 0;

% Time-series storage
bestVal      = nan(T,5);
bestRouteStr = strings(T,5);
bestPath_t   = cell(T,1);

meanThr    = nan(T,1);  meanLat  = nan(T,1);
meanSNR    = nan(T,1);  meanPDR  = nan(T,1);
aliveCount = nan(T,1);  numPaths_t = nan(T,1);
rangeR_t   = nan(T,1); selectedPDR = nan(T,1);
selectedSNR = nan(T,1);

% Collected across all steps for histograms/CDF
allHops = [];  allPDR = [];

% Failure log  T x N
failLog = false(T,N);

fprintf("=== ESP32-S3FN8 + SX1262 Multi-Mode Mesh Simulation ===\n");
fprintf("Area: %.0fm x %.0fm | N=%d | Steps=%d | dt=%.4fs\n",areaW,areaH,N,T,dt);
fprintf("Mode=%s | BW=%.0f MHz | f=%.0f MHz | Tx=%.0f dBm | NF=%.0f dB\n",radioMode,B_Hz/1e3,f_Hz/1e6,Tx_dBm,NF_dB);
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

    %-- 2. Provisional radio graph + SNR-based node failures -----------
    % Links are created according to the SX1262 link budget: received power
    % must be above -139 dBm sensitivity. SNR is then used ONLY for node failure.
    provisionalAlive = true(N,1);
    [A0, link0] = buildLoRaRadioGraph(pos,provisionalAlive,B_Hz,Tx_dBm,...
        NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);
    % Node failure rule: a node fails only if its strongest available SNR < 10 dB.
    % src/dst are forced alive so the route search always has endpoints.
    alive = true(N,1);
    aliveCount(t) = sum(alive);
    failLog(t,:)  = ~alive';

    %-- 3. Build final graph after SNR-based failures ------------------
    [A, link] = buildLoRaRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB, ...
        f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);
    % Remove links whose SNR is below threshold
    badLinks = link.snr_dB < snrFailThreshold_dB;
    A(badLinks) = 0;
    A = double(A | A');   % keep adjacency symmetric

    % Recalculate link metrics after removing bad links
    link = linkMetricsRF(A,pos,B_Hz,Tx_dBm,NF_dB,f_Hz,...
        rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);

    G = graph(A);
    
    activeLinks(t) = nnz(triu(A));

    % Display/evaluation radio range estimate based on sensitivity.
    R = estimateLoRaRange(Tx_dBm,rxSensitivity_dBm,f_Hz,extraLoss_dB);

    %-- 5. Snapshot capture ------------------------------------------
    if any(t==snapSteps)
        snapIdx           = snapIdx+1;
        snapPos{snapIdx}  = pos;
        snapA{snapIdx}    = A;
        snapAlive{snapIdx}= alive;
        snapR(snapIdx)    = R;
    end

    %-- 6. Reachability + path search --------------------------------
    bins      = conncomp(G);
    reachable = (bins(src)==bins(dst));
    bestPathNow = [];

    if reachable
        paths = allSimplePaths(A,src,dst,maxHops,maxEnumPaths);
        numPaths_t(t) = numel(paths);

        if ~isempty(paths)
            results = pathMetrics(paths,link,B_Hz,macEff,packet_B,pos,perHopAirtime_s,dutyCycle);
            Ttbl    = results.table;

            meanThr(t) = mean(Ttbl.Throughput_Mbps);
            meanLat(t) = mean(Ttbl.Latency_us);
            meanSNR(t) = mean(Ttbl.BottleneckSNR_dB);
            meanPDR(t) = mean(Ttbl.PDR);

            [bestVal(t,1),iThr] = max(Ttbl.Throughput_Mbps);
            [bestVal(t,2),iLat] = min(Ttbl.Latency_us);
            [bestVal(t,3),iSNR] = max(Ttbl.BottleneckSNR_dB);
            [bestVal(t,4),iPDR] = max(Ttbl.PDR);
            [bestVal(t,5),iCap] = max(Ttbl.BottleneckCapacity_Mbps);

            selectedPDR(t) = Ttbl.PDR(iThr); %PDR of selected path with best througput
            selectedSNR(t) = Ttbl.BottleneckSNR_dB(iThr); %SNR of selected path with best througput

            bestRouteStr(t,1) = Ttbl.Route(iThr);
            bestRouteStr(t,2) = Ttbl.Route(iLat);
            bestRouteStr(t,3) = Ttbl.Route(iSNR);
            bestRouteStr(t,4) = Ttbl.Route(iPDR);
            bestRouteStr(t,5) = Ttbl.Route(iCap);

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
    text(ax1,1.65,areaH-0.15,'● Alive','Color',[0.35 0.65 1],'FontSize',7);
    text(ax1,2.65,areaH-0.15,'× Failed','Color',[1 0.3 0.3],'FontSize',7);
    text(ax1,3.65,areaH-0.15,'— Best path','Color',[1 0.85 0],'FontSize',7);

   
    axis(ax1,[0 areaW 0 areaH]); grid(ax1,'on');
    ax1.GridColor=[0.25 0.25 0.25]; ax1.GridAlpha=0.5;

    if reachable && ~isnan(bestVal(t,1))
        ttl = sprintf("t=%.1f s | R=%.2f m | alive=%d | paths=%d | best Thr=%.1f Mbps",...
            t*dt,R,aliveCount(t),numPaths_t(t),bestVal(t,1));
    else
        ttl = sprintf("t=%.1f s | R=%.2f m | alive=%d | *** NO PATH src->dst ***",...
            t*dt,R,aliveCount(t));
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
    plot(axA,tV,meanThr(1:t),'c-','LineWidth',1.8,'DisplayName','Mean');
    plot(axA,tV,bestVal(1:t,1),'y--','LineWidth',1.2,'DisplayName','Best');
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
    plot(axB,tV,selectedPDR(1:t),'g-','LineWidth',1.8);
    plot(axB,tV,bestVal(1:t,4),'y--','LineWidth',1.0);
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
    ylabel(axD2,'BottleNeck SNR (dB)','Color','w');
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

%% =========================================================================
%%  POST-RUN FIGURES
%% =========================================================================

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
    link_k  = linkMetricsRF(A_k,pos_k,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);

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
    title(axM,sprintf("t=%.1fs  R=%.2fm  alive=%d",...
        snapSteps(k)*dt,snapR(k),sum(alive_k)),...
        'Color','w','FontSize',8);
end
sgtitle(tl3,'RF 100-Node: Topology Snapshots  (green=SRC  magenta=DST  red=failed)',...
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
plot(ax,tVec,meanThr,'c-','LineWidth',1.8);
plot(ax,tVec,bestVal(:,1),'y--','LineWidth',1.2);
legend(ax,{'Mean','Best'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
ylabel(ax,'Mbps','Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,'Throughput (Mbps)','Color','w','FontWeight','bold'); xlim(ax,[0 T*dt]);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
plot(ax,tVec,meanLat,'r-','LineWidth',1.8);
plot(ax,tVec,bestVal(:,2),'y--','LineWidth',1.2);
legend(ax,{'Mean','Best (min lat)'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northeast');
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

sgtitle(tl4,'RF 100-Node: Performance Metrics Over Time',...
    'Color','w','FontSize',12,'FontWeight','bold');

%% ==================== SUMMARY REPORT ===================================
fprintf("\n==========================================\n");
fprintf("         RF MESH SIMULATION SUMMARY\n");
fprintf("==========================================\n");
fprintf("Steps=%d | Sim time=%.1f s\n",T,T*dt);
fprintf("Mean alive nodes:   %.1f / %d\n",mean(aliveCount,'omitnan'),N);
fprintf("Mean throughput:    %.2f Mbps\n",mean(meanThr,'omitnan'));
fprintf("Mean latency:       %.2f us\n",mean(meanLat,'omitnan'));
fprintf("Mean SNR:           %.2f dB\n",mean(meanSNR,'omitnan'));
fprintf("Mean PDR:           %.4f\n",mean(meanPDR,'omitnan'));
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
fprintf("Nodes fail only when their strongest available link is below %.1f dB; otherwise they stay alive but may still lose packets.\n",snrFailThreshold_dB);
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
        Tx_dBm = 21; 
        rxSensitivity_dBm = -139;
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

function R=findMinConnectedRange(posA)
if size(posA,1)<=1, R=0; return; end
n=size(posA,1); Dmax=0;
for i=1:n, for j=i+1:n
    Dmax=max(Dmax,hypot(posA(i,1)-posA(j,1),posA(i,2)-posA(j,2)));
end; end
lo=0; hi=Dmax;
for iter = 1:50
    if hi-lo<1e-6, break; end
    mid=(lo+hi)/2;
    A=adjSimple(posA,mid); G=graph(A);
    if numel(unique(conncomp(G)))==1, hi=mid; else, lo=mid; end
end
R=hi;
end

function A=adjSimple(pos,R)
n=size(pos,1); A=zeros(n);
for i=1:n, for j=i+1:n
    if hypot(pos(i,1)-pos(j,1),pos(i,2)-pos(j,2))<=R
        A(i,j)=1; A(j,i)=1;
    end
end; end
end

function A=buildAdjAlive(pos,alive,R)
N=size(pos,1); A=zeros(N);
for i=1:N
    if ~alive(i), continue; end
    for j=i+1:N
        if ~alive(j), continue; end
        if hypot(pos(i,1)-pos(j,1),pos(i,2)-pos(j,2))<=R
            A(i,j)=1; A(j,i)=1;
        end
    end
end
end

function [A,link] = buildLoRaRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB)
% Build graph using the selected radio link budget.
% A link exists if received power is above the receiver sensitivity.
N=size(pos,1);
A=zeros(N);
for i=1:N
    if ~alive(i), continue; end
    for j=i+1:N
        if ~alive(j), continue; end
        d=max(hypot(pos(i,1)-pos(j,1),pos(i,2)-pos(j,2)),0.1);
        rx = calcRxPowerLoRa(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB);
        if rx >= rxSensitivity_dBm
            A(i,j)=1; A(j,i)=1;
        end
    end
end
link = linkMetricsRF(A,pos,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB);
end

function link=linkMetricsRF(A,pos,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB)
N=size(A,1);
snrLin=nan(N); snr_dB=nan(N); cap_bps=nan(N); rx_dBm=nan(N);
noise_dBm=-174+10*log10(B_Hz)+NF_dB;
for i=1:N, for j=i+1:N
    if A(i,j)==1
        d=max(hypot(pos(i,1)-pos(j,1),pos(i,2)-pos(j,2)),0.1);
        rx=calcRxPowerLoRa(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB);
        sdb=rx-noise_dBm; slin=10^(sdb/10);
        shannonCap=B_Hz*log2(1+slin);
        cap=min(shannonCap,phyDataRate_bps);  % LoRa data-rate limitation
        rx_dBm(i,j)=rx; rx_dBm(j,i)=rx;
        snr_dB(i,j)=sdb; snr_dB(j,i)=sdb;
        snrLin(i,j)=slin; snrLin(j,i)=slin;
        cap_bps(i,j)=cap; cap_bps(j,i)=cap;
    end
end; end
link.A=A; link.rx_dBm=rx_dBm; link.snrLin=snrLin; link.snr_dB=snr_dB; link.cap_bps=cap_bps;
link.rxSensitivity_dBm = rxSensitivity_dBm;
end

function rx_dBm = calcRxPowerLoRa(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB)
% Log-distance path loss model. PL exponent 2.7 is an indoor/urban assumption.
PL_exp=2.7;
PL0_dB=20*log10(4*pi*1/3e8*f_Hz);
shadow = shadowStd_dB*randn;
pl=PL0_dB+10*PL_exp*log10(d)+extraLoss_dB+shadow;
rx_dBm=Tx_dBm-pl;
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
        if u==dst, cnt=cnt+1; pc{end+1,1}=stk; return; end %#ok<AGROW>
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

function results=pathMetrics(pc,link,B_Hz,macEff,pktB,pos,perHopAirtime_s,dutyCycle)
% Path metrics for the selected radio mode.
% Throughput is based on bottleneck PHY capacity, duty cycle, MAC efficiency and route-level PDR.
% PDR can be < 1 because every hop may lose packets due to weak SNR,
% longer hop distance, multi-hop accumulation, and local contention.
np=numel(pc); c=3e8;
rt=strings(np,1); hp=zeros(np,1);
thr=zeros(np,1); lat=zeros(np,1);
bSNR=zeros(np,1); pdr=zeros(np,1); bCap=zeros(np,1);
for k=1:np
    p=pc{k}; H=numel(p)-1; hp(k)=H;
    hC=zeros(H,1); hS=zeros(H,1); hP=zeros(H,1); hL=zeros(H,1);
    for i=1:H
        u=p(i); v=p(i+1);
        d=max(hypot(pos(u,1)-pos(v,1),pos(u,2)-pos(v,2)),0.1);

        % Realistic per-hop delay. For LoRa this is packet Time-on-Air;
        % for Wi-Fi/BLE this is a small packet-airtime/processing delay.
        hL(i)=perHopAirtime_s + d/c;

        % Capacity and SNR are physics-based.
        hC(i)=link.cap_bps(u,v); 
        hS(i)=link.snr_dB(u,v);

        % Packet-success model that allows PDR < 1.
        % Nodes only fail below 10 dB, but links near the threshold can still
        % deliver packets with errors, so PDR decreases before node failure.
        snrDB = hS(i);
        if snrDB >= 20
            hopSucc = 0.995;
        elseif snrDB >= 15
            hopSucc = 0.985;
        elseif snrDB >= 10
            hopSucc = 0.950;
        else
            hopSucc = 0.500;
        end

        % Distance penalty: longer hops are more fragile.
        distPenalty = 0.0005*max(0,d-10);

        % Congestion/contention penalty: many neighbors around the transmitting
        % node increase collisions and medium access delay.
        nodeDegree = sum(link.A(u,:));
        degreePenalty = 0.0005*max(0,nodeDegree-15);

        hP(i)=max(0.90, min(0.995, hopSucc - distPenalty - degreePenalty));
    end
    bc=min(hC); bCap(k)=bc/1e6;
    bSNR(k)=min(hS);

    % Route-level PDR is the product of hop success probabilities.
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
end

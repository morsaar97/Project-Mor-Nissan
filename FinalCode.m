function tactical_mesh_sim_selected_metrics()
% =========================================================================
%  Area: 30 m x 200 m | Custom tactical RF / LoRa / Wi-Fi / BLE selectable PHY model
%
%  VISUALIZATIONS:
%   Fig 1  - Live animated topology (updates every step):
%              * Edge colors = SNR quality (red->yellow->green)
%              * Node trails showing recent movement
%              * Best throughput route highlighted in gold
%              * Weak links with SNR < 10 dB (changable) are excluded from routing
%              * SNR colorbar
%   Fig 2  - Live 4-panel dashboard (updates every step):
%              * Mean & best throughput vs time
%              * PDR area fill vs time
%              * Active SNR-valid connections area chart
%              * SNR + radio range dual-axis
%   Fig 3  - 10-frame topology montage (post-run)
%   Fig 4  - 4-panel time-series metric plots (post-run)
%
%  Mobility:   Random Waypoint
%  Link rule:  Links with SNR < 10 (changeable) dB are excluded from the routing graph
%  PHY:        selectable LoRa / Wi-Fi / BLE Mesh model. LoRa uses SF, duty cycle and Time-on-Air.
%
%  CHANGE LOG (duty-cycle fix):
%   Previously `dutyCycle` was threaded through configureRadioMode() but every
%   mode branch reassigned it to 1.0, so the regulatory/MAC duty-cycle limit
%   never actually constrained throughput for any mode, including LoRa. This
%   version adds explicit, mode-specific duty-cycle limits (configurable
%   below) and actually applies the LoRa regulatory duty cycle. Wi-Fi and BLE
%   Mesh are not duty-cycle-regulated the same way LoRa/ISM sub-bands are, so
%   their limits default to 1.0 with contention handled instead via macEff /
%   nodeDegree penalties -- see notes at each parameter.
% =========================================================================
clc; close all;
rng(42);

%% ==================== CONFIGURABLE PARAMETERS ==========================
N   = 30;   src = 1;   dst = N;

areaW = 30;  areaH = 200;      % 30 x 200 m tactical corridor area

% Mobility
dt   = 0.5;  T  = 60;     % 0.5-second step, total simulation time = 30 seconds
vMin = 0.5;   vMax = 2;     % m/s

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

% ---- Regulatory / MAC duty-cycle limits (fraction of airtime a node may transmit) ----
% LoRa on ISM sub-bands (e.g. EU868) is frequently duty-cycle-limited by
% regulation, independent of Time-on-Air. Typical EU868 sub-band limits are
% 1% (g1) or 10% (g1-g3, LBT/AFA exempt bands) -- pick per your regulatory
% region / band plan. FCC 915 MHz operation instead uses dwell-time + hop
% rules rather than a duty-cycle percentage; treat loraDutyCycleLimit as an
% effective cap either way.
loraDutyCycleLimit   = 1.0;  % NOT REALISTIC,MOSTLY A LOT LESS
wifiDutyCycleLimit   = 1.0;   % no regulatory duty-cycle limit; contention handled via macEff instead
bleDutyCycleLimit    = 1.0;   % no regulatory duty-cycle limit; advertising duty cycle not modeled here
customDutyCycleLimit = 1.0;   % user-configurable radio: set your own limit here

snrFailThreshold_dB = 10;

if radioMode == "LoRa"

    switch loraSF
        case 7
            snrFailThreshold_dB = snrFailThreshold_dB-14; %-7.5
        case 10
            snrFailThreshold_dB = snrFailThreshold_dB-22.5; %-15
        case 12
            snrFailThreshold_dB = snrFailThreshold_dB-27.5; %-20
        otherwise
            error('Unsupported LoRa spreading factor');
    end
end

% LoRa throughput is typically well under 1 Mbps, so the live dashboard and
% post-run time-series throughput panels display it in kbps for readability;
% every other figure/table/fprintf keeps Mbps as the canonical unit.
if radioMode == "LoRa"
    thrScale = 1000;
    thrUnit  = 'kbps';
else
    thrScale = 1;
    thrUnit  = 'Mbps';
end

thresholdLabel = sprintf('%.1f dB threshold',snrFailThreshold_dB);

% MAC
packet_B = 127; % packet size in bytes
macEff   = 0.70; % MAC efficiency. we assume that the usful infornation 
% transmission time is only 70% of the total channel occupation time. 

% Configure radio parameters according to selected mode
[B_Hz,f_Hz,Tx_dBm,NF_dB,rxSensitivity_dBm,phyDataRate_bps,perHopAirtime_s,dutyCycle] = ...
    configureRadioMode(radioMode,loraSF,loraBW_Hz,loraCR,loraPreamble,packet_B, ...
    loraDutyCycleLimit,wifiDutyCycleLimit,bleDutyCycleLimit,customDutyCycleLimit);

% Scenario control
% "OPEN": open area or with few obstacle/body/urban
% "URBAN": adds obstacle/body/urban loss so some links fail
% "HARSH": extreme scenario with obstacle loss so a lot of links fail
% only when their strongest SNR is below 10 dB (changable).
% for pathLossExponent: 2.0–2.5: open outdoor environment, 
% 2.5–3.0: suburban or partly obstructed,
% 3.0–4.0: urban, indoor, or heavily obstructed.
% Extra Loss=constant attenuation value applied to all links within a given scenario.
% added to path loss. reduces recived power,snr,less links,smaller pdr and
% throughput for the whole netwotk.
% Shadow loss= random variations in received signal strength for every
% link,even if in the same distance. causes changes in snr and pdr,routing
% changes,lass stability. 
scenario = "URBAN";

switch scenario

    case "OPEN"
        extraLoss_dB = 7;
        shadowStd_dB = 3;
        pathLossExponent = 2.7; 

    case "URBAN"
        extraLoss_dB = 15;
        shadowStd_dB = 4;
        pathLossExponent = 3.2; 

    case "HARSH"
        extraLoss_dB = 25;
        shadowStd_dB = 8;
        pathLossExponent = 3.7;

    otherwise
        extraLoss_dB = 0;
        shadowStd_dB = 4;
        pathLossExponent = 2.7; 
end

% Path search restrictions
maxEnumPaths = 100; %max pathes to search
maxHops      = 7; %max num of hops in a path

%% VISUALIZATION
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
set(gcf,'Name',sprintf('%s Mesh Live Topology',radioMode),'NumberTitle','off',...
    'Position',[10 430 700 580],'Color',[0.08 0.08 0.12]);

figure(2);
set(gcf,'Name',sprintf('%s Mesh Live Dashboard',radioMode),'NumberTitle','off',...
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
    [A_all, link_all] = buildRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB, ...
    f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB,pathLossExponent);
    
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
    R = estimateRadioRange(Tx_dBm,rxSensitivity_dBm,f_Hz,extraLoss_dB,pathLossExponent);
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
            results = pathMetrics(paths,link,B_Hz,macEff,packet_B,pos,...
                perHopAirtime_s,dutyCycle,radioMode,loraSF,snrFailThreshold_dB);
            Ttbl   = results.table;
    
            % Mean values over all available routes
            meanThr(t) = mean(Ttbl.Throughput_Mbps);
            meanLat(t) = mean(Ttbl.Latency_us);
            meanSNR(t) = mean(Ttbl.BottleneckSNR_dB);
            meanPDR(t) = mean(Ttbl.PDR);

            % Best routes according to each metric
            % ============================================================
            % Select route by maximum throughput.
            % If several routes have nearly the same throughput,
            % choose the one with the minimum latency.
            % ============================================================
            % Maximum throughput among all candidate routes
            maxThr = max(Ttbl.Throughput_Mbps);
            % Allowed throughput difference from the maximum.
            % 0.005 means routes within 0.5% of the maximum throughput.
            throughputTolerance = 0.005;
            % Find routes whose throughput is practically equal to the maximum
            candidateIdx = find(Ttbl.Throughput_Mbps >= (1-throughputTolerance) * maxThr);
            % Among those routes, choose the route with minimum latency
            [~,localIdx] = min(Ttbl.Latency_us(candidateIdx));
            % Convert the local candidate index back to the table index
            iThr = candidateIdx(localIdx);

            % Store the throughput of the route that was actually selected
            bestVal(t,1) = Ttbl.Throughput_Mbps(iThr);

            % Best routes according to the other metrics
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
    set(gcf,'Name',sprintf('%s Mesh Live Topology',radioMode),...
        'NumberTitle','off','Position',[10 430 700 580],'Color',[0.08 0.08 0.12]);
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
        
        snrColorMin = snrFailThreshold_dB;
        snrColorMax = snrFailThreshold_dB + 15;
        tc=(link.snr_dB(u,v)-snrColorMin)/(snrColorMax-snrColorMin);
        tc = max(0,min(1,tc));
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
    set(gcf,'Name',sprintf('%s Mesh Live Dashboard',radioMode),...
        'NumberTitle','off','Position',[720 430 680 560],'Color',[0.08 0.08 0.12]);
    tV = (1:t)*dt;

    axD = @(r,c) subplot(2,2,(r-1)*2+c,'Parent',gcf);

    % Panel A: Throughput
    axA = axD(1,1);
    set(axA,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
        'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);
    hold(axA,'on'); grid(axA,'on');
    plot(axA,tV,thrScale*meanThr(1:t),'Color',[0.2 0.8 1.0],'LineWidth',1.4,'DisplayName','Mean Network');
    plot(axA,tV,thrScale*selectedThr(1:t),'c-','LineWidth',2.2,'DisplayName','Selected Route');
    plot(axA,tV,thrScale*bestVal(1:t,1),'y--','LineWidth',1.2,'DisplayName','Best Possible');
    legend(axA,'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',7,'Location','northwest');
    ylabel(axA,thrUnit,'Color','w');
    title(axA,['Throughput (' thrUnit ')'],'Color','w','FontWeight','bold');
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
    'LineWidth',1.2,'Label',thresholdLabel);
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
        snrColorMin = snrFailThreshold_dB;
        snrColorMax = snrFailThreshold_dB + 15;
        tc = (link_k.snr_dB(u,v)-snrColorMin)/(snrColorMax-snrColorMin);
        tc = max(0,min(1,tc));
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
%  FIG 4 — 4-panel time-series metrics
%----------------------------------------------------------------
figure('Name','Mesh Metrics Over Time','NumberTitle','off',...
    'Position',[10 40 1100 720],'Color',[0.08 0.08 0.12]);
tl4 = tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

darkAx = @(ax) set(ax,'Color',[0.12 0.12 0.18],'XColor','w','YColor','w',...
    'GridColor',[0.3 0.3 0.3],'GridAlpha',0.5);

ax=nexttile(tl4); darkAx(ax); hold(ax,'on'); grid(ax,'on');
plot(ax,tVec,thrScale*meanThr,'Color',[0.2 0.8 1.0],'LineWidth',1.4);
plot(ax,tVec,thrScale*selectedThr,'c-','LineWidth',2.3);
plot(ax,tVec,thrScale*bestVal(:,1),'y--','LineWidth',1.2);
legend(ax,{'Mean Network','Selected Route','Best Possible'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
ylabel(ax,thrUnit,'Color','w'); xlabel(ax,'Time (s)','Color','w');
title(ax,['Throughput (' thrUnit ')'],'Color','w','FontWeight','bold'); xlim(ax,[0 T*dt]);

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
yline(ax,snrFailThreshold_dB,'--','Color',[1 0.6 0],'LineWidth',1.2,'Label',thresholdLabel);
legend(ax,{'Mean Network SNR','Selected Route SNR','Best Candidate-Route SNR'},'TextColor','w','Color',[0.15 0.15 0.2],'FontSize',8,'Location','northwest');
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
fprintf("Applied duty cycle:  %.2f%%\n",100*dutyCycle);

fprintf("\n--- Best routes (last valid step) ---\n");
lastT = find(~isnan(bestVal(:,1)),1,'last');
if ~isempty(lastT)
    fprintf("Best Throughput: %s\n  -> %.2f Mbps\n",bestRouteStr(lastT,1),bestVal(lastT,1));
    fprintf("Best Latency:    %s\n  -> %.2f us\n",bestRouteStr(lastT,2),bestVal(lastT,2));
    fprintf("Best SNR:        %s\n  -> %.2f dB\n",bestRouteStr(lastT,3),bestVal(lastT,3));
    fprintf("Best PDR:        %s\n  -> %.4f\n",bestRouteStr(lastT,4),bestVal(lastT,4));
end
fprintf("\n--- PDR<1 scenario explanation ---\n");
fprintf(['PDR can drop below 1 when links are near the %.1f dB ','SNR threshold, when routes use many hops, or when ',...
         'contention is high.\n'],snrFailThreshold_dB);
fprintf(['Links with SNR below %.1f dB are excluded ',...
         'from routing; nodes remain physically active.\n'],...
         snrFailThreshold_dB);
fprintf("==========================================\n");

%% Save all results

folderName = sprintf('%s_N%d_TX%d_TH%d',char(radioMode),N,Tx_dBm,snrFailThreshold_dB);
resultsFolder = fullfile('C:\Users\morsa\OneDrive\שולחן העבודה\Simulation Results',folderName);

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

%% Save all figures
figs = findall(groot,'Type','figure');

for k = 1:length(figs)
    exportgraphics(figs(k),...
        fullfile(resultsFolder,sprintf('Figure_%d.png',k)),...
        'Resolution',300);
end

%% Save workspace
save(fullfile(resultsFolder,'SimulationResults.mat'));

Summary = table(...
    {char(radioMode)},...
    N,...
    Tx_dBm,...
    snrFailThreshold_dB,...
    mean(selectedThr,'omitnan'),...
    mean(selectedLat,'omitnan'),...
    mean(selectedPDR,'omitnan'),...
    mean(selectedSNR,'omitnan'),...
    mean(bestVal(:,5),'omitnan'),...
    'VariableNames',{'Technology',...
                     'N',...
                     'Tx_dBm',...
                     'Threshold_dB',...
                     'Throughput_Mbps',...
                     'Latency_us',...
                     'PDR',...
                     'BottleneckSNR_dB',...
                     'Capacity_Mbps'});
excelFile = fullfile('C:\Users\morsa\OneDrive\שולחן העבודה\Simulation Results',...
                     'AllResults.xlsx');
if ~isfile(excelFile)

    writetable(Summary,excelFile);

else

    oldTable = readtable(excelFile);

    newTable = [oldTable; Summary];

    writetable(newTable,excelFile);

end

end % function

%% =========================================================================
%%  LOCAL HELPER FUNCTIONS
%% =========================================================================

function [B_Hz,f_Hz,Tx_dBm,NF_dB,rxSensitivity_dBm,phyDataRate_bps,perHopAirtime_s,dutyCycle] = ...
    configureRadioMode(radioMode,loraSF,loraBW_Hz,loraCR,loraPreamble,packet_B, ...
    loraDutyCycleLimit,wifiDutyCycleLimit,bleDutyCycleLimit,customDutyCycleLimit)
% Configure the simulation according to the selected radio on the board.
% NOTE (duty-cycle fix): each mode now returns its OWN duty-cycle limit
% instead of silently overwriting whatever was passed in with 1.0. 
% LoRa enforces the regulatory sub-band limit; Wi-Fi/BLE Mesh/Custom are not
% duty-cycle-regulated the same way, so they default to 1.0 (no additional
% throughput derating beyond what macEff / contention already capture) but
% are still fully configurable via the top-level parameters.
NF_dB = 6; % noise that the reciver adds
switch radioMode
    case "LoRa"
        B_Hz = loraBW_Hz;
        f_Hz = 915e6;
        Tx_dBm = 10; %tx power from transmitter
    %lora uses Chirp Spread Spectrum (CSS) technology so it can identify 
    % a signal with negative SNR = good sensativity= can handle weak signal
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
        perHopAirtime_s = loraTimeOnAir(packet_B,loraSF,loraBW_Hz,loraCR,loraPreamble);
        dutyCycle = loraDutyCycleLimit;
    case "WiFi"
        B_Hz = 20e6;
        f_Hz = 2.4e9;
        Tx_dBm = 18;
        rxSensitivity_dBm = -90;
        phyDataRate_bps = 150e6;
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 300e-6;
        dutyCycle = wifiDutyCycleLimit;
    case "BLEMesh"
        B_Hz = 2e6;
        f_Hz = 2.4e9;
        Tx_dBm = 8;
        rxSensitivity_dBm = -96;
        phyDataRate_bps = 1e6;
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 2e-3;
        dutyCycle = bleDutyCycleLimit;
    case "Custom"
        B_Hz = 4e6;
        f_Hz = 915e6;
        Tx_dBm = 10;
        rxSensitivity_dBm = -105; %if rx_dBm<rxSensitivity_dBm link fails
        phyDataRate_bps = 4e6;  %phsical data rate limit that can be transmitted by the technology
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 0.5e-3;
        dutyCycle = customDutyCycleLimit;
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



function [A,link] = buildRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB,pathLossExponent)
% Build graph using the selected radio link budget.
% A link exists if received power is above the receiver sensitivity.
N=size(pos,1);
A=zeros(N);
rx_dBm  = nan(N);
snr_dB  = nan(N);
snrLin  = nan(N);
cap_bps = nan(N);
noise_dBm = -174 + 10*log10(B_Hz) + NF_dB; % overall total noise

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
        rx = calcRxPower(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB,pathLossExponent); %final received power
        %snr calculations in db and linear
        sdb  = rx - noise_dBm; % snr in dBm
        slin = 10^(sdb/10);

        shannonCap = B_Hz*log2(1+slin); % theoretic max cap, in bits per sec
        cap = min(shannonCap,phyDataRate_bps); %data-rate limitation,max cap of that can be trans

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


function rx_dBm = calcRxPower(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB,pathLossExponent)
% Log-distance path-loss model.
% pathLossExponent is defined by the selected environment.
% Prevent log10(0) and keep the model referenced to 1 meter.
d = max(d,1);
PL0_dB = 20*log10(4*pi*f_Hz/3e8); %Free-space path loss at the reference path loss at 1 meter.
fastFade_dB = 0.8*randn; % Small-scale fading.
% Large-scale shadowing caused by buildings, terrain, vehicles, etc.
shadow_dB = shadowStd_dB*randn;
% Total path loss.
pathLoss_dB=PL0_dB+10*pathLossExponent*log10(d)+extraLoss_dB+shadow_dB;
% Received power.
rx_dBm = Tx_dBm - pathLoss_dB + fastFade_dB;
end

function R = estimateRadioRange(Tx_dBm,rxSensitivity_dBm,f_Hz,extraLoss_dB,pathLossExponent)
% Approximate max range using sensitivity and no random shadowing.
PL0_dB = 20*log10(4*pi*f_Hz/3e8); % Free-space path loss at 1 meter.
% Maximum allowable propagation loss.
maxPathLoss_dB = Tx_dBm - rxSensitivity_dBm -extraLoss_dB;
% Solve the log-distance model for distance.
R = 10^((maxPathLoss_dB - PL0_dB) / (10*pathLossExponent));
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

function results = pathMetrics(pc,link,B_Hz,macEff,pktB,pos,...
    perHopAirtime_s,dutyCycle,radioMode,loraSF,routingThreshold_dB)
% Path metrics for the selected radio mode.
% Throughput is based on bottleneck PHY capacity, duty cycle, MAC efficiency and route-level PDR.
% PDR can be below 1 because each hop may lose packets due to
% weak SNR, multi-hop accumulation, and local contention.
np=numel(pc); %number of paths
c=3e8;
rt=strings(np,1); hp=zeros(np,1);
thr=zeros(np,1); lat=zeros(np,1);
bSNR=zeros(np,1); pdr=zeros(np,1); bCap=zeros(np,1);
sampleSNR = [];
sampleSNRonlyPDR = [];
sampleFinalPDR   = [];
for k=1:np
    p=pc{k}; %path k
    H=numel(p)-1; hp(k)=H; %number of hops in path k
    hC=zeros(H,1); hS=zeros(H,1); hP=zeros(H,1); hL=zeros(H,1); %cap,snr,pdr,latency of each hop in path k
    for i=1:H
        u=p(i); v=p(i+1);
        d=max(hypot(pos(u,1)-pos(v,1),pos(u,2)-pos(v,2)),0.1); %distance between 2 nodes in path 

        % Realistic per-hop delay. For LoRa this is packet Time-on-Air;
        % for Wi-Fi/BLE this is a small packet-airtime/processing delay.
        propDelay_s = d / c;   % propagation delay=distance/c
        if radioMode == "LoRa"
            txDelay_s = perHopAirtime_s; %tansmission delay
        else
            txDelay_s = (pktB*8) / link.cap_bps(u,v); %tansmission delay=num of bits in packet (127*8)/cap data rate of link
        end
        procDelay_s = 0.5e-3;  % fixed 0.5 ms processing delay per hop
        nodeDegree = sum(link.A(u,:));
        queueDelay_s = 0.1e-3 * max(0,nodeDegree-5); % congestion delay. No queueing penalty up to 5 neighboring nodes. Every additional neighbor adds 0.1 ms of queueing delay.

        hL(i) = propDelay_s + txDelay_s + procDelay_s + queueDelay_s; %latency of hop i in path k

        % Capacity and SNR are physics-based.
        hC(i)=link.cap_bps(u,v); %cap of hop i in path k
        hS(i)=link.snr_dB(u,v); %snr of hop i in path k

        % Packet-success model that allows PDR < 1.
        % Links with SNR below the routing threshold are excluded.
        % Links close to the threshold may still experience packet errors.
        snrDB = hS(i);
        
       if radioMode == "LoRa"

            % The 50% packet-success point is placed slightly above
            % the routing threshold.
            snrMid_dB = routingThreshold_dB - 2;

            % Controls how quickly PDR rises with SNR.
            snrSlope = 1.0;
        else

            % For Wi-Fi, BLE Mesh, and Custom radio modes.
            snrMid_dB = routingThreshold_dB - 2;
            snrSlope = 1.0;

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
        hP(i)=max(0.001, min(0.9999, hopSucc - degreePenalty)); %pdr of hop i in path k

        % Save samples for post-run analysis
        sampleSNR(end+1,1)        = snrDB;
        sampleSNRonlyPDR(end+1,1) = hopSucc;
        sampleFinalPDR(end+1,1)   = hP(i);

    end
    % Bottleneck capacity of the route: the min Shannon cap among all route links
    bc=min(hC); 
    bCap(k)=bc/1e6; % Bottleneck capacity of path k
    % Bottleneck SNR of the route
    bSNR(k)=min(hS); % Bottleneck SNR of path k

    % Route-level PDR is the mult product of hop success probabilities.
    % for multi-hop routes.
    pdr(k)=prod(hP); % PDR of path k as sum of hop's pdr

    % Optional retransmission effect: lower PDR increases expected latency.
    maxRetries = 3; %max retries to resend packet (4 tries overall)
    expectedAttempts = min(1 ./ max(hP,0.05),maxRetries + 1); %everage num of tries to sucsses
    routeServiceTime_s = sum(hL .* expectedAttempts); %overall time of service for path k
    lat(k) = routeServiceTime_s * 1e6; % Latency of path k
    % Effective route throughput based on Shannon bottleneck capacity
    thr(k) = dutyCycle * macEff * bc * pdr(k) / 1e6; % Throughput of path k
    % Route string
    rt(k) = strjoin("N"+string(p),"->");
    % Effective route throughput based on goodput modal - DELETED
    %payloadBits = pktB * 8;
    %routeGoodput_bps = payloadBits/ routeServiceTime_s;
    %thr(k) = dutyCycle * macEff * routeGoodput_bps / 1e6; % effective goodput includes duty cycle and delivery success
    %rt(k)=strjoin("N"+string(p),"->"); 
end
T2=table(rt,hp,thr,lat,bSNR,pdr,bCap,...
    'VariableNames',["Route","Hops","Throughput_Mbps","Latency_us",...
                     "BottleneckSNR_dB","PDR","BottleneckCapacity_Mbps"]);
results.table=T2;
results.linkSNR        = sampleSNR;
results.snrOnlyHopPDR  = sampleSNRonlyPDR;
results.finalHopPDR    = sampleFinalPDR;
end


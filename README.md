# Tactical RF Mesh Network Simulator
A MATLAB-based simulator for evaluating mobile multi-hop mesh networks using realistic wireless propagation and routing metrics. 
The simulator models a network of mobile nodes equipped with different radio technologies (LoRa, Wi-Fi, Bluetooth Mesh, or a custom RF configuration), 
computes link quality from physical-layer models, and selects the optimal communication route according to network performance metrics.
The project was developed as part of an Electrical Engineering final project investigating adaptive routing algorithms for tactical mesh networks.

## Features:
- Mobile mesh network using Random Waypoint Mobility
- Multiple radio technologies: LoRa (SX1262),Wi-Fi,Bluetooth Mesh,Custom RF model
- Realistic wireless channel model: Log-distance path loss,Shadow fading,Fast fading
- Shannon-capacity based link throughput
- Dynamic topology updates
- Multi-hop routing
- Route performance evaluation
- SNR threshold sensitivity analysis
- Real-time visualization

## Network Model
The simulator models a network consisting of:  
- 50 mobile nodes
- 30 × 200 meter area
- One source node
- One destination node
- Random waypoint mobility

 ```matlab
%% ==================== CONFIGURABLE PARAMETERS ==========================
N   = 50;   src = 1;   dst = N;
areaW = 30;  areaH = 200;      % 30 x 200 m tactical corridor area
% Mobility
dt   = 0.5;  T  = 60;    % 0.5-second step, total simulation time = 30 seconds
vMin = 0.5;   vMax = 2;     % m/s
```

Each simulation step:
1. Nodes move.
2. Wireless links are recalculated.
3. Link SNR is estimated.
4. Invalid links are removed.
5. All possible routes are searched.
6. The best route is selected by a parameter of choice (throughput,latency ect..)

 ```matlab
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
......
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
```

## Wireless Channel Model
Each wireless link is modeled using:
### Path Loss
A log-distance path loss model:

PL = PL<sub>0</sub> + 10n log<sub>10</sub>(d) + L<sub>extra</sub> + Shadow

where:
- distance attenuation
- environmental loss
- shadow fading
- fast fading
are all considered.
### Received Power
P<sub>RX</sub>=P<sub>TX</sub> −PL
### Noise
Receiver noise is calculated using:

N=−174+10n log<sub>10</sub>(B)+NF

where:
- bandwidth
- receiver noise figure
depend on the selected radio technology.
### Signal-to-Noise Ratio
SNR=P<sub>RX</sub>−Noise
### Channel Capacity
The theoretical link capacity is computed using Shannon's theorem

C=B log<sub>2</sub>(1+SNR) 

The actual link rate is limited by the maximum PHY data rate of the selected radio.

```matlab
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
```

## Routing
The simulator constructs a graph where:
- nodes represent devices
- edges represent valid wireless links
A link is considered valid only if: 
1. received power exceeds receiver sensitivity
2. SNR is above the routing threshold
Links below the threshold are removed from the routing graph.

The simulator enumerates all feasible paths (up to a configurable hop limit) and evaluates every candidate route.

```matlab
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
```

## Route Metrics
Every candidate route is evaluated according to:
### Throughput
Effective throughput is computed using:
1. bottleneck link capacity
2. MAC efficiency
3. duty cycle
4. packet delivery ratio
### Latency
Latency includes:
1. propagation delay
2. transmission time
3. processing delay
4. queueing delay
5. retransmission penalty
### Packet Delivery Ratio (PDR)
Each hop has its own success probability derived from
1. SNR
2. node congestion
The end-to-end PDR equals the product of all hop success probabilities.
### Bottleneck SNR
The route SNR is defined as the minimum SNR among all links on the selected path.
### Bottleneck Capacity
The route capacity is limited by the weakest link.

```matlab
function [A,link] = buildRadioGraph(pos,alive,B_Hz,Tx_dBm,NF_dB,f_Hz,rxSensitivity_dBm,phyDataRate_bps,extraLoss_dB,shadowStd_dB,pathLossExponent)
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
        rx = calcRxPower(d,Tx_dBm,f_Hz,extraLoss_dB,shadowStd_dB,pathLossExponent); %final received power
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
```

## Radio Modes
The simulator supports four PHY configurations.

### Mode | Frequency | Typical Use
LoRa | 915 MHz | Long-range, low-rate tactical communication

Wi-Fi | 2.4 GHz | High throughput

Bluetooth Mesh | 2.4 GHz | Low-power mesh

Custom RF | User-defined | Research scenarios

```matlab
function [B_Hz,f_Hz,Tx_dBm,NF_dB,rxSensitivity_dBm,phyDataRate_bps,perHopAirtime_s,dutyCycle] = ...
    configureRadioMode(radioMode,loraSF,loraBW_Hz,loraCR,loraPreamble,dutyCycle,packet_B)
% Configure the simulation according to the selected radio on the board.
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
        perHopAirtime_s = loraTimeOnAir(packet_B,loraSF,loraBW_Hz,loraCR,loraPreamble);
    case "WiFi"
        B_Hz = 20e6;
        f_Hz = 2.4e9;
        Tx_dBm = 18;
        rxSensitivity_dBm = -90;
        phyDataRate_bps = 150e6;
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 300e-6;
        dutyCycle = 1.0;
    case "BLEMesh"
        B_Hz = 2e6;
        f_Hz = 2.4e9;
        Tx_dBm = 8;
        rxSensitivity_dBm = -96;
        phyDataRate_bps = 1e6;
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 2e-3;
        dutyCycle = 1.0;
    case "Custom"
        B_Hz = 4e6;
        f_Hz = 915e6;
        Tx_dBm = 0;
        rxSensitivity_dBm = -105;
        phyDataRate_bps = 4e6;
        perHopAirtime_s = (packet_B*8)/phyDataRate_bps + 0.5e-3;
        dutyCycle = 1.0;
    otherwise
        error('Unknown radioMode.');
end
end
```

## Mobility
Nodes move according to the Random Waypoint Mobility Model.
Each node:
- selects a random destination
- moves at a random speed
- chooses a new destination upon arrival
This continuously changes the network topology.

```matlab
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
```

## Visualization
The simulator generates several figures.
### Figure 1- Live network topology  
Displays:
- moving nodes
- valid communication links
- selected route
- SNR-colored links
- source and destination nodes
### Figure 2- Live performance dashboard
Includes:
- Throughput
- Packet Delivery Ratio
- Active links
- Selected-route bottleneck SNR
### Figure 3- Topology snapshots
Ten snapshots showing the evolution of the network during the simulation.
### Figure 4- Performance over time
Time histories of: 
- Throughput
- Latency
- SNR
- Packet Delivery Ratio
### Figure 5- Performance vs SNR Threshold
Illustrates how changing the routing threshold affects Throughput,PDR,Latency,Active links
### Figure 6- Selected Route PDR vs Bottleneck SNR
Shows the relationship between physical-layer signal quality and end-to-end packet delivery performance.

## Performance Metrics
The simulator reports:
- Mean throughput
- Selected-route throughput
- Mean latency
- Selected-route latency
- Mean network SNR
- Selected-route SNR
- Mean Packet Delivery Ratio
- Radio range estimate
- Best routes
- Fallen links
- Route statistics

```matlab
function results = pathMetrics(pc,link,B_Hz,macEff,pktB,pos,...
    perHopAirtime_s,dutyCycle,radioMode,loraSF,routingThreshold_dB)
% Path metrics for the selected radio mode.
% Throughput is based on bottleneck PHY capacity, duty cycle, MAC efficiency and route-level PDR.
% PDR can be below 1 because each hop may lose packets due to
% weak SNR, multi-hop accumulation, and local contention.
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
        degreePenalty = 0.0002*max(0,nodeDegree-20);
        
        % Final per-hop PDR used by the simulation
        hP(i)=max(0.001, min(0.9999, hopSucc - degreePenalty));

        % Save samples for post-run analysis
        sampleSNR(end+1,1)        = snrDB;
        sampleSNRonlyPDR(end+1,1) = hopSucc;
        sampleFinalPDR(end+1,1)   = hP(i);

    end
    % Bottleneck capacity of the route:
    % the minimum Shannon capacity among all route links
    bc=min(hC); bCap(k)=bc/1e6; 
    % Bottleneck SNR of the route
    bSNR(k)=min(hS);

    % Route-level PDR is the mult product of hop success probabilities.
    % This naturally creates scenarios with PDR < 1, especially for weak or
    % multi-hop routes.
    pdr(k)=prod(hP);

    % Optional retransmission effect: lower PDR increases expected latency.
    maxRetries = 3;
    expectedAttempts = min(1 ./ max(hP,0.05),maxRetries + 1);
    routeServiceTime_s = sum(hL .* expectedAttempts);
    lat(k) = routeServiceTime_s * 1e6;
    % Effective route throughput based on Shannon bottleneck capacity
    thr(k) = dutyCycle * macEff * bc * pdr(k) / 1e6;
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
```

## Applications
This simulator can be used for: 
- Tactical communication research
- Mesh routing algorithm development
- Wireless network analysis
- LoRa performance evaluation
- Multi-hop routing optimization
- Academic research and education

## Future Work
Possible extensions include:
- Hardware-in-the-loop testing with ESP32 + SX1262 boards
- Energy consumption models
- Multiple simultaneous traffic flows
- Interference modeling
- Adaptive transmission power


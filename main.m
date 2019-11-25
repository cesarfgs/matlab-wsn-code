clear all
close all
clc

%% GPU config
%gpu = gpuDevice;
%gpu(1);


%% Main configuration values for this simulation

dataset.nodeNo = 50; %Number of nodes
dataset.nodePosition(1,:) = [1 50 50]; %(Sender node fixed position)
dataset.nodePosition(2,:) = [2 900 900]; %(Receiver node fixed position)
dataset.NeighborsNo = 5;
dataset.range = 250; %Tolerance distance to became neighbor of one node (Euclidean distance based)
dataset.atenuationFactor = 1.8; %Atenuation factor in freespace - ranges from 1.8 to 4 due environment
dataset.minEnergy = 80; % Mw - Miliwatts (70% energy)
dataset.maxEnergy = 100; % Mw - Miliwatts (Full energy (100%) - 1 mAh charge capacity within 1 Volt energy)
dataset.energyconsumptionperCicle = 0.35;
dataset.energyrecoveryperCicle = 0.2;
STenergy=10000;
packet=0;

% Node position sortition

for a = 3 : dataset.nodeNo
    
   dataset.nodeId = a; 
   garbage.x = randi([1 900]); %Xpos sortition
   garbage.y = randi([1 900]); %Ypos sortition
   dataset.nodePosition(a,:) = [dataset.nodeId garbage.x garbage.y]; %NodeID, X and Y position into nodePosition table
   
end

% Euclidean Distance calc from one node to all others

for i = 1 : dataset.nodeNo
    for j = 1: dataset.nodeNo
        garbage.x1 = dataset.nodePosition(i,2); 
        garbage.x2 = dataset.nodePosition(j,2); 
        garbage.y1 = dataset.nodePosition(i,3); 
        garbage.y2 = dataset.nodePosition(j,3);
        
        dataset.euclidiana(i,j) = sqrt(  (garbage.x1 - garbage.x2) ^2 + (garbage.y1 - garbage.y2)^2  ); 
        
    end
end

% Edges matrix definition due "range" variable value

dataset.weights = lt(dataset.euclidiana,dataset.range);

% Graph construction

G=graph(dataset.weights,'omitselfloops'); %Graph creation based on adjacency matrix (Edges matrix) built above

% Euclidean distance extraction for all existente end-to-end formed by
% "distance tolerance" (range variable value)

for a = 1 : height(G.Edges)
    garbage.s = G.Edges.EndNodes(a,1);
    garbage.t = G.Edges.EndNodes(a,2);
    garbage.Z(a,:) = dataset.euclidiana(garbage.s,garbage.t);
 end
G.Edges.Euclidiana = garbage.Z(:,1);

%Initial energy sortition (from 70% to 100% - minEnergy and maxEnergy variable valeu) 

[dataset.nodePosition(:,4)] = dataset.maxEnergy -(dataset.maxEnergy-dataset.minEnergy)*rand(dataset.nodeNo,1);
dataset.nodePosition(1:2,4)=STenergy;

%All "G" (Graph object) based nodes degree to use as "node processing
%status overload" (more connections, busier!)

for a = 1: length(dataset.nodePosition(:,1))
   
    dataset.nodePosition(a,5) = degree(G,dataset.nodePosition(a,1));
    
end

% Pathloss calc of each Edges based in a freespace (1.8 factor)

[G.Edges.Pathloss] = (10*dataset.atenuationFactor)*log10(G.Edges.Euclidiana);

%End points coordinates and energy migration to G object

for a = 1 : height(G.Edges)
	garbage.Sourcenode = G.Edges.EndNodes(a,1);
	garbage.Targetnode = G.Edges.EndNodes(a,2);
	G.Edges.SourcenodeXpos(a) = dataset.nodePosition(garbage.Sourcenode,2);
	G.Edges.SourcenodeYpos(a) = dataset.nodePosition(garbage.Sourcenode,3);
	G.Edges.TargetnodeXpos(a) = dataset.nodePosition(garbage.Targetnode,2);
	G.Edges.TargetnodeYpos(a) = dataset.nodePosition(garbage.Targetnode,3);
    %G.Edges.SourcenodeEnergy(a) = dataset.nodePosition(garbage.Sourcenode,4);
    %G.Edges.TargetnodeEnergy(a) = dataset.nodePosition(garbage.Targetnode,4);
    G.Edges.ActiveEdge(a) = 1;
end

% Graph objects plot

figure('units','normalized','innerposition',[0 0 1 1],'MenuBar','none')
subplot(1,1,1) %1,3,1 (Line number,collumn number, graph id) - if you want to show more than 1 graph in same windows
garbage.Xmax = 1500;
garbage.Xmin = 0;
garbage.Ymax = 1500;
garbage.Ymin = 0;
p = plot(G,'XData',(dataset.nodePosition(:,2)),'YData',(dataset.nodePosition(:,3))); %,'EdgeLabel',G.Edges.Weight
line(dataset.nodePosition(1:2,2),dataset.nodePosition(1:2,3),'color','green','marker','o','linestyle','none','markersize',100)
garbage.ax = gca;
garbage.ax.XAxis.TickValues = 0:100:1000;
garbage.ax.YAxis.TickValues = 0:100:1000;
grid on
hold on
pause(2)

%% finding routes

% Finding shortest path route
G2 = shortestpathtree(G,1,2);


%% Initialize patch existance test for loops

while ~isempty(G2.Edges)
    G2 = shortestpathtree(G,1,2);
    
    % Test if there is connection between node 1 and 2. If not, terminate!
    if isempty(G2.Edges)
        break
    end
    
    %Find edges found by shorthestpathtree in main G object
    for a = 1 : height(G2.Edges)
        source = G2.Edges.EndNodes(a,1);
        target = G2.Edges.EndNodes(a,2);
        garbage.edgesrow(a,:) = findedge(G,source,target);
    end

    %Find nodes involved in routing event
    garbage.routingnodes = unique(G2.Edges.EndNodes);

    %Code block for energy usage (decrease) and packet send count

    while min(dataset.nodePosition(:,4))>0 
        for a = 1 : length(garbage.routingnodes)
            node=garbage.routingnodes(a,1);
            dataset.nodePosition(node,4)=dataset.nodePosition(node,4)-dataset.energyconsumptionperCicle^rand()+dataset.energyrecoveryperCicle^rand();
            packet=packet+1
        end
    end

    %Find dead node ID to discorver its neighbors and disable relative edges 
    [garbage.deadnoderow] = find(dataset.nodePosition(:,4)<=0);
    for a = 1 : length(garbage.deadnoderow)
        deadnode=garbage.deadnoderow(a,1);
        for b = 1 : height(G.Edges)
            if ismember(G.Edges.EndNodes(b,1),deadnode) == 1 || ismember(G.Edges.EndNodes(b,2),deadnode) == 1
                G.Edges.ActiveEdge(b)=0;
                deadnode
                pause(2)
            end
        end
    end
    [garbage.deadedgerow]=find(G.Edges.ActiveEdge==0);
    G = rmedge(G,garbage.deadedgerow(:,1));

    %Mark node energy as NaN
    [garbage.deaddatasetnoderow]=find(dataset.nodePosition(:,4)<=0);
    for a = 1 : length(garbage.deaddatasetnoderow)
        b=garbage.deaddatasetnoderow(a,1);
        dataset.nodePosition(b,4)=NaN;
    end

   
    figure;p = plot(G,'XData',(dataset.nodePosition(:,2)),'YData',(dataset.nodePosition(:,3))); %,'EdgeLabel',G.Edges.Weight
    pause(2)   

end

%Message if there is no path between source and target even in the first iteration
disp('NO CONNECTIONS BETWEEN SOURCE (NODE1) AND TARGET (NODE2)')
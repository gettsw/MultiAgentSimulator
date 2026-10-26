function makeScenarioLayouts()
% Builds the ready-made map scenarios of scenarioCatalog: for each, a background image
% (Configurations/backgrounds/<Name>.png, drawn here) and a layout file (Configurations/<Name>.mat)
% whose walls come from the same geometry as the picture, so they line up exactly. Run it again
% after editing a scenario:
%   matlab -batch "addpath(genpath(pwd)); makeScenarioLayouts"
% Also restyles the lab floor plan (LabLayout.png -> backgrounds/LabFloor.png), records it as the
% background of ReportLayout.mat, and gives ReportLayout the walls traced from the plan.
here = fileparts(mfilename('fullpath'));
bgDir = fullfile(here, 'backgrounds');
if ~isfolder(bgDir), mkdir(bgDir); end
makers = {@cityPatrol, @officeFloor, @officeDock, @harborPatrol, @campus, @museum};
for k = 1:numel(makers)
    [name, s, drawFcn] = makers{k}();
    s.background = name + ".png";
    render(drawFcn, fullfile(bgDir, name + ".png"));
    save(fullfile(here, name + ".mat"), 's');
    fprintf('%s: %d targets, %d edges, %d walls, %d agents\n', name, size(s.targets, 1), ...
        size(s.edges, 1), size(s.walls, 1), numel(s.agentStarts));
end
lab = labPixels(fullfile(fileparts(here), 'LabLayout.png'));
restyleLabFloor(lab, fullfile(bgDir, 'LabFloor.png'));
tmp = load(fullfile(here, 'ReportLayout.mat'), 's');
s = tmp.s;
s.background = "LabFloor.png";
paperWalls = [20 65 20 75; 20 50 60 50; 60 40 60 60];   % the reference scenario's three walls
s.walls = [paperWalls; traceLabWalls(lab)];
save(fullfile(here, 'ReportLayout.mat'), 's');
fprintf('ReportLayout: %d walls (%d traced from the floor plan)\n', size(s.walls, 1), size(s.walls, 1) - 3);
end

function lab = labPixels(src)
% Pixel classes of the Stevens lab floor plan (LabLayout.png): open floor (one region, which
% also reaches outside through the doors), other floor (benches, door leaves), the green area,
% dark outlines and text, walls, the building's extent, and the diamond-shaped block in the
% middle of the floor (dropped from the map: the reference scenario has walls there instead)
I = im2double(imread(src)); I = I(:, :, 1:3);
g = mean(I, 3); sat = max(I, [], 3) - min(I, [], 3);
lab.I = I;
lab.green = I(:, :, 2) - I(:, :, 1) > 0.12;
lab.floor = g >= 0.90 & ~lab.green;
lab.dark = g < 0.42 & ~lab.green;
lab.wall = ~lab.floor & ~lab.dark & ~lab.green & sat < 0.10 & g < 0.75;
lab.light = ~lab.floor & ~lab.dark & ~lab.green & ~lab.wall;
L = bwlabel(lab.floor, 4);
[~, k] = max(accumarray(L(L > 0), 1));
lab.open = L == k;
rows = find(sum(lab.wall, 2) > 50); cols = find(sum(lab.wall, 1) > 50);   % long wall runs
lab.inside = false(size(g));
lab.inside(rows(1):rows(end), cols(1):cols(end)) = true;
F = (lab.open | lab.green) & lab.inside;
H = bwlabel(imfill(F, 'holes') & ~F, 8);                  % holes in the floor: text, the block
[~, k] = max(accumarray(H(H > 0), 1));
lab.block = imdilate(H == k, strel('disk', 2));
end

function restyleLabFloor(lab, dst)
% The lab floor plan recolored in the palette of the other maps, with its geometry and labels
% unchanged: a warm floor, light slate benches and door leaves, slate walls, and a green
% exterior around the building
setc = @(O, M, c) O .* ~M + reshape(c, 1, 1, 3) .* M;
out = setc(zeros(size(lab.I)), lab.floor, [0.85 0.88 0.91]);
out = setc(out, lab.open & lab.inside, [0.965 0.953 0.925]);
out = setc(out, lab.open & ~lab.inside, [0.85 0.88 0.82]);
out = setc(out, lab.wall, [0.37 0.42 0.48]);
out = setc(out, lab.light, [0.80 0.82 0.85]);
out = setc(out, lab.green, [0.49 0.72 0.55]);
out = setc(out, lab.dark, [0.20 0.23 0.27]);
out = setc(out, lab.block, [0.965 0.953 0.925]);
imwrite(out, dst);
end

function walls = traceLabWalls(lab)
% Walls along the edge of the lab's open floor (benches, room walls, door leaves; the green area,
% text labels and the dropped block count as open floor), as segments on the [0, 100]^2 canvas
% the image is stretched to
[H, W] = size(lab.open);
F = imfill((lab.open | lab.green) & lab.inside, 'holes');
F = bwareaopen(F, 1000, 4);
walls = zeros(0, 4);
for B = bwboundaries(F, 4)'
    P = reducepoly(B{1}, 0.003);                           % [row col], within about 4 px
    x = (P(:, 2) - 1) / (W - 1) * 100;
    y = (H - P(:, 1)) / (H - 1) * 100;
    walls = [walls; x(1:end-1), y(1:end-1), x(2:end), y(2:end)]; %#ok<AGROW>
end
walls = round(walls, 2);
end

%% ---------------------------------------------------------------------------------------------
function [name, s, draw] = cityPatrol()
% Police patrol in a street grid: patrol cars drive the roads (every edge runs along a road
% centerline, so routes stay on the roads) between intersections and five sites; the nine
% city blocks, the park included, are obstacles
name = "CityPatrol";
R = [8 36 64 92]; half = 4;                                % road centerlines (x and y), half width
blk = [R(1:3) + half; R(2:4) - half]';                     % block ranges (3 x 2), same in x and y
walls = zeros(0, 4);
for i = 1:3
    for j = 1:3
        walls = [walls; rectWalls(blk(i, 1), blk(j, 1), blk(i, 2), blk(j, 2))]; %#ok<AGROW>
    end
end
[gx, gy] = meshgrid(R, R);
targets = [gx(:), gy(:)];                                  % 16 intersections
sites = [22 8; 50 36; 22 64; 64 78; 92 50];
targets = [targets; sites];                               % Police HQ, Bank, School, Mall, Hospital
edges = zeros(0, 2);                                       % consecutive points along each road
for r = R
    for dim = 1:2
        on = find(abs(targets(:, dim) - r) < 1e-9);
        [~, order] = sort(targets(on, 3 - dim));
        on = on(order);
        edges = [edges; on(1:end-1), on(2:end)]; %#ok<AGROW>
    end
end
A = ones(size(targets, 1), 1); A(17:end) = [1 2 1.5 2 2.5];   % sites need more frequent patrols
% site buildings: block (i, j), the two lots they take (1 SW, 2 SE, 3 NW, 4 NE), roof, name
poiDefs = {1 1 [1 2] [0.36 0.53 0.83] 'POLICE HQ'; 2 1 [3 4] [0.55 0.62 0.70] 'BANK'
           1 3 [1 2] [0.98 0.80 0.35] 'SCHOOL';   3 3 [1 3] [0.86 0.66 0.74] 'MALL'
           3 2 [2 4] [0.97 0.97 0.97] ''};
hq = 17;
s = layout(targets, edges, walls, [hq, find(targets(:, 1) == 8 & targets(:, 2) == 64), ...
    find(targets(:, 1) == 64 & targets(:, 2) == 36), find(targets(:, 1) == 92 & targets(:, 2) == 92)], 15, A);
draw = @drawCity;

    function drawCity(ax)
        land = [0.949 0.937 0.914]; road = [1 1 1]; roadEdge = [0.80 0.80 0.80];
        avenue = [0.992 0.886 0.576]; avenueEdge = [0.93 0.76 0.35]; walk = [0.91 0.89 0.85];
        fill(ax, [0 100 100 0], [0 0 100 100], land, 'EdgeColor', 'none');
        isAve = @(r, dim) (dim == 1 && r == 64) || (dim == 2 && r == 36);    % Central Ave, Main St
        for dim = 1:2                                                   % road surfaces
            for r = R
                if isAve(r, dim), c = avenue; e = avenueEdge; else, c = road; e = roadEdge; end
                if dim == 1, fill(ax, r + [-half half half -half], [0 0 100 100], c, 'EdgeColor', e);
                else, fill(ax, [0 100 100 0], r + [-half -half half half], c, 'EdgeColor', e); end
            end
        end
        for r = R                                                       % junction boxes on top
            for q = R
                c = road; if r == 64 || q == 36, c = avenue; end
                fill(ax, r + [-half half half -half], q + [-half -half half half], c, 'EdgeColor', 'none');
            end
        end
        for dim = 1:2                                                   % lane markings
            for r = R
                for k = 1:3
                    span = [blk(k, 1) blk(k, 2)];
                    col = [1 1 1] * 0.82; if isAve(r, dim), col = [1 1 1]; end
                    if dim == 1, plot(ax, [r r], span, '--', 'Color', col, 'LineWidth', 1.2);
                    else, plot(ax, span, [r r], '--', 'Color', col, 'LineWidth', 1.2); end
                end
            end
        end
        for r = R                                                       % crosswalks
            for q = R
                for d = [-1 1]
                    for k = -3:1.5:3
                        col = [1 1 1] * 0.84; if r == 64, col = [1 1 1]; end
                        rectangle(ax, 'Position', [r + k - 0.35, q + d * (half + 1) - 0.8, 0.7, 1.6], ...
                            'FaceColor', col, 'EdgeColor', 'none');
                        col = [1 1 1] * 0.84; if q == 36, col = [1 1 1]; end
                        rectangle(ax, 'Position', [r + d * (half + 1) - 0.8, q + k - 0.35, 1.6, 0.7], ...
                            'FaceColor', col, 'EdgeColor', 'none');
                    end
                end
            end
        end
        roofs = [0.886 0.871 0.843; 0.843 0.824 0.788; 0.910 0.894 0.863; 0.871 0.847 0.812];
        for i = 1:3                                                     % blocks: sidewalk, buildings
            for j = 1:3
                x0 = blk(i, 1); x1 = blk(i, 2); y0 = blk(j, 1); y1 = blk(j, 2);
                rectangle(ax, 'Position', [x0 y0 x1-x0 y1-y0], 'FaceColor', walk, 'EdgeColor', [0.78 0.76 0.72]);
                if i == 2 && j == 2                                     % park
                    rectangle(ax, 'Position', [x0+1 y0+1 x1-x0-2 y1-y0-2], 'FaceColor', [0.776 0.902 0.765], 'EdgeColor', 'none');
                    plot(ax, [x0+1 x1-1], [y0+1 y1-1], 'Color', [0.95 0.93 0.86], 'LineWidth', 2);
                    plot(ax, [x0+1 x1-1], [y1-1 y0+1], 'Color', [0.95 0.93 0.86], 'LineWidth', 2);
                    rectangle(ax, 'Position', [46 47 8 6], 'Curvature', 1, 'FaceColor', [0.667 0.855 1], 'EdgeColor', 'none');
                    tree(ax, [43 43; 57 43; 43 57; 57 57; 50 43.5; 50 56.5]);
                    label(ax, 50, 51.5 - 6.5, 'City Park', 0);
                    continue;
                end
                lots = [0 0 0.48 0.48; 0.52 0 1 0.48; 0 0.52 0.48 1; 0.52 0.52 1 1];
                lot = @(q) lots(q, :) .* [x1-x0-2 y1-y0-2 x1-x0-2 y1-y0-2] + [x0+1 y0+1 x0+1 y0+1];
                site = find([poiDefs{:, 1}] == i & [poiDefs{:, 2}] == j);
                taken = [];
                if ~isempty(site)
                    taken = poiDefs{site, 3};
                    Ls = [lot(taken(1)); lot(taken(2))];
                    box = [min(Ls(:, 1:2)), max(Ls(:, 3:4)) - min(Ls(:, 1:2))];
                    poi(ax, box, poiDefs{site, 4}, poiDefs{site, 5});
                end
                for q = setdiff(1:4, taken)
                    L = lot(q);
                    c = roofs(mod(i + 2*j + q, 4) + 1, :);
                    rectangle(ax, 'Position', [L(1)+0.5 L(2)-0.5 L(3)-L(1) L(4)-L(2)], 'FaceColor', c * 0.88, 'EdgeColor', 'none');
                    rectangle(ax, 'Position', [L(1) L(2) L(3)-L(1) L(4)-L(2)], 'FaceColor', c, 'EdgeColor', c * 0.85);
                end
            end
        end
        rectangle(ax, 'Position', [80.5 50.5 3 3], 'FaceColor', [0.85 0.2 0.2], 'EdgeColor', 'none');   % hospital cross
        plot(ax, [82 82], [50.9 53.1], 'w', 'LineWidth', 3); plot(ax, [80.9 83.1], [52 52], 'w', 'LineWidth', 3);
        label(ax, 82, 46, 'HOSPITAL', 90, [0.7 0.15 0.15]);
        label(ax, 64, 17.5, 'Central Ave', 90); label(ax, 81, 36, 'Main St', 0);
        label(ax, 22, 92, 'North St', 0); label(ax, 8, 78, 'West St', 90);
    end
end

function poi(ax, box, color, txt)
% A site building: roof of the given color with its name
rectangle(ax, 'Position', box + [0.6 -0.6 0 0], 'FaceColor', color * 0.8, 'EdgeColor', 'none');
rectangle(ax, 'Position', box, 'FaceColor', color, 'EdgeColor', color * 0.7, 'LineWidth', 1.2);
if txt ~= "", label(ax, box(1) + box(3)/2, box(2) + box(4)/2, txt, 0, [1 1 1] * 0.2); end
end

%% ---------------------------------------------------------------------------------------------
function [name, s, draw] = officeFloor()
% One floor of an office and lab building: a T of corridors from the lobby, eight rooms with
% doors (gaps in the walls); a security robot patrols rooms and halls
name = "OfficeFloor";
door = 4;                                                  % half width of a door gap
gapped = @(x0, x1, y, gaps) hWallGaps(x0, x1, y, gaps, door);
walls = [gapped(4, 96, 4, 50)                              % outer walls, lobby entrance
         4 4 4 96; 96 4 96 96; 4 96 96 96
         gapped(4, 96, 56, [15 38 62 85])                  % north rooms' doors on the hall
         26 56 26 96; 74 56 74 96
         vWallGaps(50, 56, 96, 76, door)                   % Office 2 | Lab, inner door
         gapped(4, 44, 44, 15); gapped(56, 96, 44, 85)     % south rooms' doors on the hall
         4 24 44 24; gapped(56, 96, 24, 76)                % Lab A | Lab B; Meeting | Server door
         vWallGaps(44, 4, 44, 14, door); vWallGaps(56, 4, 44, 14, door)];   % lobby corridor walls
targets = [15 76; 38 76; 62 76; 85 76; 15 34; 24 14; 85 34; 76 20; ...
           15 50; 38 50; 50 50; 62 50; 85 50; 50 14];
names = ["Office 1", "Office 2", "Lab", "Lounge", "Lab A", "Lab B", "Meeting", "Server", ...
         "H1", "H2", "H3", "H4", "H5", "Lobby"];
edges = [9 10; 10 11; 11 12; 12 13; 1 9; 2 10; 3 12; 4 13; 5 9; 7 13; 6 14; 8 14; 14 11; 2 3; 7 8];
A = ones(numel(names), 1); A(names == "Server") = 2; A(names == "Lab" | names == "Lab A") = 1.5;
s = layout(targets, edges, walls, [14 9 13], 15, A);
draw = @drawOffice;

    function drawOffice(ax)
        outside = [0.85 0.88 0.82]; hall = [0.965 0.953 0.925]; wallC = [0.30 0.34 0.40];
        tints = {[0.90 0.94 0.98], [0.92 0.96 0.89], [0.98 0.94 0.89], [0.95 0.93 0.97]};
        fill(ax, [0 100 100 0], [0 0 100 100], outside, 'EdgeColor', 'none');
        fill(ax, [4 96 96 4], [4 4 96 96], hall, 'EdgeColor', 'none');
        rooms = [4 56 26 96; 26 56 50 96; 50 56 74 96; 74 56 96 96; 4 24 44 44; 4 4 44 24; 56 24 96 44; 56 4 96 24];
        for k = 1:size(rooms, 1)
            r = rooms(k, :);
            fill(ax, r([1 3 3 1]), r([2 2 4 4]), tints{mod(k - 1, 4) + 1}, 'EdgeColor', 'none');
        end
        for x = 6:4:94, plot(ax, [x x], [44.5 55.5], 'Color', [0.92 0.90 0.86], 'LineWidth', 0.5); end   % hall tiles
        for x = [7 16], desk(ax, [x 84 7 4]); desk(ax, [x 66 7 4]); end                     % Office 1
        for x = [30 41], desk(ax, [x 84 7 4]); desk(ax, [x 66 7 4]); end                    % Office 2
        for y = [64 72 80 88], bench(ax, [55 y 8 2.5]); bench(ax, [64.5 y 8 2.5]); end         % Lab benches
        rectangle(ax, 'Position', [78 84 14 6], 'Curvature', 0.6, 'FaceColor', [0.80 0.70 0.86], 'EdgeColor', [0.6 0.5 0.66]);  % sofa
        rectangle(ax, 'Position', [79 62 10 6], 'Curvature', 1, 'FaceColor', [0.86 0.78 0.66], 'EdgeColor', [0.66 0.58 0.46]);  % coffee table
        tree(ax, [92 60; 92 92]);
        for y = [28 36], bench(ax, [8 y 30 3]); end                                          % Lab A
        for y = [8 18], bench(ax, [8 y 14 3]); end                                           % Lab B
        rectangle(ax, 'Position', [62 28 18 8], 'Curvature', 1, 'FaceColor', [0.86 0.78 0.66], 'EdgeColor', [0.66 0.58 0.46]); % meeting table
        for x = 62:5:87, rectangle(ax, 'Position', [x 6 3 9], 'FaceColor', [0.25 0.27 0.30], 'EdgeColor', [0.15 0.15 0.15]); end % racks
        rectangle(ax, 'Position', [52.5 26 2.5 9], 'Curvature', 0.5, 'FaceColor', [0.86 0.78 0.66], 'EdgeColor', [0.66 0.58 0.46]);  % reception
        fill(ax, [45 55 55 45], [0.5 0.5 4 4], [0.70 0.66 0.60], 'EdgeColor', 'none');      % entrance mat
        for k = 1:size(walls, 1)
            w = walls(k, :);
            plot(ax, w([1 3]), w([2 4]), 'Color', wallC, 'LineWidth', 5);
        end
        doors = [15 56 0 1; 38 56 0 1; 62 56 0 1; 85 56 0 1; 15 44 0 -1; 85 44 0 -1; 76 24 0 -1
                 50 76 1 0; 44 14 -1 0; 56 14 1 0];                    % door center, into the room
        th = linspace(0, pi/2, 20);
        for k = 1:size(doors, 1)                                       % door leaves and swings
            c = doors(k, 1:2); n = doors(k, 3:4); t = [n(2) -n(1)];    % normal into the room, along the wall
            for side = [-1 1]                                          % double doors
                hinge = c + side * door * t; tip = hinge + door * n;
                arc = hinge + door * (-side * cos(th') * t + sin(th') * n);
                plot(ax, [hinge(1) tip(1)], [hinge(2) tip(2)], 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2);
                plot(ax, arc(:, 1), arc(:, 2), ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1);
            end
        end
        names2 = ["Office 1", "Office 2", "Lab", "Lounge", "Lab A", "Lab B", "Meeting", "Server room"];
        cen = [15 92.5; 38 92.5; 62 92.5; 85 92.5; 26 41; 32 21; 76 41; 88 21];
        for k = 1:numel(names2), label(ax, cen(k, 1), cen(k, 2), char(names2(k)), 0); end
        label(ax, 50, 30, 'Lobby', 0); label(ax, 20, 50, 'Hall', 0);
    end
end

function [name, s, draw] = officeDock()
% The Office floor with a charging dock by the lobby entrance (a ChargingStation, 300 W) and
% small batteries (2.5 Wh, about four minutes of operation; the robots stop when empty): a
% scenario for the battery-aware policy, whose robots must recharge. The three robots draw
% about 40 W each, so one dock can keep them going only if it charges much faster than that
[~, s, drawOffice] = officeFloor();
name = "OfficeDock";
dock = size(s.targets, 1) + 1;
s.targets(dock, :) = [50 7];
s.edges(end+1, :) = [14 dock];                             % Lobby - Dock
s.A(dock) = 0; s.R0(dock) = 0;
s.chargers = dock; s.chargePower = 300;
s.agentBatteryWh = 2.5; s.agentStopWhenEmpty = 1;
draw = @drawDock;

    function drawDock(ax)
        drawOffice(ax);
        rectangle(ax, 'Position', [45.5 4.8 9 4.6], 'Curvature', 0.3, ...
            'FaceColor', [0.30 0.33 0.38], 'EdgeColor', [0.18 0.20 0.23], 'LineWidth', 1.2);
        fill(ax, [48.3 46.3 47.8 46.9 49.3 47.8 48.9], [9.0 6.9 6.9 5.2 7.5 7.5 9.0], ...
            [1 0.82 0.1], 'EdgeColor', 'none');            % lightning bolt
        text(ax, 51.8, 7.1, 'DOCK', 'HorizontalAlignment', 'center', 'FontSize', 9, ...
            'FontWeight', 'bold', 'Color', [1 1 1], 'FontName', 'Helvetica');
    end
end

function desk(ax, box)
rectangle(ax, 'Position', box, 'FaceColor', [0.86 0.78 0.66], 'EdgeColor', [0.66 0.58 0.46]);
rectangle(ax, 'Position', [box(1) + box(3)/2 - 1, box(2) - 1.6, 2, 1.4], 'Curvature', 1, ...
    'FaceColor', [0.45 0.50 0.58], 'EdgeColor', 'none');                                     % chair
end

function bench(ax, box)
rectangle(ax, 'Position', box, 'FaceColor', [0.80 0.84 0.88], 'EdgeColor', [0.55 0.60 0.66]);
end

function w = hWallGaps(x0, x1, y, gaps, half)
% Horizontal wall from x0 to x1 at y with door gaps centered at gaps
cuts = sort([x0, reshape([gaps - half; gaps + half], 1, []), x1]);
w = [cuts(1:2:end)', repmat(y, numel(cuts) / 2, 1), cuts(2:2:end)', repmat(y, numel(cuts) / 2, 1)];
end

function w = vWallGaps(x, y0, y1, gaps, half)
% Vertical wall from y0 to y1 at x with door gaps centered at gaps
h = hWallGaps(y0, y1, x, gaps, half);
w = h(:, [2 1 4 3]);
end

%% ---------------------------------------------------------------------------------------------
function [name, s, draw] = museum()
% A small museum for the beginner's example (Examples/EventDetection): four galleries around a
% plus-shaped hall, each open to the atrium at its inner corner. Every wall and target lies on
% the GUI's 5-unit click grid, so the layout can be rebuilt by clicking on the image; its
% targets are event targets (SimpleTarget). The outer walls are walls too: without them, the
% shortest way around an inner wall that ends at the outer wall would pass outside.
name = "Museum";
inner = [40 10 40 30; 10 40 30 40; 40 90 40 70; 10 60 30 60
         60 10 60 30; 70 40 90 40; 60 90 60 70; 70 60 90 60];
outer = [10 10 45 10; 55 10 90 10; 90 10 90 90; 90 90 10 90; 10 90 10 10];   % entrance at x 45-55
walls = [inner; outer];
targets = [20 20; 20 80; 80 20; 80 80; 50 15; 15 50; 50 85; 85 50; 50 50];
edges = [9 5; 9 6; 9 7; 9 8; 1 9; 2 9; 3 9; 4 9; 1 5; 2 6; 4 7; 3 8];
s = layout(targets, edges, walls, [9 1 4], 4, ones(9, 1));   % slow indoor robots
s.targetType = "Simple Target (example)";
draw = @drawMuseum;

    function drawMuseum(ax)
        outside = [0.85 0.88 0.82]; hall = [0.965 0.953 0.925]; wallC = [0.30 0.34 0.40];
        tints = {[0.90 0.94 0.98], [0.92 0.96 0.89], [0.98 0.94 0.89], [0.95 0.93 0.97]};
        fill(ax, [0 100 100 0], [0 0 100 100], outside, 'EdgeColor', 'none');
        fill(ax, [10 90 90 10], [10 10 90 90], hall, 'EdgeColor', 'none');
        rooms = [10 10 40 40; 10 60 40 90; 60 10 90 40; 60 60 90 90];
        for k = 1:4
            r = rooms(k, :);
            fill(ax, r([1 3 3 1]), r([2 2 4 4]), tints{k}, 'EdgeColor', 'none');
        end
        rectangle(ax, 'Position', [43 43 14 14], 'Curvature', 1, 'FaceColor', [0.93 0.87 0.78], ...
            'EdgeColor', [0.80 0.72 0.60], 'LineWidth', 1.5);                    % atrium rug
        art = {[0.85 0.45 0.35], [0.35 0.55 0.80], [0.95 0.75 0.30], [0.45 0.65 0.45]};
        for k = 1:4                                                        % paintings, pedestals
            r = rooms(k, :); c = art{k};
            xs = r(1) + [6 16]; ys = r(2) + [7 19];
            if r(2) > 50, yw = r(4) - 1.6; else, yw = r(2) + 0.6; end
            for x = xs, rectangle(ax, 'Position', [x yw 7 1], 'FaceColor', c, 'EdgeColor', c * 0.7); end
            if r(1) > 50, xw = r(3) - 1.6; else, xw = r(1) + 0.6; end
            for y = ys, rectangle(ax, 'Position', [xw y 1 7], 'FaceColor', c * 0.9, 'EdgeColor', c * 0.6); end
            px = r(1) + 22 * (r(1) < 50) + 8 * (r(1) > 50);
            py = r(2) + 8 * (r(2) < 50) + 22 * (r(2) > 50);
            rectangle(ax, 'Position', [px - 1.5 py - 1.5 3 3], 'FaceColor', [0.97 0.97 0.97], ...
                'EdgeColor', [0.6 0.6 0.6]);                                     % pedestal
            rectangle(ax, 'Position', [px - 0.8 py - 0.8 1.6 1.6], 'Curvature', 1, 'FaceColor', c, 'EdgeColor', 'none');
        end
        fill(ax, [45 55 55 45], [5 5 10 10], [0.70 0.66 0.60], 'EdgeColor', 'none');   % entrance mat
        for k = 1:size(walls, 1), plot(ax, walls(k, [1 3]), walls(k, [2 4]), 'Color', wallC, 'LineWidth', 5); end
        label(ax, 25, 36, 'Gallery A', 0); label(ax, 25, 64, 'Gallery B', 0);
        label(ax, 75, 36, 'Gallery C', 0); label(ax, 75, 64, 'Gallery D', 0);
        label(ax, 50, 41, 'Atrium', 0); label(ax, 50, 3, 'Entrance', 0);
    end
end

%% ---------------------------------------------------------------------------------------------
function [name, s, draw] = harborPatrol()
% A harbor on the west coast (piers, breakwater) and two islands offshore; boats patrol buoys
name = "HarborPatrol";
coast = [0 0; 16 0; 18 12; 14 22; 19 34; 15 48; 20 60; 16 74; 21 88; 18 100; 0 100];
islandA = [44 64; 47 72; 55 76; 62 71; 61 62; 53 57; 46 59];
islandB = [68 26; 71 33; 78 34; 81 28; 77 22; 71 22];
piers = [17 18 30 18; 16.5 28 30 28];
breakwater = [30 40 44 46];
walls = [polyWalls(coast(2:end-1, :), false); polyWalls(islandA, true); polyWalls(islandB, true); ...
    piers; breakwater];
targets = [33.5 18; 33.5 28; 40 36; 30 80; 38 66; 55 86; 72 66; 60 45; 60 26; 78 12; 90 88; 92 48];
edges = [1 2; 1 3; 2 3; 3 8; 3 5; 5 4; 4 6; 5 6; 6 11; 6 7; 7 11; 7 8; 7 12; 8 9; 9 10; 10 12; 8 12; 11 12; 5 8];
A = ones(size(targets, 1), 1); A([1 2]) = 1.5;              % the port is busier
s = layout(targets, edges, walls, [1 6 10], 15, A);
draw = @drawHarbor;

    function drawHarbor(ax)
        sea = [0.624 0.808 0.937]; shallow = [0.72 0.87 0.96]; sand = [0.93 0.89 0.78];
        land = [0.86 0.90 0.78]; town = [0.88 0.85 0.80];
        fill(ax, [0 100 100 0], [0 0 100 100], sea, 'EdgeColor', 'none');
        for y = 3:6:99, plot(ax, [22 100], [y y + 1], 'Color', [0.67 0.84 0.95], 'LineWidth', 0.6); end
        outline(ax, coast + [3 0], shallow); outline(ax, coast + [1.2 0], sand); outline(ax, coast, land);
        fill(ax, [0 15 13 0], [6 6 40 40], town, 'EdgeColor', 'none');                  % port town
        for r = [1 9; 1 15; 1 21; 6 9; 6 15; 6 21; 1 27; 6 27; 1 33; 6 33]'
            rectangle(ax, 'Position', [r(1) r(2) 4 4], 'FaceColor', [0.80 0.62 0.52], 'EdgeColor', [0.6 0.45 0.38]);
        end
        for k = 1:size(piers, 1)
            p = piers(k, :);
            fill(ax, [p(1) p(3) p(3) p(1)], [p(2)-0.9 p(4)-0.9 p(4)+0.9 p(2)+0.9], [0.62 0.62 0.62], 'EdgeColor', [0.4 0.4 0.4]);
        end
        plot(ax, breakwater([1 3]), breakwater([2 4]), 'Color', [0.45 0.45 0.45], 'LineWidth', 5);
        for isl = {islandA, islandB}
            P = isl{1}; c = mean(P, 1);
            outline(ax, c + 1.25 * (P - c), shallow); outline(ax, c + 1.1 * (P - c), sand); outline(ax, P, land);
        end
        tree(ax, [52 68; 56 70; 50 63; 74 28; 76 31]);
        rectangle(ax, 'Position', [56.5 62.5 2.4 2.4], 'Curvature', 1, 'FaceColor', [0.95 0.95 0.95], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 2);
        plot(ax, [30 98], [96 60], '--', 'Color', [0.35 0.55 0.75], 'LineWidth', 1);   % shipping lane
        plot(ax, [36 98], [99 66], '--', 'Color', [0.35 0.55 0.75], 'LineWidth', 1);
        label(ax, 8, 46, 'Port Town', 90); label(ax, 54, 80, 'North Isle', 0); label(ax, 75, 19, 'South Isle', 0);
        label(ax, 83, 82, 'Shipping lane', -28, [0.25 0.45 0.7]);
    end
end

%% ---------------------------------------------------------------------------------------------
function [name, s, draw] = campus()
% A zoomed-out building complex: six buildings and a pond (obstacles), a quad, parking
name = "Campus";
bldgs = [12 60 34 80; 42 66 60 84; 68 58 92 78; 40 34 60 50; 70 20 92 42; 10 18 28 44];
names = ["Library", "Hall A", "Labs", "Student Center", "Gym", "Dorms"];
t = linspace(0, 2*pi, 11)'; t(end) = [];
pond = [47 + 9 * cos(t), 14 + 5 * sin(t)];
walls = zeros(0, 4);
for k = 1:size(bldgs, 1), walls = [walls; rectWalls(bldgs(k, 1), bldgs(k, 2), bldgs(k, 3), bldgs(k, 4))]; end %#ok<AGROW>
walls = [walls; polyWalls(pond, true)];
targets = [23 56; 51 62; 80 54; 50 30; 66 31; 32 31; 38 57; 22 91; 94 88; 47 23; 80 82];
edges = [1 7; 7 2; 2 11; 11 9; 9 3; 3 2; 3 5; 5 4; 4 7; 4 6; 6 1; 1 8; 8 7; 4 10; 10 6; 10 5];
A = ones(size(targets, 1), 1); A([2 3]) = 1.5;              % busy lecture hall and labs
s = layout(targets, edges, walls, [8 3 6 10 2], 15, A);
routes = plannedRoutes(s);                                 % footpaths follow the planned routes
draw = @drawCampus;

    function drawCampus(ax)
        grass = [0.812 0.910 0.722]; walk = [0.95 0.93 0.88]; road = [0.70 0.70 0.70];
        fill(ax, [0 100 100 0], [0 0 100 100], grass, 'EdgeColor', 'none');
        fill(ax, [0 100 100 0], [97 97 100 100], road, 'EdgeColor', 'none');             % campus road
        plot(ax, [0 100], [98.5 98.5], '--', 'Color', [1 1 1], 'LineWidth', 1);
        fill(ax, [97 100 100 97], [0 0 100 100], road, 'EdgeColor', 'none');
        rectangle(ax, 'Position', [4 85 38 11], 'FaceColor', [0.58 0.58 0.60], 'EdgeColor', [0.45 0.45 0.45]);  % parking
        for x = 6:3:40, plot(ax, [x x], [85.5 89], 'w', 'LineWidth', 1); plot(ax, [x x], [92 95.5], 'w', 'LineWidth', 1); end
        rectangle(ax, 'Position', [35.5 52 5 10], 'Curvature', 0.3, 'FaceColor', [0.74 0.86 0.66], 'EdgeColor', walk, 'LineWidth', 2); % quad
        for k = 1:numel(routes)                                                         % footpaths
            plot(ax, routes{k}(:, 1), routes{k}(:, 2), 'Color', walk, 'LineWidth', 6);
        end
        patch(ax, pond(:, 1), pond(:, 2), [0.667 0.855 1], 'EdgeColor', [0.5 0.72 0.9], 'LineWidth', 1.5);
        roofs = [0.73 0.75 0.78; 0.85 0.64 0.54; 0.73 0.75 0.78; 0.85 0.64 0.54; 0.73 0.75 0.78; 0.85 0.64 0.54];
        for k = 1:size(bldgs, 1)
            b = bldgs(k, :);
            rectangle(ax, 'Position', [b(1)+0.6 b(2)-0.6 b(3)-b(1) b(4)-b(2)], 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'none'); % shadow
            rectangle(ax, 'Position', [b(1) b(2) b(3)-b(1) b(4)-b(2)], 'FaceColor', roofs(k, :), 'EdgeColor', [0.45 0.45 0.45]);
            plot(ax, [b(1)+1.5 b(3)-1.5], ([b(2) b(4)] + [1.5 -1.5]) * [0.5; 0.5] * [1 1], 'Color', roofs(k, :) * 0.85, 'LineWidth', 2);
            label(ax, (b(1)+b(3))/2, (b(2)+b(4))/2 + 2, char(names(k)), 0);
        end
        tree(ax, [6 50; 6 70; 38 90; 63 56; 64 88; 95 10; 84 10; 34 44; 64 46; 30 4; 64 6; 8 6]);
        label(ax, 23, 82.5, 'Parking', 0); label(ax, 37.5, 61.5, 'Quad', 0); label(ax, 47, 14, 'Pond', 0, [0.25 0.45 0.7]);
        rectangle(ax, 'Position', [92.5 86.5 3 3], 'FaceColor', [0.2 0.4 0.75], 'EdgeColor', 'w'); % bus stop sign
        label(ax, 92, 84, 'Bus', 0);
    end
end

%% ---------------------------------------------------------------------------------------------
function routes = plannedRoutes(s)
% The edges' paths as the default planner draws them
m = ScenarioModel(DijkstraCornerPlanner());
m.importLayout(s);
routes = arrayfun(@(e) {e.curvePoints}, m.edges);
end

function s = layout(targets, edges, walls, starts, speed, A)
s = struct('targets', targets, 'edges', edges, 'walls', walls, 'agentStarts', starts, ...
    'agentSpeed', speed, 'uncertaintyMode', 'Static Uniform', 'A', A, 'B', 10, ...
    'R0', zeros(size(targets, 1), 1));
end

function w = rectWalls(x0, y0, x1, y1)
w = [x0 y0 x1 y0; x1 y0 x1 y1; x1 y1 x0 y1; x0 y1 x0 y0];
end

function w = polyWalls(P, closed)
if closed, Q = [P; P(1, :)]; else, Q = P; end
w = [Q(1:end-1, :), Q(2:end, :)];
end

function render(drawFcn, file)
% Draws on a 100 x 100 canvas and saves a 1000 x 1000 px image covering exactly [0, 100]^2
fig = figure('Visible', 'off', 'Units', 'pixels', 'Position', [0 0 1000 1000], 'Color', 'w');
ax = axes(fig, 'Position', [0 0 1 1]);
hold(ax, 'on'); axis(ax, 'off');
drawFcn(ax);
set(ax, 'XLim', [0 100], 'YLim', [0 100], 'DataAspectRatio', [1 1 1]);
set(fig, 'PaperUnits', 'inches', 'PaperPosition', [0 0 10 10], 'PaperSize', [10 10]);
print(fig, file, '-dpng', '-r100');
close(fig);
end

function outline(ax, P, color)
patch(ax, P(:, 1), P(:, 2), color, 'EdgeColor', 'none');
end

function tree(ax, P)
for k = 1:size(P, 1)
    rectangle(ax, 'Position', [P(k, :) - 1.3, 2.6, 2.6], 'Curvature', 1, ...
        'FaceColor', [0.42 0.66 0.40], 'EdgeColor', [0.32 0.52 0.30]);
end
end

function label(ax, x, y, txt, rot, color)
if nargin < 6, color = [0.35 0.33 0.30]; end
text(ax, x, y, txt, 'HorizontalAlignment', 'center', 'Rotation', rot, 'FontSize', 11, ...
    'FontWeight', 'bold', 'Color', color, 'FontName', 'Helvetica');
end

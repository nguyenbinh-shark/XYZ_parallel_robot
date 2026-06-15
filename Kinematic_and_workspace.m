% ===== THÔNG SỐ KHÂU (cùng đơn vị) =====
l0 = 5.5  % O->D theo trục x
l1 = 7  % OA
l2 = 13  % AB
l3 = 13  % CB
l4 = 7  % DC
xB = 6  % <-- nhập xB
yB = 7  % <-- nhập yB

 
    % ====== Nhánh trái (O-A-B) ======
    a = 2*xB*l1;
    b = 2*yB*l1;
    c = xB^2 + yB^2 + l1^2 - l2^2;
    [theta1_t1, theta1_t2, ok1] = angle(a, b, c);
    if ~ok1, ok = false; return; end

    % In nghiệm nhánh trái
    fprintf('OA:  t1 = %.6f rad (%.3f°),  t2 = %.6f rad (%.3f°)\n', ...
        theta1_t1, rad2deg(theta1_t1), theta1_t2, rad2deg(theta1_t2));

    % ====== Nhánh phải (D-C-B) ======
    d = 2*(xB - l0)*l4;
    e = 2*yB*l4;
    f = (l0 - xB)^2 + yB^2 + l4^2 - l3^2;
    [theta2_t1, theta2_t2, ok2] = angle(d, e, f);
    if ~ok2, ok = false; return; end

    % In nghiệm nhánh phải
    fprintf('DC:  t1 = %.6f rad (%.3f°),  t2 = %.6f rad (%.3f°)\n', ...
        theta2_t1, rad2deg(theta2_t1), theta2_t2, rad2deg(theta2_t2));


    % ====== Chọn nghiệm theo yB ======
    if yB >= 0
        theta1 = theta1_t1;
        theta2 = theta2_t2;
    else
        theta1 = theta1_t2;
        theta2 = theta2_t1;
    end
fprintf('theta1 = %.6f rad (%.3f°), theta2 = %.6f rad (%.3f°)\n', ...
        theta1, rad2deg(theta1), theta2, rad2deg(theta2));

% Kích thước khung vẽ tự động
Rmax = max(l1+l2, l3+l4) + 1;   % +1 margin
xlim_plot = [-Rmax, l0+Rmax];
ylim_plot = [-Rmax, Rmax];

%% ===== LƯỚI TỌA ĐỘ =====
nx = 500; ny = 500;                    % độ phân giải
[xg, yg] = meshgrid(linspace(xlim_plot(1), xlim_plot(2), nx), ...
                    linspace(ylim_plot(1), ylim_plot(2), ny));

rO = hypot(xg, yg);
rD = hypot(xg - l0, yg);

condLeft  = (abs(l1 - l2) <= rO) & (rO <= l1 + l2);
condRight = (abs(l4 - l3) <= rD) & (rD <= l4 + l3);
reachable = condLeft & condRight;

%% ===== VẼ WORKSPACE =====
figure('Color','w'); hold on; axis equal; box on;
imagesc(linspace(xlim_plot(1), xlim_plot(2), nx), ...
        linspace(ylim_plot(1), ylim_plot(2), ny), ...
        flipud(reachable));    % flipud để trục y hướng lên
set(gca,'YDir','normal');
colormap([1 1 1; 0.1 0.6 1]); % trắng = không tới, xanh = tới được
xlabel('x'); ylabel('y'); title('Workspace điểm B (giao hai vành khuyên)');
xlim(xlim_plot); ylim(ylim_plot);

% Vẽ O, D
plot(0,0,'ko','MarkerFaceColor','k'); text(0,0,'  O','VerticalAlignment','bottom');
plot(l0,0,'ko','MarkerFaceColor','k'); text(l0,0,'  D','VerticalAlignment','bottom');

% Vẽ biên các vành khuyên (tham khảo trực quan)
tt = linspace(0,2*pi,600);
plot( (l1+l2)*cos(tt), (l1+l2)*sin(tt), 'k:', 'LineWidth',0.8);
plot( abs(l1-l2)*cos(tt), abs(l1-l2)*sin(tt), 'k:', 'LineWidth',0.8);
plot( l0 + (l3+l4)*cos(tt), (l3+l4)*sin(tt), 'k:', 'LineWidth',0.8);
plot( l0 + abs(l3-l4)*cos(tt), abs(l3-l4)*sin(tt), 'k:', 'LineWidth',0.8);
legend({'','','',''},'Location','bestoutside'); % giữ bố cục gọn

%% ===== (TUỲ CHỌN) VẼ CƠ CẤU TẠI MỘT ĐIỂM B =====
plot(xB, yB, 'r.', 'MarkerSize',1);

% Kiểm tra nhanh bằng tam giác:
rO_B = hypot(xB,yB);
rD_B = hypot(xB-l0,yB);
    if ~(abs(l1-l2) <= rO_B && rO_B <= l1+l2 && abs(l4-l3) <= rD_B && rD_B <= l4+l3)
        warning('B nằm ngoài workspace.');
    end   
        % Vẽ cơ cấu
        O = [0,0]; D = [l0,0];
        A = [l1*cos(theta1), l1*sin(theta1)];
        C = [l0 + l4*cos(theta2), l4*sin(theta2)];
        plot([O(1) A(1) xB C(1) D(1)], [O(2) A(2) yB C(2) D(2)], '-o', 'LineWidth',2,'MarkerFaceColor',[1 .1 .2]);
xlim([-19.0 25.0])
ylim([-19.0 19.0])
function [t1, t2, ok] = angle(A, B, C)
    disc = A^2 + B^2 - C^2;
        if disc < 0
            ok = false;
            t1 = NaN; t2 = NaN;
            return;
        end
    s = sqrt(disc);
    denom = A + C;
    t1 = 2*atan((B + s)/denom);
    t2 = 2*atan((B - s)/denom);
    ok = true;
end
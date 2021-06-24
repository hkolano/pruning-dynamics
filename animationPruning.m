function animationPruning(p,t,X,exportVideo,playbackRate)

%Prep figure
figure(2);
clf;
axis equal;
hold on;

FPS = 120;
perp_forces = [];

%Prep video writer
if exportVideo
    filename = 'PruningDemo.mp4';
    v = VideoWriter(filename,'MPEG-4');
    v.FrameRate = FPS;
    open(v);
end

tic;
for t_plt = t(1):playbackRate*1.0/FPS:t(end)
    
    %Clear frame and reset properties that seem to be lost on clf
    clf;
    hold on;
    axis equal;
    
    % Interpolate state at this time step
    x_state = interp1(t',X',t_plt);
    
    %% Draw the branch
    t1 = 0:.002:2*pi;
    % Get branch circle outline points
    xbranch = p.r_branch*cos(t1);
    ybranch = p.r_branch*sin(t1);
    % translate points to the actual location, as given by X(5) (X_b) and
    % X(7) (Y_b)
    [xbranch,ybranch] = rtPoints(xbranch,ybranch,0,x_state(5),x_state(7));
    % fill with brown
    branch = polyshape(xbranch,ybranch);
    plot(branch, 'FaceColor', [122/255,49/255,7/255], 'FaceAlpha', .9);
%     fill(xbranch, ybranch,[87/255,26/255,7/255]);

    
    %% Draw cutter joint
    % Update x and y position of joint center
    [xjoint,yjoint] = rtPoints(p.joint_x,p.joint_y,0,x_state(1), x_state(3));
    fill(xjoint,yjoint,[.75,.75,.75]);
%     plot(xjoint, yjoint, 'r', 'LineWidth', 2);

    %% Draw top cutter
    [xtop,ytop] = rtPoints(p.top_x',p.top_y',0,x_state(1), x_state(3));
%     topcutter = polyshape(xtop, ytop);
%     plot(topcutter, 'FaceColor', [.7,.7,.7]);
    fill(xtop,ytop,[.7,.7,.7]);
    
    [xbot,ybot] = rtPoints(p.bot_x',p.bot_y',0,x_state(1), x_state(3));
%     botcutter = polyshape(xbot,ybot);
%     plot(botcutter, 'FaceColor', [.7,.7,.7]);
    fill(xbot,ybot,[.7,.7,.7]);
    
    %% Draw Force arrows
%     plot([0, x_state(5)], [0, x_state(7)], 'r', 'LineWidth', 2)
    top_cutter = polyshape(p.top_x+x_state(1), p.top_y+x_state(3));
    bottom_cutter = polyshape(p.bot_x+x_state(1), p.bot_y+x_state(3));

    top_hit = overlaps(top_cutter, branch);
    bot_hit = overlaps(bottom_cutter, branch);
    
    % Find intersect between the cutter and the branch
    if top_hit == 1
%         disp('Animation: top hit')
        plot_forces(p, x_state, top_cutter, branch)
    elseif bot_hit == 1
%         disp('Animation: bottom hit')
        plot_forces(p, x_state, bottom_cutter, branch)
    else
%         disp('No contact')
        lowright_text = ['Friction: None'];
        text(0.04, -0.035, lowright_text);
    end
        
       
%% Set proximal link parameters and draw it
%     %Half-width of proximal link
%     base_hw = 1/2;
%     %Height of proximal link
%     base_height = 2;
%     %Generic angle vector to draw a the hemisphere up top
%     t = 0:.001:pi;
%     %Stitch together points around perimeter of proximal link
%     xbase = [base_hw,base_hw*cos(t),-base_hw];
%     ybase = [0,base_hw*sin(t)+base_height,0];
%     %Fill it up wit purp
%     fill(xbase,ybase,[0.4940, 0.1840, 0.5560]);

%% Do the same thing with distal link, but make it a hair smaller
%     distal_hw = .9*base_hw;
%     distal_height = 1.5;
%     %Two hemisphere angle curves: One for the top, one for the bottom
%     t1 = pi:.001:2*pi;
%     t2 = 0:.001:pi;
%     %Stitch together points around perimeter of distal link
%     %With center of bottom hemisphere at origin
%     xdistal = [distal_hw*cos(t1),distal_hw*cos(t2)];
%     ydistal = [distal_hw*sin(t1),distal_height+distal_hw*sin(t2)];
%     %Rotate/Translate perimeter points by desired joint angle, and
%     %translate up to joint position
%     [xdistal,ydistal] = rtPoints(xdistal,ydistal,theta,0,base_height);
%     %Fill up proximal link wit green
%     fill(xdistal,ydistal,[0.4660, 0.6740, 0.1880]);

    %Draw the grey pivot-point circle between proximal and distal links
%     rpivot = .75*distal_hw;
%     t = 0:.001:2*pi;
%     %Get x,y points of circle perimeter
%     xpivot = rpivot*cos(t);
%     ypivot = rpivot*sin(t);
%     %Translate them up to joint location
%     [xpivot,ypivot] = rtPoints(xpivot,ypivot,0,0,base_height);
%     %Fill wit grey
%     fill(xpivot,ypivot,[.75,.75,.75]);
    

    %Draw the tendon-rerouting bushings 
%     bo = .7;
%     rbushing = 1/3*rpivot;
%     xbush = rbushing*cos(t);
%     ybush = rbushing*sin(t);
%     xbushs = .5*xbush;
%     ybushs = .5*ybush;
%     %Draw bushing on proximal link with two color circles
%     fill(xbush,ybush+(base_height-bo),[0.9294,0.6941,0.1255]);
%     fill(xbushs,ybushs+(base_height-bo),[1,1,1]);
%     %Translate bushing points up to distal link
%     [xbush,ybush] = rtPoints(xbush,ybush+bo,theta,0,base_height);
%     [xbushs,ybushs] = rtPoints(xbushs,ybushs+bo,theta,0,base_height);
%     %Draw bushing on distal link with two color circles
%     fill(xbush,ybush,[0.9294,0.6941,0.1255]);
%     fill(xbushs,ybushs,[1,1,1]);

    %Draw servo
%     servo_width = 2.5;
%     servo_hh = .75;
%     servox = [-base_hw/2,-base_hw/2,servo_width-base_hw/2,servo_width-base_hw/2];
%     servoy = [-1+servo_hh,-1-servo_hh,-1-servo_hh,-1+servo_hh];
%     fill(servox,servoy,[0,0,0]);

    %Draw servo horn
%     rhorn = servo_hh*.75;
%     t = [0:.001:2*pi];
%     xhorn = rhorn*cos(t);
%     yhorn = rhorn*sin(t);
%     xhorn = xhorn+rhorn-rbushing;
%     yhorn = yhorn - 1;
%     fill(xhorn,yhorn,[1,1,1]);

%     %Draw flexor tendon up to distal link connection
%     t1 = [pi:-0.001:pi-int_angle];
%     t2 = [-int_angle:.001:int_angle+theta];
%     xflexor1 = [-rbushing,rbushing*cos(t1),rpivot*cos(t2)];
%     yflexor1 = [-1,rbushing*sin(t1)+base_height-bo,rpivot*sin(t2)+base_height];
%     plot(xflexor1,yflexor1,'r','LineWidth',2);

%     %Draw flexor tendon from beginning of distal link to distal tip
%     t1 = [5*pi/4:-0.001:pi];
%     xflexor2=[rpivot*cos(int_angle),rbushing*cos(t1),0];
%     yflexor2=[rpivot*sin(int_angle),rbushing*sin(t1)+bo,distal_height+distal_hw];
%     [xflexor2,yflexor2] = rtPoints(xflexor2,yflexor2,theta,0,base_height);
%     plot(xflexor2,yflexor2,'r','LineWidth',2);
    
%     %Draw torsional spring
%     ssl = .75;
%     xbl = [-rpivot,-rpivot];
%     ybl = [base_height-ssl,base_height];
%     turns = 3;
%     rspring = 1/8;
%     phi = linspace(0,turns*2*pi,200);
%     bendangs = linspace(0,theta,200);
%     xcoil = (rspring*cos(phi)-(rpivot+rspring)).*cos(bendangs);
%     ycoil = (rspring*cos(phi)-(rpivot+rspring)).*sin(bendangs)+base_height;
%     xtl = [-rpivot,-rpivot];
%     ytl = [0,ssl];
%     [xtl,ytl] = rtPoints(xtl,ytl,theta,0,base_height);
%     xSpring = [xbl,xcoil,xtl];
%     ySpring = [ybl,ycoil,ytl];
%     plot(xSpring,ySpring,'b','LineWidth',2);

%     %Draw flexor tendon around servo horn
%     t1 = [pi:.001:pi-theta];
%     plot(rhorn*cos(t1)+rhorn-rbushing,rhorn*sin(t1)-1,'r','LineWidth',2);

%     %Draw 2 grey areas inside of servo horn showing servo movement
%     rins = .8*rhorn;
%     startt = pi/20;
%     t = [startt:.001:pi-startt];
%     xins = rins*cos(t);
%     yins = rins*sin(t);
%     [xins1,yins1] = rtPoints(xins,yins,-theta,0,0);
%     xins1 = xins1+rhorn-rbushing;
%     yins1 = yins1-1;
%     fill(xins1,yins1,[.5,.5,.5]);
%     [xins2,yins2] = rtPoints(xins,yins,pi-theta,rhorn-rbushing,-1);
%     fill(xins2,yins2,[.5,.5,.5]);

    % Write current time step on plot
    timetext = ['Time: ',num2str(t_plt)];
    text(-0.07, 0.035, timetext);
    
    disp('Time = ')
    disp(t_plt)

    %Set axis, dimensions of figure
    axis([-.075, .075, -.04, .04]);
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    title('Pruning Animation')
    fig = gcf;
    fig.Position(3:4) = [752,500];
    movegui
   

    %Draw figure to video framw
    if exportVideo
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
        while( toc < 1.0/FPS)
            pause(.002)
        end
        drawnow
        tic;
    end % if exportVideo
end % t_plt it = ...

if exportVideo
    close(v);
end

%Make rotation matrix for rotation/translation function
function R = makeRotMatrix(theta)

    R = [cos(theta),-sin(theta);sin(theta),cos(theta)];

end

%Batch rotates/translates points as a group action
function [nx,ny] = rtPoints(x,y,theta,dx,dy)

    %Get the rotation matrix to act on points
    R = makeRotMatrix(theta);

    %Compose points as a 2xN matrix of columns of point pairs
    allPoints = [x;y];
    %Batch rotate all points around the origin
    rotatedPoints = R*allPoints;
    %Translate rotated x,y points by desired offset
    nx = rotatedPoints(1,:) + dx;
    ny = rotatedPoints(2,:) + dy;

end

function plot_forces(p, x_state, cutter, branch)
        F_Kx = -p.kx*x_state(5); %-p.b*x_state(6);
        F_Ky = -p.ky*x_state(7); %-p.b*x_state(8);
        F_K = sqrt(F_Ky^2+F_Kx^2);

        th_Fk = atan2(F_Ky,F_Kx); % Angle of net restoring force (from +x)
        poly_int = intersect(cutter, branch);
        [C_intx, C_inty] = centroid(poly_int);
%         plot([C_intx, x_state(5)], [C_inty, x_state(7)], 'b', 'LineWidth', 2)
        
        % Find angle of the normal
        dy = C_inty-x_state(7);
        dx = C_intx-x_state(5);
        th_N = atan2(dy,dx); % Angle of normal force

        % Angle to project F_K onto F_N vector 
        th_projection = th_Fk-th_N;
%         disp([th_Fk, th_Fk2, th_N, th_projection]);

        F_N = -F_K*cos(th_projection); % Restoring force perp. to surface 
        F_rem = F_K*sin(th_projection); % Restoring force parallel to surface
        F_Nx = F_N*cos(th_N);%*-sign(dx);
        F_Ny = F_N*sin(th_N);%*-sign(dy);
        
%         disp('X dir')
%         disp(F_Kx + F_Nx)
%         disp('Y dir')
%         disp(F_Ky + F_Ny)
        
        
          if abs(x_state(6)-x_state(2)) < 0.001 && abs(x_state(8)-x_state(4)) < 0.001 % If branch isn't moving, add stiction
%                disp('Not moving; adding stiction')
               max_Ff = p.mu_s*F_N;
               if abs(F_rem) <= max_Ff % If friction can equal the remaining force
%                    disp('Friction is stopping motion')
                   F_f = - F_rem;
                   frict_text = ['Friction: sticking'];
               else 
%                    disp('Friction is insufficient; starting to move')
        %            F_f = 0;
                   F_f = -max_Ff*sign(F_rem);
                   frict_text = ['Friction: starting to slide'];
               end
           else % If branch is moving, apply kinetic friction
        %        disp('Branch is moving; adding kinetic friction')
%                F_f = 0;
               F_f = -p.mu_k*F_N*sign(F_rem);
               frict_text = ['Friction: sliding'];
           end % if stiction
           F_f = 0;

           F_fx = F_f*sin(th_N);
           F_fy = -F_f*cos(th_N);
           
        tot_Fx = F_Kx + F_Nx;
        tot_Fy = F_Ky + F_Ny;
        ang_tot_F = atan2(tot_Fx, tot_Fy);
        relative_ang = th_N - ang_tot_F;
        F_proj = sqrt(tot_Fx^2+tot_Fy^2)*cos(relative_ang)
        
        quiver(x_state(5), x_state(7), tot_Fx, tot_Fy, 'k', 'LineWidth', 2)
        quiver(x_state(5), x_state(7), F_Kx/10, F_Ky/10, 'r', 'LineWidth', 2); % Restoring force to equil.
%         quiver(x_state(5), x_state(7), -dx, -dy, 'b', 'LineWidth', 2); % Normal vector
        quiver(x_state(5), x_state(7), F_Nx/10, F_Ny/10, 'g', 'LineWidth', 2); % Normal force
        quiver(x_state(5), x_state(7), F_fx/10, F_fy/10, 'm', 'LineWidth', 2); % Friction Force
%         annotation('arrow', [(C_intx+.2)*10/3, (x_state(5)+.2)*10/3], [(C_inty+.1)*10/2, (x_state(7)+.1)*10/2]);
        text(0.04, -0.035, frict_text);
    end
    
    
end

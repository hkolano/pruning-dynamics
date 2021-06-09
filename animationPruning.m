function animationPruning(p,t,X,exportVideo,playbackRate)

%Prep figure
figure(1);
clf;
axis equal;
hold on;

%Prep video writer
filename = 'PruningDemo.mp4';
v = VideoWriter(filename,'MPEG-4');
v.FrameRate = 60;
open(v);

%Set up vector of joint angles to visualize
thetas = linspace(0,-pi/3,120);
thetas = [thetas,fliplr(thetas)];

%Guess at wrapping angle around bushings because math is hard
int_angle = pi/4;

%For each frame of the video, animate a joint angle
for i = 1:numel(thetas)
    
    %Get current joint angle
    theta = thetas(i);
    
    %Clear frame and reset properties that seem to be lost on clf
    clf;
    hold on;
    axis equal;

    %Set proximal link parameters and draw it
    %Half-width of proximal link
    base_hw = 1/2;
    %Height of proximal link
    base_height = 2;
    %Generic angle vector to draw a the hemisphere up top
    t = 0:.001:pi;
    %Stitch together points around perimeter of proximal link
    xbase = [base_hw,base_hw*cos(t),-base_hw];
    ybase = [0,base_hw*sin(t)+base_height,0];
    %Fill it up wit purp
    fill(xbase,ybase,[0.4940, 0.1840, 0.5560]);

    %Do the same thing with distal link, but make it a hair smaller
    distal_hw = .9*base_hw;
    distal_height = 1.5;
    %Two hemisphere angle curves: One for the top, one for the bottom
    t1 = pi:.001:2*pi;
    t2 = 0:.001:pi;
    %Stitch together points around perimeter of distal link
    %With center of bottom hemisphere at origin
    xdistal = [distal_hw*cos(t1),distal_hw*cos(t2)];
    ydistal = [distal_hw*sin(t1),distal_height+distal_hw*sin(t2)];
    %Rotate/Translate perimeter points by desired joint angle, and
    %translate up to joint position
    [xdistal,ydistal] = rtPoints(xdistal,ydistal,theta,0,base_height);
    %Fill up proximal link wit green
    fill(xdistal,ydistal,[0.4660, 0.6740, 0.1880]);

    %Draw the grey pivot-point circle between proximal and distal links
    rpivot = .75*distal_hw;
    t = 0:.001:2*pi;
    %Get x,y points of circle perimeter
    xpivot = rpivot*cos(t);
    ypivot = rpivot*sin(t);
    %Translate them up to joint location
    [xpivot,ypivot] = rtPoints(xpivot,ypivot,0,0,base_height);
    %Fill wit grey
    fill(xpivot,ypivot,[.75,.75,.75]);

    %Draw the tendon-rerouting bushings 
    bo = .7;
    rbushing = 1/3*rpivot;
    xbush = rbushing*cos(t);
    ybush = rbushing*sin(t);
    xbushs = .5*xbush;
    ybushs = .5*ybush;
    %Draw bushing on proximal link with two color circles
    fill(xbush,ybush+(base_height-bo),[0.9294,0.6941,0.1255]);
    fill(xbushs,ybushs+(base_height-bo),[1,1,1]);
    %Translate bushing points up to distal link
    [xbush,ybush] = rtPoints(xbush,ybush+bo,theta,0,base_height);
    [xbushs,ybushs] = rtPoints(xbushs,ybushs+bo,theta,0,base_height);
    %Draw bushing on distal link with two color circles
    fill(xbush,ybush,[0.9294,0.6941,0.1255]);
    fill(xbushs,ybushs,[1,1,1]);

    %Draw servo
    servo_width = 2.5;
    servo_hh = .75;
    servox = [-base_hw/2,-base_hw/2,servo_width-base_hw/2,servo_width-base_hw/2];
    servoy = [-1+servo_hh,-1-servo_hh,-1-servo_hh,-1+servo_hh];
    fill(servox,servoy,[0,0,0]);

    %Draw servo horn
    rhorn = servo_hh*.75;
    t = [0:.001:2*pi];
    xhorn = rhorn*cos(t);
    yhorn = rhorn*sin(t);
    xhorn = xhorn+rhorn-rbushing;
    yhorn = yhorn - 1;
    fill(xhorn,yhorn,[1,1,1]);

    %Draw flexor tendon up to distal link connection
    t1 = [pi:-0.001:pi-int_angle];
    t2 = [-int_angle:.001:int_angle+theta];
    xflexor1 = [-rbushing,rbushing*cos(t1),rpivot*cos(t2)];
    yflexor1 = [-1,rbushing*sin(t1)+base_height-bo,rpivot*sin(t2)+base_height];
    plot(xflexor1,yflexor1,'r','LineWidth',2);

    %Draw flexor tendon from beginning of distal link to distal tip
    t1 = [5*pi/4:-0.001:pi];
    xflexor2=[rpivot*cos(int_angle),rbushing*cos(t1),0];
    yflexor2=[rpivot*sin(int_angle),rbushing*sin(t1)+bo,distal_height+distal_hw];
    [xflexor2,yflexor2] = rtPoints(xflexor2,yflexor2,theta,0,base_height);
    plot(xflexor2,yflexor2,'r','LineWidth',2);
    
    %Draw torsional spring
    ssl = .75;
    xbl = [-rpivot,-rpivot];
    ybl = [base_height-ssl,base_height];
    turns = 3;
    rspring = 1/8;
    phi = linspace(0,turns*2*pi,200);
    bendangs = linspace(0,theta,200);
    xcoil = (rspring*cos(phi)-(rpivot+rspring)).*cos(bendangs);
    ycoil = (rspring*cos(phi)-(rpivot+rspring)).*sin(bendangs)+base_height;
    xtl = [-rpivot,-rpivot];
    ytl = [0,ssl];
    [xtl,ytl] = rtPoints(xtl,ytl,theta,0,base_height);
    xSpring = [xbl,xcoil,xtl];
    ySpring = [ybl,ycoil,ytl];
    plot(xSpring,ySpring,'b','LineWidth',2);

    %Draw flexor tendon around servo horn
    t1 = [pi:.001:pi-theta];
    plot(rhorn*cos(t1)+rhorn-rbushing,rhorn*sin(t1)-1,'r','LineWidth',2);

    %Draw 2 grey areas inside of servo horn showing servo movement
    rins = .8*rhorn;
    startt = pi/20;
    t = [startt:.001:pi-startt];
    xins = rins*cos(t);
    yins = rins*sin(t);
    [xins1,yins1] = rtPoints(xins,yins,-theta,0,0);
    xins1 = xins1+rhorn-rbushing;
    yins1 = yins1-1;
    fill(xins1,yins1,[.5,.5,.5]);
    [xins2,yins2] = rtPoints(xins,yins,pi-theta,rhorn-rbushing,-1);
    fill(xins2,yins2,[.5,.5,.5]);

    %Set axis, dimensions of figure
    axis([-1,3,-2,4]);
    fig = gcf;
    fig.Position(3:4) = [400,600];
    movegui

    %Draw figure to video framw
    drawnow;
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);

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

end
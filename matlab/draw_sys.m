function [myVideo] = draw_sys(name,l,r,t,x_b,y_b,x_p,y_p,window_size)

    pt_rad = 0.01; % Base point circule radius 

    f=figure();
    
    theta=0; %LOL

    b_1 = [0; (l+r)];
    b_2 = [(l+r)*sin(2*pi/3); (l+r)*cos(2*pi/3)];
    b_3 = [(l+r)*sin(-2*pi/3); (l+r)*cos(-2*pi/3)];

    % Initialize video
    myVideo = VideoWriter(strcat(name,'.mp4'),'MPEG-4'); %open video file
    myVideo.FrameRate = 20;  %can adjust this, 5 - 10 works well for me
    open(myVideo)
    
    
    i_pos=1;

    for i = 1: length(t)
        
        if(t(i)<=0)
            i_pos=i;
        end
    
        clf;
        b_t = eye(2); 
    
    
        p1 = [x_b(i);y_b(i)] + b_t*[r*sin(0); r*cos(0)];    % Vector in base coordinates from center to attachment
        f1 = b_1 - p1;        % Vector from actuator start to end
        L1 = norm(f1);                      % Length of actuator
    
        p2 = [x_b(i);y_b(i)] + b_t*[r*sin(2*pi/3); r*cos(2*pi/3)];    % Vector in base coordinates from center to attachment
        f2 = b_2 - p2;        % Vector from actuator start to end
        L2 = norm(f2);                      % Length of actuator
        
    
        p3 = [x_b(i);y_b(i)] + b_t*[r*sin(-2*pi/3); r*cos(-2*pi/3)];    % Vector in base coordinates from center to attachment
        f3 = b_3 - p3;        % Vector from actuator start to end
        L3 = norm(f3);                      % Length of actuator


        % Plot all pen points thus far
        plot(x_p(i_pos:i),y_p(i_pos:i),'r.-','LineWidth',0.3,'MarkerSize',3);
        hold on;

        scatter([b_1(1),b_2(1),b_3(1)],[b_1(2),b_2(2),b_3(2)],50,'bo','filled');
        
        scatter(x_p(i),y_p(i),50,'mx');

    
        
        line([p1(1),p2(1)], [p1(2), p2(2)], 'Color', 'b','LineWidth',2);
        hold on;
        line([p2(1),p3(1)], [p2(2), p3(2)], 'Color', 'b','LineWidth',2);
        line([p1(1),p3(1)], [p1(2), p3(2)], 'Color', 'b','LineWidth',2);
        title( sprintf('time = %.3f sec', t(i)) );
        axis([-1 1 -1 1]* window_size);


        drawnow;
        pause(0.0001) %Pause and grab frame
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);

    end
    close(myVideo)

end
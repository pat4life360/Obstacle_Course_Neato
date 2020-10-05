% grad descent following sources & sinks at center points of obstacles? 
% Key for #2: how to obtain center points of objects

% OG GradDescent Code

% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.


function guantletPaul(posX, posY, headingX, headingY)
% PAUL: stop before placement
    pub = rospublisher('raw_vel');
    stopMsg = rosmessage(pub);
    stopMsg.Data = [0 0];
    send(pub, stopMsg);
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);
    sub = rossubscriber('/scan');

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
    x = [];
    y = [];
    syms x y ;
%     Func = x*y-x^2-y^2-2*x-2*y+4;
    

%     GUANTLET FUNC DEF
    
    % Adding in barrel
    Func =  -2*log(sqrt((x-.75).^2 + (y+2.5).^2));

    % Adding in obstacles
    Func = Func + .6*log(sqrt((x-1).^2 + (y+.7).^2));
    Func = Func + .7*log(sqrt((x-1.41).^2 + (y+2).^2)); 
    Func = Func + .6*log(sqrt((x+.25).^2 + (y+1).^2)) ;

    % Adding in borders
    b = -1.5;
    for a = -3.37:0.5:1
        Func = Func - .1*(log(sqrt((x-2.5).^2 + (y-a).^2))) - .1*(log(sqrt((x+1.5).^2 + (y-a).^2))) - .1*(log(sqrt((x-b).^2 + (y+3.37).^2))) - .1*(log(sqrt((x-b).^2 + (y-1).^2)));
        b = b+ 4/9;
    end
    
    disp("func completed")
    disp(Func)
    
    gradDescent(Func, [posX;posY]);
        
    
    function max = gradDescent(Func, startingPoint)
        stopMsg = rosmessage(pub);
        stopMsg.Data = [0 0];
        send(pub, stopMsg);
        phi = 0;
        

        grad = gradient(Func, [x,y]);
        delta = 1.1;
        x_pos = startingPoint(1,:);
        y_pos = startingPoint(2,:);
        gradTemp = [headingX; headingY];

        lambda = 1;
    %     start = rostime('now');

    % PAUL: loop for longer
        while norm(gradTemp)>.01
            d = .235;

            gradTempOld = gradTemp;
            x_pos = startingPoint(1,:);
            y_pos = startingPoint(2,:);
            disp("completed math stage 1")
            gradTemp = subs(grad, [x y], [x_pos, y_pos]);
            gradTemp = round(double(gradTemp),4);
            disp("completed math stage 1.1 subs")
            startingPointOld = startingPoint;
            startingPoint = startingPoint + gradTemp.*lambda;
            lambda = lambda*delta;
            disp("completed math stage 2")
            
            
            angle1 = atan2(gradTemp(2,:),gradTemp(1,:));
            angle2 = atan2(gradTempOld(2,:),gradTempOld(1,:));
            disp("angle1") 
            disp(angle1)
            disp("angle2")
            disp(angle2)
            % PAUL: angdiff takes first input is starting angle and second is
            % ending angle (I switched this)
            angle = angdiff(angle2, angle1);
            
            
%             angle = double(acos(dot(gradTemp, gradTempOld)./(norm(gradTemp).*norm(gradTempOld))));
            

            disp("completed angle calcs")
            phi = phi + angle; 
            distance = double(norm(startingPoint - startingPointOld));
            disp("completed math stage 3")
            disp('gradtemp')
            disp(double(gradTemp))
            disp("norm Gradtemp")
            disp(double(norm(gradTemp)))
            disp('distance')
            disp(double(distance))
            disp('angle')
            disp(double(angle))

    %         Angular Function code
            if angle < 0 
                vL = 0.1;
                vR = -0.1; 
            else 
                vL = -0.1;
                vR = 0.1;
            end 
            omega = (vR - vL)/d;
            stopMsg.Data = [vL,vR];
            send(pub, stopMsg);
            tic;
            % PAUL :the old way you were figuring out when to end the loop did not
            % actually work.  Adi, I remember us working together and thinking that
            % this method was sound, but I just checked it now and it definitely doesn't work.            while true
            temp1 = rostime('now');
            while true
                elapsed = rostime('now') - temp1;
                if elapsed.seconds*omega>angle
                    break
                end
            end 
            toc;
            stopMsg.Data = [0,0];
            send(pub, stopMsg);

    %         Drive straight Function code 
%             temp2 = rostime('now');
            vL = 0.4;
            vR = 0.4;
            v = (vL + vR)/2;
            stopMsg.Data = [vL,vR];
            send(pub, stopMsg);
            
%             ORIGINAL WHILE LOOP CODE - DRIVE STRAIGHT FOR CALCULATED
%             DISTANCE
            % PAUL :the old way you were figuring out when to end the loop did not
            % actually work.  Adi, I remember us working together and thinking that
            % this method was sound, but I just checked it now and it definitely doesn't work.
            temp1 = rostime('now');
            while true
                elapsed = rostime('now') - temp1;
                if elapsed.seconds*v>distance
                    break
                end
            end 
            stopMsg.Data = [0,0];
            send(pub, stopMsg);
            
            
%             NEW WHILE LOOP CODE - PERFORM LIDAR DETECTION APROX EVERY
%             SECOND AND DETERMINE IF EU. DISTANCE IS < THRESHOLD - IF IT
%             IS, RECALC GRADIENT 
            
            
%             LIDAR SCAN & UPDATE FUNC
            % wait a while for the Neato to start scanning
%             pause(2);
%             % Collect data at the room origin
%             scan_message = receive(sub);
%             r_1 = scan_message.Ranges(1:end-1);
%             theta_1 = deg2rad([0:359]');
%             
% %             to access angle, use fi; to access x and y positions, use
% %             startingPoint
%             global_frame_pos_x=[r_1.*cos(theta_1+phi)-0.084*cos(phi)+startingPoint(1,:)];
%     
%             global_frame_pos_y= [r_1.*sin(theta_1+phi)-0.084.*cos(phi)+startingPoint(2,:)];
% %             PLOT
%               plot(global_frame_pos_x, global_frame_pos_y,"o");
%             title('Plot of Lidar Gauntlet Data');
%             ylabel('Y pos');
%             xlabel('X pos');

            
% Recalculate Func
       
     
        end

    max = double(startingPoint);
    end
end
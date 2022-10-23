classdef TurtleBotTask < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       
    end
    
    methods
        
        function obj = TurtleBotTask(mode, colour)
            %% Sensors and Control (SPR2022) - Assessment Task 3 
            % Group Project 7:  Turtlebot robot following each other 
            % Elmer Junior Caballero, Jithin Abraham, Shayer Shah


            %% SETUP
            % Connect to virtual machine's ROS master

            rosshutdown; % Shut down any running global ROS entities created by rosinit
            simulationIP = '192.168.233.132'; % This IP depends on the virtual machine running Gazebo
            rosinit(simulationIP);

            % Configure ROS subscribers and publishers
            cameraSubscriber = rossubscriber('/tb3_0/camera/rgb/image_raw'); % Topic for follower turtlebot3 image data
            receive(cameraSubscriber,10); % Program stalls here until image data from the simulation is received
            laserSubscriber = rossubscriber('/tb3_0/scan');
            receive(laserSubscriber,10);
            [velPub,velMsg] = rospublisher('/tb3_0/cmd_vel'); % Topic for follower turtlebot3 linear/angular velocity

            % Visualise the follower turtlebot3 camera view
            followerView = vision.DeployableVideoPlayer('Name', 'FollowerView','CustomSize', [640 480]);

            % Load control parameters
                controlParameters.Ts = 0.1;           % Sample time
                controlParameters.bufSize = 5;        % Filter buffer size
                controlParameters.maxDisp = 300;      % Max object displacement [pixels]
                controlParameters.minSize = 20;       % Min object size [pixels]
                controlParameters.maxSize = 20000;      % Max object size [pixels]
                controlParameters.maxCounts = 5;      % Max outlier counts before stopping
                controlParameters.linVelGain = 2e-3;  % Linear control gain
                controlParameters.angVelGain =1e-4;  % Angular control gain
                controlParameters.maxLinVel = 0.26;    % Max linear speed
                controlParameters.maxAngVel = 0.75;   % Max angular speed
                controlParameters.posDeadZone = 30;   % Steering control marker position dead zone [pixels] 
                controlParameters.targetSizeAdv = 2000;
                controlParameters.targetSizeBas = 300;
                controlParameters.targetDistance = 0.5;
                controlParameters.sizeDeadZone = 30;  % Linear speed control size dead zone [pixels]
                controlParameters.distanceDeadZone = 20;  % Linear speed control size dead zone [pixels]
                controlParameters.speedRedSize = 100; % Minimum pixel value before turning speed is ramped down

                fig = findall(0,"Name", "MATLAB App");
                handle = fig.RunningAppInstance;
                
                referenceImg = imread('heart.jpg');
                disp('Program Started')
                
                
                
            %% LOOP
            while(1) 
                
                drawnow();
                stopState = handle.STOPButton.Value;
                
                if stopState == 1
                   break; 
                end
                
                %% SENSE
                % Grab images
                followerImg = readImage(cameraSubscriber.LatestMessage);
                laserScan = laserSubscriber.LatestMessage.Ranges;

                %% PROCESS
                % Object detection algorithm
                resizeScale = 0.5;
                if mode == 'basic'
                    [centerX,centerY, state, matchSize] = obj.processImageBasic(followerImg,resizeScale, colour);
                end
                
                if mode == 'advanced'
                    [centerX,centerY, state, matchSize] = obj.processImageAdvanced(referenceImg,followerImg,resizeScale);
                end
                % Object tracking algorithm
                [v,w] = obj.calculateMovement(centerX, matchSize, laserScan, size(followerImg,2),controlParameters, mode, state);


                %% CONTROL
                % Package ROS message and send to the robot
                velMsg.Linear.X = v;
                velMsg.Angular.Z = w;
                send(velPub,velMsg);

                %% VISUALIZE
                % Annotate image and update the video player
                 followerImg = insertShape(followerImg,'Circle',[centerX centerY 50],'LineWidth',2);
                 step(followerView,followerImg);

            end

            disp('Program Stopped')

        end
    end
    
    methods(Static)
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function [centerX,centerY, state, blobSize] = processImageBasic(img,scale, colour)
        % Detects circle location and size in image

            % Initialize variables
            centerX = 0;
            centerY = 0;
            blobSize = 0;

            % Resize the image
            img = imresize(img,scale,'Antialiasing',false);

            % Create mask based on chosen histogram thresholds
            % For this example, we convert to L*a*b* space and then threshold
            % Thresholds for blue marker
            if colour == "BLUE"
                LMin = 0;
                LMax = 70;
                aMin = -100;
                aMax = 100;
                bMin = -100;
                bMax = -15;
            end
            if colour == "RED"
                LMin = 4.259;
                LMax = 32.593;
                aMin = 16.667;
                aMax = 62.963;
                bMin = 4.815;
                bMax = 48.519;
            end

            if colour == "GREEN"
                LMin = 30.593;
                LMax = 75.370;
                aMin = -100.000;
                aMax = -20.852;
                bMin = 20.333;
                bMax = 100.000;
            end

            labImg = rgb2lab(img);
            imgBW = (labImg(:,:,1)>=LMin)&(labImg(:,:,1)<=LMax)& ...
                    (labImg(:,:,2)>=aMin)&(labImg(:,:,2)<=aMax)& ...
                    (labImg(:,:,3)>=bMin)&(labImg(:,:,3)<=bMax);

            % Detect blobs
            persistent detector
            if isempty(detector)
                detector = vision.BlobAnalysis( ...
                            'BoundingBoxOutputPort',false, ...
                            'AreaOutputPort',false, ...
                            'MajorAxisLengthOutputPort', true, ...
                            'MinimumBlobArea',300, ...
                            'MaximumCount', 10);
            end
            [centroids,majorAxes] = detector(imgBW);

            % Estimate the blob location and size, if any are large enough
            if ~isempty(majorAxes)

                % Find max blob major axis
                [blobSize,maxIdx] = max(majorAxes);
                blobSize = double(blobSize(1));

                % Find location of largest blob
                maxLoc = centroids(maxIdx,:);
                centerX = double(maxLoc(1));
                centerY = double(maxLoc(2));

            end

            % Rescale outputs
            centerX = centerX/scale;
            centerY = centerY/scale;
            blobSize = blobSize/scale;
            if blobSize == 0
                state = 0;
            else
                state = 1;
            end
        end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        function [centerX,centerY, state, matchSize] = processImageAdvanced(capturedImg, img, scale)
        % Detects circle location and size in image

            persistent previousX

            if isempty(previousX); previousX = 0; end

            img = imresize(img,scale,'Antialiasing',false);
            capturedImg = imresize(capturedImg,scale,'Antialiasing',false);

            boxImage = rgb2gray(capturedImg);
            sceneImage = rgb2gray(img);

            boxPoints = detectSURFFeatures(boxImage);
            scenePoints = detectSURFFeatures(sceneImage);

            [boxFeatures, boxPoints] = extractFeatures(boxImage, boxPoints);
            [sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);

            boxPairs = matchFeatures(boxFeatures, sceneFeatures);

            matchedBoxPoints = boxPoints(boxPairs(:, 1), :);
            matchedScenePoints = scenePoints(boxPairs(:, 2), :);

            if matchedScenePoints.Count < 5 
                centerX = previousX;
                centerY = 0;
                state = 0;
                matchSize = 1000;
            else
                strongest = matchedScenePoints.selectStrongest(5);
                strongestPoints = strongest.Location;
                centerX = 2*mean(strongestPoints(:, 1));
                centerY = 2*mean(strongestPoints(:, 2));
                previousX = centerX;
                state = 1;
                matchSize = 1000;
            end       
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [v,w] = calculateMovement(x,blobSize, laserScan,imgWidth,controlParameters, mode, state)
        % Tracks location and distance of detected circle by assigning 
        % linear and angular velocities

            %% Initialize persistent variables
            persistent xBuffer xPrev sizeBuffer outlierCount tStart tFlag
            if isempty(tFlag); tFlag = 0; end
            if isempty(xBuffer); xBuffer = zeros(1,controlParameters.bufSize); end
            if isempty(sizeBuffer); sizeBuffer = zeros(1,controlParameters.bufSize); end
            if isempty(xPrev); xPrev = x; end
            if isempty(outlierCount); outlierCount = 0; end


            %% Input processing

            if (state == 0) & (tFlag == 0)
                tStart = tic;
                tFlag = 1;
            end

            if (state == 1)
                tFlag = 0;
            end

            if mode == "basic"

                if abs(x-xPrev) < controlParameters.maxDisp   
                    outlierCount = 0;
                else
                    outlierCount = min(outlierCount+1,1000); % Increment with saturation
                end
            end
            % Update and average the measurement buffers
             xBuffer = [xBuffer(2:controlParameters.bufSize) x];
             sizeBuffer = [sizeBuffer(2:controlParameters.bufSize) blobSize];
             xFilt = mean(xBuffer);
             sizeFilt = mean(sizeBuffer);
             xPrev = x;

            %% Angular velocity control: Keep the marker centered
            w = 0;

            if mode == "basic"
                if outlierCount < controlParameters.maxCounts        
                    posError = xFilt - imgWidth/2;
                    if abs(posError) > controlParameters.posDeadZone
                        speedReduction = max(sizeFilt/controlParameters.speedRedSize,1);
                        w = -controlParameters.angVelGain*posError*speedReduction;
                    end
                    if w > controlParameters.maxAngVel
                        w = controlParameters.maxAngVel;
                    elseif w < -controlParameters.maxAngVel
                        w = -controlParameters.maxAngVel;
                    end
                end
            else
                posError = xFilt - imgWidth/2;
                if abs(posError) > controlParameters.posDeadZone
                    speedReduction = max(sizeFilt/controlParameters.speedRedSize,1);
                    w = -controlParameters.angVelGain*posError*speedReduction;
                end
                if w > controlParameters.maxAngVel
                    w = controlParameters.maxAngVel;
                elseif w < -controlParameters.maxAngVel
                    w = -controlParameters.maxAngVel;
                end
            end

            %% Linear velocity control: Keep the marker at a certain distance
            v = 0;

            if mode == "basic"
                if outlierCount < controlParameters.maxCounts      
                    sizeError = controlParameters.targetSizeBas - sizeFilt;
                    if abs(sizeError) > controlParameters.sizeDeadZone
                        v = controlParameters.linVelGain*sizeError;
                    end
                    if v > controlParameters.maxLinVel
                        v = controlParameters.maxLinVel;
                    elseif v < -controlParameters.maxLinVel
                        v = -controlParameters.maxLinVel;
                    end
                    if any(laserScan(1:46) < 0.35) || any(laserScan(316:360) < 0.35)
                        v = 0;
                    end
                end
            else
                sizeError = controlParameters.targetSizeAdv - sizeFilt;
                if abs(sizeError) > controlParameters.sizeDeadZone
                    v = controlParameters.linVelGain*sizeError;
                end
                if v > controlParameters.maxLinVel
                    v = controlParameters.maxLinVel;
                elseif v < -controlParameters.maxLinVel
                    v = -controlParameters.maxLinVel;
                end
                if any(laserScan(1:46) < 0.35) || any(laserScan(316:360) < 0.35)
                        v = 0;
                end
            end

            %% Scan autonomously for a while if the outlier count has been exceeded
            if mode == "basic"
                if (blobSize == 0) || (outlierCount > 50 && outlierCount < 500)
                   v = 0;
                   w = controlParameters.maxAngVel/2; 
                end
            end

            if (mode == "advanced") & (state == 0)
                tEnd = toc(tStart);
                if tEnd > 5
                    v = 0;
                    w = controlParameters.maxAngVel/2;
                end
            end


        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end


classdef TurtleBotTask < handle
   %% Sensors and Control (SPR2022) - Assessment Task 3 
    % Group Project 7:  Turtlebot robot following each other 
    % Elmer Junior Caballero, Jithin Abraham, Shayer Shah
    
    properties
       
    end
    
    methods
        
        function obj = TurtleBotTask(mode, colour)
            % Function TurtleBotTask:
            % Main constructor of the TurtleBotTask class
            % Sets up the MATLAB-ROS connectivity and program parameters
            % Runs the main program loop
            
            %% CONFIGURATION     
            rosshutdown;
            simulationIP = '192.168.233.133'; 
            rosinit(simulationIP);

            % SETUP ROS SUBSCRIBERS AND PUBLISHERS
            cameraSubscriber = rossubscriber('/tb3_0/camera/rgb/image_raw');
            receive(cameraSubscriber,10);
            laserSubscriber = rossubscriber('/tb3_0/scan');
            receive(laserSubscriber,10);
            [velPub,velMsg] = rospublisher('/tb3_0/cmd_vel');

            followerView = vision.DeployableVideoPlayer('Name', 'FollowerView','CustomSize', [640 480]);
                
            % CREATE A STRUCTURE 'controlParameters' WITH TUNED CONTROL SETTINGS
            controlParameters.Ts = 0.1;           
            controlParameters.bufSize = 5;      
            controlParameters.maxDisp = 300;     
            controlParameters.minSize = 20;      
            controlParameters.maxSize = 20000;    
            controlParameters.maxCounts = 5;    
            controlParameters.linVelGain = 2e-3;  
            controlParameters.angVelGain = 1e-4;  
            controlParameters.maxLinVel = 0.26;   
            controlParameters.maxAngVel = 1;   
            controlParameters.posDeadZone = 30;   
            controlParameters.targetSizeAdv = 2000;
            controlParameters.targetSizeBas = 300;
            controlParameters.targetDistance = 0.5;
            controlParameters.sizeDeadZone = 20;  
            controlParameters.speedRedSize = 100; 

                fig = findall(0,"Name", "MATLAB App");
                handle = fig.RunningAppInstance;
                
                referenceImg = imread('heart.jpg');
                disp('Program Started')
                
            %% BEGIN MAIN LOOP
            while(1) 
                
                drawnow();
                stopState = handle.STOPButton.Value;
                
                if stopState == 1
                   break; 
                end
                
                %% GATHER DATA FROM SIMULATION
                followerImg = readImage(cameraSubscriber.LatestMessage);
                laserScan = laserSubscriber.LatestMessage.Ranges;

                %% CALL IMAGE PROCESSING FUNCTIONS 
                resizeScale = 0.5;
                if mode == 'basic'
                    [centerX,centerY, state, matchSize] = obj.processImageBasic(followerImg,resizeScale, colour);
                end
                
                if mode == 'advanced'
                    [centerX,centerY, state, matchSize] = obj.processImageAdvanced(referenceImg,followerImg,resizeScale);
                end

                [v,w] = obj.calculateMovement(centerX, matchSize, laserScan, size(followerImg,2),controlParameters, mode, state);


                %% CONTROLLING THE SIMULATED TURTLEBOT FOLLOWER
                velMsg.Linear.X = v;
                velMsg.Angular.Z = w;
                send(velPub,velMsg);

                %% UPDATE THE CAMERA VIEW OF THE TURTLEBOT FOLLOWER
                 followerImg = insertShape(followerImg,'Circle',[centerX centerY 50],'LineWidth',2);
                 step(followerView,followerImg);

            end

            disp('Program Stopped')

        end
    end
    
    methods(Static)
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function [centerX,centerY, state, blobSize] = processImageBasic(img,scale, colour)
            % Function processImageBasic: 
            % Detects a given colour (RGB) in the camera view and determines
            % the size of the matched blob and coordinates (x,y) in the frame.
            % Template used from:   
            % MathWorks Student Competitions Team (2022). 
            % Getting Started with MATLAB, Simulink, and ROS 
            %(https://github.com/mathworks-robotics/getting-started-ros), GitHub. 
            % Retrieved October 24, 2022.
        

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
                            'MinimumBlobArea',controlParameters.targetSizeBas, ...
                            'MaximumCount', 10);
            end
            [centroids,majorAxes] = detector(imgBW);

            % Estimate the blob location and size
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
            % Function processImageAdvanced: 
            % Detects a given pattern in the camera view and determines the
            % the matched pattern's coordinates (x,y) in the frame.

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
            
            figure(1); ax = axes;
            showMatchedFeatures(boxImage,sceneImage,matchedBoxPoints,matchedScenePoints,'montage','Parent',ax);

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
            % Function calculateMovement:
            % Determines the linear and angular velocities required to
            % track the targeted object.
            % Template used from:   
            % MathWorks Student Competitions Team (2022). 
            % Getting Started with MATLAB, Simulink, and ROS 
            %(https://github.com/mathworks-robotics/getting-started-ros), GitHub. 
            % Retrieved October 24, 2022.

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

            %% ANGULAR VELOCITY CALCULATION
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

            %% LINEAR VELOCITY CALCULATION
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

            %% Stop linear movement if the outlier count has been exceeded
            if mode == "basic"
                if (blobSize == 0) || (outlierCount > 50 && outlierCount < 500)
                   v = 0;
                   w = controlParameters.maxAngVel/2; 
                end
            end
            
            %% Stop linear movement if the pattern hasn't been detected for more than 5 seconds
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


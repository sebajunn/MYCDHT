clear all; close all; dbstop error

global MyContext

global ABCD;
ABCD.flagPause=0;
flag12 = 0;
flag1 = 0;
flag2 = 0;
flag3 = 0;
flag4 = 0;
flag5 = 0;
flag6 = 0;
flag7 = 0;
flag8 = 0;
flag9 = 0;
flag10 = 0;
flag11 = 0;
flag12 = 0;

% PART 1
MyFile = 'IMU_data.mat';  
load(MyFile) ;
load('NavMap.mat');
myVehicle.x = 0; myVehicle.y = 0; myVehicle.h = 0;

times = double(IMU.times)/10000;
% IMU.times is an array that contains the timestamps of the measurements.
% Each sample has an associated timestamp (sample time)

times = times - times(1) ;  % just, in order to refer to t0=0 (not necessary).

% IMU.Gyros(1,:) is Wx  (local Roll rate)
% IMU.Gyros(2,:) is Wy  (local Pitch rate)
% IMU.Gyros(3,:) is Wz  (local Yaw rate)
% All measurements are expressed in radians/second

Wx = double(IMU.DATAf(4,:));
Wy = double(IMU.DATAf(5,:));
Wz = double(IMU.DATAf(6,:));

k = 180/pi; % constant, useful for converting radian to degrees.
calibrationN = 3000; % number of elements that are within 15 seconds

meanInputX = Wx(1:calibrationN);
meanInputY = Wy(1:calibrationN);
meanInputZ = Wz(1:calibrationN);

bias(1) = mean(meanInputX);
bias(2) = mean(meanInputY);
bias(3) = mean(meanInputZ);

unbiasInput = Wz - bias(3);
unbiasWz = -(unbiasInput);

timeLength = length(times);
unbiasedAttitude = zeros(timeLength,3);

if (flag1 == 0)
    for n = 1:length(times)
        
        if n > 1
          dt_input = times(n) - times(n-1);
          dt = dt_input;
        else
          dt_initial = 0.005;
          dt = dt_initial; 
        end

        if n == 1
            initialx=0;
            initialy=0;
            initialz=0;
            currentAttitude = [initialx,initialy,initialz];
        else
            currentAttitude = newAttitude;
        end
        xBias = Wx(n) - bias(1);
        yBias = Wy(n) - bias(2);
        zBias =  -(Wz(n) - bias(3));
  
        newAttitude = gyroIntegrate(xBias, yBias, zBias, dt, currentAttitude); 
        unbiasedAttitude(n,:) = newAttitude;
        biasedAttitude = unbiasedAttitude - flag1;
    end
end


figure(2); clf(); hold on;
flagSub1 = 0;
flagSub2 = 0;
flagSub3 = 0;
uAttx = unbiasedAttitude(:,1)*k;
uAtty = unbiasedAttitude(:,2)*k;
uAttz = unbiasedAttitude(:,3)*k;

if (flagSub1 == 0)
    subplot(3,1,1), plot(times,uAttx);
    catchFlag = 1;
    title('Estimated Roll');
    xlabel('Time (seconds)');
    ylabel('Angle (degrees)');
    grid on;
end

if (flagSub2 == 0)
    subplot(3,1,2), plot(times,uAtty);
    catchFlag = 1;
    title('Estimated Pitch');
    xlabel('Time (seconds)');
    ylabel('Angle (degrees)');
    grid on;
end

if (flagSub3 == 0)
    subplot(3,1,3), plot(times,uAttz);
    catchFlag = 1;
    title('Estimated Yaw');
    xlabel('Time (seconds)');
    ylabel('Angle (degrees)');
    grid on;
end

% PART 2
myFile = 'DepthData01.mat';   
load(myFile);        % here we load the file, specified by the caller.

L = CR.N; % get number of data points  
myCRH = CR.H(1,:) - CR.H(1);
crH = myCRH;
timeConversion = 10000
laserTimes = double(crH)/timeConversion;
% load API 
API = IniAPIGetPointCloudFromDepth();
% This is a small API, for corverting depth data into 3D points. 

% The values are expressed for images 640x480, but internally adapted
% to our resolution of 160x120 

API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
fprintf('(using API ver=[%.1f])\n',API.Info.version);



global flag;

flag = 0;
if flag2 == 0;
    % initialise some variables
    previousTransformMatrix = eye(3);
    myQuality = 75;
    zerosInput = zeros(1,L);
    initialAngle = 0;
    myThetaY = initialAngle;
    myThetaX = initialAngle;
    myTrMatrix = eye(3);
    myPitchData = zerosInput;
    myRollData = zerosInput;
    myYawData = zerosInput;
end

initialContextX = 0;
initialContextY = 0;
initialContextNx = 40;
initialContextNy = 40;
initialContextX2 = 200;
initialContextY2 = 200;

MyContext.x1 = initialContextX ;MyContext.x2 = initialContextX2 ;
MyContext.y1 = initialContextY ;MyContext.y2 = initialContextY2 ;
MyContext.Nx = initialContextNx ;
MyContext.Ny = initialContextNy ;

% .....................
% figure(5) ; clf ; plot(0,0) ;
% axis([MyContext.x1,MyContext.x2,MyContext.y1,MyContext.y2]) ;
if (flag3 == 0)
    CreateOG() ;
end


figure(6); clf ; 
createColorMapping = 1;
mappingContext = MyContext.M;
if (createColorMapping == 1)
    MyContext.handle = imagesc(mappingContext) ; 
    colormap hot ;
    set(gca,'ydir','normal','xdir','normal') ;
end

%PopulateOG(Landmarks.xy(1,:),Landmarks.xy(2,:)) ;
%set(MyContext.handle, 'cdata', MyContext.M);

for i=1:L             
    if flag4 == 0;
        timeToc = 358;
        if i == timeToc
            pause; 
        end
    end
    
    Depth = CR.R(:,:,i);  %get a copy of depth image #i;

    % Call API method for obtaining its associated 3D points.
    % scale factor =0.1, so the resulting 3D points are expressed in Cm.
    [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ; 

    % choose region of interest
    
    ROIX = 50:110;
    ROIY = 70:120;
    
    RegionOfInterset.X = ROIX;
    RegionOfInterset.Y = ROIY;
    if (flag5 == 0)
        myXROI = RegionOfInterset.X;
        myYROY = RegionOfInterset.Y;
        xxROI = xx(myYROY,myXROI); 
        yyROI = yy(myYROY,myXROI); 
        zzROI = zz(myYROY,myXROI);
    end
    
    % fit plane using least square method
    xReShape = reshape(xxROI,numel(xxROI),1);
    yReShape = reshape(yyROI,numel(yyROI),1);
    ReShapePlane = ones(numel(yyROI),1);
    
    vectorA = [xReShape yReShape ReShapePlane];
    vectorB = reshape(zzROI,numel(zzROI),1);

    % solve matrix
    soleSolveN = (vectorA' * vectorA);
    matrixSolve = soleSolveN \ vectorA' * vectorB;
    vectorNormal = [-matrixSolve(1) -matrixSolve(2) 1];
    % normalise normal vector
    myNormal = norm(vectorNormal);
    normalisedVector = vectorNormal/myNormal;

    % hat vector to check fit
    AveChat = vectorA * matrixSolve ;
    flag100 = 0;
    if (flag100 == 1)
        AveChat;
    end
    
    bVectorChat = vectorA * matrixSolve;
    myResidual = vectorB - bVectorChat;
    resSquare = myResidual.^2;
    sumResSquare = sum(resSquare);
    myCatch = 0;
    if (flag5 == myCatch)
        logic1 = sumResSquare < myCatch;
        logic2 = ~(sumResSquare > myQuality);
        
        if((sumResSquare > myCatch) && (sumResSquare < myQuality))
            
            % solving equations analytically
            asinInput1 = -normalisedVector(1);
            asinInput2 = normalisedVector(2);
            myThetaY = asin(asinInput1);
            asinInput3 = cos(myThetaY);
            myThetaX = asin(asinInput2/asinInput3);
            myThetaTemp = myThetaY + myThetaX;
            myThetaZ = 0;

            % generate transformation matrix
            myTrMatrix = [cos(myThetaY)*cos(myThetaZ) -cos(myThetaX)*sin(myThetaZ)+sin(myThetaX)*sin(myThetaY)*cos(myThetaZ)  sin(myThetaX)*sin(myThetaZ)+cos(myThetaX)*sin(myThetaY)*cos(myThetaZ);
                        cos(myThetaY)*sin(myThetaZ)  cos(myThetaX)*cos(myThetaZ)+sin(myThetaX)*sin(myThetaY)*sin(myThetaZ) -sin(myThetaX)*cos(myThetaZ)+cos(myThetaX)*sin(myThetaY)*sin(myThetaZ);
                        -sin(myThetaY)    sin(myThetaX)*cos(myThetaY)          cos(myThetaX)*cos(myThetaY)];
            
        flagx = 0;
        if (flagx == 0)
            xxtemp = reshape(yy,1,numel(xx));   
            xx = reshape(xx,1,numel(xx));
        end
        if (flagx == 0)
            yytemp = reshape(yy,1,numel(yy));
            yy = reshape(yy,1,numel(yy));
        end    
        if (flagx == 0)
        zztemp = reshape(zz,1,numel(zz));
        zz = reshape(zz,1,numel(zz));
        end

        transformPlot = myTrMatrix*[xx;yy;zz];

        % store the good fitting trans. matrix
        previousTransformMatrix = myTrMatrix;

        
        else
            % if the plane fit is bad, use previous trans. matrix
            if (flag7 == 0)
                mySizexx = numel(xx);
                xx = reshape(xx,1,mySizexx);
                mySizeyy = numel(yy);
                yy = reshape(yy,1,mySizeyy);
                mySizezz = numel(zz);  
                zz = reshape(zz,1,mySizezz);
            end
            %myCoordTr = [myXX;myYY;myZZ];
            transformPlot = previousTransformMatrix*[xx;yy;zz];
        end
    end

    flagTransform = 0;
    flagReshape = 0;
    
    if (flagTransform == 0)
    xx = transformPlot(1,:);
    %size(xx(1));
    yy = transformPlot(2,:);
   % size(yy(1));
    zz = transformPlot(3,:);
   % size(zz(1));
    end
    if (flagReshape == 0)
        xx = reshape(xx,120,160);
      %  size(xx(1));
        yy = reshape(yy,120,160);
      %  size(yy(1));
        zz = reshape(zz,120,160);
      %  size(zz(1));
    end
    flagGlobalCheck = 0;
    flagYaw = 0;
    if (flagGlobalCheck == 0)
        if(~flag)
            angleUpdate1 = myThetaY; 
            angleUpdate2 = myThetaX;
            if (flagYaw ~= 0)
                iniYaw = rad2deg(0);
            end
            iniPitch = rad2deg(angleUpdate1);
            iniRoll = rad2deg(angleUpdate2);
            fprintf('Initial Pitch = %f deg, Initial Roll = %f deg \n', iniPitch, iniRoll);
            pause(1);
        else
        angleUpdate1 = myThetaY; 
        angleUpdate2 = myThetaX;

        % display the camera's estimated pitch and roll
        myYawData(i) = rad2deg(0);
        existingData = 0;
        if (existingData == 0)
            myPitchData(i) = rad2deg(angleUpdate1) - iniPitch;
            myRollData(i) = rad2deg(angleUpdate2) - iniRoll;
        end
        %fprintf('Pitch = %f deg, Roll = %f deg \n', pitchData(i), rollData(i));
        end
    end
    % update graphics to match transformed 3d cloud
    
    % get rid of faulty points
    faultyPoints = 0;
    xflag = 0;
    yflag = 0;
    zflag = 0;
    if (faultyPoints == 0)
        if(xflag == 0)
            xx(xx==0) = NaN;
        end
        if(yflag == 0)
            yy(yy==0) = NaN;
        end    
        if(zflag == 0)
            zz(zz==0) = NaN;
        end    
    else
        if(xflag == 0)
            xx(xx==0) = 0;
        end
        if(yflag == 0)
            yy(yy==0) = 0;
        end    
        if(zflag == 0)
            zz(zz==0) = 0;
        end      
    end
    
    floorheight = 30;
    zz = zz + floorheight; % floor is at 0
    
    %%% Part 4
    
    
    
    % circular area in front of vehicle
    trRange = sqrt(xx.^2 + yy.^2);
    
    % risky point checking 
    riskFlag = 0;
    riskLogic1 = trRange < 60;
    riskLogic2 = zz > 10;
    riskLogic3 = zz < 50;
    
    risk = riskLogic1 & riskLogic2 & riskLogic3;
    if (riskFlag == 0)
        riskxx = xx(risk);
        risktemp = yy(risk);
        riskyy = yy(risk);
        risktemp1 = yy(risk);
        riskzz = zz(risk);
    end

    
    %%% Part 3
    % simulate a laser scan
    laserScansL = zz>15;
    laserScansU = zz<16;
    ii = find(laserScansL & laserScansU);
    identityFlag = 0;
    if (riskFlag == 0)
        if (identityFlag == 0)
            xxL = xx(ii);
            xxP = xx(ii);
            yyL = yy(ii);
            yyP = yy(ii);
            zzL = zz(ii);
            zzP = zz(ii);
        end
    end
    
    separationy = diff(yyL);
    separationz = diff(zzL);
    separation = diff(xxL);
    poleCenters = [NaN;NaN];
    poleCentersDiff = [1;1];
    start = 1;
    end1 = 1;
    poleN = 0;
    loopCatch = numel(xxL)-1;
    
    
    for n = 1:loopCatch
        xxDifference = xxL(n) - xxL(n+1);
        absDiff = abs(xxDifference)>10;
        nElements = numel(xxL)-1;
        nCatch = n == nElements;
        if absDiff || nCatch % first check if the current point is 10cm from next, therefore making it the last point of the cluster
           if (end1 == 1)
               flagDefineX = 0;
               flagDefineY = 0;
               flagDefineLast = 0;
               if (flagDefineX == 0)
                   firstPointX = xxL(start);
                   endingPointX = xxL(end1);
               end
               if (flagDefineY == 0)
               firstPointY = yyL(start);
               endingPointy = yyL(end1);
               end
               if (flagDefineLast == 0)
                   lastPointX = xxL(n);
                   lastPointY = yyL(n);
               end
               xDiff = firstPointX - lastPointX;
               yDiff = firstPointY - lastPointY;
               size = sqrt((xDiff).^2 + (yDiff).^2);
               
               sizeL = (size >= 0.5);
               sizeU = (size <= 4);     
               
               size_condition = sizeL && sizeU;
               if (size_condition) && (start == 1)
                   if (size_condition)
                       xSum = firstPointX + lastPointX;
                       ySum = firstPointY + lastPointY;
                       poleN = poleN + 1;
                       poleCenters(1,poleN) = (xSum)./2; % x coord
                       poleCenters(2,poleN) = (ySum)./2; % y coord                           
                       start = n + 1;
                   end

               elseif (size_condition) && (separation(start-1) < 0 && separation(n) > 0) % check if the cluster is within the size range to be a pole
                   if (size_condition)
                       xSum = firstPointX + lastPointX;
                       ySum = firstPointY + lastPointY;
                       poleN = poleN + 1;
                       poleCenters(1,poleN) = (xSum)./2; % x coord
                       poleCenters(2,poleN) = (ySum)./2; % y coord                           
                       start = n + 1;
                   end
               else
                   start = n + 1;
               end
           end
        end
    end
    
    fprintf('No. of Poles visible: %d\n', poleN);
    for ii = 1:poleN
        %fprintf('Coord. of Pole %d: X = %.3f, Y = %.3f\n', ii, poleCenters(1,ii), poleCenters(2,ii));
    end
       
    
    if flag
            % Global
            gPoleCenters = TransformGlobal(poleCenters,myVehicle);
            gridCentresL = gPoleCenters(1,:);
            gridCentresU = gPoleCenters(2,:);
            %if (flag)
            set(hGPoles, 'xdata', gridCentresL, 'ydata', gridCentresU, 'color', 'b');
            AOOI = FindAOOI(gPoleCenters, Landmarks);
            if (flag)
                if ~isempty(AOOI.localIndex) % don't do anything if no AOOIs seen
                       numberOfAOOI = length(AOOI.globalIndex) == 1;
                       assertNonZero = AOOI.globalIndex ~= 0;
                       AOOILogic = numberOfAOOI && assertNonZero;
                       
                    if AOOILogic
                        globalAOOI = AOOI.globalIndex;
                        localAOOI = AOOI.localIndex;
                        if (AOOI.globalIndex ~= 0)
                            dx = Landmarks.xy(1,globalAOOI) - gPoleCenters(1,localAOOI);
                            %unnecessary below
                            dz = Landmarks.xy(1,globalAOOI) - gPoleCenters(1,localAOOI); 
                            dy = Landmarks.xy(2,globalAOOI) - gPoleCenters(2,localAOOI);
                        end
                        vehicleMoving = 1;
                        vehicleStillMoving = 1;
                       
                        if (vehicleMoving == 1)
                            vehicleDx = myVehicle.x + dx;
                            myVehicle.x = vehicleDx;
                            vehicleDy = myVehicle.y + dy;
                            myVehicle.y = vehicleDy;
                            unbiasTheAttitude = unbiasedAttitude(i*54,3);
                            
                            vehicleH = initH + unbiasTheAttitude;
                        end    
                            myVehicle.h = vehicleH;
                        if (vehicleStillMoving == 1)     
                            gPoleUpdateX = gPoleCenters(1,:) + dx;
                            gPoleUpdatey = gPoleCenters(1,:) + dy;
                            
                            gPoleCenters(1,:) = gPoleUpdateX;
                            gPoleCenters(2,:) = gPoleUpdatey;
                        end
                    end
                    moreThanOneAOOI = length(AOOI.globalIndex) > 1;    
                   
                    if moreThanOneAOOI
                        fminFlag = 0;
                        errorFlag = 0;
                        shiftFlag = 0;
                        
                        ogPC = gPoleCenters; % save original untransformed pole center positions
                        initInitial = [1,1,1];
                        initGuess = [0,0,0];
                        %options = optimset('Display', 'final');
                        % error = Adjustment(AOOI, gPC, LMs, veh, xshift, yshift, angle)
                        tic;
                        if (fminFlag == 0)
                            mySol = fminsearch(@(guess) Adjustment(AOOI, poleCenters, Landmarks, myVehicle, guess(1), guess(2), guess(3)), initGuess);%, options);
                        end
                        if (errorFlag == 0)
                            error1 = Adjustment(AOOI, poleCenters, Landmarks, myVehicle, mySol(1), mySol(2), mySol(3));
                        end
                        %disp(error);
                        
                        if (shiftFlag == 0)
                            [xshift, yshift] = SecondAdjust(AOOI, poleCenters, Landmarks, myVehicle, mySol(1), mySol(2), mySol(3));
                        end

                        solXshift = mySol(1) + xshift;
                        solYShift = mySol(2) + yshift;
                        
                        mySol(1) = solXshift;
                        mySol(2) = solYShift;
                        
                        error2 = Adjustment(AOOI, poleCenters, Landmarks, myVehicle, mySol(1), mySol(2), mySol(3));
                        %disp(error);
                        toc;
                        solflag = 0;
                        
                        if (solflag == 0)
                            VehicleX = myVehicle.x + mySol(1);
                        end
                        if (solflag == 0)
                            VehicleY = myVehicle.y + mySol(2);
                        end
                        if (solflag == 0)
                            Vehicleh = myVehicle.h + mySol(3);
                        end
                        vehicleFlag = 0;
                        
                        if (vehicleFlag == 0)
                            myVehicle.x = VehicleX;
                            myVehicle.y = VehicleY;
                            myVehicle.h = Vehicleh;
                        end
                        
                        gPoleCenters = TransformGlobal(poleCenters,myVehicle);

                    end
                    error1Hist(end+1) = error1;
                    error2Hist(end+1) = error2;

                    raySector = linspace(0,200,101);
                    zerosSector = zeros(1,length(linspace(0,200,101)));
                    
                    ray = [raySector; zerosSector];
                    angle = 60;
                    angView = deg2rad(angle);
                    myAngle = angView/2 ;
                    angView = linspace(-myAngle,myAngle,angle);
                    rays = RayTransform(ray, myVehicle, angView);
                    
                    if (vehicleFlag == 0)
                        rayIndex = myPopulateRays(rays);
                        PopulatePC(gPoleCenters, rayIndex);
                        myPopulateVeh(myVehicle);
                    end
                             
                    set(MyContext.handle, 'cdata', MyContext.M);
                    vehicleX = myVehicle.x;
                    vehicleY = myVehicle.y;
                    vehicleH = myVehicle.h;
                    initalDcos = initD*cos(myVehicle.h);
                    initalDsin = initD*sin(myVehicle.h);
                    vehicleHist(:,end+1) = [vehicleX; vehicleY; vehicleH];
                    vHist1 = vehicleHist(1,:);
                    vHist2 = vehicleHist(2,:);
                    histError1 = error1Hist(:);
                    histError2 = error2Hist(:);
                    histLength1 = 1:length(error1Hist);
                    histLength2 = 1:length(error2Hist);
                    
                    gPoleCenters1 = gPoleCenters(1,:);
                    gPoleCenters2 = gPoleCenters(2,:);
                    
                    set(hGPoles, 'xdata', gPoleCenters1, 'ydata', gPoleCenters2, 'color', 'r');
                    set(hVehicle, 'xdata', vehicleX, 'ydata', vehicleY, 'udata', initalDcos, 'vdata', initalDsin);
                    
                    set(myhPath, 'xdata', vHist1, 'ydata', vHist2);
                    
                    set(hError1, 'ydata', histError1(:), 'xdata', histLength1);
                    set(hError2, 'ydata', histError2(:), 'xdata', histLength2);
                end
            end
            %disp(vehicle);

            % Plot
            
            flag1 = 0;
            if (flag1 == 0 )
                depthInput = Depth;
                
                set(h1,'cdata',Depth);
                xxInput = xx(1:end);
                yyInput = yy(1:end);
                xxInput = xx(1:end);
                set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));
            end
            
            xRisky = riskxx;
            yRisky = riskyy;
            zRisky = riskzz;
            xLaser = xxL;
            yLaser = yyL;
            zLaser = zzL;
            
            if (flag1 == 0 )
                set(hRisk, 'xdata',xRisky,'ydata',yRisky,'zdata',zRisky);
                myRisky = xxL + yyL + zzL;
                set(hLaser,'xdata',xLaser,'ydata',yLaser,'zdata',zLaser);
            end
            
            %s = sprintf('Depth at time [%.3f] secs',laserTimes(i));
            if (flag1 == 0)
                set(titleHandle,'string',sprintf('Depth at time [%.3f] secs',laserTimes(i)));
            end
  
            if (flag1 == 0)
                set(h2dLaser, 'xdata', xxL);
                set(h2dLaser, 'ydata', yyL);
                %set(h2dLaser, 'xdata', xxL, 'ydata', yyL);
                set(hPoles, 'xdata', poleCenters(1,:));
                set(hPoles, 'ydata', poleCenters(2,:));
                %set(hPoles, 'xdata', poleCenters(1,:), 'ydata', poleCenters(2,:));

                %s = sprintf('Simulated laser scan at time [%.3f] secs',laserTimes(i));
                set(laserTitleHandle,'string',sprintf('Simulated laser scan at time [%.3f] secs',laserTimes(i)));
            end
            

        else   %First time: we create Matlab graphical objects
            flag=1;
            flag2 = 0;
            flag3 = 0;
            figure(1) ; clf() ;  
            gridMap = [100,2000];
            subplot(211) ; 
            subplotCreate = 1;
            if (subplotCreate == 1)
                h1 = imagesc(Depth,gridMap); % displayed with scaled colours
            end
            % image, scalling color for values in range from 100mm to 2000.
            if (subplotCreate == 1)
                set(gca(),'xdir','reverse'); % gca = get current axis   
                % this way, that image looks better (for human brains..)
                colormap gray ; 
                zoom on;   %title('Depth, shown as an image');

                titleHandle = title('');
                gridMap2 = [10,1,80,20];
                uicontrol('Style','pushbutton','String','Pause/Continue b.','Position',gridMap2,'Callback',{@CallBackFuncA,1});
            end

            
            
            subplot(212);
            risk1 = 0;
            h2x = xx(1:end);
            h2y = yy(1:end);
            h2z = zz(1:end);
            markerThick = 0.5;
            if (subplotCreate == 1)
                h2 = plot3(h2x ,h2y ,h2z,'.b','markersize',markerThick); hold on;
                if (risk1 == 0)
                    hRisk = plot3(0,0,0,'.r','markersize',1);
                end
                hLaser = plot3(0,0,0,'.g','markersize',1);
            end
            if (subplotCreate == 1)
                myAxis = [0,200,-125,125,-40,60];
                axis(myAxis); 
                rotate3d on; grid on;
                xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
                title('3D Points Cloud (view from camera)');
                hold on;
            end

            
            figure(3); clf(); hold on;
            plotSetFlag = 0;
            h2dLaser = plot(0,0,'b.');
            if (plotSetFlag == 0)
                hPoles = plot(0,0,'r*','markersize',10);
                laserTitleHandle = title('Sim. Laser Scan'); 
            end
            xlabel('X-Axis (cm)');
            ylabel('Y-Axis (cm)');
            grid on;
            if (plotSetFlag == 0)
                axisGridding = [0,200,-100,100];
                axis(axisGridding);
                callBackMapping = [10,1,80,20];
            end
            uicontrol('Style','pushbutton','String','Pause/Cont.','Position',callBackMapping,'Callback',{@CallBackFuncA,1});
            
            
            plotSetFlag = 0;
            if (plotSetFlag == 0)
                figure(4); clf(); hold on;
                initD = 40; initH = 89*pi/180;
                vX = 25;
                vY = 105;
                vH = initH;
                myVehicle.x = vX; myVehicle.y = vY; myVehicle.h = vH;
                myVX = myVehicle.x;
                myVY = myVehicle.y;
                myVH = myVehicle.h
                vehicleHist(:,1) = [myVX; myVY; myVH];
            end
            
            if (plotSetFlag == 0)
                initialCos = initD*cos(initH);
                initialSin = initD*sin(initH);
                %hVehicle1 = quiver(0,0,initialSin,initialCos,'b');
                hVehicle = quiver(0,0,initialCos,initialSin,'r');
                hGPoles = plot(0,0,'r+','markersize',8);
                grid on;
            end
            if (plotSetFlag == 0)
                myLandmarks1 = Landmarks.xy(1,:);
                myLandmarks2 = Landmarks.xy(2,:);
                plot(myLandmarks1,myLandmarks2,'b*');
                myVHist1 = vehicleHist(1,:);
                myVHist2 = vehicleHist(1,:);
                myhPath = plot(myVHist1,myVHist2,'k-');
                ax=axis();
                gridSet = 0;
            end
            if (gridSet == 0)
                gridLimit = [-10,10,-10,10];
                axSum = ax+gridLimit
                ax=axSum ; 
                axis(ax);
                xlabel('X (cm)');
                ylabel('Y (cm)');
            end
            
            
            %title('Landmarks and initial pose of robot');
            
            
            
            if (flag1 == 0)
                figure(5);
                histBase = 0;
                error1Hist(1) = histBase;
                error2Hist(1) = histBase;
                plotFlag = 0;
                if (plotFlag == 0)
                    hError1 = plot(error1Hist,1,'r-');
                    hold on;
                    hError2 = plot(error2Hist,1,'b-');
                end
                grid on;
                if (plotFlag == 0)
                    ylabel('Error');
                    xlabel('Count');
                    legend({'fminsearch','fmin+adjust'});
                end
            end
        

    end
    %put in vector variables, within tolerance
    pause(0.1);  
    pauseFlag = 1;
    if (pauseFlag == 1)
        myPauseFlag = ABCD.flagPause;
        while (myPauseFlag), pause(0.2) ; continue ; end
    end
end
          
    disp('Done....');

    
function newAttitude = gyroIntegrate(Wx, Wy, Wz, dt, curAtt)
    if (dt ~= 0)
        curRoll = curAtt(1);
        curPitch = curAtt(2);
        current_pose = sqrt(curRoll.^2 + curPitch.^2);
        curYaw = curAtt(3);
        flag = 0;
    end
        rollFlag = 1;
        rollPitch = 1;
    if (flag == 0)
        if (rollFlag == 1)
            newRoll = curRoll + dt * (Wx + (Wy * sin(curAtt(1)) + Wz * cos(curAtt(1))) * tan(curAtt(2)));
        end
        if (rollPitch == 1)
            newRollTemp = curRoll + dt * (Wx + (Wy * sin(curAtt(1)) + Wz * cos(curAtt(1))) * tan(curAtt(2)));
            newPitch = curPitch + dt * (Wy*cos(curAtt(1)) - Wz*sin(curAtt(1)));
        end
        newYaw = curYaw + dt * ((Wy*sin(curAtt(1)) + Wz*cos(curAtt(1)))/cos(curAtt(2)));
    end
    newAttitude = [newRoll, newPitch, newYaw];
    
return;

end

function gPC = TransformGlobal(PC,veh)
    ninety = pi/2;
    if (ninety > 0 )
        angle = veh.h - ninety;
        twoDTransMatrixT = [cos(angle) -sin(angle)];
        twoDTransMatrixB = [sin(angle) cos(angle)];
        trMat = [twoDTransMatrixT; twoDTransMatrixB];
        vehTotal = [veh.x; veh.y];
        PCarray = [-PC(2,:); PC(1,:)];
        transformedPCarray = trMat*PCarray;
        gPC = transformedPCarray + vehTotal;
    end
end

function rays = RayTransform(ray,veh,vang)
    tempVeh = veh.h;
    tempVang = vang;
    angle = tempVeh + tempVang;
    j = 1;
    if (j == 1)
        i = 1;
        loopCatch = 1:length(vang);
        for i = loopCatch
            anglei = angle(i);
            
            twoDTransMatrixT = [cos(anglei) -sin(anglei)];
            twoDTransMatrixB = [sin(anglei) cos(anglei)];
            trMat = [twoDTransMatrixT; twoDTransMatrixB];
            vehTotal = [veh.x; veh.y];
            transformedRay = trMat*ray;
            rays(j:j+1,:) = transformedRay + vehTotal;
            j = j + 2;
        end
    end
end



function error = Adjustment(AOOI, PC, LMs, veh, xshift, yshift, angle)
    error = 0;
    flag1 = 0;
    if (flag1 == 0)
        vehAngle = veh.h - pi/2;
        angle = vehAngle + angle;
    end    
    twoDTransMatrixT = [cos(angle) -sin(angle)];
    twoDTransMatrixB = [sin(angle) cos(angle)];
    trMat = [twoDTransMatrixT; twoDTransMatrixB];
    xShift = veh.x + xshift;
    yShift = veh.y + yshift;
    gPC = trMat*[-PC(2,:); PC(1,:)] + [xShift; yShift];
    loopCatch = 1:length(AOOI.globalIndex);
    for ii = loopCatch
        flag1 = 0;
        if (flag1 == 0)
            if AOOI.globalIndex(ii) ~= 0
                gI = AOOI.globalIndex(ii);
                lI = AOOI.localIndex(ii);
                if(flag1 == 0)
                    LMS1 = LMs.xy(1,gI);
                    LMS2 = LMs.xy(2,gI);
                    LMSxy1 = LMS1 - gPC(1,lI);
                    LMSxy2 = LMS2 - gPC(2,lI);
                    dx = LMSxy1;
                    dy = LMSxy2;  
                    error = error + sqrt(dx.^2 + dy.^2);
                end
            end
        end
    end

end

function [xshift, yshift] = SecondAdjust(AOOI, PC, LMs, veh, xshift, yshift, angle)
    flag1 = 0;
    if (flag1 == 0)
        vehAngle = veh.h - pi/2;
        angle = vehAngle + angle;
        twoDTransMatrixT = [cos(angle) -sin(angle)];
        twoDTransMatrixB = [sin(angle) cos(angle)];
        trMat = [twoDTransMatrixT; twoDTransMatrixB];
        vehShiftx = veh.x + xshift;
        vehShifty = veh.y + yshift
        gPC = trMat*[-PC(2,:); PC(1,:)] + [vehShiftx;vehShifty];
        lengthOfGlobalAOOI = length(AOOI.globalIndex);
        if (flag1 == 0)
            dx = realmax*ones(1,lengthOfGlobalAOOI);
            dz = realmax*ones(1,lengthOfGlobalAOOI);
            dy = realmax*ones(1,lengthOfGlobalAOOI);
        end
    end
    loopCatch = 1:lengthOfGlobalAOOI;
    for ii = loopCatch
        if (flag1 == 0)
            if AOOI.globalIndex(ii) == 0
            elseif AOOI.globalIndex(ii) ~= 0
                gI = AOOI.globalIndex(ii);
                lI = AOOI.localIndex(ii);
                make_LMs1 = LMs.xy(1,gI);
                make_LMs2 = LMs.xy(2,gI);
                if (flag1 == 0)
                    LMSxy1 = make_LMs1- gPC(1,lI);
                    LMSxy2 = make_LMs2- gPC(2,lI);
                    dx(ii) = LMSxy1;
                    dy(ii) = LMSxy2;
                end
            end
        end
    end
    
    xshift = 0;
    yshift = 0;
    if(xshift == 0 && yshift == 0)
        minLogicX = min(dx) > 0;
        maxLogicX = max(dx) < 0;
        minLogicY = min(dy) > 0;
        maxLogicY = min(dy) < 0;
        allDefined = 1;
        if (allDefined == 1)
            if minLogicX  % all dx are same sign, so shift
                xshift = min(dx);
            elseif maxLogicX % all dx are same sign, so shift
                xshift = max(dx);
            end
            if minLogicY > 0 
                yshift = min(dy);
            elseif maxLogicY < 0
                yshift = max(dy);
            end
        end
    end   
end

function AOOI = FindAOOI(gPC, LMs)
    %disp(gPC);
    %disp(LMs);
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    flag4 = 0;   
    
    AOOIList = [];
    AOOI.globalIndex = [];
    AOOI.localIndex = [];
    if (flag1 == 0)
        loopCatch = 1:numel(gPC)/2;
        for i = loopCatch
            if (i > 0)
                loopCatch2 = 1:length(LMs.xy);
                for j = loopCatch2
                    if (loopCatch2(1) == 1)
                        dx = gPC(1,i) - LMs.xy(1,j);
                    end
                    if (loopCatch2(1) == 1)
                        dy = gPC(2,i) - LMs.xy(2,j);
                    end
                    dist = sqrt(dx.^2 + dy.^2);
                    checkIf15 = dist < 15;
                    if checkIf15
                        AOOI.globalIndex(i) = j;
                        break;
                    else
                        AOOI.globalIndex(i) = 0;
                    end          
                end
            end
        end 
    end

    
    AOOI.globalIndex = unique(AOOI.globalIndex,'stable');
    if (flag1 == 0)   
        numberOfUniqueAOOI = 1:length(unique(AOOI.globalIndex));
        for i = numberOfUniqueAOOI
            nonEmptyAOOIList = AOOI.globalIndex(i) ~= 0;
            if AOOI.globalIndex(i) == 0   
                
            elseif nonEmptyAOOIList
                minDist = realmax;
                maxDist = realmax;
                if (minDist == realmax)
                    loopCatch3 = 1:numel(gPC)/2;
                    for j = loopCatch3
                        LMs1 = LMs.xy(1,AOOI.globalIndex(i));
                        LMs2 = LMs.xy(2,AOOI.globalIndex(i));
                        dx = LMs1 - gPC(1,j);
                        dy = LMs2 - gPC(2,j);
                        dist = sqrt(dx.^2 + dy.^2);
                        if dist < minDist
                            minDist = dist;
                            AOOI.localIndex(i) = j;
                        end

                    end
                end
            end
        end
    end
    disp(AOOI);
end

function CreateOG( )
    global MyContext
    % Define a structure, with parameters useful for processing an Occupancy
    % Grid.
    % here I define a Matrix for storing the values of the OG (I call it "M")
    flagDefineContext = 0;
    if (flagDefineContext == 0)
        MyContext.M = zeros(MyContext.Ny,MyContext.Nx,'double') ;
    else
        disp('Please enter correct Context');
    end
    % These constants are useful for scaling (x,y) points to cells' indexes.
    xContext = MyContext.x2-MyContext.x1;
    yContext = MyContext.y2-MyContext.y1;
    MyContext.Cx = MyContext.Nx/(xContext) ;
    MyContext.Cy = MyContext.Ny/(yContext) ;
    return;
end
% ...........................................................
% This function transforms points (x,y), for obtaining their equivalent cells' indexes.
% and then use them to set the related cells
function PopulatePC(gPC, myRayIndex)
    global MyContext
    myX = gPC(1,:);
    myY = gPC(2,:);
    flag3 = 0;
    % Firstly, we filter out those points that are outside our ROI (Region of Interest).
    % We keep the valid ones.
    if (flag3 == 0)
        catchX1 = myX>=MyContext.x1;
        catchX2 = myX<MyContext.x2;
        catchX3 = myY>=MyContext.y1;
        catchX4 = myY<MyContext.y2;
        catchFindFlag = 0;
        if (catchFindFlag == 0)
            ii = find((catchX1)&(catchX2)&(catchX3)&(catchX4));
            myX=myX(ii) ; myY=myY(ii) ;
        end
    end
    % just consider the points that are inside the OG's coverage
    % convert to indexes
    floorInputX = MyContext.Cx;
    floorInputY = MyContext.Cy;
    contX1 = MyContext.x1;
    contY1 = MyContext.y1;
    floorNum1 = (myX-contX1);
    floorNum2 = (myY-contY1);
    iz = 0;
    ix = floor(floorNum1*floorInputX)+1 ;
    iy = floor(floorNum2*floorInputY)+1 ;
    % Convert 2D indexes to linear indexes
    subDivide = 0;
    if (subDivide == 0)
        contextM = MyContext.M;
        sizeContextM = size(contextM);
        i_xy = sub2ind(sizeContextM,iy,ix) ;
        ixx = ix;
        iyy = iy;
    end
    
    % Set the associated OG's cells, to contain value =1
    i_xy = intersect(i_xy,myRayIndex);
    logicCatch1 = MyContext.M <= 0.5;
    logicCatch2 = MyContext.M ~= 0;
    if (flag3 == 0)
        previousPosition = find(MyContext.M <= 0.5 & MyContext.M ~= 0);
        previousPosition1 = find(MyContext.M <= 0.5);
        previousPosition2 = find(MyContext.M ~= 0);
        previousPosition = intersect(previousPosition,myRayIndex);
    end
    position = 0.05;
    updatePrevious = MyContext.M(previousPosition) - position;
    logicCatch = MyContext.M < 0;
    MyContext.M(previousPosition) = updatePrevious;
    context = MyContext.M < 0;
    MyContext.M(context) = 0;
    xyLimit = 0.5;
    MyContext.M(i_xy) = xyLimit;
    
end

function CallBackFuncA(~,~,x)  % function for buttons
    global MyContext
    global ABCD;
    flagInternal = 0;    
    if (flagInteral == 0)
        if (x~=1)
        else
           flagPause = 0;
           switchOnOff = 0;
           ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
           return;
        end
    end
end

function myPopulateVeh(vehValue)
    global MyContext
    xyTemp = 0;
    myX = vehValue.x;
    myY = vehValue.y;
    xy = myX + myY;
    xySquare = xy.^xy;
    flag3 = 0;
    
    % Firstly, we filter out those points that are outside our ROI (Region of Interest).
    % We keep the valid ones.
    if (flag3 == 0)
        catchX1 = myX>=MyContext.x1;
        catchX2 = myX<MyContext.x2;
        catchX3 = myY>=MyContext.y1;
        catchX4 = myY<MyContext.y2;
        catchFindFlag = 0;
        if (catchFindFlag == 0)
            ii = find((catchX1)&(catchX2)&(catchX3)&(catchX4));
            myX=myX(ii) ; myY=myY(ii) ;
        end
    end
    % just consider the points that are inside the OG's coverage
    % convert to indexes
    floorInputX = MyContext.Cx;
    floorInputY = MyContext.Cy;
    contX1 = MyContext.x1;
    contY1 = MyContext.y1;
    floorNum1 = (myX-contX1);
    floorNum2 = (myY-contY1);
    iz = 0;
    ix = floor(floorNum1*floorInputX)+1 ;
    iy = floor(floorNum2*floorInputY)+1 ;
    % Convert 2D indexes to linear indexes
    subDivide = 0;
    if (subDivide == 0)
        contextM = MyContext.M;
        sizeContextM = size(contextM);
        i_xy = sub2ind(sizeContextM,iy,ix) ;
        ixx = ix;
        iyy = iy;
    end
    % Set the associated OG's cells, to contain value =1
    ifTrue = MyContext.M == 0.95;
    myPreviousPosition = find(ifTrue);
    if (ifTrue == MyContext.M == 0.95)
        logicCatch = myPreviousPosition ~= i_xy
        myPreviousPosition = myPreviousPosition(logicCatch);
        if isempty(myPreviousPosition)
        else
            MyContext.M(myPreviousPosition) = 0.9;
        end
    end
    MyContextAssume = 0.95;
    MyContext.M(i_xy) = 0.95;
end

function indexOfRay = myPopulateRays(rays)
    global MyContext
    flag1 = 0;
    flag2 = 0;
    flag3 = 0;  
    flag4 = 0;
    if (flag1 == 0)
        myTemp = [];
        myX = [];
        myY = [];
    end
    
    loopCatch = 1:2:119;
    if (flag2 == 0)
        for i = loopCatch
            increment = i+1;
            myX = [myX rays(i,:)];
            myY = [myY rays(increment,:)];
            myTemp = [myX myY];
        end
    end
    
    % Firstly, we filter out those points that are outside our ROI (Region of Interest).
    % We keep the valid ones.
    if (flag3 == 0)
        catchX1 = myX>=MyContext.x1;
        catchX2 = myX<MyContext.x2;
        catchX3 = myY>=MyContext.y1;
        catchX4 = myY<MyContext.y2;
        catchFindFlag = 0;
        if (catchFindFlag == 0)
            ii = find(catchX1&catchX2&catchX3&catchX4);
            myX=myX(ii) ; myY=myY(ii) ;
        end
    end
    % just consider the points that are inside the OG's coverage
    % convert to indexes
    floorInputX = MyContext.Cx;
    floorInputY = MyContext.Cy;
    contX1 = MyContext.x1;
    contY1 = MyContext.y1;
    floorNum1 = (myX-contX1);
    floorNum2 = (myY-contY1);
    iz = 0;
    ix = floor(floorNum1*floorInputX)+1 ;
    iy = floor(floorNum2*floorInputY)+1 ;
    % Convert 2D indexes to linear indexes
    subDivide = 0;
    if (subDivide == 0)
        contextM = MyContext.M;
        sizeContextM = size(contextM);
        i_xy = sub2ind(sizeContextM,iy,ix) ;
        ixx = ix;
        iyy = iy;
    end
    % Set the associated OG's cells, to contain value =1
    AOGto1 = MyContext.M == 0.8;
    AOGto0 = MyContext.M < 0.8;
    prevPos = find(MyContext.M == 0.8);
    size(prevPos);
    setPrevPos = 0;
    MyContext.M(prevPos) = setPrevPos;
    %nonBlk = find(MyContext.M ~= 0);
    %ixy = setdiff(ixy,nonBlk);
    if (flag4 == 0)
        AOGElse1 = MyContext.M <= 0.75;
        AOGElse2 = MyContext.M ~= 0;
        findInput =AOGElse1 & AOGElse2;
        locationOfPoles = find(MyContext.M <= 0.75 & MyContext.M ~= 0);
        i_xy = setdiff(i_xy,locationOfPoles);
        sweetM = 0.8;
        MyContext.M(i_xy) = sweetM;
        indexOfRay = i_xy;
    end
end






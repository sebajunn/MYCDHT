%
%Kalman Filter
%Cameron Murray Z3417671

function [] = lab_4()

global flag;
flag = 0;

time_conv_factor = 10000;

title_flag = 0;
trace_flag = 0;
bias_flag = 0;
plot_flag = 0;
start_index = 1;
if_no_local_OOI = 0;
sdev_assigned = 0;
state_0 = 0;
p_0 = 0;
kalman_gain_flag = 0;
update_flag = 0 ;

IMU_data = load('DataForProject02/IMU_dataC.mat');
IMU_times = double(IMU_data.IMU.times);
IMU_times = IMU_times/time_conv_factor;

if (start_index ~= 0)
    IMU_times = IMU_times - IMU_times(1);
end

% 2D change of attitude representation
IMU_omega = IMU_data.IMU.DATAf(4:6,:)';
IMU_omega(:,3) = -1*IMU_omega(:,3);

laser_data = load('DataForProject02/Laser__2C.mat');
laser_times = double(laser_data.dataL.times);
laser_times = laser_times/time_conv_factor;
if (start_index ~= 0)
    laser_times = laser_times - laser_times(1);
end
laser_scans = laser_data.dataL.Scans;

speed_data = load('DataForProject02/Speed_dataC.mat');
velocity = speed_data.Vel.speeds;

% Kalman filter parameters
if (state_0 == 0)
    Xdr = [0 0 pi/2 0];
    Xe = [0 0 pi/2 0];        
end

if (p_0 == 0)
    P = zeros(4,4);
    P(4,4) = (4*pi/180)^2;
end

if (sdev_assigned == 0)
    stdDevGyro = 1.5*pi/180 ;        
    stdDevSpeed = 0.05 ; 
    sdev_rangeMeasurement = 0.15;
    sdev_angleMeasurement = 1*pi/180; 
end

% Begin configuration initialization

handles = initialize_plots(IMU_times);

%Extract linear trend to estimate IMU data
start_time = IMU_times - IMU_times(1) < 20;
start_index = find(start_time,1,'last');
range_of_omega = IMU_omega(1:start_index,3);
estimated_omega_offset = mean(range_of_omega);


% Data fusion
% Initialize and plot starting positions of OOIs
% Process first scan

range_of_scans = laser_scans(:,1);
[range_0, intensity_0] = extract_laser_data(range_of_scans);
plot_scan(range_0, intensity_0, Xe, handles);
OOI_global_list = ExtractOOIs(range_0, intensity_0);


global_x = OOI_global_list.Centers(1,:)';
global_y = OOI_global_list.Centers(2,:)';

if (flag == 0)
    [OOI_global_list.Centers(1,:),OOI_global_list.Centers(2,:)] = transform_position(global_x,global_y, Xe);
end

global_x = OOI_global_list.Centers(1,:);
global_y = OOI_global_list.Centers(2,:);
                           
set(handles.object_global_centers,'xdata',global_x);
set(handles.object_global_centers,'ydata',global_y);
                              
for i = 1:OOI_global_list.N
    global_positions = [OOI_global_list.Centers(1,i)+0.2,OOI_global_list.Centers(2,i)];
    set(handles.object_global_labels(i),'position',global_positions);
    set(handles.object_global_labels(i),'String',['#',num2str(i)]);
end

%Initialize
if (if_no_local_OOI == 0)
    local_OOI_list = OOI_global_list;    
end
                              
% Iterate through data set, using IMU times as clock
current_scan = 1;
N = length(IMU_times)-1;

%more EKF parameters
q_1 = (0.01)^2;
q_2 = (0.01)^2;
q_3 = (0.05*pi/180)^2;

Q_1 = diag([q_1,q_2,q_3,0]);
P_u = diag([0 stdDevSpeed]);

Xe_History= zeros(4,length(IMU_times)) ;

for i = 2:N;

    %Dt = IMU_times(i) - IMU_times(i-1);
    
    %Process new IMU data
    
    if (flag == 0)
        MeasuredBearings = []; 
        MeasuredRanges = [];     
        ObservedLandmarks = [];
    end
    
    % process new laser scan if it arrives
    % process a laser scan when IMU_times(i) is the time right after a
    % laser scan  | |.| | |.| | |.|<----- this one here is when we process
    % the laser data
    
    
    chosen_laser_scan = IMU_times(i) > laser_times(current_scan) % 
    is_valid_scan = current_scan < length(laser_times);
    num_measurements = 0;
    
    if (chosen_laser_scan && is_valid_scan)
        current_scan = current_scan + 1; %process the laser scan that is right behind the time 
        current_extract = laser_scans(:,current_scan);
        
        [range_i, intensity_i] = extract_laser_data(current_extract);

        local_OOI_list = ExtractOOIs(range_i,intensity_i);

        local_centers_x = local_OOI_list.Centers(1,:)';
        local_centers_y = local_OOI_list.Centers(2,:)';
        
        if (flag == 0)
            [adjusted_centers_X,adjusted_centers_Y] = transform_position(local_centers_x, local_centers_y, Xe);
        end

        plot_scan(range_i, intensity_i, Xe, handles);
        
        if (plot_flag == 0)
            set(handles.object_local_centers, 'xdata',adjusted_centers_X);
        end
        if (plot_flag == 0)
            set(handles.object_local_centers, 'ydata',adjusted_centers_Y);
        end

    %Associate objects

        %Shouldn't just be inventing this attribute here
        if (flag == 0)
            global_list_vector = zeros(local_OOI_list.N, 1);
            local_OOI_list.global_ID = global_list_vector;
        end


        for j = 1:local_OOI_list.N
            x_glob_centers = OOI_global_list.Centers(1,:);
            y_glob_centers = OOI_global_list.Centers(2,:);

            if (flag == 0)
                x_dists = x_glob_centers - adjusted_centers_X(j);
                y_dists = y_glob_centers - adjusted_centers_Y(j);
            end

            %Euclidian distance sqrt(x_dists.^2 + y_dists.^2)
            [mindist,mini] = min(sqrt(x_dists.^2 + y_dists.^2));  
            flag_string_label = 0;
            if mindist < 0.4

                adjusted_centers = [adjusted_centers_X(j)-1,adjusted_centers_Y(j)-0.5];
                set(handles.object_local_labels(j), 'position',adjusted_centers);

                if (flag_string_label == 0)
                    set(handles.object_local_labels(j), 'String',['Best match: #',num2str(mini), ', Error: ', num2str(mindist), ' m']);
                end

                    local_OOI_list.global_ID(j) = mini;
                if (flag == 0)     
                    my_measured_ranges = sqrt(local_OOI_list.Centers(1,j)^2 + local_OOI_list.Centers(2,j)^2);
                    MeasuredRanges = [MeasuredRanges, my_measured_ranges];
                end

                if (flag == 0)
                    my_measured_bearings = atan2(local_OOI_list.Centers(2,j), local_OOI_list.Centers(1,j));
                    MeasuredBearings = [MeasuredBearings, my_measured_bearings];
                end

                if (flag == 0)
                    ObservedLandmarks = [ObservedLandmarks, local_OOI_list.global_ID(j)];   
                end
                    num_measurements = num_measurements + 1;

            else
                if (flag_string_label == 0)
                    set(handles.object_local_labels(j),'position',[0,0]);
                    set(handles.object_local_labels(j),'String','');
                end
            local_OOI_list.global_ID(j) = -1;
            end

        end

        loop_catch = local_OOI_list.N+1:length(handles.object_local_labels);
        for j = loop_catch
              set(handles.object_local_labels(j),'position',[0,0]);
              set(handles.object_local_labels(j),'String','');
        end

    end
        
    
    
    
    
    
    
    
    %Compute new P matrix and new predicted value
    Dt = IMU_times(i) - IMU_times(i-1);
    imu_gyro = double(IMU_omega(i,3));
    encoder_speed = double(velocity(i));
    
    j_1 = [1,0,-Dt*encoder_speed*sin(Xe(3)), 0];
    j_2 = [0,1,Dt*encoder_speed*cos(Xe(3)),0];
    j_3 = [ 0,0,1,-1*Dt];
    j_4 = [0,0,0,1];
    
    J = [j_1 ;j_2 ;j_3; j_4] ;

    
    
    f_1 = [encoder_speed*cos(Xe(3)) 0];
    f_2 = [encoder_speed*sin(Xe(3)) 0];
    f_3 = [0 1];
    f_4 = [0, 0];
    
    F_u = [f_1; f_2; f_3; f_4];
    F_u = Dt*F_u;

    % MODIFIED: added J*Q_u*J' term to include input uncertainty
    P = J*P*J'+ (Q_1 + F_u*P_u*F_u'); 
    % predicted/expected update
    if (flag == 0)
        Xe = int_state(imu_gyro, encoder_speed, Dt, Xe)';  
    end

    %Compute updated range/angle readings for any landmarks that have been 
    %detected and associated. Perform an EKF update on these measurements
    M = num_measurements;
    for j = 1:M
        tick = j;
	    disp(local_OOI_list.global_ID(j))

        % OOI_global_list.Centers(2,ObservedLandmarks(j)) = landmarks x & y
        
	    eDX = (OOI_global_list.Centers(1,ObservedLandmarks(j))-Xe(1)) ;      % (xu-x), 
	    eDY = (OOI_global_list.Centers(2,ObservedLandmarks(j))-Xe(2)) ;      % (yu-y)
	    eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
	    % New 2D measurement matrix:
        h_1 = [  -eDX/(sqrt( eDX*eDX + eDY*eDY )) , -eDY/(sqrt( eDX*eDX + eDY*eDY )) , 0 , 0];
        h_2 = [eDY/((sqrt( eDX*eDX + eDY*eDY ))^2), -eDX/((sqrt( eDX*eDX + eDY*eDY ))^2), -1, 0];
        
	    H = [h_1;h_2];
		     
        % Jacobian of h(X); size 2x4

	    ExpectedRange = eDD ;  
	    ExpectedBearing = atan2(eDY,eDX) - Xe(3)
        ExpectedBearing = wrapToPi(ExpectedBearing  + pi/2);
        
        range_difference = MeasuredRanges(j) - ExpectedRange;
        bearing_difference = MeasuredBearings(j) - ExpectedBearing;
        
	    z = [range_difference; bearing_difference]; 

	    %disp('Measured bearings')

	    % ------ covariance of the noise/uncetainty in the measurements
        sdev_range_square = sdev_rangeMeasurement*sdev_rangeMeasurement;
        sdev_angle_square = sdev_angleMeasurement*sdev_angleMeasurement
	    R = diag([sdev_range_square, sdev_angle_square]);
        
        if (kalman_gain_flag == 0)
            S = R;
            S = S + H*P*H' ;
            iS=inv(S);
            K = P*H'*iS;            % Kalman gain
        end

        % we complete EKF update to obtain X(k+1|k+1) and P(k+1|k+1)
        expected_update = K*z
        covariance_update = P*H'*iS*H*P;
        
        Xe = Xe+expected_update            % update the  expected value         
        P = P-covariance_update ;     % update the Covariance
       
    end

    Xe_History(:,i) = Xe;
    XeDR_History = Xdr;
    
    s = sprintf('Processed: IMU scan %d, Laser scan %d', i, current_scan);
    
    if (title_flag == 0)
        set(handles.object_title, 'string', s);
    end
    if (trace_flag == 0)
        set(handles.kalman_trace,'xdata',Xe_History(1,1:i));
        set(handles.kalman_trace,'ydata',Xe_History(2,1:i));
    end
    if (bias_flag == 0)
        set(handles.bias_kalman,'xdata',IMU_times(1:i),...
            'ydata', Xe_History(4,1:i))
    end
    if (bias_flag == 0)
        set(handles.bias_initial,'xdata',IMU_times(1:i));
        set(handles.bias_initial,'ydata', estimated_omega_offset*ones(1,i));     
    end
    
    pause(0.0001);
end

end








function new_state = int_state(omega, speed, dt, old_state)
        new_state = zeros(1,3);
        phi = old_state(3);
        x = old_state(1);
        y = old_state(2);
        b = old_state(4);
        
		new_state(1) = speed*cos(phi)*dt + x; 
		new_state(2) = speed*sin(phi)*dt + y; 
		new_state(3) = (omega-old_state(4))*dt + phi; 
        new_state(4) = b;
end

function [ranges, intensity] = extract_laser_data(scan)
         mask_flag = 0;
         mask_l_13 = uint16(2^13-1);
         ranA = bitand(scan,mask_l_13);
         double_ranA = double(ranA);
         multiplier = 0.01;
         ranges = multiplier*double_ranA;

         %Extract intensity data
         if (mask_flag == 0)
            maskE000 = bitshift(uint16(7),13);
            intensity = bitand(scan, maskE000);
         end
end

function plot_scan(ranges, intensity, state, handles)
        ang = [0:360]';
        radToDeg = pi/180;
        ANG = ang*0.5*radToDeg;
        X_position = -cos(ANG).*ranges;
        Y_position_before = sin(ANG).*ranges;
        Y_position =  Y_position_before + 0.46;
        data_flag = 0;
        bright_flag = 0;
        [X_transformed,Y_transformed] = transform_position(X_position,Y_position,state);       
        
        if (data_flag == 0)
            set(handles.object_data_points,'xdata',X_transformed,'ydata',Y_transformed);
        end
        if (bright_flag == 0)
            set(handles.object_bright_points,'xdata',X_transformed(find(intensity~=0)),...
                'ydata',Y_transformed(find(intensity~=0)));      
        end
end

function [transformed_X, transformed_Y] = transform_position(X, Y, state)
       rot_input = state(3)*180/pi - 90;
       x = [0 0 state(1)];
       y = [0 0 state(2)];
       z = [0 0 0];
       homo_3 = ones(1,length(X));
       transform_matrix = rotz(rot_input) + [x;y;z];
       untouched_coords = [X'; Y'; homo_3];
       transformed_coords = transform_matrix*untouched_coords;
       transformed_X = transformed_coords(1,:);
       transformed_Y = transformed_coords(2,:);
end


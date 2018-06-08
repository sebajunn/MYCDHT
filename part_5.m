%Cameron Murray Z3417671

function [] = lab_4_2()
% Kalman filter parameters

global flag;
flag = 0;

time_conv_factor = 10000;

initialize = 0;
title_flag = 0;
trace_flag = 0;
bias_flag = 0;
plot_flag = 0;
start_index = 1;
if_no_local_OOI = 0;
sdev_assigned = 0;
state_0 = 0;
p_0 = 0;
range_flag = 0;
kalman_gain_flag = 0;
update_flag = 0 ;
kalman_para_set = 0;
update_state_values = 0;

% Kalman filter parameters
if (state_0 == 0)
    Xe = [0 0 pi/2];        
end

Xe_History = zeros(3) ;

if (p_0 == 0)
    P = zeros(3,3);
end
if (sdev_assigned == 0)
    stdDevGyro = 1.5*pi/180 ;        
    stdDevSpeed = 0.05 ; 
    sdev_rangeMeasurement = 0.15;
    sdev_angleMeasurement = 1*pi/180; 
end

q_1 = (0.01)^2;
q_2 = (0.01)^2;
q_3 = (0.05*pi/180)^2;

Q = diag([q_1,q_2,q_3]);
Q_u = diag([stdDevGyro stdDevSpeed]);




%Extract linear trend to estimate IMU data

% Data fusion
               
                              
% Iterate through data set, using IMU times as clock
current_scan = 1;
if (kalman_para_set == 0)
    OOI_global_list = [];
    times = [];
    time = 0;     
    velocity = 0;
    imu_gyro = 0;
    num_measurements=0;
end

% Begin configuration initialization

handles = initialize_plots_live;

udp_receiver = dsp.UDPReceiver('RemoteIPAddress','127.0.0.1',...
                               'LocalIPPort',1112,...
                               'MaximumMessageLength',1024,...
                               'MessageDataType','uint8');
                           
step(udp_receiver);



while (1)
    
    
    buf_size = -1;
    %Fetch a UDP packet
    buf = step(udp_receiver);
    if(isempty(buf))
        empty_buffer = -1;
        buf_size = empty_buffer;
    elseif (~isempty(buf))
        buffer_5_6 = typecast(buf(5:6),'uint16');
        buf_size = buffer_5_6;
    end
    got_frame = 0;
    prev_time = time;
    
    if buf_size == 8
        buffer_9_12 = buf(9:12);
        buffer_13_14 = buf(13:14);
        buffer_15_16 = buf(15:16);
        fprintf('IMU Packet\n');
        
       % time = double(typecast(buffer_9_12, 'uint32'))*1e-4;
       % velocity = double(typecast( buffer_13_14, 'int16'))*1e-3;
       % imu_gyro = -1*double(typecast(buffer_15_16, 'int16'))*0.02*pi/180;
        
        if (update_state_values == 0)
            time = double(typecast(buffer_9_12, 'uint32'))*1e-4;
        end
        if (update_state_values == 0)
            velocity = double(typecast( buffer_13_14, 'int16'))*1e-3;
        end
        if (update_state_values == 0)
            imu_gyro = -1*double(typecast(buffer_15_16, 'int16'))*0.02*pi/180;
        end
        if (initialize == 0) 
            MeasuredRanges = [];
            ObservedLandmarks = [];
            MeasuredBearings = [];   
        end
        num_measurements = 0;
        
        if(~isempty(times))
        elseif(isempty(times))
             times = time;
        end
        increment = 1;
        got_frame = increment;
        
    elseif buf_size == 736
        got_frame = 1;
        fprintf('LIDAR Packet\n');
        buffer_9_12_32 = buf(9:12);
        buffer_15_736 = buf(15:736);
        if (state_0 == 0)
            time = double(typecast(buffer_9_12_32, 'uint32'))*1e-4;
            current_scan = typecast(buffer_15_736,'uint16');
        end
        if (state_0 == 0)
        [range_i, intensity_i] = extract_laser_data(current_scan);
        end
        intensity_flag = 0;
        if (isempty(OOI_global_list))
                    % Initialize and plot starting positions of OOIs
                    % Process first scan
                    if (range_flag == 0)
                        range_0 = range_i;
                    end
                    if (intensity_flag == 0)
                        intensity_0 = intensity_i;    
                    end
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
        else
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
        end

            loop_catch = local_OOI_list.N+1:length(handles.object_local_labels);
            for j = loop_catch
                  set(handles.object_local_labels(j),'position',[0,0]);
                  set(handles.object_local_labels(j),'String','');
            end
        
            
   % elseif buf_size > 0
    %    fprintf('Bad buffer size, %d bytes\n', buf_size);
    %    continue;
    else
        continue;
    end

    %if(~got_frame)
    %                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    %   disp(skipped)
     %  continue; 
   % end
    
    
   %Dt = double(time)
    Dt = double(time) -double(times(end));
    times = [times, double(time)];

    j_1 = [1,0,-Dt*velocity*sin(Xe(3))];
    j_2 = [0,1,Dt*velocity*cos(Xe(3))];
    j_3 = [0,0,1];
    
    J = [j_1 ;j_2 ;j_3] ;

    f_1 = [velocity*cos(Xe(3)) 0];
    f_2 = [velocity*sin(Xe(3)) 0];
    f_3 = [0 1];
    
    F_u = [f_1; f_2; f_3];
    F_u = Dt*F_u;

    %include input uncertainty
    P = J*P*J'+ (Q + F_u*Q_u*F_u'); 
    % predicted/expected update
    if (flag == 0)
        Xe = int_state(imu_gyro, velocity, Dt, Xe)';  
    end

    %Compute updated range/angle readings for any landmarks that have been 
    %detected and associated. Perform an EKF update on these measurements
    M = num_measurements;
 
    for j = 1:M
        
        tick = j;
	   % disp(local_OOI_list.global_ID(j))

        % OOI_global_list.Centers(2,ObservedLandmarks(j)) = landmarks x & y
        
	    eDX = (OOI_global_list.Centers(1,ObservedLandmarks(j))-Xe(1)) ;      % (xu-x), 
	    eDY = (OOI_global_list.Centers(2,ObservedLandmarks(j))-Xe(2)) ;      % (yu-y)
	    eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
	    % New 2D measurement matrix:
        h_1 = [  -eDX/(sqrt( eDX*eDX + eDY*eDY )) , -eDY/(sqrt( eDX*eDX + eDY*eDY )) , 0 ];
        h_2 = [eDY/((sqrt( eDX*eDX + eDY*eDY ))^2), -eDX/((sqrt( eDX*eDX + eDY*eDY ))^2), -1];
        
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
        sdev_angle_square = sdev_angleMeasurement*sdev_angleMeasurement;
	    R = diag([4*sdev_range_square, 4*sdev_angle_square]);
        
        if (kalman_gain_flag == 0)
            S = R;
            S = S + H*P*H' ;
            iS=inv(S);
            K = P*H'*iS;            % Kalman gain
        end

        % we complete EKF update to obtain X(k+1|k+1) and P(k+1|k+1)
        expected_update = K*z
        covariance_update = P*H'*iS*H*P;
        
        Xe = Xe+expected_update;            % update the  expected value 
        P = P-covariance_update ;     % update the Covariance
       
    end

    Xe_History = [Xe_History, Xe]
    
    if (trace_flag == 0)
        set(handles.vehicle_trace,'xdata',Xe(1,:));
        set(handles.vehicle_trace,'ydata',Xe(2,:));
    end
    if (trace_flag == 0)
        set(handles.kalman_trace,'xdata',Xe_History(1,:));
        set(handles.kalman_trace,'ydata',Xe_History(2,:));
    end
    drawnow;
end

end

function new_state = int_state(omega, speed, dt, old_state)
        new_state = zeros(1,3);
        phi = old_state(3);
        x = old_state(1);
        y = old_state(2);
        
		new_state(1) = speed*cos(phi)*dt + x; 
		new_state(2) = speed*sin(phi)*dt + y; 
		new_state(3) = (omega)*dt + phi; 
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


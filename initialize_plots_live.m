%Author: Cameron Murray Z3417671
%Program: Utility function to initalize plot handles for Part 3

function [handles] = initialize_plots_live()
    initialize_flag = 0;
    subplot_flag = 0;
    title_flag = 0;
    label_flag = 0;
    center_flag = 0;
    point_flag = 0;
    trace_flag = 0;
    bias_flag = 0;
    
    limit = [-10,10];
    %{
    if ( initialize_flag == 0 )
        handles.master_fig = figure(1);
        clf; hold on;
        if (subplot_flag == 0)
            handles.att_subplot = subplot(121);
        end
        if (trace_flag == 0)
            handles.att_trace = plot(0,0,'b-');
        end
        if (title_flag == 0)
            handles.att_title = title('Vehicle Attitude vs Time');
        end
        xlabel('Time (s)');
        ylabel('Attitude (deg.)');
        grid on;
        y_range = [0,500]
        ylim(y_range);

        if (subplot_flag == 0)
            handles.xy_subplot = subplot(122);
        end
        if (trace_flag == 0)
            handles.xy_trace = plot(0,0,'b-');
        end
        if (title_flag == 0)
            handles.xy_title = title('Vehicle XY Position (Dead Reckoning)');
        end
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        grid on;
    end
  %}
    if ( initialize_flag == 0 )
        
        empty_vector = zeros(1,10);
        handles.object_figure = figure(2);
        clf; hold on;
        
        if (title_flag == 0)
            handles.object_title = title('OOI Plot');
        end
        if (label_flag == 0)
            handles.object_global_labels = text(empty_vector,empty_vector,'','Color','green');
            handles.object_local_labels = text(empty_vector,empty_vector,'','Color','magenta');
        end
        if (center_flag == 0)
            handles.object_global_centers = plot(0,0,'g *');   
            handles.object_local_centers = plot(0,0,'m *');
        end
        if (point_flag == 0)
            handles.object_data_points = plot(0,0,'k. ');
            handles.object_bright_points = plot(0,0,'rx ');
        end
        if (trace_flag == 0)
            handles.kalman_trace = plot(0,0,'r- ');
            handles.vehicle_trace = plot(0,0,'b- ');  
        end
        
        axis equal;
        axis manual;
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        xlim(limit);
        ylim(limit);
        grid on;
        
    end
    
    if ( initialize_flag == 0 )
        handles.bias_figure = figure(3);
        clf; hold on;
        done = 1;
        %if (bias_flag == 0)
            handles.bias_title = title('Gyroscope bias estimate');
            bias_plot = done;
            handles.bias_initial = plot(0,0,'b-')
            bias_plot = done;
            handles.bias_kalman = plot(0,0,'r-')
       % end
        xlabel('Time (s)');
        ylabel('Bias estimate (rad s^-1)');
    end 

    handles.velocity_figure = figure(4);
    clf; hold on;
    handles.velocity_title = title('Longitudinal Velocity');
    handles.velocity_real = plot(0,0,'b-')
    handles.velocity_EKF = plot(0,0,'r-')
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
end
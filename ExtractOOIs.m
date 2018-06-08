%Cameron Murray, z3417671
%Extract OOI from LIDAR
%scans.

function [ooi_list] = ExtractOOIs(ranges, intensity)

    deg_to_pi = pi/180;
    angle_range = [0:360]';
    my_angles = angle_range*0.5*deg_to_pi;

    X_pos = -cos(my_angles).*ranges;
    Y_pos = sin(my_angles).*ranges + 0.46;
    
    X_pos_input = diff(X_pos).^2;
    Y_pos_input = diff(Y_pos).^2;
    
    position_logic = (find(sqrt(X_pos_input+Y_pos_input) > 0.2))';
    
    %Find the gaps between the edges of the points
    object_edges = [0 position_logic];

    %if the threshold is breached (that is the distance between two points
    %then we create a new objects and assign a center, reflective check,
    %and a size
    no_objects = length(object_edges);
    num_obs = no_objects-1;
    temp_object_list = num_obs;
    empty_vector_1D = zeros(1, num_obs);
    empty_vector_2D = zeros(2, num_obs);
    ooi_list = struct('invCenters', empty_vector_2D, ...
                      'N',num_obs, ...
                      'Centers',empty_vector_2D, ...
                      'Intensity', empty_vector_1D, ...
                      'Diameters',empty_vector_1D, ...
                      'Color',empty_vector_1D);

    for i = 1:ooi_list.N
        pos_edges = object_edges(i)+1:object_edges(i+1);
        
        object_X = X_pos(pos_edges);
        object_Y = Y_pos(pos_edges);

        object_X_mean = mean(object_X);
        object_Y_mean = mean(object_Y);
        
        
        %Use the mean of the X,Y points as the centre
        ooi_list.Centers(:,i) = [object_X_mean,object_Y_mean];

        %Find Eucl. difference for each of the points, from the 1st to the
        %Nth point, if any point lies outside the indicated distance limits
        %(0.05 < x < 0.3 then we reject it
        
        range_X = range(object_X);
        range_Y = range(object_Y);
        
        cluster_diameter = norm(range(object_X),range(object_Y));
        ooi_list.Diameters(i) = cluster_diameter;
        greater_than_diameter_max = ooi_list.Diameters(i) > 0.3;
        greater_than_diameter_min = ooi_list.Diameters(i) < 0.05;
        remove_object = -1;
        
        if (greater_than_diameter_max || greater_than_diameter_min)
            remove_this_object = remove_object;
            ooi_list.Diameters(i) = remove_this_object; 
        end

        %Check if any pixels are reflective
        pixel_is_bright = max(intensity(pos_edges) > 0);
        bright = 1;
        dull = 0;
        
        if ~(pixel_is_bright)
            ooi_list.Color(i) = dull;
            ooi_list.Diameters(i) = remove_object;
        else
            ooi_list.Color(i) = bright;
        end

    end	
    objects_that_we_dont_consider = find(ooi_list.Diameters < 0);
    
    % Clean up invalid objects
    ooi_list.Color(objects_that_we_dont_consider) = [];
    ooi_list.Centers(:,objects_that_we_dont_consider) = [];
    ooi_list.Diameters(objects_that_we_dont_consider) = [];
    
    number_of_remaning_objects = length(ooi_list.Diameters);
    ooi_list.N = number_of_remaning_objects;
end

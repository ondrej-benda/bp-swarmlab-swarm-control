%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if DRONE_TYPE == "quadcopter" || DRONE_TYPE == "point_mass"
    map.width =300; % the map is of size (width)x(width)
    map.max_height = 100; % maximum height of trees
elseif DRONE_TYPE == "fixed_wing"
    map.width = 4000; % the map is of size (width)x(width)
    map.max_height = 100; % maximum height of trees
end

if exist('ACTIVE_ENVIRONMENT', 'var')
    map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT; % for those functions calling map
end
    
if ~exist('ACTIVE_ENVIRONMENT', 'var') || ACTIVE_ENVIRONMENT == false     
    return 
end

map.ACTIVE_GASS=true;
map.gass_center=[100;100];
map.gass_radius=map.width/3;
map.gass_matrix=create_gass(map.gass_radius);
if RANDOM_GASS_LOCATION
    if RNG_LOCK
        rng(RNG_LOCK);
    end
    map.gass_center=[map.width*rand();map.width*round(rand())];
end

if exist('ACTIVE_ENVIRONMENT', 'var') && ACTIVE_ENVIRONMENT == true
    map.bl_corner_north = 0;
    map.bl_corner_east = 0;
    
    map.nb_blocks = 50; %the number of blocks
    if ~RANDOM_BUILDINGS
        map.nb_blocks=round(sqrt(map.nb_blocks));
    end
    
        
    
    
    %Nastavení velikosti bloku
    
    
    map.street_width_perc = 0.5; % puvodni 0.5, percentage of block that is empty
    map.building_width = map.width/map.nb_blocks*(1-map.street_width_perc);
    map.street_width = map.width/map.nb_blocks*map.street_width_perc;
    
    map.building_width_range = [map.width/20,map.width/10]';%30-60
    map.building_region = [0,0;map.width,map.width];
    map.min_dst_between_objects = 10;%50

    map.building_shape = 'cylinder';
%     map.building_shape = 'parallelepiped';

    %Create buildings parameters
    if exist('RANDOM_BUILDINGS','var')
        if RANDOM_BUILDINGS
            map = create_rnd_buildings(map,RNG_LOCK);
        else
%         map = create_shifted_buildings(map);
        map = create_buildings(map);
        end
    else
%         map = create_shifted_buildings(map);
        map = create_buildings(map);
    end
    
    
end


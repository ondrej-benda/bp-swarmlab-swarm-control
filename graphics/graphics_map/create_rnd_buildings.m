function map = create_rnd_buildings(map,RNG_LOCK)
% CREATE_BUILDINGS - Create the city buildings in grid. The heights are
% random.
%
% Inputs:
%   map: structure of map parameters
%  
% Outputs:
%   map: structure of map parameters
%
    
%     map.buildings_heights = map.max_height*rand(map.nb_blocks*map.nb_blocks,1);
%     
%     for i=1:map.nb_blocks
%         buildings_north(i) = [0.5*map.width/map.nb_blocks*(2*(i-1)+1)];
%     end
%     map.buildings_north = repmat(buildings_north', map.nb_blocks, 1);
%     map.buildings_east  = repmat(buildings_north, map.nb_blocks, 1);
%     map.buildings_east  = map.buildings_east(:);
      if RNG_LOCK
          rng(RNG_LOCK);
      end
      map.buildings_heights = map.max_height*ones(map.nb_blocks,1);
      nb_blocks=map.nb_blocks;
      map.building_width = rand(nb_blocks,1)*(map.building_width_range(2)-map.building_width_range(1))+map.building_width_range(1);
      map.buildings_north=[rand()*(map.building_region(2,1)-map.building_region(1,1))+map.building_region(1,1)];
      map.buildings_east=[rand()*(map.building_region(2,2)-map.building_region(1,2))+map.building_region(1,2)];
      for objekt = 2:nb_blocks
          while true
              temp_north = rand()*(map.building_region(2,1)-map.building_region(1,1))+map.building_region(1,1);
              temp_east = rand()*(map.building_region(2,2)-map.building_region(1,2))+map.building_region(1,2);
              not_crossing=true;
              for objekt2 = 1:length(map.buildings_north)
                  if norm([map.buildings_north(objekt2),map.buildings_east(objekt2)]-[temp_north,temp_east])<(map.building_width(objekt)/2+map.building_width(objekt2)/2+map.min_dst_between_objects)
                      not_crossing=false;
                      break;
                  end
              end
              if not_crossing
                  map.buildings_north=[map.buildings_north,temp_north];
                  map.buildings_east=[map.buildings_east,temp_east];
                  break;
              end
          end
      end
      map.buildings_north=[map.buildings_north]';
      map.buildings_east=[map.buildings_east]';
end
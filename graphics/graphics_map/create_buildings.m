function map = create_buildings(map)

    
    map.buildings_heights = map.max_height*rand(map.nb_blocks*map.nb_blocks,1);
    
    for i=1:map.nb_blocks
        buildings_north(i) = [0.5*map.width/map.nb_blocks*(2*(i-1)+1)];
    end
    map.buildings_north = repmat(buildings_north', map.nb_blocks, 1);
    map.buildings_east  = repmat(buildings_north, map.nb_blocks, 1);
    map.buildings_east  = map.buildings_east(:);
    map.building_width = repmat(map.building_width, length(map.buildings_north), 1);

end
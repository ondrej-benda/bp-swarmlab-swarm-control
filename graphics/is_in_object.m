function waypoint = is_in_object(map,waypoint)
is_in=true;
% rng(1);
while is_in
    is_in=false;
    for i=1:length(map.buildings_north)
        vect=waypoint-[map.buildings_north(i);map.buildings_east(i)];
        if norm(vect)<map.building_width(i)
            waypoint=waypoint+map.building_width(i)*(2*rand(2,1)-1);
            is_in=true;
        end
    end
end



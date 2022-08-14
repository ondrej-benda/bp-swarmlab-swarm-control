%% Clear console and workspace and add project root to path

close all;
clearvars -except app;

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename),'../..');
addpath(genpath(project_root));


%% Simulation options

DRONE_TYPE = "point_mass"; % swarming mode supports quadcopter and point_mass
ACTIVE_ENVIRONMENT = true;
DEBUG = false;
VIDEO =true;
CENTER_VIEW_ON_SWARM = false; 
SWARM_ALGORITHM = "vasarhelyi"; % either vasarhelyi or olfati_saber

RANDOM_BUILDINGS= true;
SWARM_VIEWER_DRAW_WAYPOINTS = true;
RANDOM_GASS_LOCATION=true;
RNG_LOCK=1;%if 0-> not locked, if 1,2,3..->seed
NALADENY_PARAMETRY=true;

if DEBUG || VIDEO
    results_dirname = strcat('results/results_swarm');
    date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
    subfolder = strcat(erase(mfilename,"example_"), '_', date_string);
    results_dirname = strcat(results_dirname, '/', subfolder);
    if ~exist(results_dirname, 'dir')
        mkdir(results_dirname)
    end
end

fontsize = 12;


%% Get changes from GUI

if exist('app', 'var')
    % Simulation parameters
    p_sim.end_time = app.sim_time;
    
    % Drone parameters
    DRONE_TYPE = app.drone_type;
    
    % Swarming parameters
    SWARM_ALGORITHM = "olfati_saber";
    p_swarm.nb_agents = app.nb_agents;
    p_swarm.d_ref = app.d_ref;
    p_swarm.v_ref = app.v_ref;
    
    % Map parameters
    ACTIVE_ENVIRONMENT = app.active_environment;
    
    % Debug plot
    DEBUG = app.debug_plot;
end

if DRONE_TYPE == "point_mass"
   SWARM_VIEWER_TYPE = "agent";
elseif DRONE_TYPE == "quadcopter"
   SWARM_VIEWER_TYPE = "drone";
end


%% Call parameters files

run('param_sim');
run('param_battery');
run('param_physics');
if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
    run('param_drone'); 
elseif DRONE_TYPE == "point_mass"
    run('param_drone'); 
end
run('param_map');
run('param_swarm');
run('param_pso');

%% Init Swarm object, Wind, Viewer and other variables

%Inicializace genetickeho algoritmu

evolucni_ladeni=true;
pocet_generaci=10;
pocet_sad=24;
pocet_map=5;
rozsah_parametru=[0,2;0,5;0,5;5,100];
fitness=zeros(pocet_sad,1);
parametry_genetic=[0.7,1/(length(rozsah_parametru(:,1))*(pocet_sad/5))];

generace=1;
iterace=0;
hodnoty_parametru=genetic_algo_init(rozsah_parametru,pocet_sad);

hist_fitness=zeros(pocet_generaci,pocet_sad);
hist_casu=zeros(pocet_generaci,pocet_sad);
hist_hodnot_parametru=zeros(pocet_sad,length(rozsah_parametru(:,1)),pocet_generaci);

cas_sady=0;
cas_generace=zeros(pocet_sad,1);

for j=1:pocet_map
    if RNG_LOCK
        RNG_LOCK=j+1;
        rng(RNG_LOCK);
    end
    mapy(j) = create_rnd_buildings(map,RNG_LOCK);
    mapy(j).gass_center=is_in_object(mapy(j),[map.width*rand();map.width*round(rand())]);
end
disp('Type CTRL-C to exit');
RNG_LOCK=6;
%%
while true
    pso.linear_w=false;
    cas_sady=0;
    fitness_sady=0;
    iterace=iterace+1;
    
    pso.ws=hodnoty_parametru(iterace,1);
    pso.d=hodnoty_parametru(iterace,2);
    pso.s=hodnoty_parametru(iterace,3);
    pso.radius_seeking=hodnoty_parametru(iterace,4);
    
    for mapa=1:pocet_map
        
        map=mapy(mapa);
        p_swarm.P0 = [is_in_object(map,[map.width/2;map.width/2]);-50];
        p_swarm.Pos0 = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents);
        p_swarm.cylinders = [map.buildings_north'; map.buildings_east'; map.building_width' / 2];
        
        % Init swarm and set positions
        swarm = Swarm();
        swarm.algorithm = SWARM_ALGORITHM;

        for i = 1 : p_swarm.nb_agents
            swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
                 map);
        end
        swarm.set_pos(p_swarm.Pos0);

        % Init wind
        wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)

        % Init video
        % if VIDEO    
        %     video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
        %     video_filepath = strcat(results_dirname, '/', video_filename);
        %     video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
        % end

        % Init viewer
        swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
        swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
        swarm_viewer.draw_waypoints=SWARM_VIEWER_DRAW_WAYPOINTS;
        states_handle = [];

        PATH_TYPE = "city_dubins"; %straight_line_rrt

        if RNG_LOCK
            rng(RNG_LOCK);
        end

        %Check if goal of swarm and center of gass are not in building (if true,
        %it changes its position)
        p_swarm.goal_of_swarm=is_in_object(map,p_swarm.goal_of_swarm);
%         map.gass_center=is_in_object(map,map.gass_center);


        % swarm.as_swarm_to_goal(p_swarm.goal_of_swarm);
        swarm.as_swarm_to_goal(p_swarm.P0(1:2));

        %% Main simulation loop


        najdeno=false;
        for time = p_sim.start_time:p_sim.dt:p_sim.end_time
            % Check if program terminated from GUI
            if exist('app', 'var')
                switch app.StartsimulationSwitch.Value
                    case 'Off'
                        close all;
                        return;
                end
            end

            %Checks if swarm reached goal
            if swarm.as_swarm==true
                if swarm.as_swarm_reached_goal==true
                    swarm.as_swarm=false;
                    swarm.as_pso=true;
                end
            end
            
            %PSO
            if swarm.as_pso
                if mod(time,(pso.frequency))==0
                    swarm.compute_score(map);
                    if swarm.best_score==0
                        swarm.pso_exploring(map,pso);
                        seeking_start=time;
                    elseif swarm.best_score>=0.96
                        najdeno=true;
                        cas_sady=cas_sady+time-seeking_start;
                        disp([num2str(generace),'.',num2str(iterace),'.',num2str(mapa),'Cas: ',num2str(time)]);
                        break;
                    else
                        swarm.pso_seeking(pso,time,seeking_start);
                    end
                end
            elseif swarm.as_swarm==false
                swarm.as_drones_reached_goal(map);
            end

            % Compute velocity commands from swarming algorithm
            [~, collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

            % Update swarm states and plot the drones
           swarm.update_state(wind, time);

            % Update video
            if VIDEO
                swarm_viewer.update(time, swarm, map); 
            end
        end
        
        %Evolucni algo
        if ~najdeno
            cas_sady=cas_sady+p_sim.end_time;
            disp([num2str(generace),'.',num2str(iterace),'.',num2str(mapa),'Cas: ',num2str(p_sim.end_time)]);
        end
        if ~evolucni_ladeni
            break;
        end
    end
    cas_generace(iterace)=cas_sady;
    if iterace==pocet_sad
            fitness=spocti_fitness_pso_e(cas_generace);
            disp('');
            disp(['Generace:',num2str(generace),' Min:',num2str(min(cas_generace)),' Sum:',num2str(sum(cas_generace))]);
            [M,I]=max(fitness);
            disp(['Parametry: ',num2str(hodnoty_parametru(I,:))])
            disp('');

            hist_hodnot_parametru(:,:,generace)=hodnoty_parametru;
            hist_casu(generace,:)=cas_generace;
            hist_fitness(generace,:)=fitness;
        
            generace=generace+1;
            iterace=0;
            hodnoty_parametru=genetic_algo(hodnoty_parametru,fitness,parametry_genetic,rozsah_parametru);
    end
    if ~evolucni_ladeni||generace>pocet_generaci
            break;
    end
end
if VIDEO
    video.close(); 
end

% Close all plots
close all;

if DEBUG && ~isempty(results_dirname)
    %% Plot offline viewer
    
    swarm_viewer_off = SwarmViewerOffline(p_sim.dt_video, ...
    CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map);


    %% Analyse swarm state variables
    
    time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
    pos_ned_history = swarm.get_pos_ned_history();
    pos_ned_history = pos_ned_history(2:end,:);
    vel_ned_history = swarm.get_vel_xyz_history();
    accel_history = [zeros(1, p_swarm.nb_agents*3); ...
        diff(vel_ned_history,1)/p_sim.dt];
    
    % Save workspace
    wokspace_path = strcat(results_dirname,'/state_var');
    save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
        'accel_history');
    
    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);

    
    %% Analyse performance
    
    % Compute swarm performance
    [safety, order, union, alg_conn, safety_obs, min_d_obs] = ...
        compute_swarm_performance(pos_ned_history, vel_ned_history, ...
        p_swarm, results_dirname);
    
    % Plot performance
    [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
        union, alg_conn, safety_obs, min_d_obs, p_swarm, fontsize, results_dirname);
    
    
end


disp('Simulation completed successfully');
disp('Pocet kolizi');
disp(sum(swarm.collisions_history(:,2)));
disp('Pocet splnenych');
disp(swarm.drones(1).pocet_splnenych);


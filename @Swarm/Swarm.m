classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic agents of type
    % "Drone"
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Swarm general properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % drones:
    %           vector of Drone objects
    % nb_agents:
    %           size of the above vector
    % equivalent_drone:
    %           for path planner, drone at the barycenter ...
    %           of the swarm for command computations
    % pos_ned:

    properties
        drones % a vector of Drone objects
        nb_agents % size of the above vector
        equivalent_drone % for path planner, drone at the barycenter ...
                         % of the swarm for command computations
        algorithm SwarmAlgorithm
        collisions_history
        as_swarm
        as_pso
        swarm_goal
        
        %Variables for PSO
        best_score
        best_coord
        
        pocet_splnenych
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm()
            self.drones = [];
            self.nb_agents = 0;
            self.collisions_history = [];
            self.as_swarm=false;
            self.as_pso=false;
            self.best_score=0;
            self.best_coord=[];
            self.pocet_splnenych=0;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_drone(self, drone_type, p_drone, p_battery, p_sim, p_physics, map)
            self.nb_agents = self.nb_agents + 1;
            drone = Drone(drone_type, p_drone, p_battery, p_sim, p_physics, map);
            self.drones = [self.drones; drone];     
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_n_drones(self, drone, n)
            for i = 1:n
                self.add_drone(drone);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function init_rand_pos(self, map_size)

            for i = 1:self.nb_agents
                self.drones(i).init_rand_pos(map_size);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos(self, pos)

            for i = 1:self.nb_agents
                self.drones(i).set_pos(pos(:, i));
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel(self, vel)

            for i = 1:self.nb_drones
                self.drones(i).set_vel(vel(:, i));
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Pos_ned = get_pos_ned(self)
            % Return positions of the agent of the swarm in a matrix shape
            % of size 3 x nb_agents
            %
            %        agent_1   agent_2   ...   agent_N
            %   pn
            %   pe
            %   pd

            Pos_ned = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                Pos_ned(:, i) = drone.pos_ned;
            end

        end
        
        function waypoints_ned = get_waypoints_ned(self)
            waypoints_ned = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                waypoints_ned(:, i) = drone.waypoints(1:3,1);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Vel_ned = get_vel_ned(self)
            % Return velocities of the agents of the swarm in a matrix shape
            % of size 3 x nb_agents
            %        agent_1   agent_2   ...   agent_N
            %   vn
            %   ve
            %   vd
            
            Vel_ned = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);

                phi = drone.attitude(1);
                theta = drone.attitude(2);
                psi = drone.attitude(3);
                Rbi = Rb2i(phi, theta, psi);
                Vel_ned(:, i) = Rbi * drone.vel_xyz;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_state(self, state)
            Pos_ned = state(repmat([true true true false false false], ...
                self.nb_drones,1));
            Pos_ned = reshape(Pos_ned,3,[]);
            Vel_xyz = state(repmat([false false false true true true], ...
                self.nb_drones,1));
            Vel_xyz = reshape(Vel_xyz,3,[]);
            
            self.set_pos(Pos_ned);
            self.set_vel(Vel_xyz);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function state = get_state(self)
            Pos_ned = self.get_pos_ned();
            Vel_ned = self.get_vel_ned();
            state = [Pos_ned; Vel_ned];
            state = state(:);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Path_len = get_path_len(self)

            Path_len = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);

                Path_len(1, i) = drone.path_len;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel_commands(self, commands)

            for i = 1:self.nb_agents
                drone = self.drones(i);
                drone.command(1) = 0;
                drone.command(2:4) = commands(:, i);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Qt = get_Qt(self)
            Qt = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                Qt(i) = self.drones(i).Qt;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Q = get_Q(self)
            Q = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                Q(i) = self.drones(i).Q;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function colors = get_colors(self)
            colors = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                colors(:, i) = self.drones(i).color;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_state(self, wind, time)

            for i = 1:self.nb_agents
                self.drones(i).update_state(wind, time);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [vel_commands, collisions] = update_command(self, p_swarm, r_coll, dt)

            % Select the swarm algorithm and call the associated update
            if self.algorithm == "vasarhelyi"
                [vel_commands, collisions] = self.compute_vel_vasarhelyi(p_swarm, r_coll, dt);
            elseif self.algorithm == "olfati_saber"
                [vel_commands, collisions] = self.compute_vel_olfati_saber(p_swarm, r_coll, dt);
            end
            if isempty(self.collisions_history)
                self.collisions_history = collisions;
            else
                self.collisions_history = [self.collisions_history; collisions];
            end
            self.set_vel_commands(vel_commands);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_planner_swarm(self, path_type, time)
            % Creates an equivalent drone which will receive swarm
            % commands
            self.equivalent_drone = get_barycenter(self);
            self.equivalent_drone.plan_path(path_type, time);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function equivalent_drone = get_barycenter(self)
            pos = zeros(3, 1);
            vel = zeros(3, 1);

            for i = 1:self.nb_agents
                pos = pos + self.drones(i).pos_ned;
                vel = vel + self.drones(i).vel_xyz;
            end

            pos = pos / self.nb_agents;
            vel = vel / self.nb_agents;
            equivalent_drone = Drone(self.drones(1).drone_type, ...
                self.drones(1).p_drone, self.drones(1).p_battery, ...
                self.drones(1).p_sim, self.drones(1).p_physics, ...
                self.drones(1).map);
            equivalent_drone.set_pos(pos);
            equivalent_drone.set_vel(vel);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_planner_individual(self, path_type, time)
            % Each agent creates its waypoints independently
            for i = 1:self.nb_agents
                self.drones(i).plan_path(path_type, time);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_manager_individual(self, time)
            % Each agent creates its path independently
            for i = 1:self.nb_agents
                self.drones(i).path_manager_wing(time);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_follower_individual(self, time)
            % Each agent follows its path independently
            for i = 1:self.nb_agents
                self.drones(i).follow_path(time);
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function pos_ned_history = get_pos_ned_history(self)
            for i = 1:self.nb_agents
                pos_ned_history(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3)) = self.drones(i).pos_ned_history;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function vel_xyz_history = get_vel_xyz_history(self)
            vel_xyz_history = [];
            for i = 1:self.nb_agents
                vel_xyz_history(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3)) = self.drones(i).vel_xyz_history;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % // TODO: add this function to the SwarmViewer
        fig_handle = draw_agents_energy(self, time, period, fig_handle, axes_lim);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_state(self, time, T, period, is_active_cyl, ...
            collisions, map, dirname);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_video(self, time, T, period, fig_handle, path);
        
        
        %Added methods
        function as_swarm_to_goal(self,goal)
            self.swarm_goal=goal;
            self.as_swarm = true;
            for i = 1:self.nb_agents
                self.drones(i).waypoints(1:3,1)=[goal;-50];
            end
        end
        
        function reached_goal = as_swarm_reached_goal(self)
            reached_goal=false;
            pos = zeros(3, 1);
            for i = 1:self.nb_agents
                pos = pos + self.drones(i).pos_ned;
            end

            pos = pos / self.nb_agents;
            
            if norm(self.swarm_goal-pos(1:2))<10
                reached_goal=true;
                self.pocet_splnenych=self.pocet_splnenych+1;
            end
        end
        
        function as_drones_reached_goal(self,map)
            %rng shuffle
            for i = 1:self.nb_agents
                if norm(self.drones(i).waypoints(1:2,1)-self.drones(i).pos_ned(1:2))<2
%                     goal=rand(2,1)*map.width;
                    goal=(rand(2,1)*map.width/2+[map.width/4;map.width/4]);%pouzit pro trenink dronu, pro waypointy na mensim prostoru
                    goal=is_in_object(map,goal);
                    self.drones(i).waypoints(1:3,1)=[goal;-50];
                    self.drones(i).pocet_splnenych=self.drones(i).pocet_splnenych+1;
                end
            end
        end
        
        function drones_to_goals(self,goals)
            self.as_swarm = false;
             for i = 1:self.nb_agents
                self.drones(i).waypoints(1:3,1) = [goals(:,i);-50];
            end
        end
        
        function goals = calculate_goals(self,center,r)
            part=2*pi/(self.nb_agents+1);
            goals=zeros(2,self.nb_agents+1);
            for i = 1:(self.nb_agents+1)
                goals(:,i)=[cos(part*i-pi);sin(part*i-pi)];
            end
            goals=goals*r+center;
        end
        
        function compute_score(self,map)
            score=0;
            for i = 1:self.nb_agents
                rel_pos=self.drones(i).pos_ned(1:2)-map.gass_center;
                if max(ceil(abs(rel_pos))) <= map.gass_radius
                    score=map.gass_matrix(round(rel_pos(1)+map.gass_radius+1),round(rel_pos(2)+map.gass_radius+1));
                end
                if score>self.drones(i).best_score
                    self.drones(i).best_score=score;
                    self.drones(i).best_coord=[self.drones(i).pos_ned(1:2);-50];
                end
                if score>self.best_score
                    self.best_score=score;
                    self.best_coord=[self.drones(i).pos_ned(1:2);-50];
                end
            end
        end
        
        
        function pso_seeking(self,pso,time,seeking_start)
            if pso.linear_w
                pso_time=min(time-seeking_start,pso.t_max);
            end
%             self.compute_score(map);
            for i = 1:self.nb_agents
%                 velocity=pso.ws*(self.drones(i).waypoints(1:2,1)-self.drones(i).pos_ned(1:2));
                if pso.linear_w
                    omega=(pso.w_max-pso.w_min)*(pso.t_max-pso_time)/pso.t_max+pso.w_min;
                else
                    omega=pso.ws;
                end
%                 disp(omega)
                velocity=omega*self.drones(i).pso_velocity;
%                 disp('poloha')
%                 disp(self.drones(i).pos_ned(1:2))
%                 disp('cc_personal')
%                 disp(self.drones(i).best_coord)
%                 disp('setrvacna')
%                 disp(velocity)
                if self.drones(i).best_score>0
                    velocity=velocity+pso.d*rand*(self.drones(i).best_coord(1:2)-self.drones(i).pos_ned(1:2))...
                        +pso.s*rand*(self.best_coord(1:2)-self.drones(i).pos_ned(1:2));     
%                     disp('personalni')
%                     personalni=pso.d*rand*(self.drones(i).best_coord(1:2)-self.drones(i).pos_ned(1:2));
%                     disp(personalni)
%                     disp('tymu')
%                     disp(pso.s*rand*(self.best_coord(1:2)-self.drones(i).pos_ned(1:2)))
                elseif self.best_score>0
                    velocity=velocity+pso.s*rand*(self.best_coord(1:2)-self.drones(i).pos_ned(1:2));
                    
%                     disp('tymu')
%                     disp(pso.s*rand*(self.best_coord(1:2)-self.drones(i).pos_ned(1:2)))
                end
                if norm(velocity)~=0&&norm(velocity)>pso.radius_seeking
                    velocity=velocity/norm(velocity)*pso.radius_seeking;
                end
                self.drones(i).pso_velocity=velocity;
                self.drones(i).waypoints(1:3,1) = [self.drones(i).pos_ned(1:2,1);-50]+[velocity;0];
            end
        end
        
        function pso_exploring(self,map,pso)
%             disp('exploring')
%             self.compute_score(map);
            for i = 1:self.nb_agents
%                 velocity=pso.we*(self.drones(i).waypoints(1:2,1)-self.drones(i).pos_ned(1:2))+pso.r*(rand(2,1)*2-1);
                velocity=pso.we*(self.drones(i).pso_velocity)+pso.r*(rand(2,1)*2-1);
                
                if max(abs(self.drones(i).pos_ned(1:2,1)-(map.width/2)))>(map.width/2+map.width*0.15)
                    velocity=-self.drones(i).pos_ned(1:2,1)/5;
                end
                
                if norm(velocity)>pso.radius_exploring
                    velocity=velocity/norm(velocity)*pso.radius_exploring;
                end
                self.drones(i).pso_velocity=velocity;
                self.drones(i).waypoints(1:3,1) = [self.drones(i).pos_ned(1:2,1);-50]+[velocity;0];
            end
        end
        
        
    end

end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vasarhelyi Paramteres
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if NALADENE_PARAMETRY
    if exist('swarm', 'var')
        if swarm.as_swarm
            p_swarm.r0_rep = 46.4689;
            p_swarm.p_rep = 0.1179;

            p_swarm.r0_shill = 2.1790;
            p_swarm.v_shill = 11.8292;
            p_swarm.p_shill = 4.4067;
            p_swarm.a_shill = 9.3940;

            p_swarm.v_ref = 7.9394;

            p_swarm.r0_fric = 119.1492;
            p_swarm.C_fric = 0.2323;
            p_swarm.v_fric = 0.0201;
            p_swarm.p_fric = 4.5322;
            p_swarm.a_fric = 9.9904;
        else
            p_swarm.r0_rep = 30.6936;
            p_swarm.p_rep = 0.1256;

            p_swarm.r0_shill = 0.8502;
            p_swarm.v_shill = 8.9052;
            p_swarm.p_shill = 5.5358;
            p_swarm.a_shill = 7.1451;

            p_swarm.v_ref = 4.6207;

            p_swarm.r0_fric = 119.7;
            p_swarm.C_fric = 0.1378;
            p_swarm.v_fric = 2.3962;
            p_swarm.p_fric = 6.2906;
            p_swarm.a_fric = 3.5514;
        end
    end
else
%% Repulsion
% Repulsion range
p_swarm.r0_rep = p_swarm.d_ref; % radius of repulsion
% Repulsion gain
p_swarm.p_rep = 0.1;%0.03


%% Friction

% Stopping point offset of alignment
p_swarm.r0_fric = 85.3;
% Coefficient of velocity alignment
p_swarm.C_fric = 0.05;
% Velocity slack of alignement
p_swarm.v_fric = 0.63;
% Gain of braking curve
p_swarm.p_fric = 3.2;
% Acceleration of braking curve
p_swarm.a_fric = 4.16;


%% Obstacles and wall parameters

% Stopping point offset of walls
p_swarm.r0_shill = 1;%0.3
% Velocity of virtual shill agents
p_swarm.v_shill = 13.6; %13.6
% Gain of bracking curve for walls
p_swarm.p_shill = 3.55;%3.55
% Acceleration of braking curve for walls
p_swarm.a_shill = 5;    %3.02
end


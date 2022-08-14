%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parametrs of PSO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pso.linear_w=true;

if NALADENE_PARAMETRY2
    pso.frequency=3.1340;
    pso.we=1.3792;
    pso.r=13.8951;
    pso.radius_exploring=95.4496;
    
    pso.ws=1.4154;
    pso.d=3.7008;
    pso.s=3.6316;
    pso.radius_seeking=57.8644;

    if pso.linear_w
        pso.d=1.3462;
        pso.s=1.4590;
        pso.radius_seeking=40.6491;
    end
    pso.w_max=1.5127;
    pso.w_min=0.2931;
    pso.t_max=92.2115;
else
    pso.frequency=5;%5

    %exploring
    pso.we=0.9;
    pso.r=20;
    pso.radius_exploring=60;

    %seeking
    pso.ws=0.5;%0.5
    pso.d=0.8;%0.8
    pso.s=2;
    pso.radius_seeking=50;

    %linear omega
    pso.linear_w=true;
    pso.w_max=3;
    pso.w_min=0.5;
    pso.t_max=100;
end
pso.frequency=pso.frequency-mod(pso.frequency,p_sim.dt);




function fitness = spocti_fitness_dron(swarm)
    pocet_bodu=0;
    for dron=1:swarm.nb_agents
        pocet_bodu=pocet_bodu+swarm.drones(dron).pocet_splnenych;
    end
    pocet_bodu=pocet_bodu/(1+log(sum(swarm.collisions_history(:,2))+1));
    pocet_bodu=pocet_bodu/(1+log(sum(swarm.collisions_history(:,1))+1));
    fitness=pocet_bodu;
end


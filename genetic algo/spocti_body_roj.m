function fitness = spocti_body_roj(swarm)
    pocet_bodu=swarm.pocet_splnenych;
    pocet_bodu=pocet_bodu/(1+log10(sum(swarm.collisions_history(:,2))+1)*3);
    pocet_bodu=pocet_bodu/(1+log(sum(swarm.collisions_history(:,1))+1)*3);
    fitness=pocet_bodu;
end


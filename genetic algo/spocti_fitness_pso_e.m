function fitness = spocti_fitness_pso_e(casy)
    fitness=interp1([min(casy),max(casy)],[100,1],casy);
end


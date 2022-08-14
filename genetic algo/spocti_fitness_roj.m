function fitness = spocti_fitness_roj(body,soudrznost)
    body=body+body.*(log(min(soudrznost))./log(soudrznost));
    fitness=body;
end


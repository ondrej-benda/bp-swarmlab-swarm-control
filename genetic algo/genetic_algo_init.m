function nahodne_hodnoty = genetic_algo_init(rozsah_parametru,pocet_iteraci)
    rng(3);
    pocet_parametru=length(rozsah_parametru(:,1));
    nahodne_hodnoty=zeros(pocet_parametru,1);
    for i=1:pocet_iteraci
        for k=1:pocet_parametru
            nahodne_hodnoty(i,k)=(rozsah_parametru(k,2)-rozsah_parametru(k,1))*rand()+rozsah_parametru(k,1);
        end
    end
end


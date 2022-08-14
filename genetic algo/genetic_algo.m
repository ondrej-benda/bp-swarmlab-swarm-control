function nova_generace=genetic_algo(hodnoty_parametru,fitness,parametry_genetic,rozsah_parametru)
%parametry_genetic=[sance na crossover,sance na mutaci];
%sance na crossover- hodnota 0 až 1 určující pravděpodobnost ze dojde ke
%crossoveru
%sance na mutaci- hodnota 0 az 1 urcujici ppst ze dojde u parametru ke
%mutaci=zmena parametru o max +-10% rozsahu intervalu
ruleta=sum(fitness);
pocet_parametru=length(hodnoty_parametru(1,:));
nova_generace=zeros(length(fitness),pocet_parametru);
for i=2:2:length(fitness)
    rodice_int=[0;0];
    for rodic=1:2   
       vyber=rand()*ruleta;
       k=1;
       aktualni_hodnota=0;
       while true
           aktualni_hodnota=aktualni_hodnota+fitness(k);
           if aktualni_hodnota>vyber
               break;
           end
           k=k+1;
       end
       rodice_int(rodic)=k;
    end
    
    potomci=zeros(2,pocet_parametru);
    %crossover
    if parametry_genetic(1)>=rand()
        crossover=ceil(rand()*(pocet_parametru-1));
            potomci(1,1:crossover)=hodnoty_parametru(rodice_int(1),1:crossover);
            potomci(1,crossover:end)=hodnoty_parametru(rodice_int(2),crossover:end);
            potomci(2,1:crossover)=hodnoty_parametru(rodice_int(2),1:crossover);
            potomci(2,crossover:end)=hodnoty_parametru(rodice_int(1),crossover:end);
    else
        for potomek=1:2
            potomci(potomek,:)=hodnoty_parametru(rodice_int(potomek),:);
        end
    end
    rozsah=rozsah_parametru(:,2)-rozsah_parametru(:,1);
    %mutace
    for potomek=1:2
        for parametr=1:pocet_parametru
            if parametry_genetic(2)>=rand()
                potomci(potomek,parametr)=rozsah(parametr)*0.1*(rand()*2-1)+potomci(potomek,parametr);
            end
        end
    end
    nova_generace(i-1:i,:)=potomci;
end
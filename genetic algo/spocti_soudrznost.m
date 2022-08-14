function soudrznost = spocti_soudrznost(pos_history)
    pocet_vzorku=size(pos_history,1);
    pocet_agentu=size(pos_history,2)/3;
    x_pos=zeros(pocet_vzorku,pocet_agentu);
    y_pos=zeros(pocet_vzorku,pocet_agentu);
    for agent=1:pocet_agentu
        x_pos(:,agent)=pos_history(:,(agent-1)*3+1);
        y_pos(:,agent)=pos_history(:,(agent-1)*3+2);
    end
    teziste_x=sum(x_pos,2)/pocet_agentu;
    teziste_y=sum(y_pos,2)/pocet_agentu;
    kvadrat=((x_pos-teziste_x(:)*ones(1,pocet_agentu))+(y_pos-teziste_y(:)*ones(1,pocet_agentu))).^2;
    soudrznost=mean(sum(kvadrat,2));
end


% temp=0;
% [M,I]=max(hist_fitness,[],2);
% for i=1:10
%     temp(i)=hist_hodnot_parametru(I(i),1,i)
% end
figure()
hold on
plot(prubeh_dron(1,:),'Color','r')
% plot(prubeh_dron(1,:,2),'Color','b')
plot(mean(prubeh_dron(1,:))*ones(1,10),'--','Color','r')
% plot(mean(prubeh_dron(1,:,2))*ones(1,10),'--','Color','b')
% man=sprintf('Man., počet kolizí: %.0f',sum(sum(prubeh_dron(2:3,:,1))));
% aut=sprintf('Auto., počet kolizí: %.0f',sum(sum(prubeh_dron(2:3,:,2))));
% legend(man, aut,'Man. avg.','Auto. avg')
legend('Čas','avg')
xlabel('Mapa')
% ylabel('Hodnota parametru r0\_rep')
ylabel('Čas dokončení')

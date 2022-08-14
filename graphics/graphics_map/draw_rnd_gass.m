function draw_rnd_gass(map)
    stred=map.gass_center;
    polomer=map.gass_radius;
%     krok=polomer/100;
    krok=1;
    x = stred(1)-polomer:krok:stred(1)+polomer;
    y=stred(2)-polomer:krok:stred(2)+polomer;
    [X,Y] = meshgrid(x, y);                                   
    X=reshape(X,[],1);
    Y=reshape(Y,[],1);
    Z=reshape(map.gass_matrix',[],1);
    s=scatter(Y,X,'filled');
    s.AlphaData=Z;
    s.MarkerFaceAlpha='flat';
    s.SizeData =10;
end
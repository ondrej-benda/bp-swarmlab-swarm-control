function draw_gass(map)
    stred=map.gass_center;
    polomer=map.gass_radius;
    krok=map.width/100;
    x = stred(1)-polomer:krok:stred(1)+polomer;
    y=stred(2)-polomer:krok:stred(2)+polomer;
    [X,Y] = meshgrid(x, y);                                   
    X=reshape(X,[],1);
    Y=reshape(Y,[],1);
    Z=sqrt((X-stred(1)).^2 + (Y-stred(2)).^2);
    Z=min(Z,polomer);
    Z=interp1([0,polomer],[1,0],Z);
    s=scatter(Y,X,'filled');
    s.AlphaData=Z;
    s.MarkerFaceAlpha='flat';
    s.SizeData =30;
end
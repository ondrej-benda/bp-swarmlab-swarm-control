function plyn = create_gass(radius)
skalovac=100/radius;
ppst_ubytku=0.5;
velikost=radius*2+1;
plyn=zeros(velikost);
c=ones(2,1).*(velikost+1)/2;
skok=1/(radius/6);
vitr=rand();
for x=-1:1
    for y=-1:1
        plyn(c(1)+y,c(2)+x)=1-skok;
    end
end
plyn(c(1),c(2))=1;

i=2;
flip=1;
while true
    if (i+1)*2>=velikost
        break;
    end
    
    if rand()>vitr
       flip=flip*(-1);
    end
    for y=[-i,i]
        for x=(-i+1)*flip:flip:(i-1)*flip
            okoli=max([plyn(y+c(1)+1,x+c(2)),plyn(y+c(1)-1,x+c(2)),plyn(y+c(1),x+c(2)+1),plyn(y+c(1),x+c(2)-1)]);
            if rand()<ppst_ubytku&&abs(okoli)>(skok/10)
                plyn(y+c(1),x+c(2))=okoli-skok;
            else
                plyn(y+c(1),x+c(2))=okoli;
            end
        end
    end
    
    if rand()>vitr
       flip=flip*(-1);
    end
    for x=[-i,i]
        for y=flip*(-i+1):flip:flip*(i-1)
             okoli=max([plyn(y+c(1)+1,x+c(2)),plyn(y+c(1)-1,x+c(2)),plyn(y+c(1),x+c(2)+1),plyn(y+c(1),x+c(2)-1)]);
            if rand()<ppst_ubytku&&abs(okoli)>(skok/10)
                plyn(y+c(1),x+c(2))=okoli-skok;
            else
                plyn(y+c(1),x+c(2))=okoli;
            end
        end
    end
    for x=[-i,i]
        for y=[-i,i]
            okoli=max([plyn(y+c(1)+1,x+c(2)),plyn(y+c(1)-1,x+c(2)),plyn(y+c(1),x+c(2)+1),plyn(y+c(1),x+c(2)-1)]);
            if rand()<(ppst_ubytku)&&abs(okoli)>(skok/10)
                plyn(y+c(1),x+c(2))=okoli-skok;
            else
                plyn(y+c(1),x+c(2))=okoli;
            end
        end
    end
    i=i+1;
end
plyn=max(plyn,0);
end



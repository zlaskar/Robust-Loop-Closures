% by Juho Kannala
%Vector form of rotation to matrix form (Rodriguez formula)
function R=rotationmat(t)

    for i = 1 : size(t,1)
        t1=t(i,:);
        phi=norm(t1);
        
        if 0
            if phi==0
                sincphi=1;
                sc=0;
            else
                sc=(1-cos(phi))/(phi^2);
                sincphi=sin(phi)/phi;
            end
            R{i}=cos(phi)*eye(3)+sincphi*[0 -t1(3) t1(2); t1(3) 0 -t1(1); -t1(2) t1(1) 0]+sc*t1*t1';
        end
        
        if phi==0
            R{i}=eye(3);
        else
            th=t1/phi;
            thx=[0 -th(3) th(2); th(3) 0 -th(1); -th(2) th(1) 0];
            R{i}=eye(3)+sin(phi)*thx+(1-cos(phi))*thx*thx;
        end
    end
end
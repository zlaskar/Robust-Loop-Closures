% by Juho Kannala
%Matrix form of rotation to vector form (Rodriguez formula)
function t=rotationpars(R)
    for i = 1 : size(R,2)
        [V,D]=eig(R{i}-eye(3));
        evs=diag(D);
        [minv,mini]=min(evs);
        vect=V(:,mini);
        
        cosphi=0.5*(trace(R{i})-1);
        sinphi=0.5*vect'*[ R{i}(3,2)-R{i}(2,3); R{i}(1,3)-R{i}(3,1); R{i}(2,1)-R{i}(1,2)];
        
% % %         if cosphi==0
% % %          phi=pi/2;
% % %         else
% % %          phi=atan(sinphi/cosphi);
% % %         end
        
        phi=atan2(real(sinphi),real(cosphi));
        
        t(i,:)=real(phi*vect);
    end


end

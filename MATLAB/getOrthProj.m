%%Function to get the orthogonal projection
% % % 
% % % Input : 
% % % R : Rotation Matrix 3*3 matrix
% % % C : Camera Center 3*1 vec
% % % x3D : 3D point in world Co-ordinate
% % % 
% % % Output : projecton of the 3D point on the principal ray

function orthDep = getOrthProj(R, C, x3D, K)

        H = [R -R*C'];                                        % [R|t] t = -RC' as C is 3*1 vec
        P = K*H;                                              % camera matrix/ projection matrix
        M = P(:,1:3);                                         % M = K*R
        w = M(3,:)*(x3D' - C');
        orthDep = sign(det(M))*w/(norm(M(3,:)));              % depth
        
end
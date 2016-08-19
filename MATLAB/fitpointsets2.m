function [R,t,s]=fitpointsets2(X,Y,transform)
%FITPOINTSETS does the least-squares fitting of two 3-D point sets
%
%   [R,T]=FITPOINTSETS(X,Y) fits the point sets X and Y, where both
%   X and Y are 3*N-matrices. The rotation matrix R and translation
%   vector T are such that norm(Y-R*X-T*ones(1,N),fro) is minimised
% 
%   [R,T,S]=FITPOINTSETS(X,Y,'similarity') finds the optimal
%   similarity transformation instead of euclidean transformation.
%   R, T and scale factor S are such that
%   norm(Y-S*R*X-T*ones(1,N),fro) is minimised
%
%
%   See also
%    Arun, Huang, Blostein: Least-Squares Fitting of Two 3-D Point Sets,
%    PAMI, Vol. 9, No. 5, 1987
%

if nargin<3
  transform='euclidean';
end

N=size(X,2);
x=mean(X,2);
y=mean(Y,2);

Q=X-x(:,ones(1,N));
Qp=Y-y(:,ones(1,N));

H=zeros(2,2);
for i=1:N
  H=H+Q(:,i)*(Qp(:,i))';
end

[U,S,V]=svd(H);
X=V*U';
detX=det(X);

if detX>0
  R=X;
  %t=y-R*x;
elseif detX<0 & rank(H)<2
  R=[V(:,1) -V(:,2)]*U';
  %t=y-R*x;
else
  R=[];
  t=[];
  s=[];
  warning('In fitpointsets: Algorithm failed');
  return;
end

if strcmp(transform,'similarity')
  s=0;denomin=0;
  for i=1:N
    s=s+(Qp(:,i))'*R*Q(:,i);
    denomin=denomin+Q(:,i)'*Q(:,i);
  end  
  s=s/denomin;
else
  s=1;
end

t=y-s*R*x;
function [ inter ] = intersection( liste_p, liste_azim )
% Author: Marcelo Kaihara

v=zeros(3,3,3);
vp=zeros(3,1,3);

for i=1:3
    v(:,:,i) = eye(3)-liste_azim(i,:)'*liste_azim(i,:);
    vp(:,:,i) = v(:,:,i) * liste_p(i,:)';
end

sum_v = sum(v,3);
sum_vp = sum(vp,3);

inter = (mldivide(sum_v,sum_vp))';

end


function warpI2=warpFLColor(im1,vx,vy)
if isfloat(im1)~=1
    im1=im2double(im1);
end

if exist('vy')~=1
    vy=vx(:,:,2);
    vx=vx(:,:,1);
end
nChannels=size(im1,3);
for i=1:nChannels
    [im,~]=warpFL(im1(:,:,i),vx,vy);
    warpI2(:,:,i)=im;
end
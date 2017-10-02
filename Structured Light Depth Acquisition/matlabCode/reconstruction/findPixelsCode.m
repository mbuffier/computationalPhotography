function resultCodedPixels = findPixelsCode(floderName, imageName, first, digit, format,rgb)

% load the sequence for the data u
if rgb == 1
    matrixImagesURGB = load_sequence_color(floderName, imageName, first, first+19, digit, format) ;
    [M, N, ~, nbframe] = size(matrixImagesURGB) ;
    matrixImagesU = zeros(M,N,nbframe) ;
    
    matrixImagesVRGB = load_sequence_color(floderName, imageName, first+20, first+39, digit, format) ;
    [M, N, ~, nbframe] = size(matrixImagesVRGB) ;
    matrixImagesV = zeros(M,N,nbframe) ;
    
    % convert the RGB images into grayscale
    for i=1:nbframe
        matrixImagesU(:,:,i) = matrixImagesURGB(:,:,1,i) ;
        matrixImagesV(:,:,i) =  matrixImagesVRGB(:,:,1,i) ;
    end
else
    matrixImages = im2double(load_sequence(floderName, imageName, 0, 19, 4, format)) ;
    [M, N, ~] = size(matrixImages) ;
end

% compute both code with the given matrix 
codeU = findCode(matrixImagesU) ;
codeV = findCode(matrixImagesV) ;

% set the threshold 
test = codeU(:,:,2)+codeV(:,:,2) ;
mask = test > 4.5 ; 
mask3D = repmat(mask, [1,1,2]) ;

% compute the result by putting everything bellow the threshold to zero 
unionCode = cat(3,codeU(:,:,1), codeV(:,:,1)) ;

resultCodedPixels = mask3D.*unionCode + (1-mask3D).*zeros(M,N,2)  ;
end

function result = findCode(matrix)
[M, N, ~, ~] = size(matrix) ;

% 2 dimensions : 1 for the u code and the other for the v code 
result = zeros(M,N,2) ;

% compute a code for each pixel
for i =1:M
    for j=1:N
        % look for all the bands
        current_code = zeros(1,10) ;
        
        sum_diff = 0 ;
        index=1 ;
        for k=1:2:20
            % compute the u code
            int1 = matrix(i,j,k) ;
            int2 = matrix(i,j,k+1) ;
            
            % I inverse as the gray code is inverted in the images (u encodes rows and v colums) 
            if int1 <= int2
                current_code(index) = 0 ;
            else
                current_code(index) = 1 ;
            end
            
            index = index+1 ;
            sum_diff = sum_diff+abs(int1-int2) ;
        end
        
        %fill the resulting matrix assuming there is no wrong pixel (easy
        %cube)  
        result(i,j,1) = bi2de(current_code)+1 ;
        result(i,j,2) = sum_diff ;
    end
end
end


% code before the images were too high resolution 


% % 2 dimensions : 1 for the u code and the other for the v code 
% resultCodedPixels = zeros(M,N,2) ;
% 
% % compute a code for each pixel
% for i =1:M
%     for j=1:N
%         % look for all the bands
%         current_u = zeros(1,10) ;
%         current_v = zeros(1,10) ;
%         
%         sum_diff = 0 ;
%         index=1 ;
%         for k=1:2:20
%             % compute the u code
%             int1 = matrixImages(i,j,k) ;
%             int2 = matrixImages(i,j,k+1) ;
%             
%             % I inverse as the gray code is inverted in the images 
%             if int1 <= int2
%                 current_u(index) = 0 ;
%             else
%                 current_u(index) = 1 ;
%             end
%             
%             % compute the v code
%             int3 = matrixImages(i,j,k+20) ;
%             int4 = matrixImages(i,j,k+21) ;
%             
%             if int3 <= int4
%                 current_v(index) = 0 ;
%             else
%                 current_v(index) = 1 ;
%             end
%             
%             index = index+1 ;
%             sum_diff = sum_diff+abs(int1-int2)+abs(int3-int4) ;
%             
%         end
%         
%         %if the sum differences isn't big enough, we are not sure about
%         % this pixel
%         
%         if sum_diff < 32/255
%             resultCodedPixels(i,j,1) = 0 ;
%             resultCodedPixels(i,j,2) = 0 ;
%         else
%             % fill the resulting matrix
%             resultCodedPixels(i,j,1) = bi2de(current_u)+1 ;
%             resultCodedPixels(i,j,2) = bi2de(current_v)+1 ;
%         end
%         
%         %fill the resulting matrix assuming there is no wrong pixel (easy
%         %cube)  
%         resultCodedPixels(i,j,1) = bi2de(current_u)+1 ;
%         resultCodedPixels(i,j,2) = bi2de(current_v)+1 ;
%     end
% end

function r = lab6(path, prefix, first, last, digit, suffix)
disp('Load the images, compute the distance frame, the graph and the image flows') ;

matrix = load_sequence_color(path, prefix, first, last, digit, suffix) ;
[M, N, ~, nbFrames] = size(matrix) ;

disp('Images loaded') ;

% images to double
matrix = double(matrix) ;

%************ Step 1 : load the flow **********
if ~exist('flows.mat')
    error('Flow are needed for compution, you can compute them in Optical flow, computeflow.m') ;
else
    load('flows.mat') ;
    disp('Flows matrix computed')
end


%************ Step 2 : compute the distance frame for the sequence and create the graph **********
if ~exist('distFrame.mat')
    
    % *********** inter-step : compute the trajectory with respect to the first
    %               image for each image ********
    [MFlows, NFlows] = size(flows_a(:,:,1,1)) ;
    
    dist_trajectory = zeros(MFlows, NFlows, 2, nbFrames) ;
    
    for i = 2:nbFrames
        index = (i-1)*(i-2)/2+1 ;
        dist_trajectory(:,:,1,i) = -flows_a(:,:,1,index) ;
        dist_trajectory(:,:,2,i) = -flows_a(:,:,2,index) ;
    end
    
    % compute the distances
    distFrame = zeros(nbFrames, nbFrames) ;
    for i = 1:nbFrames
        for j = 1:nbFrames
            dist = computeDistColor(matrix(:,:,:,i), matrix(:,:,:,j), dist_trajectory(:,:,1,i), dist_trajectory(:,:,1,j),...
                dist_trajectory(:,:,2,i), dist_trajectory(:,:,2,j)) ;
            distFrame(i,j) = dist ;
        end
    end
    save('distFrame.mat', 'distFrame') ;
    disp('Distance frame matrix computed')
    
else
    load('distFrame.mat') ;
    disp('Distance frame matrix computed')
end

% create the minimum spanning tree for the total graph
sparseDistFrame = sparse(double(distFrame)) ;
undirectedGraphTotal = tril(sparseDistFrame + sparseDistFrame') ;
[Tree, ~] = graphminspantree(undirectedGraphTotal) ;

tree_full = full(Tree) ;
tree_full = tree_full > 0 ;

% create the thresholded matrix
distFrameThres = distFrame < 60 ;

lastMask = or(distFrameThres,tree_full) ;

% will be the distFrame value if lastMask ==1 or a high distance
% of 70 if lastMask == 0 
newDistFrame = (lastMask.*distFrame+ (1 - lastMask).*70) ;

sp_newDistFrame = sparse(double(newDistFrame)) ;
und_graph_final = tril(sp_newDistFrame + sp_newDistFrame') ;
disp('Graph created')
%h = view(biograph(und_graph_final,[],'ShowArrows','off','ShowWeights','on')) ;

% ********* Step 3 : ask the user to choose a starting frame ***********
lastnumber = num2str(last) ;
text = ['Chose a number between ' num2str(first)+1 ' and ' lastnumber ' as a strating frame : '] ;
choosen_frame = input(text) ;

fig1=figure ;
imshow(matrix(:,:,:,choosen_frame)) ;

disp('You can choose a starting point and several destinations') ;

[wanted_positions_j,wanted_positions_i] = getline(fig1) ;

% create the directions the user wants
wanted_direction = zeros(2, size(wanted_positions_j,1)-1) ;
for i=1:size(wanted_positions_j,1)-1
    wanted_direction(1,i) =  wanted_positions_i(i+1) - wanted_positions_i(i) ;
    wanted_direction(2,i) =  wanted_positions_j(i+1) - wanted_positions_j(i) ;
end

pos_start = [wanted_positions_i(1); wanted_positions_j(1)] ;
% ********* Loop for the next frame to handle multiple directions chooses ***********
number_directions = size(wanted_positions_j, 1) ;

disp('The algorithm is computing paths between frames') ;

for k = 1:number_directions-1
    
    % ********* Step 4 : compute the shortest path for all the images with respect to the chosen one ***********
    paths = cell(1,nbFrames);
    for j=1:nbFrames
        [~,path,~] = graphshortestpath(und_graph_final,choosen_frame,j,'directed',false) ;
        paths{1,j} = path ;
    end
    
    % ********* Step 5 : compute the position for this pixel for every image
    %                    according to the shortest path and the flows ***********
    pixel_positions = zeros(2,nbFrames) ;
    pixel_displacements = zeros(2,nbFrames) ;
    
    for i=1:nbFrames
        % extract the path 
        path = paths{1,i} ;
        current_position = double(pos_start) ;
        current_displacement = [0; 0] ;
        
        if size(path) == 1
            pixel_positions(:,i) = [0; 0] ;
        else
            for j= 2:size(path,2)
                
                % compute the flows according to the index
                [index, sign] = findIndex(path(j-1), path(j)) ;
                
                vx = imresize(sign.*flows_a(:,:,1,index), 1/0.3) ;
                vy =  imresize(sign.*flows_a(:,:,2,index), 1/0.3) ;
      
                % compute the displacement and the pixel pos for each
                current_position = uint16(current_position) ;
                y_disp = vy(current_position(1), current_position(2));
                x_disp = vx(current_position(1), current_position(2));
                
                current_displacement = current_displacement + [y_disp;x_disp]*(1/0.3) ;
                current_position = double(current_position) + [y_disp ;x_disp]*(1/0.3) ;
            end
            pixel_positions(:,i) = uint16(current_position) ;
            pixel_displacements(:,i) = current_displacement ;
        end
    end
    
    % ********* Step 6 : find the images with the closest correspondance ***********
    wanted_displacement = repmat(wanted_direction(:,k), [1, nbFrames]) ;
    
    % compute the distance from the previous pixel position
    dist = sum((pixel_displacements-wanted_displacement).^2, 1) ;
    closest_frame_index = find(dist==min(dist)) ;
    
    % ********* Step 7 : Store the best path and the position estimate ***********
    if k==1
        first_position = [wanted_positions_i(1); wanted_positions_j(1)] ;
        position_found = [first_position, uint16(pixel_positions(:,closest_frame_index(1)))] ;
        winningPath = paths{1,closest_frame_index(1)} ;
    else
        position_found = [position_found, uint16(pixel_positions(:,closest_frame_index(1)))] ;
        winningPath = [winningPath(1,1:end-1), paths{1,closest_frame_index(1)}] ;
    end
    
    choosen_frame = winningPath(1, end:end) ;
    pos_start = position_found(:,end:end) ;
end

% ********* Step 8 :  Show the found path  ***********
figure,
imshow(matrix(:,:,:,winningPath(1))); hold on;

for this_dest = 1:size(wanted_positions_j, 1)-1
    plot([wanted_positions_j(this_dest,1) wanted_positions_j(this_dest+1,1)], [wanted_positions_i(this_dest,1) wanted_positions_i(this_dest+1,1)],'g-','LineWidth',2);
    hold on;
    plot([position_found(2,this_dest) position_found(2,this_dest+1)], [position_found(1,this_dest) position_found(1,this_dest+1)],'r-','LineWidth',2);
    hold on ;
end;

% ********* Step 9 :  Interpolation between images of the path   ***********
answer = input('Do you want to interpolate between frames ? (1 or 0)') ;

if answer == 1
    % ask how many images to add 
    nb_frame_add = input('How many images do you want to create in between ?') ;
    
    % initialization and creation of the storage 
    numb_frame_before = size(winningPath, 2) ;
    num_frame_total = numb_frame_before+(numb_frame_before-1)*nb_frame_add ;
    result_matrix = zeros(M/2, N/2, 3, num_frame_total, 'uint8') ;
    index_matrix = 1 ;
    result_matrix(:,:,:,index_matrix) = im2uint8(imresize(matrix(:,:,:,winningPath(1)),0.5)) ;
    index_matrix = index_matrix+1 ;
        
    % creation of the right number of images between frames 
    for i=2:numb_frame_before
        % compute the flows between 2 original images
        [index, sign] = findIndex(winningPath(i-1), winningPath(i)) ;
        vx = sign.*flows_a(:,:,1,index) ;
        vy = sign.*flows_a(:,:,2,index) ;
           
        % add frames 
        for p=1:nb_frame_add
            % compute the intermediare flows and resize it 
            inter_flowsX = imresize((vx./nb_frame_add+1)*(1/0.3)*p, 1/0.3) ;
            inter_flowsY = imresize((vy./nb_frame_add+1)*(1/0.3)*p, 1/0.3) ;
            
            % blur the flows to avoid artifacts 
            inter_flowsY = imgaussfilt(inter_flowsY,20) ;
            inter_flowsX = imgaussfilt(inter_flowsX,20) ;
            
            % create the image using warpFLColor and store it 
            newImage = warpFLColor(matrix(:,:,:,winningPath(i-1)),-inter_flowsX,-inter_flowsY) ;
            result_matrix(:,:,:,index_matrix) = im2uint8(imresize(newImage,0.5)) ;
            index_matrix = index_matrix+1 ;
        end
        result_matrix(:,:,:,index_matrix) = im2uint8(imresize(matrix(:,:,:,winningPath(i)),0.5)) ;
        index_matrix = index_matrix+1 ;
    end
    % crop the image to avoid seeing artefact 
    result_matrix = result_matrix(10:end-10, 10:end-10, :,:) ;
else
    % create the final matrix
    result_matrix = zeros(M/2, N/2, 3, size(winningPath, 2), 'uint8') ;
    
    for i= 1:size(winningPath, 2)
        result_matrix(:,:,:,i) = im2uint8(imresize(matrix(:,:,:,winningPath(i)),0.5)) ;
    end
end

% ********************* Step 10 : store the result   ***************
% create the video
createVideo(result_matrix, 'result_final') ;

% save the images
save_sequence(result_matrix, 'resultImages', 'result_', 0, 4) ;

end

% compute the color distance
function dist = computeDistColor(img1, img2, vx1, vx2, vy1, vy2)
dist_disp_X = computeDist1Chanel(vx1, vx2) ;
dist_disp_Y = computeDist1Chanel(vy1, vy2) ;

distR = computeDist1Chanel(img1(:,:,1), img2(:,:,1)) ;
distG = computeDist1Chanel(img1(:,:,2), img2(:,:,2)) ;
distB = computeDist1Chanel(img1(:,:,3), img2(:,:,3)) ;

dist = sqrt(distR^2+distG^2+distB^2+dist_disp_X^2+dist_disp_Y^2) ;
end

% compute the distance for one chanel image
function dist = computeDist1Chanel(img1, img2)
diff = (img1-img2).^2 ;
dist = sum(diff(:)) ;
dist = dist/(size(img1,1)*size(img1,2)) ;
end

% export the final video
function r =  createVideo(matrix, name)
% matrix: matrix of images to export
% path: path of the files
% name : name of the filename

numFrame = size(matrix,4) ;
% initialization of the video
filename = ['resultVideo/' name '.avi'];
v = VideoWriter(filename,'Uncompressed AVI');
open(v) ;

for i=1:numFrame
    for j=1:7
        writeVideo(v,matrix(:,:,:,i)) ;
    end
end

end

function [index, sign] = findIndex(indexI, indexJ)
if indexI > indexJ
    index = (indexI-1)*(indexI-2)/2+indexJ ;
    sign = 1 ;
else
    index = (indexJ-1)*(indexJ-2)/2+indexI ;
    sign = -1 ;
end
end


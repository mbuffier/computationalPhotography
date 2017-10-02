function r = createPLY(matrix, nameFile)

% function to create a PLY file to visualiaze it in Meshlab
% matrix need to be a column matrix 

[N, M] = size(matrix) ;

% in case of color information 
if M==6
    fileID = fopen([nameFile '.txt'],'w');
    fprintf(fileID,'ply\n');
    fprintf(fileID,'format ascii 1.0\n');
    fprintf(fileID,'element vertex %d\n', N);
    fprintf(fileID,'property float x\n');
    fprintf(fileID,'property float y\n');
    fprintf(fileID,'property float z\n');
    fprintf(fileID,'property uchar red\n');
    fprintf(fileID,'property uchar green\n');
    fprintf(fileID,'property uchar blue\n');
    fprintf(fileID,'end_header\n');
    
    for i=1:N
        fprintf(fileID,'%f, %f, %f, %u, %u, %u\n',matrix(i,1),matrix(i,2),matrix(i,3), uint8(matrix(i,4)), uint8(matrix(i,5)), uint8(matrix(i,6)));
    end
    
% without color information 
elseif M==3
    fileID = fopen([nameFile '.txt'],'w');
    fprintf(fileID,'ply\n');
    fprintf(fileID,'format ascii 1.0\n');
    fprintf(fileID,'element vertex %d\n', N);
    fprintf(fileID,'property float x\n');
    fprintf(fileID,'property float y\n');
    fprintf(fileID,'property float z\n');
    fprintf(fileID,'end_header\n');
    
    for i=1:N
        fprintf(fileID,'%f, %f, %f \n',matrix(i,1),matrix(i,2),matrix(i,3));
    end
    
    
else 
    error('The matrix is not in the good format ') ;
end

end
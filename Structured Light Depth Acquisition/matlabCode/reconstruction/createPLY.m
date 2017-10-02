function r = createPLY(matrix, nameFile)

% function to create a PLY file to visualiaze it in Meshlab
[N, ~] = size(matrix) ;

fileID = fopen(['../pointClouds/' nameFile '.txt'],'w');
fprintf(fileID,'ply\n');
fprintf(fileID,'format ascii 1.0\n');
fprintf(fileID,'element vertex %d\n', N);
fprintf(fileID,'property float x\n');
fprintf(fileID,'property float y\n');
fprintf(fileID,'property float z\n');
fprintf(fileID,'end_header\n');

for i=1:N 
    fprintf(fileID,'%f, %f, %f\n',matrix(i,1),matrix(i,2),matrix(i,3));
end

end
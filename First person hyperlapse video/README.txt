The code folder contains the code of my project describes in the PDF. 

The SfM folder contains 2 folders : result which contained the result of the SFM algorithm. However, result is empty to  because it was to heavy to be uploaded on moodle. I only upload the images of the short UCL sequence, they are in «imagesUCL » in SfM.

The videos folder contained the original video (only for UCL) as well as a result folder to store the resulting video.

The .json files in the code folder are the result from the SfM algorithm stored in json format (both for UCL and Paris) shown in the PDF description. 

The total pipeline can be run this way : 
- first obtain frames (stored in SfM folder) from the video (store in ‘videos’) using fromVideoToImages.m

- use those to compute a point cloud using SfM_PoseEstimation (python SfM_PoseEstimation.py ‘imagesPath’ ‘resultPath’)

- From the json obtained, run main[…](jsonFileName, imageFolder, numberOfFrame). […] indicates one of the 3 versions, imageFolder is the one in the SfM folder and numberOfFrame the number of frame to have in the final video. 

For example you can run : 
mainHomography('sfm_data_UCL3.json’, 'imagesUCL', 317) ;


Thank you ! 

Maud 

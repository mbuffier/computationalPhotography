{\rtf1\ansi\ansicpg1252\cocoartf1504\cocoasubrtf810
{\fonttbl\f0\fswiss\fcharset0 Helvetica;\f1\fnil\fcharset0 Menlo-Regular;}
{\colortbl;\red255\green255\blue255;\red0\green0\blue0;\red0\green0\blue0;\red255\green255\blue255;
}
{\*\expandedcolortbl;;\csgenericrgb\c0\c0\c0;\csgray\c0;\csgray\c100000;
}
\paperw11900\paperh16840\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\qc\partightenfactor0

\f0\b\fs28 \cf0 Project Part 2 Buffier : README \
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\fs24 \cf0 Project description
\fs28  \

\b0\fs24 In this folder we have a pdf document with a presentation of my project. You can access videos by clicking on them. If it\'92s not working, I also provide the powerpoint version. \
\

\b Code folder\

\b0 The code folder contains the code of my project describes in the PDF. \
\
The SfM folder contains 2 folders : result which contained the result of the SFM algorithm. However, result is empty to  because it was to heavy to be uploaded on moodle. I only upload the images of the short UCL sequence, they are in \'abimagesUCL\'a0\'bb in SfM.\
\
The videos folder contained the original video (only for UCL) as well as a result folder to store the resulting video.\
\
The .json files in the code folder are the result from the SfM algorithm stored in json format (both for UCL and Paris) shown in the PDF description. \
\
The total pipeline can be run this way : \
- first obtain frames (stored in SfM folder) from the video (store in \'91videos\'92) using 
\f1\fs22 \cf2 \CocoaLigature0 fromVideoToImages.m\
\
- use those to compute a point cloud using SfM_PoseEstimation (python\cf3 \cb4  SfM_PoseEstimation.py \'91imagesPath\'92 \'91resultPath\'92\cf2 \cb1 )\
\
- From the json obtained, run main[\'85](jsonFileName, imageFolder, numberOfFrame). [\'85] indicates one of the 3 versions, imageFolder is the one in the SfM folder and numberOfFrame the number of frame to have in the final video. \
\
For example you can run : \
mainHomography('sfm_data_UCL3.json\'92, 'imagesUCL', 317) ;\
\
\
Thank you ! \
\
Maud }
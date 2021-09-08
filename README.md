# Sentrybot_ws
This repository stores all the codes and information about the Sentrybot project that was developed in Summer 2021. 
------------------------------------------------------------------------------------------------------------------
This project is about a welcoming robot called Sentrybot that can navigate autonomously to take anyone approaching for direction to a commonly visited place in Bergeron building. Apart from using ROS and Slam_toolbox it uses Speech Recognition Pocketsphinx to spot the keywords from the visitors.
------------------------------------------------------------------------------------------------------------------
**Instructions on How to Use the Code:**

Publishing a picture on the monitor:
  - Simply change the name of the file under the image_publisher node in sentrybot_gazebo.launch launch file or other gazebo launch files that you would like to launch.
  - You can comment the node if you don't want the picture to be seen on the monitor at the start of the launch file.
--------------------------------------------------------------------------------------------------------------------------------------
Working with the camera:
  - you can type in the following command to see the camera view of the robot or to see any object that's being placed in front of the camera. rosrun image_view image_view image:=/sentrybot/camera/image_raw
--------------------------------------------------------------------------------------------------------------------------------------
Streaming a video on the monitor:
  - Videos can be streamed on the screen of the monitor by changing the value of the argument "video_stream_provider" by the exact location of the video.
--------------------------------------------------------------------------------------------------------------------------------------
Mapping Sherman and Bergeron buildings:
  - Sherman and Bergeron worlds or any other place (Gazebo or real life structures) can be mapped using the online_sync.launch located in sentrybot_description/slam_toolbox-noetic-devel/slam_toolbox/launch file with the mapping mode that can be modified in the related config file in sentrybot_description/slam_toolbox-noetic-devel/slam_toolbox/config. To save a mapped structure using rviz, go to panels/add_new_panel and choose SlamToolboxPlugin. Once the GUI is opened you can give it a name and click the Save Map button. However, ready-made maps are available for both buildings using the following Google Drive links:
    - Sherman building: https://drive.google.com/file/d/1GQlb0vqm0hGnr7P2gEVsY-TgTIJ1EQ0L/view?usp=sharing
    - Bergeron building: https://drive.google.com/file/d/1inqQ13eGKwfEpdesvCqHVc5P3-L_DDy8/view?usp=sharing
  - To load your mapped pose-graph, modify the name of the map_file_name located in sentrybot_description/slam_toolbox-noetic-devel/slam_toolbox/config/mapper_params_online_sync.yaml and provide any starting pose if applicable.
  - For localization please use the online_sync.launch file by changing its mode to localization from mapping in the related config files form mapping.
<<<<<<< HEAD
--------------------------------------------------------------------------------------------------------------------------------------
*** Note: Further instructions about the usage of slam_toolbox package is available in the readme file of its package. ***
=======
--------------------------------------------------------------------------------------------------------------------------------------

Working with Audio Classification:
  - Audio_source node captures audio that is saved forever in ROS
  - Audio_classification takes this audio from the log and temporarily creates an audio
  - This audio file is the used to calculate audio classification
  - The file functions.py is required for it to run
  - The folders Audio_only and Audio_Text contain the models trained to work for audio classification and are required
  - The files helper.py, features.py and graph_formatting.py are also required for the functions defined in functions.py
  - To calculate features, start the audio_source node, and then run the audio_classification node. The results can be seen via ```rostopic echo /sentrybot/sentimentanalysis``` command
  - The results are posted via a custom ROS msg calledn SentimentAnalysis.msg
--------------------------------------------------------------------------------------------------------------------------------------
Some Screenshots of the work can be found on this link:
  - https://drive.google.com/drive/folders/1ZVcDkdwSghNCSYnVnfP8n7O6LT7Tvtxa?usp=sharing
--------------------------------------------------------------------------------------------------------------------------------------
The navigation and speech_recognition were developed and implemented by Adnan Fahad Faizi, a Computer Science student at Lassonde School of Engineering in LURA summer program under the supervision of Prof. Michael Jenkin.
--------------------------------------------------------------------------------------------------------------------------------------

By Faizi, Adnan Fahad


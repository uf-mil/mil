-- If you want to detect a buoy

rosrun sub8_perception HOG.py <image_topic>

-- If you want to detect a new shape

---- Selecting ROI ----
1. create a new file in this directory called 'train_vid' and fill it with your training images
2. run roi.py and draw a border around your image, you can draw as many borders as you want until you get the right one.
It always saves the last one.
3. Press c.
4. Do this for every image in your train_vid file

---- Training ----
5. Run HOG_SVM_trainer.py


6. rosrun sub8_perception HOG.py <image_topic>
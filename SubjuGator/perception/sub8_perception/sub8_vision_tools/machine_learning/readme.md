Sub Machine Learning Stack

# Ingestion
Use segmentation/manual_grabcut to segment a bag file, this outputs a .p file containing the labelled data

Use any of the training tools to output a trained classifier that will run on a whole image


# Plans
* Hard negative mining
* Use an RBM-SVC to generate more features for boosting (Should eliminate redundant features)
* Really easy training pipeline
* Unified detector/classifier class for easy interfacing


# Dream Pipeline

1. Record a short bag
2. Manually segment a useful subset (For now, just manually skip images that aren't useful)
    a. Save as a .pkl
3. Run one of the ML trainers
    a. save the resulting classifier as a .pkl
4. Use load the pkl at runtime
5. Go to town

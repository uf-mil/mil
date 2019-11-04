PIPELINE_CONFIG_PATH=~/../tensorflow/models/research/object_detection/transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/pipeline.config
MODEL_DIR=~/../tensorflow/models/research/object_detection/transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/
NUM_TRAIN_STEPS=50000
NUM_EVAL_STEPS=99
python ../model_main.py \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --model_dir=${MODEL_DIR} \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --num_eval_steps=${NUM_EVAL_STEPS} \
    --alsologtostderr

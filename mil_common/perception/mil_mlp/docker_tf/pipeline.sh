docker run --runtime=nvidia --rm -it \
-v $PWD/transfer_learning:/tensorflow/models/research/object_detection/transfer_learning:rw \
--privileged -p 6006:6006 mil-common/docker_tf:latest
# --privileged -p 6006:6006 grymestone:tensorpipeline01

# Prepare data for generate_tfrecords.py
python process.py
# Generate TF records
python generate_tfrecord.py --csv_input=train_labels.csv --image_dir=train --output_path=train.record
python generate_tfrecord.py --csv_input=test_labels.csv --image_dir=test --output_path=test.record
# Move the record files to their proper locations
mv train.record ../data/train.record
mv test.record ../data/test.record
# Cleanup
# rm -r Annotations Images test train 
# rm test_labels.csv train_labels.csv 
python cvt_pascal.py
python split_data.py
python xml_to_csv.py
python generate_tfrecord.py --csv_input=train_labels.csv --image_dir=train --output_path=train.record
python generate_tfrecord.py --csv_input=test_labels.csv --image_dir=test --output_path=test.record
rm -r Annotations Images test train 
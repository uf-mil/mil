import os
import random
import shutil

home = os.path.expanduser("~")
# Source folder containing all files

source_folder = os.path.join(home, "Robosub 2024 Data", "buoy_v1")

# Destination folders for train and test sets
train_folder = "train"
test_folder = "test"
val_folder = "val"

# Create train and test folders if they don't exist
# os.makedirs(os.path.join(image_folder, train_folder))
# os.makedirs(os.path.join(image_folder, test_folder))
# os.makedirs(os.path.join(label_folder, train_folder))
# os.makedirs(os.path.join(label_folder, test_folder))

# List all files in the source folder
for i in range(1):
    folder = f"s{i+1}"
    print(folder)
    image_folder = os.path.join(source_folder, folder, "images")
    label_folder = os.path.join(source_folder, folder, "labels")

    files = os.listdir(image_folder)
    files = [
        entry for entry in files if os.path.isfile(os.path.join(image_folder, entry))
    ]
    random.shuffle(files)  # Shuffle the list of files

    # Calculate split indices
    split_index = int(0.8 * len(files))  # 80% for train, 20% for test and val
    offset = int((len(files) - split_index) / 2)
    test_val_split_index = split_index + offset

    # Split files into train and test sets
    train_files = files[:split_index]
    test_files = files[split_index:test_val_split_index]
    val_files = files[test_val_split_index:]

    # Move train files
    print("\nOrganizing Training Data\n")
    for file in train_files:
        base = os.path.splitext(os.path.basename(file))[0]

        # Move images
        image_file = base + ".png"
        image_train_path = os.path.join(source_folder, train_folder, "images")
        print("Moving image:\n" + image_file + " => " + image_train_path)
        src = os.path.join(image_folder, image_file)
        dst = os.path.join(image_train_path, image_file)
        shutil.move(src, dst)

        # Move labels
        label_file = base + ".txt"
        label_train_path = os.path.join(source_folder, train_folder, "labels")
        print("Moving label:\n" + label_file + " => " + label_train_path)
        src = os.path.join(label_folder, label_file)
        dst = os.path.join(label_train_path, label_file)
        shutil.move(src, dst)

    print("\nOrganizing Testing Data\n")
    # Move test files
    for file in test_files:
        base = os.path.splitext(os.path.basename(file))[0]
        # Move images
        image_file = base + ".png"
        image_test_path = os.path.join(source_folder, test_folder, "images")
        print("Moving image:\n" + image_file + " => " + image_test_path)
        src = os.path.join(image_folder, image_file)
        dst = os.path.join(image_test_path, image_file)
        shutil.move(src, dst)

        # Move labels
        label_file = base + ".txt"
        label_test_path = os.path.join(source_folder, test_folder, "labels")
        print("Moving label:\n" + label_file + " => " + label_test_path)
        src = os.path.join(label_folder, label_file)
        dst = os.path.join(label_test_path, label_file)
        shutil.move(src, dst)

    print("\nOrganizing Validation Data\n")
    # Move test files
    for file in val_files:
        base = os.path.splitext(os.path.basename(file))[0]
        # Move images
        image_file = base + ".png"
        image_val_path = os.path.join(source_folder, val_folder, "images")
        print("Moving image:\n" + image_file + " => " + image_val_path)
        src = os.path.join(image_folder, image_file)
        dst = os.path.join(image_val_path, image_file)
        shutil.move(src, dst)

        # Move labels
        label_file = base + ".txt"
        label_val_path = os.path.join(source_folder, val_folder, "labels")
        print("Moving label:\n" + label_file + " => " + label_val_path)
        src = os.path.join(label_folder, label_file)
        dst = os.path.join(label_val_path, label_file)
        shutil.move(src, dst)


print("Splitting and moving files completed successfully.")

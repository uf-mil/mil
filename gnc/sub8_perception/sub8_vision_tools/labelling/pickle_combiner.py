import pickle
import argparse
import sys

if __name__ == '__main__':
    usage_msg = ("Pass the path to an arbitrary number of pickle files.")
    desc_msg = "Used to combine multiple pickles into one."

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='pickles', nargs="*",
                        help="List of pickles to combine.")
    parser.add_argument('--output', type=str, help="Path to a file to output to (and overwrite)",
                        default='combine.p')

    args = parser.parse_args(sys.argv[1:])
    
    out_pickle = []
    print "Loading pickles."
    for pickle_file in args.pickles:
    	out_pickle += pickle.load(open(pickle_file, 'rb'))
    print "Pickles loaded - saving."
    pickle.dump(out_pickle, open(args.output, 'wb'))
    print "Done!"

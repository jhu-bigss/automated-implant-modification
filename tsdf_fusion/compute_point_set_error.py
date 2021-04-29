import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Evaluate TSDF fusion accuracy')
parser.add_argument('input_1', metavar='INPUT_FILE', help='1st csv file of reference points set')
parser.add_argument('input_2', metavar='INPUT_FILE', help='2nd csv file of TSDF fusion points set')
args = parser.parse_args()

point_set_1 = np.loadtxt(args.input_1, delimiter=',')
point_set_2 = np.loadtxt(args.input_2, delimiter=',')

for point_gt, point_exp in zip(point_set_1, point_set_2):
    diff = point_gt - point_exp
    print(diff)
    diff = np.linalg.norm(diff)
    print(diff)
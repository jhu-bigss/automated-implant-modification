import os
from utils import load_data

data_foler = 'data/'

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CHESSBOARD_WIDTH = 7
CHESSBOARD_HEIGHT = 6
CHESSBOARD_SQUARE_SIZE = 30

data_path = os.path.join(os.getcwd() + '/', data_foler)
print(data_path)
load_data.read_chessboard_image_from_dir(data_path, IMAGE_WIDTH, IMAGE_HEIGHT, CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT, CHESSBOARD_SQUARE_SIZE, calibrate_camera=True)
print("------------camera calibration completed--------------")
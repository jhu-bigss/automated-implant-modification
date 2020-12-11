import os
from utils import load_data

data_foler = 'data/'

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_CHESSBOARD_WIDTH = 7
DEFAULT_CHESSBOARD_HEIGHT = 6
DEFAULT_CHESSBOARD_SQUARE_SIZE = 30

data_path = os.path.join(os.getcwd() + '/', data_foler)
print(data_path)
load_data.read_chessboard_image_from_dir(data_path, DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_CHESSBOARD_WIDTH, DEFAULT_CHESSBOARD_HEIGHT, DEFAULT_CHESSBOARD_SQUARE_SIZE, calibrate_camera=True)
print("------------camera calibration completed--------------")
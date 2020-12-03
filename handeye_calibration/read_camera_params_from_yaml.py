import os
import yaml
import numpy as np

file_path = os.path.dirname(os.path.realpath(__file__))
camera_param_path = file_path.replace(file_path.split("/")[-1], '') + "camera_calibration/camera_params.yaml"

# Function to load yaml configuration file
def load_parameters(file_name):
    with open(file_name) as file:
        parameters = yaml.safe_load(file)

    return parameters

camera_params = load_parameters(camera_param_path)

camera_intrinsic = np.array(camera_params['camera_intrinsic']['data'])
rows = camera_params['camera_intrinsic']['rows']
cols = camera_params['camera_intrinsic']['cols']
camera_intrinsic = np.reshape(camera_intrinsic, (rows, cols))
print(camera_intrinsic)
"""NOTE: this file should not be tracked
with git. It is automagically
generated by catkin based on presets

everything in this file is editable
the varaibles here must always exist,
but they can be changed however you want

current_car and enabled_models use info
from static_params
see static_params.py if you want to add
config info for a new car or ML model
static_params.py is tracked by git
"""
from static_params import car_info, model_info

# full path to the bdd directory (the one with
#   the ./bin and ./src directories
path_to_bdd_dir='/home/mark/car/src/bdd'

# camera params
camera_fps = 30 # can be 15, 30, or 60, depending on resolution
camera_resolution_height = 720 # can be 384, 720, 1080, or 1440

# car params
# set this to the name of the current car
# you can find car names in car_info in
#   static_params.py
current_car = 'car_1'


# models
# see model_info in static_params.py
# to enable a model at runtime, add its name to the
#   enabled_models list
# feel free to add or delete items from this list at
#   will
# however, this list must always exist (empty is fine)
enabled_models = ['test_model',]

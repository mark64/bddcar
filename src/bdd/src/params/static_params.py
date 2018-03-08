"""semi-static config information
this file is tracked by git

add, but do not remove data from this
file (unless it's no longer relevant)

only config options that make sense
to track with git and shared with
everyone should be put here

that means think twice before
putting something like a file
path here
"""

# this contains settings for the different cars
# add information here for all cars used
car_info = {
    'car_1': {
        'mse_path': '/dev/arduino',
        'speed_pwm_null': 24,
        'speed_pwm_gain': 2,
        'speed_pwm_min': 0,
        'speed_pwm_max': 4096,
        'direction_pwm_null': 21,
        'direction_pwm_gain': 3,
        'direction_pwm_min': 0,
        'direction_pwm_max': 4096,
        'baudrate': 112500,
        'timeout': 1,
    },
}

# when you make a new model, add its information to
#   the model_info dictionary
# however, do not delete data from this dictionary
#   unless that model will never be used ever again
model_info = {
    'test_model': {
        'name': 'blank',        # NOTE: this software uses the model
                                # name to determine the filename for
                                # your model.
                                # symlink your model in the bdd/models
                                # directory and name the symlink <name>.py
                                # symlinks are made with:
                                # > ln -s path/to/original path/to/new/link

        'framework': 'pytorch', # or something else like 'tensorflow'
                                # if your framework isn't supported,
                                # then you will have to convert a
                                # numpy matrix to your networks's
                                # desired type
    },
}

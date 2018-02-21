class Controls():
    def __init__(self, speed, direction):
        self.speed = speed
        self.direction = direction

class ML():
    def __init__(self, model_info):
        self.weights = model_info['weights_path']
        self.model = model_info['model_code']
        self.model.share_memory()

    def output_controls(self, left_image, right_image):
        speed = 0.1
        direction = 0.2
        return Controls(speed, direction)

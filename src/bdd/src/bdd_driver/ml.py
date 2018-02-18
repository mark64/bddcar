class Controls():
    def __init__(self, speed, direction):
        self.speed = speed
        self.direction = direction

class ML():
    def __init__(self, weights_path):
        self.weights_path = weights_path

    def output_controls(self, left_image, right_image):
        speed = 0.1
        direction = 0.2
        return Controls(speed, direction)

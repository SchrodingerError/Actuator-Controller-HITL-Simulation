class SharedFloat:
    def __init__(self, initial_value:float=0.0):
        self.value = initial_value

    def get(self):
        return self.value

    def set(self, new_value):
        self.value = new_value
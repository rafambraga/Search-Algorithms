import math

class Vector2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def add(other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def subtract(other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def scale(scalar):
        return Vector2D(scalar * self.x, scalar * self.y)

    def abs():
        return math.sqrt(self.x * self.x + self.y * self.y)

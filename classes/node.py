
class Node:
    def __init__(self, node_id, x_pos, y_pos):
        self.id = node_id
        self.x = x_pos
        self.y = y_pos

    def __repr__(self):
        return str(self.id)
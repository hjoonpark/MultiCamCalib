import numpy as np

class Checkerboard():
    def __init__(self, n_cols, n_rows, sqr_size):
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.sqr_size = sqr_size
        self.chb_pts = self._create_chb_points()

    def _create_chb_points(self):
        #        columns
        #   +z o -------- +y
        # rows |
        #      |
        #      | +x

        pts = []
        for r in range(self.n_rows):
            for c in range(self.n_cols):
                pts.append([r*self.sqr_size, c*self.sqr_size, 0])
        return np.float32(pts)
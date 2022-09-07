"""Field of view computation."""

import numpy as np


class FieldOfView(object):
    """Compute field of view prefactors.

    The field of view angle twophi is given in degrees.
    out_of_view_factor is C in the paper.
    """

    def __init__(self, phi=None, out_of_view_factor=None):
        phi = phi or np.full((4,), 100.)
        out_of_view_factor = out_of_view_factor or np.full((4,), 0.5)
        self.cosphi = np.cos(phi / 180.0 * np.pi)
        self.out_of_view_factor = out_of_view_factor

    def __call__(self, vtypes, desired_direction, forces_direction):
        """Weighting factor for field of view.

        desired_direction : e, rank 2 and normalized in the last index.
        forces_direction : f, rank 3 tensor.
        """
        in_sight = (
                np.einsum("aj,abj->ab", desired_direction, forces_direction)
                > np.linalg.norm(forces_direction, axis=-1)
        )
        for i in range(0, 4):
            in_sight[vtypes == i] *= self.cosphi[i]
        out = self.out_of_view_factor * np.ones_like(in_sight)
        out[in_sight] = 1.0
        np.fill_diagonal(out, 0.0)
        return out

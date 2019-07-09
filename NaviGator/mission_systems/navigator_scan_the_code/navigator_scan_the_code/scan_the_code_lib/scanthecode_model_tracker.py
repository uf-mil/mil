"""Tracks a ScanTheCodeModel."""
from scanthecode_model import ScanTheCodeModel


class ScanTheCodeModelTracker:
    """Class that contains the functionality to track a ScanTheCode."""

    def __init__(self):
        """Initialize the ModelTracker Class."""
        self.model = None
        self.colors = []

    def update_model(self, frame, points, debug):
        """
        Update the current model that we are tracking.

        Returns if the model has observed three different colors
        """
        if self.model is None:
            self.model = ScanTheCodeModel(points, frame)

        mission_status, colors = self.model.check_for_colors(debug)
        if(mission_status):
            self.colors = colors
            return True

        self.model.points = points
        self.model.frame = frame

        return False

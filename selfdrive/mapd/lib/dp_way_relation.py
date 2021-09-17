from .WayRelation import WayRelation
from selfdrive.config import Conversions as CV

class DPWayRelation(WayRelation):
  @property
  def speed_limit(self):
    self._speed_limit = super().speed_limit
    if self._speed_limit is not None:
      return self._speed_limit

    junction_string = self.way.tags.get("junction")
    if junction_string is not None and junction_string == "roundabout":
      self._speed_limit = 25.0 * CV.KPH_TO_MS
      return self._speed_limit

    return self._speed_limit 

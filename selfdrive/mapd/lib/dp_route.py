from .Route import Route

class DPRoute(Route):
  def current_road_name(self):
    if not self.located:
      return None
    return self.current_wr.name 

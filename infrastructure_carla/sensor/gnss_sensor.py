import carla
import weakref

class GnssSensor(object):
  def __init__(self, parent_actor, carla_transform, gnss_params):
    self.sensor = None
    self.lat = 0.0
    self.lon = 0.0
    self.alt = 0.0
    if isinstance(parent_actor, carla.Actor):
      self._parent = parent_actor
      world = self._parent.get_world()
    else:
      self._parent = None
      world = parent_actor

    bp = world.get_blueprint_library().find('sensor.other.gnss')
    for key, value in gnss_params.items():
      bp.set_attribute(key, str(value))

    self.sensor = world.spawn_actor(bp, carla_transform, attach_to=self._parent)
    # We need to pass the lambda a weak reference to self to avoid circular
    # reference.
    weak_self = weakref.ref(self)
    self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

  @staticmethod
  def _on_gnss_event(weak_self, event):
    self = weak_self()
    if not self:
      return
    self.lat = event.latitude
    self.lon = event.longitude
    self.alt = event.altitude

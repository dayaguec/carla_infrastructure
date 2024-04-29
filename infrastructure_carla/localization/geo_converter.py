import ad_map_access as ad

from geometry_msgs.msg import (Pose, Point, Quaternion)

class GeoConverter:
  """Class to convert GPS Fix measures to Local provided a projection"""
  def __init__(self, geo_projection, altitude):
    self._altitude = altitude

    self._coordinate_transform = ad.map.point.CoordinateTransform()
    self._coordinate_transform.setGeoProjection(geo_projection)

  def toENU(self, geo_location):
    altitude = geo_location.altitude - self._altitude
    geo = ad.map.point.createGeoPoint(ad.map.point.Longitude(geo_location.longitude),
      ad.map.point.Latitude(geo_location.latitude),
      ad.map.point.Altitude(altitude))

    ENU_pt = self._coordinate_transform.Geo2ENU(geo)

    return Pose(position=Point(x=float(ENU_pt.x), y=float(ENU_pt.y), z=float(ENU_pt.z)),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

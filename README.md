# roadmap
Roadmap node for converting waypoints to polylines. 

Takes hardcoded waypoints and fits lanes through thw waypoints via a Clothoid and/or Cubic spline fitting. The polylines are outputted as `roadmap_msgs/RoadPolylineArray.msg` on `/roadmap/polylines`.

## Installing
All components should be installed automatically using (to be checked):

`rosdep install --from-paths src --ignore-src -r -y`

The xml reader depends on http://wiki.ros.org/asr_rapidxml.

## Usage
`roslaunch roadmap roadmap.launch`

See `settings.yaml` for high level settings
Currenly, the reader supports xml and yaml documents. See `maps/test_map.xml`, `maps/test_map.yaml` for example formats.

**Note:** Support for the yaml files is for backwards compatibility, but the xml format should be prefered.

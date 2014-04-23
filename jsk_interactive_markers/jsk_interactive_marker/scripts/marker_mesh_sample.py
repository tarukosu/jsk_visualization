#!/usr/bin/evn python
import rospy
from visualization_msgs.msg import Marker

rospy.init_node("marker_mesh_sample")

pub = rospy.Publisher('marker', Marker)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    mesh_marker = Marker()
    
    mesh_marker.header.frame_id = "/map"
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.pose.orientation.w = 1.0
    mesh_marker.scale.x = 1.0
    mesh_marker.scale.y = 1.0
    mesh_marker.scale.z = 1.0

    mesh_marker.type = Marker.MESH_RESOURCE
    #mesh_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
    mesh_marker.mesh_resource = "package://hrpsys_gazebo_tutorials/environment_models/3d-coord-arrow/meshes/nil_link_mesh.dae"
    mesh_marker.mesh_use_embedded_materials = True
    pub.publish(mesh_marker)
    r.sleep()

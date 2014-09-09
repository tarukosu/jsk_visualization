#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayMenu
import time
rospy.init_node("test_menu")
p = rospy.Publisher("test_menu", OverlayMenu)

while not rospy.is_shutdown():
  menu = OverlayMenu()
  menu.title = "First Menu"
  menu.menus = ["hoge"]
  menu.action = OverlayMenu.ACTION_SELECT
  p.publish(menu)
  time.sleep(4)

  menu.action = OverlayMenu.ACTION_CLOSE
  p.publish(menu)
  time.sleep(4)

  another_menu = OverlayMenu()
  menu.title = "Second Menu"
  menu.menus = ["fuga"]
  menu.current_index = 0
  menu.action = OverlayMenu.ACTION_SELECT
  p.publish(menu)
  time.sleep(4)
  menu.action = OverlayMenu.ACTION_CLOSE
  p.publish(menu)
  time.sleep(4)

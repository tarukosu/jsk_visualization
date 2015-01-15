/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "torus_array_display.h"
#include <jsk_topic_tools/color_utils.h>

#define PI 3.14159265

namespace jsk_rviz_plugins
{  
  TorusArrayDisplay::TorusArrayDisplay()
  {
    color_property_ = new rviz::ColorProperty("color", QColor(25, 255, 0),
                                              "color to draw the toruses",
                                              this, SLOT(updateColor()));
    alpha_property_ = new rviz::FloatProperty("alpha", 0.8,
                                              "alpha value to draw the toruses",
                                              this, SLOT(updateAlpha()));
    auto_color_property_ = new rviz::BoolProperty("auto color", false,
                                                  "change the color of the toruses automatically",
                                                  this, SLOT(updateAutoColor()));
  }
  
  TorusArrayDisplay::~TorusArrayDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete auto_color_property_;
  }

  QColor TorusArrayDisplay::getColor(size_t index)
  {
    if (auto_color_) {
      std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(index);
      return QColor(ros_color.r * 255.0, ros_color.g * 255.0, ros_color.b * 255.0,
                    ros_color.a * 255.0);
    }
    else {
      return color_;
    }
  }
  
  void TorusArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    updateColor();
    updateAlpha();
    updateAutoColor();

    large_dimension_ = 10;
    small_dimension_ = 10;
  }

  void TorusArrayDisplay::updateColor()
  {
    color_ = color_property_->getColor();
  }

  void TorusArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void TorusArrayDisplay::updateAutoColor()
  {
    auto_color_ = auto_color_property_->getBool();
  }

  void TorusArrayDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
  }

  void TorusArrayDisplay::allocateShapes(int num)
  {
    if (num > shapes_.size()) {
      for (size_t i = shapes_.size(); i < num; i++) {
        ShapePtr shape (new rviz::MeshShape(context_->getSceneManager()));
        shapes_.push_back(shape);
      }
    }
    else if (num < shapes_.size())
    {
      shapes_.resize(num);
    }
  }

  void
  TorusArrayDisplay::calcurateTriangleMesh(
                                           int large_dimension, int small_dimension,
                                           float large_radius, float small_radius,
                                           Ogre::Vector3 pos, Ogre::Quaternion q,
                                           std::vector<Triangle> &triangles,
                                           std::vector<Ogre::Vector3> &vertices,
                                           std::vector<Ogre::Vector3> &normals
                                           ){
    //Create Vertex List and Normal List
    for (int i = 0; i < large_dimension; i ++){
      float target_circle_x = large_radius * cos( ( i * 1.0/ large_dimension) * 2 * PI) ;
      float target_circle_y = large_radius * sin( ( i * 1.0 / large_dimension) * 2 * PI) ;
      for (int j = 0; j < small_dimension; j++){
        Ogre::Vector3 new_point;
        new_point.x = target_circle_x + small_radius * cos ( (j * 1.0 / small_dimension) * 2 * PI) * cos( ( i * 1.0/ large_dimension) * 2 * PI);
        new_point.y = target_circle_y + small_radius * cos ( (j * 1.0/ small_dimension) * 2 * PI) * sin( ( i * 1.0/ large_dimension) * 2 * PI);
        new_point.z = small_radius * sin ( (j * 1.0/ small_dimension) * 2 * PI);

        //new_point rotate
        new_point = q * new_point;
        //new_point translate
        new_point += pos;
        vertices.push_back(new_point);

        //GetNormals
        Ogre::Vector3 normal;
        normal.x = small_radius * cos ( (j * 1.0 / small_dimension) * 2 * PI) * cos( ( i * 1.0/ large_dimension) * 2 * PI);
        normal.y = small_radius * cos ( (j * 1.0/ small_dimension) * 2 * PI) * sin( ( i * 1.0/ large_dimension) * 2 * PI);
        normal.z = small_radius * sin ( (j * 1.0/ small_dimension) * 2 * PI);
        normal = q * normal;
        normals.push_back(normal);
      }
    }

    //Create Index List and push into triangles
    for(int i = 0; i < large_dimension; i++){
      for(int j = 0; j < small_dimension; j++){
        int target_index = i * large_dimension + j;
        int next_index = target_index + 1;
        if(next_index >= small_dimension * large_dimension)
          next_index = 0;
        int next_circle_target_index = target_index + large_dimension;
        if (next_circle_target_index >= large_dimension*small_dimension)
          next_circle_target_index -= large_dimension*small_dimension;
        int prev_circle_next_index = target_index - large_dimension + 1;
        if (prev_circle_next_index < 0)
          prev_circle_next_index += large_dimension*small_dimension;
        Triangle t1;
        t1.v1 = target_index;
        t1.v3 = next_index;
        t1.v2 = next_circle_target_index;
        Triangle t2;
        t2.v1 = target_index;
        t2.v2 = next_index;
        t2.v3 = prev_circle_next_index;
        triangles.push_back(t1);
        triangles.push_back(t2);
      }
    }
  }

  void TorusArrayDisplay::processMessage(const jsk_pcl_ros::TorusArray::ConstPtr& msg)
  {
    allocateShapes(msg->toruses.size());
    for (size_t i = 0; i < msg->toruses.size(); i++) {
      jsk_pcl_ros::Torus torus = msg->toruses[i];
      ShapePtr shape = shapes_[i];

      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      float large_radius = torus.large_radius;
      float small_radius = torus.small_radius;

      if(!context_->getFrameManager()->transform(torus.header, torus.pose,
                                                 position,
                                                 quaternion))
        {
          ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                     qPrintable( getName() ), torus.header.frame_id.c_str(),
                     qPrintable( fixed_frame_ ));
          return;
        }

      shape->clear();
      std::vector<Triangle> triangles;
      std::vector<Ogre::Vector3> vertices;
      std::vector<Ogre::Vector3> normals;

      calcurateTriangleMesh(large_dimension_, small_dimension_,
                            large_radius, small_radius,
                            position,quaternion,
                            triangles, vertices, normals);

      shape->estimateVertexCount(vertices.size());
      shape->beginTriangles();
      for (std::size_t j = 0 ; j < vertices.size() ; ++j)
        shape->addVertex(vertices[j], normals[j]);
      for (std::size_t j = 0 ; j < triangles.size() ; ++j)
        shape->addTriangle(triangles[j].v1, triangles[j].v2, triangles[j].v3);
      shape->endTriangles();
      QColor color = getColor(i);
      shape->setColor(color.red() / 255.0,
                      color.green() / 255.0,
                      color.blue() / 255.0,
                      alpha_);
    }
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::TorusArrayDisplay, rviz::Display )

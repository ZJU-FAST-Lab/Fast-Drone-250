#ifndef ELLIPSOIDS_VISUAL_H
#define ELLIPSOIDS_VISUAL_H

#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_geometry/ellipsoid.h>
#include <Eigen/Eigenvalues>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

namespace decomp_rviz_plugins {
  class EllipsoidArrayVisual {
    public:
      EllipsoidArrayVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node);

      virtual ~EllipsoidArrayVisual();

      void setMessage(const decomp_ros_msgs::EllipsoidArray::ConstPtr &msg);

      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      std::vector<std::unique_ptr<rviz::Shape>> objs_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}
#endif

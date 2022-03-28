#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <decomp_ros_msgs/EllipsoidArray.h>
#include <rviz/message_filter_display.h>
#include "ellipsoid_array_visual.h"

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual;

class EllipsoidArrayDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::EllipsoidArray> {
  Q_OBJECT
public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay();

protected:
  void onInitialize();

  void reset();

private Q_SLOTS:
  void updateColorAndAlpha();

private:
  void processMessage(const decomp_ros_msgs::EllipsoidArray::ConstPtr &msg);

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz::ColorProperty *color_property_;
  rviz::FloatProperty *alpha_property_;
};
}

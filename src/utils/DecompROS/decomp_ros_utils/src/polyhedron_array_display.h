#include <decomp_ros_msgs/PolyhedronArray.h>
#include <rviz/message_filter_display.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/load_resource.h>

#include "mesh_visual.h"
#include "bound_visual.h"
#include "vector_visual.h"
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/geometric_utils.h>

namespace decomp_rviz_plugins {
class PolyhedronArrayDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::PolyhedronArray> {
  Q_OBJECT
public:
  PolyhedronArrayDisplay();
  virtual ~PolyhedronArrayDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateMeshColorAndAlpha();
  void updateBoundColorAndAlpha();
  void updateVsColorAndAlpha();
  void updateState();
  void updateScale();
  void updateVsScale();

private:
  void processMessage(const decomp_ros_msgs::PolyhedronArray::ConstPtr &msg);
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  std::shared_ptr<MeshVisual> visual_mesh_;
  std::shared_ptr<BoundVisual> visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  rviz::ColorProperty *mesh_color_property_;
  rviz::ColorProperty *bound_color_property_;
  rviz::ColorProperty *vs_color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::FloatProperty *scale_property_;
  rviz::FloatProperty *vs_scale_property_;
  rviz::EnumProperty *state_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_E<vec_Vec3f> vertices_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

}

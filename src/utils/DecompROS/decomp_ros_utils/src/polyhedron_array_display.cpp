#include "polyhedron_array_display.h"

namespace decomp_rviz_plugins {

PolyhedronArrayDisplay::PolyhedronArrayDisplay() {
  mesh_color_property_ =
      new rviz::ColorProperty("MeshColor", QColor(0, 170, 255), "Mesh color.",
                              this, SLOT(updateMeshColorAndAlpha()));
  bound_color_property_ =
      new rviz::ColorProperty("BoundColor", QColor(255, 0, 0), "Bound color.",
                              this, SLOT(updateBoundColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 0.2,
      "0 is fully transparent, 1.0 is fully opaque, only affect mesh", this,
      SLOT(updateMeshColorAndAlpha()));

  scale_property_ = new rviz::FloatProperty("Scale", 0.1, "bound scale.", this,
                                            SLOT(updateScale()));

  vs_scale_property_ = new rviz::FloatProperty("VsScale", 1.0, "Vs scale.", this,
                                               SLOT(updateVsScale()));

  vs_color_property_ =
    new rviz::ColorProperty("VsColor", QColor(0, 255, 0), "Vs color.",
                            this, SLOT(updateVsColorAndAlpha()));



  state_property_ = new rviz::EnumProperty(
      "State", "Mesh", "A Polygon can be represented as two states: Mesh and "
                       "Bound, this option allows selecting visualizing Polygon"
                       "in corresponding state",
      this, SLOT(updateState()));
  state_property_->addOption("Mesh", 0);
  state_property_->addOption("Bound", 1);
  state_property_->addOption("Both", 2);
  state_property_->addOption("Vs", 3);

}

void PolyhedronArrayDisplay::onInitialize() { MFDClass::onInitialize(); }

PolyhedronArrayDisplay::~PolyhedronArrayDisplay() {}

void PolyhedronArrayDisplay::reset() {
  MFDClass::reset();
  visual_mesh_ = nullptr;
  visual_bound_ = nullptr;
  visual_vector_ = nullptr;
}

void PolyhedronArrayDisplay::processMessage(const decomp_ros_msgs::PolyhedronArray::ConstPtr &msg) {
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  vertices_.clear();
  vs_.clear();

  const auto polys = DecompROS::ros_to_polyhedron_array(*msg);

  for(const auto& polyhedron: polys){
    vec_E<vec_Vec3f> bds = cal_vertices(polyhedron);
    vertices_.insert(vertices_.end(), bds.begin(), bds.end());
    const auto vs = polyhedron.cal_normals();
    vs_.insert(vs_.end(), vs.begin(), vs.end());
  }

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedronArrayDisplay::visualizeMesh() {
  std::shared_ptr<MeshVisual> visual_mesh;
  visual_mesh.reset(new MeshVisual(context_->getSceneManager(), scene_node_));

  visual_mesh->setMessage(vertices_);
  visual_mesh->setFramePosition(position_);
  visual_mesh->setFrameOrientation(orientation_);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();
  visual_mesh->setColor(color.r, color.g, color.b, alpha);
  visual_mesh_ = visual_mesh;
}

void PolyhedronArrayDisplay::visualizeBound() {
  std::shared_ptr<BoundVisual> visual_bound;
  visual_bound.reset(new BoundVisual(context_->getSceneManager(), scene_node_));

  visual_bound->setMessage(vertices_);
  visual_bound->setFramePosition(position_);
  visual_bound->setFrameOrientation(orientation_);

  Ogre::ColourValue color = bound_color_property_->getOgreColor();
  visual_bound->setColor(color.r, color.g, color.b, 1.0);
  float scale = scale_property_->getFloat();
  visual_bound->setScale(scale);

  visual_bound_ = visual_bound;
}

void PolyhedronArrayDisplay::visualizeVs() {
  std::shared_ptr<VectorVisual> visual_vector;
  visual_vector.reset(new VectorVisual(context_->getSceneManager(), scene_node_));

  visual_vector->setMessage(vs_);
  visual_vector->setFramePosition(position_);
  visual_vector->setFrameOrientation(orientation_);

  Ogre::ColourValue color = vs_color_property_->getOgreColor();
  visual_vector->setColor(color.r, color.g, color.b, 1.0);

  visual_vector_ = visual_vector;
}

void PolyhedronArrayDisplay::visualizeMessage(int state) {
  switch (state) {
  case 0:
    visual_bound_ = nullptr;
    visual_vector_ = nullptr;
    visualizeMesh();
    break;
  case 1:
    visual_mesh_ = nullptr;
    visual_vector_ = nullptr;
    visualizeBound();
    break;
  case 2:
    visual_vector_ = nullptr;
    visualizeMesh();
    visualizeBound();
    break;
  case 3:
    visualizeVs();
    break;
  default:
    std::cout << "Invalid State: " << state << std::endl;
  }
}

void PolyhedronArrayDisplay::updateMeshColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = mesh_color_property_->getOgreColor();

  if(visual_mesh_)
    visual_mesh_->setColor(color.r, color.g, color.b, alpha);
}

void PolyhedronArrayDisplay::updateBoundColorAndAlpha() {
  Ogre::ColourValue color = bound_color_property_->getOgreColor();
  if(visual_bound_)
    visual_bound_->setColor(color.r, color.g, color.b, 1.0);
}


void PolyhedronArrayDisplay::updateState() {
  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedronArrayDisplay::updateScale() {
  float s = scale_property_->getFloat();
  if(visual_bound_)
    visual_bound_->setScale(s);
}

void PolyhedronArrayDisplay::updateVsScale() {
  float s = vs_scale_property_->getFloat();
  if(visual_vector_)
    visual_vector_->setScale(s);
}

void PolyhedronArrayDisplay::updateVsColorAndAlpha() {
  Ogre::ColourValue color = vs_color_property_->getOgreColor();
  if(visual_vector_)
    visual_vector_->setColor(color.r, color.g, color.b, 1);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::PolyhedronArrayDisplay, rviz::Display)

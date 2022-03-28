#include "bound_visual.h"

namespace decomp_rviz_plugins {
  BoundVisual::BoundVisual(Ogre::SceneManager *scene_manager,
                           Ogre::SceneNode *parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
  }

  BoundVisual::~BoundVisual() { scene_manager_->destroySceneNode(frame_node_); }

  void BoundVisual::setMessage(const vec_E<vec_Vec3f>& bds) {
    objs_.clear();

    if (bds.empty())
      return;

    size_t num_faces = bds.size();
    objs_.resize(num_faces);
    for (auto &it : objs_)
      it.reset(new rviz::BillboardLine(scene_manager_, frame_node_));

    int cnt = 0;
    for (const auto &vs : bds) {
      for (unsigned int i = 0; i <= vs.size(); i++) {
        if (i < vs.size()) {
          if(!std::isnan(vs[i](0)))
            objs_[cnt]->addPoint(Ogre::Vector3(vs[i](0), vs[i](1), vs[i](2)));
        }
        else {
          if(!std::isnan(vs[0](0)))
          objs_[cnt]->addPoint(Ogre::Vector3(vs[0](0), vs[0](1), vs[0](2)));
        }
      }
      cnt++;
    }
  }

  void BoundVisual::setFramePosition(const Ogre::Vector3 &position) {
    frame_node_->setPosition(position);
  }

  void BoundVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
    frame_node_->setOrientation(orientation);
  }

  void BoundVisual::setColor(float r, float g, float b, float a) {
    for (auto &it : objs_)
      it->setColor(r, g, b, a);
  }

  void BoundVisual::setScale(float s) {
    for (auto &it : objs_)
      it->setLineWidth(s);
  }
}

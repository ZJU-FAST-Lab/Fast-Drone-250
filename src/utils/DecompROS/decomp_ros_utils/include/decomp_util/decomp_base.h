/**
 * @file decomp_base.h
 * @brief Decomp Base Class
 */
#ifndef DECOMP_BASE_H
#define DECOMP_BASE_H

#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
//#include <decomp_geometry/geometry_utils.h>

/**
 * @brief Line Segment Class
 *
 * The basic element in EllipsoidDecomp
 */
template <int Dim>
class DecompBase {
  public:
    ///Null constructor
    DecompBase() {}
    /**
     * @brief Adding local bounding box around line seg
     * @param Dim Distance in corresponding axis
     *
     * This virtual bounding box is parallel to the line segment, the x,y,z axes are not w.r.t the world coordinate system, but instead, x-axis is parallel to the line, y-axis is perpendicular to the line and world z-axis, z-axis is perpendiculat to the line and y-axis
     */
    void set_local_bbox(const Vecf<Dim>& bbox) {
      local_bbox_ = bbox;
    }

    ///Import obstacle points
    void set_obs(const vec_Vecf<Dim> &obs) {
      // only consider points inside local bbox
      Polyhedron<Dim> vs;
      add_local_bbox(vs);
      obs_ = vs.points_inside(obs);
    }

    ///Get obstacel points
    vec_Vecf<Dim> get_obs() const { return obs_; }

    ///Get ellipsoid
    Ellipsoid<Dim> get_ellipsoid() const { return ellipsoid_; }

    ///Get polyhedron
    Polyhedron<Dim> get_polyhedron() const { return polyhedron_; }

    /**
     * @brief Inflate the line segment
     * @param radius the offset added to the long semi-axis
     */
    virtual void dilate(decimal_t radius = 0) = 0;

    /**
     * @brief Shrink the polyhedron
     * @param shrink_distance Shrink distance
     */
    virtual void shrink(double shrink_distance) {}
 protected:
    virtual void add_local_bbox(Polyhedron<Dim> &Vs) = 0;

    void find_polyhedron() {
      //**** find half-space
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = obs_;
      while (!obs_remain.empty()) {
        const auto v = ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v);
        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
        /*
           std::cout << "a: " << a.transpose() << std::endl;
           std::cout << "b: " << b << std::endl;
           */
      }

      polyhedron_ = Vs;
    }

    /// Obstacles, input
    vec_Vecf<Dim> obs_;

    /// Output ellipsoid
    Ellipsoid<Dim> ellipsoid_;
    /// Output polyhedron
    Polyhedron<Dim> polyhedron_;

    /// Local bounding box along the line segment
    Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()};
};
#endif

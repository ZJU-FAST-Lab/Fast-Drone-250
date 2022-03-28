/**
 * @file iterative_decomp.h
 * @brief IterativeDecomp Class
 */
#ifndef ITERATIVE_DECOMP_H
#define ITERATIVE_DECOMP_H

#include <decomp_util/ellipsoid_decomp.h>

/**
 * @brief IterativeDecomp Class
 *
 * Iteratively calls ElliseDecomp to form a safer Safe Flight Corridor that is away from obstacles
 */
template <int Dim>
class IterativeDecomp : public EllipsoidDecomp<Dim>
{
  public:
    ///Simple constructor
    IterativeDecomp() {}
    /**
     * @brief Basic constructor
     * @param origin The origin of the global bounding box
     * @param dim The dimension of the global bounding box
     */
    IterativeDecomp(const Vecf<Dim> &origin, const Vecf<Dim> &dim) :
        EllipsoidDecomp<Dim>(origin, dim) {}
    /**
     * @brief Decomposition thread
     * @param path_raw The path to dilate
     * @param iter_num Max iteration number
     * @param offset_x offset added to the long semi-axis, default is 0
     * @param res Resolution to downsample the path
     */
    void dilate_iter(const vec_Vecf<Dim> &path_raw, int iter_num = 5,
                decimal_t res = 0, decimal_t offset_x = 0) {
      vec_Vecf<Dim> path = res > 0 ? downsample(path_raw, res) : path_raw;
      this->dilate(path, offset_x);
      vec_Vecf<Dim> new_path = simplify(path);
      for (int i = 0; i < iter_num; i++) {
        if (new_path.size() == path.size())
          break;
        else {
          path = new_path;
          this->dilate(path, offset_x);
          new_path = simplify(path);
        }
      }
    }

  protected:
    /// Uniformly sample path into many segments
    vec_Vecf<Dim> downsample(const vec_Vecf<Dim> &ps, decimal_t d) {
      // subdivide according to length
      if (ps.size() < 2)
        return ps;
      vec_Vecf<Dim> path;
      for (unsigned int i = 1; i < ps.size(); i++) {
        decimal_t dist = (ps[i] - ps[i - 1]).norm();
        int cnt = std::ceil(dist / d);
        for (int j = 0; j < cnt; j++)
          path.push_back(ps[i - 1] + j * (ps[i] - ps[i - 1]) / cnt);
      }
      path.push_back(ps.back());
      return path;
    }

    /// Get closest distance
    decimal_t cal_closest_dist(const Vecf<Dim>& pt, const Polyhedron<Dim>& vs){
      decimal_t dist = std::numeric_limits<decimal_t>::infinity();
      for(const auto& it: vs.hyperplanes()){
        decimal_t d = std::abs(it.n_.dot(pt - it.p_));
        if(d < dist)
          dist = d;
      }
      return dist;
    }

    /// Remove redundant waypoints
    vec_Vecf<Dim> simplify(const vec_Vecf<Dim>& path) {
			if(path.size() <= 2)
				return path;

			Vecf<Dim> ref_pt = path.front();
			vec_Vecf<Dim> new_path;
			new_path.push_back(ref_pt);

			for(size_t i = 2; i < path.size(); i ++){
				if(this->polyhedrons_[i-1].inside(ref_pt) &&
					 cal_closest_dist(ref_pt, this->polyhedrons_[i-1]) > 0.1) {
				}
				else{
					ref_pt = path[i-1];
					new_path.push_back(ref_pt);
				}
			}
			new_path.push_back(path.back());
			return new_path;
		}
};

typedef IterativeDecomp<2> IterativeDecomp2D;

typedef IterativeDecomp<3> IterativeDecomp3D;
#endif

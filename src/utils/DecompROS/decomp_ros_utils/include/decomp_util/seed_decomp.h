/**
 * @file seed_decomp.h
 * @brief SeedDecomp Class
 */
#ifndef SEED_DECOMP_H
#define SEED_DECOMP_H

#include <decomp_util/decomp_base.h>

/**
 * @brief Seed Decomp Class
 *
 * Dilate around the given point
 */
template <int Dim>
class SeedDecomp : public DecompBase<Dim> {
  public:
    ///Simple constructor
    SeedDecomp() {};
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    SeedDecomp(const Vecf<Dim> &p) : p_(p) {}
    /**
     * @brief Inflate the seed with a sphere
     * @param radius Robot radius
     */
    void dilate(decimal_t radius) {
      this->ellipsoid_ = Ellipsoid<Dim>(radius * Matf<Dim, Dim>::Identity(), p_);
      this->find_polyhedron();
      add_local_bbox(this->polyhedron_);
    }

    /// Get the center
    Vecf<Dim> get_seed() const {
      return p_;
    }

  protected:
    ///Add the bounding box
    void add_local_bbox(Polyhedron<Dim> &Vs) {
      if(this->local_bbox_.norm() == 0)
        return;

      //**** virtual walls x-y-z
      Vecf<Dim> dir = Vecf<Dim>::UnitX();
      Vecf<Dim> dir_h = Vecf<Dim>::UnitY();

      Vecf<Dim> pp1 = p_ + dir_h * this->local_bbox_(1);
      Vecf<Dim> pp2 = p_ - dir_h * this->local_bbox_(1);
      Vs.add(Hyperplane<Dim>(pp1, dir_h));
      Vs.add(Hyperplane<Dim>(pp2, -dir_h));

      // along y
      Vecf<Dim> pp3 = p_ + dir * this->local_bbox_(0);
      Vecf<Dim> pp4 = p_ - dir * this->local_bbox_(0);
      Vs.add(Hyperplane<Dim>(pp3, dir));
      Vs.add(Hyperplane<Dim>(pp4, -dir));

      // along z
      if(Dim > 2) {
        Vecf<Dim> dir_v = Vecf<Dim>::UnitZ();
        Vecf<Dim> pp5 = p_ + dir_v * this->local_bbox_(2);
        Vecf<Dim> pp6 = p_ - dir_v * this->local_bbox_(2);
        Vs.add(Hyperplane<Dim>(pp5, dir_v));
        Vs.add(Hyperplane<Dim>(pp6, -dir_v));
      }
    }

    ///Seed location
    Vecf<Dim> p_;
};

typedef SeedDecomp<2> SeedDecomp2D;

typedef SeedDecomp<3> SeedDecomp3D;

#endif

//
// Created by hannes on 2018-01-15.
//

#ifndef TASERV2_SPLINE_BASE_H
#define TASERV2_SPLINE_BASE_H

#include <Eigen/Dense>

#include <entity/entity.h>
#include <entity/paramstore/empty_pstore.h>
#include <entity/paramstore/dynamic_pstore.h>
#include <taser/trajectories/trajectory.h>

namespace taser {
namespace trajectories {
namespace internal {
static const Eigen::Matrix4d M = (Eigen::Matrix4d() <<
                                                    1. / 6., 4. / 6.,  1. / 6., 0,
    -3. / 6.,       0,  3. / 6., 0,
    3. / 6., -6. / 6,  3. / 6., 0,
    -1. / 6.,   3./6., -3. / 6., 1./6.).finished();

static const Eigen::Matrix4d M_cumul = (Eigen::Matrix4d() <<
                                                          6. / 6.,  5. / 6.,  1. / 6., 0,
    0. / 6.,  3. / 6.,  3. / 6., 0,
    0. / 6., -3. / 6.,  3. / 6., 0,
    0. / 6.,  1. / 6., -2. / 6., 1. / 6.).finished();

struct SplineSegmentMeta : public entity::MetaData {
  double t0; // First valid time
  double dt; // Knot spacing
  size_t n; // Number of knots

  SplineSegmentMeta(double dt, double t0) :
      dt(dt),
      t0(t0),
      n(0) { };

  SplineSegmentMeta() :
      SplineSegmentMeta(1.0, 0.0) { };

  size_t NumParameters() const override {
    return n;
  }

  double MinTime() const {
      Validate();
      return t0;
  }

  double MaxTime() const {
    Validate();
    return t0 + (n-3) * dt;
  }

  void Validate() const {
    if (n < 4) {
      throw std::range_error("Spline had too few control points");
    }
  }
};

struct SplineMeta : public entity::MetaData {

  std::vector<SplineSegmentMeta> segments;

  size_t NumParameters() const override {
    /*return std::accumulate(segments.begin(), segments.end(),
                           0, [](int n, SplineSegmentMeta& meta) {
          return n + meta.NumParameters();
        });*/
    int n = 0;
    for (auto &segment_meta : segments) {
      n += segment_meta.NumParameters();
    }
    return n;
  }

  double MinTime() const {
    if (segments.size() == 1)
      return segments[0].MinTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }

  double MaxTime() const {
    if (segments.size() == 1)
      return segments[0].MaxTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }
};

template<typename T>
class AlwaysTrueValidator {
 public:
  static void Validate(const T&) {};
};

template<typename Type, int Size>
struct ControlPointInfo {
  using type = Type;
  const int size = Size;

  virtual void Validate(const Type&) = 0;
  virtual ceres::LocalParameterization* parameterization() const = 0;
};

template<
    typename T,
    typename _ControlPointInfo>
class SplineSegmentView : public TrajectoryView<T, SplineSegmentMeta> {
 public:
  using ControlPointType = typename _ControlPointInfo::type;
  using ControlPointMap = Eigen::Map<ControlPointType>;

  // Inherit constructor
  using TrajectoryView<T, SplineSegmentMeta>::TrajectoryView;

  const ControlPointMap ControlPoint(int i) const {
    return ControlPointMap(this->holder_->ParameterData(i));
  }

  ControlPointMap MutableControlPoint(int i) {
    return ControlPointMap(this->holder_->ParameterData(i));
  }

  T t0() const {
    return T(this->meta_.t0);
  }

  T dt() const {
    return T(this->meta_.dt);
  }

  size_t NumKnots() const {
    return this->meta_.n;
  }

  double MinTime() const override {
    return this->meta_.MinTime();
  }

  double MaxTime() const override {
    return this->meta_.MaxTime();
  }

  void CalculateIndexAndInterpolationAmount(T t, int& i0, T& u) const {
    T s = (t - t0()) / dt();
    i0 = PotentiallyUnsafeFloor(s);
    u = s - T(i0);
  }

 protected:
  int PotentiallyUnsafeFloor(double x) const {
    return static_cast<int>(std::floor(x));
  }

  // This way of treating Jets are potentially unsafe, hence the function name
  template<typename Scalar, int N>
  int PotentiallyUnsafeFloor(const ceres::Jet<Scalar, N>& x) const {
    return static_cast<int>(std::floor(x.a));
  };

  _ControlPointInfo control_point_info_;
};


template<typename T, typename MetaType, template<typename...> typename SegmentTemplate>
class SplineView : public TrajectoryView<T, MetaType> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using SegmentView = SegmentTemplate<T>;
  using Base = TrajectoryView<T, MetaType>;
 public:
  using ControlPointType = typename SegmentView::ControlPointType;
  using ControlPointMap = typename SegmentView::ControlPointMap;

  SplineView(const MetaType &meta, entity::ParameterStore<T> *holder) :
      Base(meta, holder) {
    std::cout << "SplineView::SplineView (with meta and holder)" << std::endl;
    size_t offset = 0;
    for (auto &segment_meta : meta.segments) {
      size_t length = segment_meta.NumParameters();
      segments.push_back(std::make_shared<SegmentView>(segment_meta, holder->Slice(offset, length)));
      std::cout << "SplineView: Adding view" << std::endl;
      offset += length;
    }
  }

  Result Evaluate(T t, int flags) const override {
    int parameter_offset = 0;
    for (auto &seg : segments) {
      if ((t >= seg->MinTime()) && (t < seg->MaxTime())) {
        return seg->Evaluate(t, flags);
      }
    }

    throw std::range_error("No segment found for time t");
  }

  const ControlPointMap ControlPoint(int i) const {
    return this->ConcreteSegmentViewOrError()->ControlPoint(i);
  }

  ControlPointMap MutableControlPoint(int i) {
    return this->MutableConcreteSegmentViewOrError()->MutableControlPoint(i);
  }

  double MinTime() const override{
    return ConcreteSegmentViewOrError()->MinTime();
  }

  double MaxTime() const override {
    return ConcreteSegmentViewOrError()->MaxTime();
  }

  double t0() const {
    return ConcreteSegmentViewOrError()->t0();
  }

  double dt() const {
    return ConcreteSegmentViewOrError()->dt();
  }

  int NumKnots() const {
    return ConcreteSegmentViewOrError()->NumKnots();
  }

  void CalculateIndexAndInterpolationAmount(T t, int& i0, T& u) const {
    return this->ConcreteSegmentViewOrError()->CalculateIndexAndInterpolationAmount(t, i0, u);
  }

 protected:
  std::vector<std::shared_ptr<SegmentView>> segments;

  const std::shared_ptr<SegmentView> ConcreteSegmentViewOrError() const {
    if (segments.size() == 1) {
      return segments[0];
    }
    else {
      std::cout << "Number of segments: " << segments.size() << std::endl;
      throw std::runtime_error("Spline had more than one segment!");
    }
  }

  std::shared_ptr<SegmentView> MutableConcreteSegmentViewOrError() {
    if (segments.size() == 1) {
      return segments[0];
    }
    else {
      std::cout << "Number of segments: " << segments.size() << std::endl;
      throw std::runtime_error("Spline had more than one segment!");
    }
  }

  virtual ceres::LocalParameterization* ControlPointParameterization() {
    return nullptr;
  }
};

using _SplineParamStore = entity::EmptyParameterStore<double>;


template<template<typename...> typename SegmentViewTemplate>
struct SplineFactory {

  template<typename T, typename MetaType>
  struct View : public SplineView<T, MetaType, SegmentViewTemplate> {
    using SplineView<T, MetaType, SegmentViewTemplate>::SplineView;
  };

  template<typename T, typename MetaType>
  struct SegmentView : public SegmentViewTemplate<T> {
    using SegmentViewTemplate<T>::SegmentViewTemplate;
  };

  struct SegmentEntity : public TrajectoryEntity<SegmentView,
                                                 SplineSegmentMeta,
                                                 entity::DynamicParameterStore<double>> {
    using Base = TrajectoryEntity<SegmentView, SplineSegmentMeta, entity::DynamicParameterStore<double>>;
    using ControlPointType = typename Base::ControlPointType;

    SegmentEntity(double dt, double t0) {
      this->meta_.dt = dt;
      this->meta_.t0 = t0;
      this->meta_.n = 0;
    }

    void AddToProblem(ceres::Problem &problem,
                      time_init_t times,
                      SplineSegmentMeta &meta,
                      std::vector<entity::ParameterInfo<double>> &parameters) const override {
      throw std::runtime_error("Segments do not know how to add to problem. This is handled by the Spline entity");
    }

    void AppendKnot(const ControlPointType& cp) {
      // 1) Validate the control point
      this->control_point_info_.Validate(cp);

      // 2) Create parameter data and set its value
      auto i = this->holder_->AddParameter(this->control_point_info_.size,
                                           this->control_point_info_.parameterization());
      this->MutableControlPoint(i) = cp;

      // 3) Update segment meta
      this->meta_.n += 1;
    }
  };
};

template<template<typename> typename SegmentViewTemplate>
class SplineEntity : public TrajectoryEntity<SplineFactory<SegmentViewTemplate>::template View,
                                             SplineMeta,
                                             _SplineParamStore> {
  using Base = TrajectoryEntity<SplineFactory<SegmentViewTemplate>::template View, SplineMeta, _SplineParamStore>;
  using ControlPointType = typename Base::ControlPointType;
  using SegmentType = typename SplineFactory<SegmentViewTemplate>::SegmentEntity;

  // Hidden constructor, not intended for user code
  SplineEntity(double dt, double t0, ceres::LocalParameterization* control_point_parameterization) :
      segment_entity_(std::make_shared<SegmentType>(dt, t0)),
      control_point_parameterization_(control_point_parameterization) {
    std::cout << "SplineEntity::SplineEntity (hidden)" << std::endl;
    this->segments.push_back(segment_entity_);
  };

 public:

  SplineEntity(double dt, double t0) :
      SplineEntity(dt, t0, this->ControlPointParameterization()) { };

  SplineEntity(double dt) :
      SplineEntity(dt, 0.0) { };

  SplineEntity() :
      SplineEntity(1.0) { };

  void AppendKnot(const ControlPointType& cp) {
    segment_entity_->AppendKnot(cp);
  }


 protected:
  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    SplineMeta &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    throw std::runtime_error("SplineEntity::AddToProblem not implemented yet!");
  }

  std::shared_ptr<SegmentType> segment_entity_;
  std::unique_ptr<ceres::LocalParameterization> control_point_parameterization_;
};

#if 0

template<template<typename> typename ViewTemplate>
class SplinedTrajectoryBase : public TrajectoryBase<ViewTemplate> {
 protected:
  // Hidden constructor, not intended for user code
  SplinedTrajectoryBase(double dt, double t0, std::unique_ptr<ceres::LocalParameterization> local_parameterization) :
      TrajectoryBase<ViewTemplate>(new dataholders::VectorHolder<double>()),
      local_parameterization_(std::move(local_parameterization)) {
    this->meta_.segments.push_back(detail::SplineSegmentMeta(dt, t0));
  };

 public:
  using Meta = typename TrajectoryBase<ViewTemplate>::Meta;
  using ControlPointType = typename ViewTemplate<double>::ControlPointType;
  using ControlPointMap = typename ViewTemplate<double>::ControlPointMap;
  using ControlPointValidator = typename ViewTemplate<double>::ControlPointValidator;
  const static int ControlPointSize = ViewTemplate<double>::ControlPointSize;

  SplinedTrajectoryBase(double dt, double t0) :
    SplinedTrajectoryBase(dt, t0, nullptr) { };

  SplinedTrajectoryBase(double dt) :
      SplinedTrajectoryBase(dt, 0.0) { };

  SplinedTrajectoryBase() :
      SplinedTrajectoryBase(1.0) { };

  double t0() const {
    return this->AsView().t0();
  }

  double dt() const {
    return this->AsView().dt();
  }

  size_t NumKnots() const {
    return this->AsView().NumKnots();
  }

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const {

    auto view = this->AsView();
    double master_dt = view.dt();
    double master_t0 = view.t0();
    int current_segment_start = 0;
    int current_segment_end = -1; // Negative signals no segment created yet

    // Times are guaranteed to be sorted correctly and t2 >= t1
    for (auto tt : times) {

      int i1, i2;
      double u_notused;
      view.CalculateIndexAndInterpolationAmount(tt.first, i1, u_notused);
      view.CalculateIndexAndInterpolationAmount(tt.second, i2, u_notused);

      // Create new segment, or extend the current one
      if (i1 > current_segment_end) {
        double segment_t0 = master_t0 + master_dt * i1;
        meta.segments.push_back(detail::SplineSegmentMeta(master_dt, segment_t0));
        current_segment_start = i1;
      }
      else {
        i1 = current_segment_end + 1;
      }

      auto& current_segment_meta = meta.segments.back();

      // Add parameters and update currently active segment meta
      for (int i=i1; i < (i2 + 4); ++i) {
        auto ptr = this->holder_->Parameter(i);
        size_t size = ControlPointSize;
        parameter_blocks.push_back(ptr);
        parameter_sizes.push_back(size);
        problem.AddParameterBlock(ptr, size, this->local_parameterization_.get());
        current_segment_meta.n += 1;
      }

      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    } // for times
  }

  ControlPointMap MutableControlPoint(size_t i) {
    return this->AsView().MutableControlPoint(i);
  }

  const ControlPointMap ControlPoint(size_t i) const {
    return this->AsView().ControlPoint(i);
  }

  void AppendKnot(const ControlPointType& cp) {
    ControlPointValidator::Validate(cp); // Or throw exception
    auto i = this->holder_->AddParameter(ControlPointSize);
    this->AsView().MutableControlPoint(i) = cp;
    // FIXME: Should check for single segment or give error
    this->meta_.segments[0].n += 1;
  }

  void ExtendTo(double t, const ControlPointType& fill_value) {
    while ((this->NumKnots() < 4) || (this->MaxTime() < t)) {
      this->AppendKnot(fill_value);
    }
  }

  const detail::SplineSegmentMeta& ConcreteSegmentMetaOrError() const {
    if (this->meta_.segments.size() == 1)
      return this->meta_.segments[0];
    else
      throw std::logic_error("Concrete spline had multiple segments. This should not happen!");
  }

 protected:
  std::unique_ptr<ceres::LocalParameterization> local_parameterization_;
};
#endif

} // namespace detail
} // namespace trajectories
} // namespace taser

#endif //TASERV2_SPLINE_BASE_H

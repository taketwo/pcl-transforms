#include <celero/Celero.h>

#include <pcl/common/io.h>

#include <transforms.hpp>
#include <pcl/common/transforms.h>

constexpr unsigned int CLOUD_SIZE = 640 * 480;
constexpr int SAMPLES = 50;
constexpr int ITERATIONS = 100;

typedef double Scalar;
Eigen::Matrix<Scalar, 4, 4> matrix;
Eigen::Transform<Scalar, 3, Eigen::Affine> tf;

template <typename PointT>
class TransformFixture : public celero::TestFixture
{
public:
  TransformFixture ()
  {
    tf.setIdentity ();
    tf.translate(Eigen::Matrix<Scalar, 3, 1>::Random ());
    tf.rotate (Eigen::Quaternion<Scalar> (0.1f, 0.2f, 0.3f, 1.0f));
    matrix = tf.matrix ();
  }

  virtual void setUp (int64_t)
  {
    cloud_in.resize (CLOUD_SIZE);
    cloud_out.resize (CLOUD_SIZE);
    for (size_t i = 0; i < CLOUD_SIZE; ++i)
    {
      Eigen::Matrix<float, sizeof (PointT) / 4, 1> r1 = Eigen::Matrix<float, sizeof (PointT) / 4, 1>::Random ();
      memcpy (&cloud_in[i], r1.data (), sizeof (PointT));
      Eigen::Matrix<float, sizeof (PointT) / 4, 1> r2 = Eigen::Matrix<float, sizeof (PointT) / 4, 1>::Random ();
      memcpy (&cloud_out[i], r2.data (), sizeof (PointT));
    }

    pcl::copyPointCloud (cloud_in, cloud_in_not_dense);
    for (size_t i = 0; i < CLOUD_SIZE; i += 2)
      *(reinterpret_cast<float*> (&cloud_in_not_dense[i]) + 3) = std::numeric_limits<float>::quiet_NaN ();
    cloud_in_not_dense.is_dense = false;
  }

  pcl::PointCloud<PointT> cloud_in;
  pcl::PointCloud<PointT> cloud_in_not_dense;
  pcl::PointCloud<PointT> cloud_out;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CELERO_MAIN

#define COMPARE(Name, Type, Function, CloudIn, Copy) \
  BASELINE_F (Name, PCL, TransformFixture<pcl::Type>, SAMPLES, ITERATIONS) \
  { \
    pcl::Function (CloudIn, cloud_out, tf, Copy); \
  } \
  BENCHMARK_F (Name, Proposed, TransformFixture<pcl::Type>, SAMPLES, ITERATIONS) \
  { \
    EIGEN_ASM_COMMENT(#Name "::begin"); \
    pcl::experimental::Function (CloudIn, cloud_out, tf, Copy); \
    EIGEN_ASM_COMMENT(#Name "::end"); \
  }

COMPARE(D_XYZ_NC, PointXYZ, transformPointCloud, cloud_in, false)
COMPARE(D_XYZRGBN_NC, PointXYZRGBNormal, transformPointCloudWithNormals, cloud_in, false)
COMPARE(D_XYZ_C, PointXYZ, transformPointCloud, cloud_in, true)
COMPARE(D_XYZRGBN_C, PointXYZRGBNormal, transformPointCloudWithNormals, cloud_in, true)
COMPARE(S_XYZ_NC, PointXYZ, transformPointCloud, cloud_in_not_dense, false)
COMPARE(S_XYZRGBN_NC, PointXYZRGBNormal, transformPointCloudWithNormals, cloud_in_not_dense, false)
COMPARE(S_XYZ_C, PointXYZ, transformPointCloud, cloud_in_not_dense, true)
COMPARE(S_XYZRGBN_C, PointXYZRGBNormal, transformPointCloudWithNormals, cloud_in_not_dense, true)

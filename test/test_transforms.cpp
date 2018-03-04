#include <gtest/gtest.h>

#include <pcl/pcl_tests.h>
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>

#include <transforms.hpp>

typedef ::testing::Types<Eigen::Transform<float, 3, Eigen::Affine>,
                         Eigen::Transform<double, 3, Eigen::Affine>,
                         Eigen::Matrix<float, 4, 4>,
                         Eigen::Matrix<double, 4,4> > TransformTypes;

template <typename Transform>
class Transforms : public ::testing::Test
{
 public:
  typedef typename Transform::Scalar Scalar;

  Transforms ()
  {
    identity_transform.setIdentity ();

    Eigen::Matrix<Scalar, 6, 1> r = Eigen::Matrix<Scalar, 6, 1>::Random ();
    Eigen::Transform<Scalar, 3, Eigen::Affine> tf;
    pcl::getTransformation (r[0], r[1], r[2], r[3], r[4], r[5], tf);
    random_transform = tf.matrix ();

    p_xyz_normal.resize (cloud_size);
    p_xyz_normal_trans.resize (cloud_size);
    for (size_t i = 0; i < cloud_size; ++i)
    {
      Eigen::Vector3f xyz = Eigen::Vector3f::Random ();
      Eigen::Vector3f normal = Eigen::Vector3f::Random ().normalized ();
      p_xyz_normal[i].getVector3fMap () = xyz;
      p_xyz_normal_trans[i].getVector3fMap () = (tf * xyz.template cast<typename Transform::Scalar> ()).template cast<float> ();
      p_xyz_normal[i].getNormalVector3fMap () = normal;
      p_xyz_normal_trans[i].getNormalVector3fMap () = (tf.rotation () * normal.template cast<typename Transform::Scalar> ()).template cast<float> ();
      p_xyz_normal[i].rgba = 0x12345678;
    }

    pcl::copyPointCloud(p_xyz_normal, p_xyz);
    pcl::copyPointCloud(p_xyz_normal_trans, p_xyz_trans);

    p_none.resize (cloud_size);
    for (size_t i = 0; i < cloud_size; ++i)
    {
      p_none[i].intensity = 42.0f;
    }
  }

  const Scalar abs_error = std::numeric_limits<Scalar>::epsilon () * 10;
  const size_t cloud_size = 100;

  Transform identity_transform;
  Transform random_transform;

  pcl::PointCloud<pcl::PointXYZ>           p_xyz, p_xyz_trans;                // XYZ
  pcl::PointCloud<pcl::PointXYZRGBNormal>  p_xyz_normal, p_xyz_normal_trans;  // XYZ and Normal
  pcl::PointCloud<pcl::Intensity>          p_none;                            // none of XYZ or Normal

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

TYPED_TEST_CASE (Transforms, TransformTypes);

TYPED_TEST (Transforms, PointCloudWithXYZ)
{
  {
    pcl::PointCloud<pcl::PointXYZ> p;
    pcl::experimental::transformPointCloud (this->p_xyz, p, this->identity_transform);
    ASSERT_METADATA_EQ (p, this->p_xyz);
    ASSERT_EQ (p.size (), this->p_xyz.size ());
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_XYZ_EQ (p[i], this->p_xyz[i]);
  }
  {
    pcl::PointCloud<pcl::PointXYZ> p;
    pcl::experimental::transformPointCloud (this->p_xyz, p, this->random_transform);
    ASSERT_METADATA_EQ (p, this->p_xyz);
    ASSERT_EQ (p.size (), this->p_xyz.size ());
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_trans[i], this->abs_error);
  }
}

TYPED_TEST (Transforms, PointCloudWithXYZAndNormal)
{
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::experimental::transformPointCloudWithNormals (this->p_xyz_normal, p, this->identity_transform);
    ASSERT_METADATA_EQ (p, this->p_xyz_normal);
    ASSERT_EQ (p.size (), this->p_xyz_normal.size ());
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_EQ (p[i], this->p_xyz_normal[i]);
      ASSERT_NORMAL_EQ (p[i], this->p_xyz_normal[i]);
    }
  }
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::experimental::transformPointCloudWithNormals (this->p_xyz_normal, p, this->random_transform);
    ASSERT_METADATA_EQ (p, this->p_xyz_normal);
    ASSERT_EQ (p.size (), this->p_xyz_normal.size ());
    for (size_t i = 0; i < p.size (); ++i)
    {
      ASSERT_XYZ_NEAR (p[i], this->p_xyz_normal_trans[i], this->abs_error);
      ASSERT_NORMAL_NEAR (p[i], this->p_xyz_normal_trans[i], this->abs_error);
    }
  }
}

TYPED_TEST (Transforms, PointCloudCopyAllFields)
{
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    pcl::experimental::transformPointCloudWithNormals (this->p_xyz_normal, p, this->random_transform, true);
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_EQ (p[i].rgba, this->p_xyz_normal[i].rgba);
  }
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p = this->p_xyz_normal;
    pcl::experimental::transformPointCloudWithNormals (p, p, this->random_transform, true);
    // Same source and target, other fields should not change
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_EQ (p[i].rgba, this->p_xyz_normal[i].rgba);
  }
}

TYPED_TEST (Transforms, PointCloudDoNotCopyAllFields)
{
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p;
    p.resize (this->p_xyz_normal.size ());
    for (size_t i = 0; i < p.size (); ++i)
      p[i].rgba = 0;
    pcl::experimental::transformPointCloudWithNormals (this->p_xyz_normal, p, this->random_transform, false);
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_EQ (p[i].rgba, 0);
  }
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> p = this->p_xyz_normal;
    pcl::experimental::transformPointCloudWithNormals (p, p, this->random_transform, false);
    // Same source and target, so other fields should not change
    for (size_t i = 0; i < p.size (); ++i)
      ASSERT_EQ (p[i].rgba, this->p_xyz_normal[i].rgba);
  }
}

int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

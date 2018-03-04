#pragma once

#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/pcl_macros.h>

#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

namespace detail
{

  /** A helper struct to apply an SO3 or SE3 transform to a 3D point.
    * Supports single and double precision transform matrices. */
  template<typename Scalar>
  struct Transformer
  {
    const Eigen::Matrix<Scalar, 4, 4>& tf;

    Transformer (const Eigen::Matrix<Scalar, 4, 4>& transform) : tf (transform) { };

    /** Apply SO3 transform (top-left corner of the transform matrix).
      * \param[in] src input 3D point (3 floats)
      * \param[out] tgt output 3D point (4 floats), can be the same as input */
    void so3 (const float* src, float* tgt) const
    {
      const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
      tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2]);
      tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2]);
      tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2]);
    }

    /** Apply SE3 transform.
      * \param[in] src input 3D point (3 floats)
      * \param[out] tgt output 3D point (4 floats), can be the same as input */
    void se3 (const float* src, float* tgt) const
    {
      const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
      tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2] + tf (0, 3));
      tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2] + tf (1, 3));
      tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2] + tf (2, 3));
    }
  };

#if defined(__SSE2__)
  /** Optimized version for single-precision transform using SSE2 intrinsics. */
  template<>
  struct Transformer<float>
  {
    __m128 c[4];

    Transformer(const Eigen::Matrix4f& tf)
    {
      for (size_t i = 0; i < 4; ++i)
        c[i] = _mm_load_ps (tf.col (i).data ());
    }

    void so3 (const float* src, float* tgt) const
    {
      __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
      __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
      __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
      _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, p2)));
    }

    void se3 (const float* src, float* tgt) const
    {
      __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
      __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
      __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
      _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, _mm_add_ps(p2, c[3]))));
    }
  };

  /** Optimized version for double-precision transform using SSE2 intrinsics. */
  template<>
  struct Transformer<double>
  {
    __m128d c[4][2];

    Transformer(const Eigen::Matrix4d& tf)
    {
      for (size_t i = 0; i < 4; ++i)
      {
        c[i][0] = _mm_load_pd (tf.col (i).data () + 0);
        c[i][1] = _mm_load_pd (tf.col (i).data () + 2);
      }
    }

    void so3 (const float* src, float* tgt) const
    {
      __m128d xx = _mm_cvtps_pd (_mm_load_ps1 (&src[0]));
      __m128d p0 = _mm_mul_pd (xx, c[0][0]);
      __m128d p1 = _mm_mul_pd (xx, c[0][1]);

      for (size_t i = 1; i < 3; ++i)
      {
        __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
        p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
        p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
      }

      _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
    }

    void se3 (const float* src, float* tgt) const
    {
      __m128d p0 = c[3][0];
      __m128d p1 = c[3][1];

      for (size_t i = 0; i < 3; ++i)
      {
        __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
        p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
        p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
      }

      _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
    }

  };
#endif

}

namespace pcl
{

namespace experimental
{

template <typename PointT, typename Scalar> void
transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                     pcl::PointCloud<PointT> &cloud_out,
                     const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                     bool copy_all_fields = true)
{
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_in.points.size ());
    if (copy_all_fields)
      cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    else
      cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  ::detail::Transformer<Scalar> tf (transform.matrix ());
  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
      tf.se3 (cloud_in[i].data, cloud_out[i].data);
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) ||
          !pcl_isfinite (cloud_in.points[i].y) ||
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      tf.se3 (cloud_in[i].data, cloud_out[i].data);
    }
  }
}

template <typename PointT, typename Scalar> void
transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                     pcl::PointCloud<PointT> &cloud_out,
                     const Eigen::Matrix<Scalar, 4, 4> &transform,
                     bool copy_all_fields = true)
{
  transformPointCloud (cloud_in, cloud_out, Eigen::Transform<Scalar, 3, Eigen::Affine> (transform), copy_all_fields);
}

template <typename PointT, typename Scalar> void
transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                pcl::PointCloud<PointT> &cloud_out,
                                const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                bool copy_all_fields = true)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    if (copy_all_fields)
      cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    else
      cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  ::detail::Transformer<Scalar> tf (transform.matrix ());
  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      tf.se3 (cloud_in[i].data, cloud_out[i].data);
      tf.so3 (cloud_in[i].data_n, cloud_out[i].data_n);
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) ||
          !pcl_isfinite (cloud_in.points[i].y) ||
          !pcl_isfinite (cloud_in.points[i].z))
        continue;

      tf.se3 (cloud_in[i].data, cloud_out[i].data);
      tf.so3 (cloud_in[i].data_n, cloud_out[i].data_n);
    }
  }
}

template <typename PointT, typename Scalar> void
transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out,
                                const Eigen::Matrix<Scalar, 4, 4> &transform,
                                bool copy_all_fields = true)
{
  transformPointCloudWithNormals (cloud_in, cloud_out, Eigen::Transform<Scalar, 3, Eigen::Affine> (transform), copy_all_fields);
}

}

}


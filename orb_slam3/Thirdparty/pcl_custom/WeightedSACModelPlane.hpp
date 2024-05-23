#include <pcl/sample_consensus/sac_model_plane.h>

namespace pcl {
template <typename PointT>
class WeightedSACModelPlane : public pcl::SampleConsensusModelPlane<PointT> {
public:
    using Ptr = boost::shared_ptr<WeightedSACModelPlane<PointT>>;
    using ConstPtr = boost::shared_ptr<const WeightedSACModelPlane<PointT>>;
  
    // Constructor
    WeightedSACModelPlane (const typename pcl::SampleConsensusModelPlane<PointT>::PointCloudConstPtr &cloud, 
                           const pcl::Indices &indices,
                           bool random = false) 
        : pcl::SampleConsensusModelPlane<PointT>::SampleConsensusModelPlane(cloud, indices, random) 
    {}

    // Override the countWithinDistance function
    std::size_t countWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold) const override
    {
        if (!this->isModelValid (model_coefficients))
            return (0);
        
        // if input is not of type PointXYZRGBA, run the base class implementation
        if (!std::is_same<PointT, pcl::PointXYZRGBA>::value)
            return pcl::SampleConsensusModelPlane<PointT>::countWithinDistance(model_coefficients, threshold);

        // Iterate through the 3d points and calculate the distances from them to the plane
        std::size_t nr_p = 0;
        double nr_p_d = 0.0;
        for (std::size_t i = 0; i < this->indices_->size (); ++i)
        {
            // Calculate the distance from the point to the plane normal as the dot product
            // D = (P-A).N/|N|
            Eigen::Vector4f pt ((*this->input_)[(*this->indices_)[i]].x,
                                (*this->input_)[(*this->indices_)[i]].y,
                                (*this->input_)[(*this->indices_)[i]].z,
                                1.0f);
            if (std::abs (model_coefficients.dot (pt)) < threshold)
            {
                nr_p_d += static_cast<int>((*this->input_)[(*this->indices_)[i]].a)/255.0;
            }
        }
        nr_p = static_cast<std::size_t>(nr_p_d);
        return nr_p;
    }
};
  
} // namespace pcl

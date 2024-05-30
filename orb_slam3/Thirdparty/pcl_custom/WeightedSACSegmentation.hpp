#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "Thirdparty/pcl_custom/WeightedSACModelPlane.hpp"

namespace pcl
{

    template <typename PointT>
    class WeightedSACSegmentation : public pcl::SACSegmentation<PointT>
    {
    public:
        using Base = pcl::SACSegmentation<PointT>;

        WeightedSACSegmentation() : Base() {}

        virtual bool initSACModel(const int model_type) override
        {
            switch (model_type)
            {
            case SACMODEL_PLANE:
            {
                this->model_.reset(new WeightedSACModelPlane<PointT>(this->input_, *this->indices_, this->random_));
                break;
            }
            default:
                // Call base class implementation for other model types
                return pcl::SACSegmentation<PointT>::initSACModel(model_type);
            }
            return true; // Return true if initialization was successful
        }
    };

} // namespace pcl

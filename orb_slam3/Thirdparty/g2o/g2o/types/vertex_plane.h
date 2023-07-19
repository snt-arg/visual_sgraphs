#ifndef G2O_VERTEX_PLANE_H_
#define G2O_VERTEX_PLANE_H_

#include "plane3d.h"
#include "io_helper.h"
#include "../../config.h"
#include "../core/base_vertex.h"
#include "../core/hyper_graph_action.h"
#include "g2o_types_slam3d_addons_api.h"

namespace g2o
{
    class G2O_TYPES_SLAM3D_ADDONS_API VertexPlane : public BaseVertex<3, Plane3D>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPlane();

        Vector3D color;

        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;
    };

} // namespace g2o
#endif

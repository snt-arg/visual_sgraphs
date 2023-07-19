#include "vertex_plane.h"
#include "../stuff/opengl_wrapper.h"

namespace g2o
{
    VertexPlane::VertexPlane() { color << 0.2, 0.2, 0.2; }

    bool VertexPlane::read(std::istream &is)
    {
        Vector4D lv;
        bool state = internal::readVector(is, lv);
        setEstimate(Plane3D(lv));
        state &= internal::readVector(is, color);
        return state;
    }

    bool VertexPlane::write(std::ostream &os) const
    {
        bool state = internal::writeVector(os, _estimate.toVector());
        state &= internal::writeVector(os, color);
        return state;
    }

} // namespace g2o

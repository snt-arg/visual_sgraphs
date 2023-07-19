#ifndef G2O_CORE_IO_HELPER_H
#define G2O_CORE_IO_HELPER_H

#include <Eigen/Core>
#include <iosfwd>

namespace g2o
{
    namespace internal
    {
        template <typename Derived>
        bool writeVector(std::ostream &os, const Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size(); i++)
                os << b(i) << " ";
            return os.good();
        }

        template <typename Derived>
        bool readVector(std::istream &is, Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size() && is.good(); i++)
                is >> b(i);
            return is.good() || is.eof();
        }
    }
}

#endif

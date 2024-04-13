#pragma once

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"
#include "g2o/types/slam2d/edge_se2.h"

namespace g2o
{

    /**
     * \brief Edge enforcing ICR constraint
     */
    class G2O_TYPES_SLAM2D_API Edge2ICR : public BaseBinaryEdge<2, SE2, VertexSE2, VertexSE2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Costruttore vuoto
        Edge2ICR() : BaseBinaryEdge<2, SE2, VertexSE2, VertexSE2>() {}

        // Definizione dell'errore,
        // potremmo anche solo fare in displacement e non includere il constraint
        // su theta
        void computeError()
        {
            const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
            const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);

            // ICR Constraint
            SE2 dT = v1->estimate().inverse() * v2->estimate();
            double rho = dT.translation().norm();
            double theta = dT.rotation().angle();
            //SE2 dTICR(rho * std::cos(theta / 2), rho * std::sin(theta / 2), theta );
            Eigen::Vector2d dTICR(rho * std::cos(theta / 2), rho * std::sin(theta / 2));

            // Error Computation
            //_error = (_inverseMeasurement * dTICR).toVector();
            _error = _measurement.translation() - dTICR;

            return;
        }

        // Default
        virtual bool read(std::istream &is)
        {
            Vector3 p;
            internal::readVector(is, p);
            setMeasurement(SE2(p));
            _inverseMeasurement = measurement().inverse();
            readInformationMatrix(is);
            return is.good() || is.eof();
        }

        // Default
        virtual bool write(std::ostream &os) const
        {
            internal::writeVector(os, measurement().toVector());
            return writeInformationMatrix(os);
        }

        // Default
        virtual void setMeasurement(const SE2 &m)
        {
            _measurement = m;
            _inverseMeasurement = m.inverse();
        }

        // Default
        virtual bool setMeasurementData(const double *d)
        {
            _measurement = SE2(d[0], d[1], d[2]);
            _inverseMeasurement = _measurement.inverse();
            return true;
        }

        // Default
        virtual bool getMeasurementData(double *d) const
        {
            Vector3 v = _measurement.toVector();
            d[0] = v[0];
            d[1] = v[1];
            d[2] = v[2];
            return true;
        }

        // Updated
        virtual int measurementDimension() const { return 3; }

    protected:
        SE2 _inverseMeasurement;
    };
} // namespace g2o
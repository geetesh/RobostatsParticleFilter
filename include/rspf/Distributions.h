#ifndef _DISTRIBUTION_H_
#define _DISTRIBUTION_H_

#include <armadillo>
#include "rspf/Parameterized.h"

namespace rspf {


    /*! \brief A continuous uniform random number generator. */
    class uniformPDF{
    public:

        uniformPDF();
        uniformPDF( double lower, double upper );
        uniformPDF( const PropertyTree& ptree );

        void SetBounds( double lower, double upper );
        double GetLowerBound() const;
        double GetUpperBound() const;

        double Sample();
        double GetProb( double meas ) const;

    protected:

        double density;
        double lowerBound;
        double scale; // Difference of upper and lower

    };

    /*! \brief A univariate Gaussian random number generator. */
    class normalPDF{
    public:

        normalPDF();
        normalPDF( double _mean, double variance );
        normalPDF( const PropertyTree& ptree );

        void SetMean( double _mean );
        void SetVariance( double variance );

        double GetMean() const;
        double GetVariance() const;

        double Sample();
        double GetProb( double meas ) const;

    protected:

        double mean;
        double sigma; // Square root of variance (std dev)

    };

    class exponentialPDF{
    public:
        exponentialPDF();
        exponentialPDF(double l, double v);

        void SetLambda(double val);
        void SetTrueVal(double val);
        double GetLambda();
        double GetTrueVal();

//        double Sample();
        double GetProb(double meas) const;

    protected:
        double trueMeas;
        double lambda;
    };

}

#endif

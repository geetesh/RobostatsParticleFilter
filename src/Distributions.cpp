#include "rspf/Distributions.h"

using namespace arma;

namespace rspf {

    uniformPDF::uniformPDF( double lower, double upper ) {
        SetBounds( lower, upper );
    }

    uniformPDF::uniformPDF( const PropertyTree& ptree ) {
        double lower = ptree.get<double>("lower_bound");
        double upper = ptree.get<double>("upper_bound");
        SetBounds( lower, upper );
    }

    void uniformPDF::SetBounds( double lower, double upper ) {
        if( lower >= upper ) {
            std::stringstream ss;
            ss << "Lower bound " << lower << " is less than upper bound " << upper;
            throw std::logic_error( ss.str() );
        }

        lowerBound = lower;
        scale = upper - lower;
        density = 1.0/scale;
    }

//    double UniformDistribution::GetLowerBound() const {
//        return lowerBound;
//    }

//    double UniformDistribution::GetUpperBound() const {
//        return lowerBound + scale;
//    }

    double uniformPDF::Sample() {
        double raw = randu();
        double transformed = lowerBound + raw*scale;
        return transformed;
    }

    double uniformPDF::GetProb( double meas ) const {

        if( meas < lowerBound || meas > (lowerBound + scale) ) {
            return 0.0;
        }
        return density;
    }

    normalPDF::normalPDF( double _mean, double variance ) {
        SetMean( _mean );
        SetVariance( variance );
    }

    normalPDF::normalPDF( const PropertyTree& ptree ) {
        SetMean( ptree.get<double>("mean") );
        SetVariance( ptree.get<double>("variance") );
    }

    void normalPDF::SetMean( double _mean ) {
        mean = _mean;
    }

    void normalPDF::SetVariance( double variance ) {
        if( variance < 0 ) {
            std::stringstream ss;
            ss << "Variance " << variance << " is negative." << std::endl;
            throw std::logic_error( ss.str() );
        }
        sigma = std::sqrt( variance );
    }

    double normalPDF::GetMean() const {
        return mean;
    }

    double normalPDF::GetVariance() const {
        return sigma*sigma;
    }

    double normalPDF::Sample() {
        double raw = randn();
//		ScalarType raw = SampleRaw();
        double transformed = sigma*raw + mean;
        return transformed;
    }

    double normalPDF::GetProb( double meas ) const {
        double a = 1/(sigma*sqrt(2*M_PI));
        double b = mean;
        double c = sigma;
        double d = 0;
        double y = a * std::exp(-std::pow((meas-b),2) / (2*std::pow(c,2))) + d;
        return y;
    }

}

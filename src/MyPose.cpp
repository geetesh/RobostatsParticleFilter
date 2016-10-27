#include "rspf/MyPose.h"
#include <cmath>

namespace rspf {

    MyPose::MyPose() :
        trans( Transform::Identity() ) {}

    rspf::MyPose::MyPose(double x, double y, double theta)
{
        Rotation rot( theta );
        Translation zero( 0, 0 );
        Translation lin( x, y );

        // Need this step for the transform composition to be valid
        Transform rTrans = rot*zero;
        
        trans = lin*rTrans;
    }

    MyPose::MyPose( const Matrix& m ) :
        trans( m ) {}
        

    // TODO Remove this because of inaccuracy!
    MyPose::MyPose( const Transform &t ) :
        trans( t ) {}

    MyPose::Matrix MyPose::ToMatrix() const {
        return trans.matrix();
    }

    MyPose::Transform MyPose::GetTransform() const {
        return trans;
    }

    MyPose::Vector MyPose::ToVector() const {
        Transform::ConstTranslationPart lin = trans.translation();
        Rotation rot(0);
        rot.fromRotationMatrix( trans.linear() );

        Vector ret;
        ret << lin(0), lin(1), rot.angle();
        return ret;
    }
    
    
   /*		double getX() const;
		double getY() const;
		double getTheta() const;*/
	double MyPose::getX() const {
		Transform::ConstTranslationPart lin = trans.translation();
		return lin(0);
	}
	double MyPose::getY() const {
		Transform::ConstTranslationPart lin = trans.translation();
		return lin(1);
	}   
	double MyPose::getTheta() const {
        Rotation rot(0);
        rot.fromRotationMatrix( trans.linear() );
		return rot.angle();
	}   
   
   
    MyPose MyPose::Inverse() const {
        return MyPose( trans.inverse() );
    }

    MyPose MyPose::operator*(const MyPose& other) const {
        return MyPose( trans*other.trans );
    }

    MyPose MyPose::operator/(const MyPose& other) const {
        return MyPose( trans*other.trans.inverse() );
    }

    void MyPose::Print(std::ostream& os) const {
        Vector vec = ToVector();
        os << vec(0) << " " << vec(1) << " " << vec(2);
    }

    std::ostream& operator<<(std::ostream& os, const MyPose& se2) {
        se2.Print(os);
        return os;
    }

}

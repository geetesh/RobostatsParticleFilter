#ifndef _MY_POSE_H_
#define _MY_POSE_H_

#include <Eigen/Geometry>
#include <iostream>

namespace rspf {

    class MyPose {
    public:

        typedef Eigen::Vector3d Vector;
        typedef Eigen::Transform<double, 2, Eigen::Isometry> Transform;
        typedef Eigen::Matrix<double, 3, 3> Matrix;
        typedef Eigen::Rotation2D<double> Rotation;
        typedef Eigen::Translation<double, 2> Translation;
        typedef Translation::VectorType TranslationVector;

        MyPose();
        explicit MyPose(double x, double y, double theta);
        explicit MyPose(const Transform& trans);
        explicit MyPose(const Matrix& m);
        explicit MyPose(const Rotation& r, const Translation& t);

        Matrix ToMatrix() const;
        Transform GetTransform() const;
        Vector ToVector() const;
        MyPose Inverse() const;
		double getX() const;
		double getY() const;
		double getTheta() const;

        MyPose operator*( const MyPose& other ) const;
        MyPose operator/( const MyPose& other ) const;

        friend std::ostream& operator<<( std::ostream& os, const MyPose& se2 );

    protected:

        Transform trans;

        virtual void Print( std::ostream& os ) const;

    };

    std::ostream& operator<<(std::ostream& os, const MyPose& se2);

}

#endif


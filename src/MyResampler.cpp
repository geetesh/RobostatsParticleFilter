#include "rspf/MyResampler.h"


namespace rspf
{

MyResampler::MyResampler(const rspf::PropertyTree &ptree):
    x_pdf(ptree.get_child("x_noise")),
    y_pdf(ptree.get_child("y_noise")),
    th_pdf(ptree.get_child("th_noise"))
{
    std::printf("\n x_noise \t: %f",x_pdf.GetVariance());
    std::printf("\n y_noise \t: %f",y_pdf.GetVariance());
    std::printf("\n th_noise \t: %f",th_pdf.GetVariance());
}

std::vector<Particle> MyResampler::resampleParticles(const std::vector<Particle> &particles)
{
    int numParticles = particles.size()/2;
    numParticles = numParticles < 1000 ? 1000 : numParticles;
    std::vector<Particle> new_particles(numParticles);
    delta = 0;
    for(int i=0;i<numParticles;i++)
        delta += particles[i].getW();

    delta = delta / (double)numParticles;

    uniformPDF r_pdf(0,delta);
    r = r_pdf.Sample();

//    for(int j=0;j<100;j++)
//        std::cout<<"\n"<<r_pdf.Sample();

//    std::cout<<"\nDelta:"<<delta;

    double weightsum = 0;
    int index = 0;
    for(int i=0;i<numParticles;i++)
    {
        weightsum += particles[i].getW();
        while(r<weightsum)
        {
            PoseSE2 pose = particles[i].getPose();
            PoseSE2 noise(x_pdf.Sample(),y_pdf.Sample(),th_pdf.Sample());
            pose = pose * noise;
//            Particle p;
//            p.setPose(pose);
            new_particles[index++].setPose(pose);
            r += delta;
        }
    }

//    std::cout<<"Length of old particle set vs new: "<<numParticles<<" , "<<index;

    return new_particles;
}



}

#ifndef PRIMITIVE
#define PRIMITIVE

#include <tuple>
#include <vector>

namespace Primitive
{
    class Primitive {
        protected:
            // start with a point and then we can overload for other primitives
            // and then finally links
            virtual double getShortestDistance(std::vector<double>) = 0;

            // Same as above start with point then overload
            virtual std::vector<double> getClosestPoint(std::vector<double>) = 0;

            // I'm not sure if this is necessary but this mentions it should be there
            // https://www.learncpp.com/cpp-tutorial/126-pure-virtual-functions-abstract-base-classes-and-interface-classes/
            virtual ~Primitive() {}

    };

    class Cylinder: public Primitive {
        private:
            float length;
            float radius;

        public:
            virtual double get_shortest_dist(std::vector<double>) = 0;
            virtual std::vector<double> get_closest_point(std::vector<double>) = 0;
    };

    class n_ellipsoid: public Primitive {
        private:
            //Primitve parameters

        public:
            virtual double get_shortest_dist(std::vector<double>) = 0;
            virtual std::vector<double> get_closest_point(std::vector<double>) = 0;
    };
}

#endif // PRIMITIVES
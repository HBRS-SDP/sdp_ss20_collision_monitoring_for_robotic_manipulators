#ifndef BASIC_SHAPES_H
#define BASIC_SHAPES_H

#include <tuple>
#include <vector>

namespace shapes
{
    class IShape
    {
        protected:
            // start with a point and then we can overload for other shapes
            // and then finally links
            virtual double get_shortest_dist(std::vector<double>) = 0;

            // Same as above start with point then overload
            virtual std::vector<double> get_closest_point(std::vector<double>) = 0;

            // I'm not sure if this is necessary but this mentions it should be there
            // https://www.learncpp.com/cpp-tutorial/126-pure-virtual-functions-abstract-base-classes-and-interface-classes/
            virtual ~IShape() {}

    };

    class Cylinder: public IShape
    {
        private:
            float length;
            float radius;

        public:
            virtual double get_shortest_dist(std::vector<double>) = 0;
            virtual std::vector<double> get_closest_point(std::vector<double>) = 0;
    };

    class n_ellipsoid: public IShape
    {
        private:
            //Shape parameters

        public:
            virtual double get_shortest_dist(std::vector<double>) = 0;
            virtual std::vector<double> get_closest_point(std::vector<double>) = 0;
    };
}

#endif // BASIC_SHAPES_H
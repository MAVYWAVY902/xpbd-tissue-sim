#ifndef __SDF_HPP
#define __SDF_HPP

#include <Eigen/Dense>

namespace Geometry
{

/** Abstract interface for a signed distance field (SDF).
 * A SDF is just a function F(x) that returns the distance to a shape boundary, where F(x) = 0.
 * Inside the shape, the distance is negative (hence it being a "signed" distance function).
 * Outside the shape, the distance is positive.
 * The gradient of the function has magnitude 1 and is in the direction of increasing distance.
 */
class SDF
{
    public:
    virtual ~SDF() = default;

    /** Evaluates F(x) - i.e. finds the distance from point x to the shape boundary.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
     */
    virtual double evaluate(const Eigen::Vector3d& x) const = 0;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const = 0;

};

} // namespace Geometry

#endif // __SDF_HPP
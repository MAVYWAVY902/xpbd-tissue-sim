#ifndef COLLISION_CONSTRAINT_HPP
#define COLLISION_CONSTRAINT_HPP

#include "solver/Constraint.hpp"

namespace Solver
{
class CollisionConstraint : public Constraint
{
    public:
    CollisionConstraint(XPBDMeshObject* vertex_obj, unsigned vertex_index, XPBDMeshObject* face_obj, unsigned face_vertex1, unsigned face_vertex2, unsigned face_vertex3)
        : Constraint(std::vector<PositionReference>({
        PositionReference(vertex_obj, vertex_index),
        PositionReference(face_obj, face_vertex1),
        PositionReference(face_obj, face_vertex2),
        PositionReference(face_obj, face_vertex3)}))
    {

    }

    /** Evaluates the current value of this constraint.
     * i.e. returns C(x)
     */
    inline double evaluate() const override
    {
        assert(0); // not implemented
        return 0;
    }

    /** Returns the gradient of this constraint in vector form.
     * i.e. returns delC(x)
     */
    inline Eigen::VectorXd gradient() const override
    {
        assert(0); // not implemented
        return Eigen::VectorXd();
    }

    /** Returns the value and gradient of this constraint.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     */
    inline Constraint::ValueAndGradient evaluateWithGradient() const override
    {
        assert(0);  // not implemented
        return ValueAndGradient(evaluate(), gradient());
    }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint calculation, if necessary
     */
    inline void evaluate(double* C, double* additional_memory) const override
    {
        double* a = additional_memory;
        double* inv_magA = a + 3;
        _a(a, inv_magA, inv_magA + 1);

        _evaluate(C, a, *inv_magA);
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values in the constraint gradient calculation, if necessary
     */
    inline void gradient(double* grad, double* additional_memory) const override
    {
        double* a = additional_memory;
        double* inv_magA = a + 3;
        _a(a, inv_magA, inv_magA + 1);

        _gradient(grad, a, *inv_magA, inv_magA + 1);     // additional memory needed for the gradient calculation (after F and X) is provided after X
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     * @param additional_memory - a pointer to some pre-allocated memory that will be used to compute intermediate values, if necessary
     */
    void evaluateWithGradient(double* C, double* grad, double* additional_memory) const override
    {
        double* a = additional_memory;
        double* inv_magA = a + 3;
        _a(a, inv_magA, inv_magA + 1);
        _evaluate(C, a, *inv_magA);
        _gradient(grad,a, *inv_magA, inv_magA + 1);
    }

    /** Returns the number of bytes of pre-allocated dynamic memory needed to do its computation. */
    inline unsigned memoryNeeded() const override
    {
        // 3 for a = (p2 - p1) x (p3 - p1)
        // 1 for 1/|a|
        // 3 for q-p1
        // 9 for (I/|a| - (aa^T)/|a|^3)
        // 3 for (q - p1)^T * (I/|a| - (aa^T)/|a|^3)
        // 3 for p_diff (i.e. (p3 - p2) or (p1 - p3))
        // 9 for skew matrix (i.e (p3 - p2)^)
        // 3 for (q - p1)^T * (I/|a| - (aa^T)/|a|^3) * skew(p_diff)
        return 34 * sizeof(double);
    }

    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    private:

    inline void _evaluate(double* C, const double* a, const double inv_magA) const
    {
        const double* q = _positions[0].position_ptr;
        const double* p1 = _positions[1].position_ptr;
        const double* p2 = _positions[2].position_ptr;
        const double* p3 = _positions[3].position_ptr;

        const bool point_not_over_triangle = !_pointOverTriangle(q, p1, p2, p3);
        // C = (q - p1)^T * a / |a| 
        *C = inv_magA * ( (q[0] - p1[0]) * a[0] + (q[1] - p1[1]) * a[1] + (q[2] - p1[2]) * a[2] ) + 1e-4;// + point_not_over_triangle*1000;

        // std::cout << "q: " << q[0] << ", " << q[1] << ", " << q[2] << std::endl;
        // std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
        // std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
        // std::cout << "p1: " << p3[0] << ", " << p3[1] << ", " << p3[2] << std::endl;
        // std::cout << "POINT OVER TRIANGLE: " << _pointOverTriangle(q, p1, p2, p3) << std::endl;
    }

    inline void _gradient(double* delC, const double* a, const double inv_magA, double* additional_memory) const
    {

        const double* q = _positions[0].position_ptr;
        const double* p1 = _positions[1].position_ptr;
        const double* p2 = _positions[2].position_ptr;
        const double* p3 = _positions[3].position_ptr;

        // delC wrt q
        delC[_gradient_vector_index[0]] = a[0] * inv_magA;
        delC[_gradient_vector_index[1]] = a[1] * inv_magA;
        delC[_gradient_vector_index[2]] = a[2] * inv_magA;


        double* qp1 = additional_memory; // stores (q - p1)
        _sub3(q, p1, qp1);
        double* D = qp1 + 3;    // stores (I/|a| - (aa^T)/|a|^3)
        _I_div_magA_minus_aaT_div_magA3(a, inv_magA, D);
        double* qp1_times_D = D + 9;    // stores (q - p1)^T * (I/|a| - (aa^T)/|a|^3) - which is a 1x3 vector
        _vec_mat_mul3(qp1, D, qp1_times_D);

        double* p_diff = qp1_times_D + 3;   // stores subtraction between two positions
        double* skew = p_diff + 3;          // stores skew matrix of p_diff
        double* res = skew + 9;             // stores the result of (q - p1)^T * (I/|a| - (aa^T)/|a|^3) * skew(p_diff)

        // delC wrt p1
        _sub3(p3, p2, p_diff);
        _skew(p_diff, skew);
        
        _vec_mat_mul3(qp1_times_D, skew, res);
        
        delC[_gradient_vector_index[3]] = -a[0] * inv_magA + res[0];
        delC[_gradient_vector_index[4]] = -a[1] * inv_magA + res[1];
        delC[_gradient_vector_index[5]] = -a[2] * inv_magA + res[2];

        // delC wrt p2
        _sub3(p1, p3, p_diff);
        _skew(p_diff, skew);

        _vec_mat_mul3(qp1_times_D, skew, res);

        delC[_gradient_vector_index[6]] = res[0];
        delC[_gradient_vector_index[7]] = res[1];
        delC[_gradient_vector_index[8]] = res[2];

        // delC wrt p3
        _sub3(p2, p1, p_diff);
        _skew(p_diff, skew);
        _vec_mat_mul3(qp1_times_D, skew, res);

        delC[_gradient_vector_index[9]] = res[0];
        delC[_gradient_vector_index[10]] = res[1];
        delC[_gradient_vector_index[11]] = res[2];

        // std::cout << "delC: " << delC[0] << ", " << delC[1] << ", " << delC[2] << ",\n" << delC[3] << ", " << delC[4] << ", " << delC[5] << ",\n" <<
        // delC[6] << ", " << delC[7] << ", " << delC[8] << ",\n" << delC[9] << ", " << delC[10] << ", " << delC[11] << std::endl; 
    }

    inline void _a(double* a, double* inv_magA, double* additional_memory) const
    {
        const double* p1 = _positions[1].position_ptr;
        const double* p2 = _positions[2].position_ptr;
        const double* p3 = _positions[3].position_ptr;

        double* p2p1 = additional_memory;
        _sub3(p2, p1, p2p1);

        double* p3p1 = additional_memory+3;
        _sub3(p3, p1, p3p1);

        // a = (p2 - p1) x (p3 - p1)
        _cross3(p2p1, p3p1, a);

        *inv_magA = 1/std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    }

    inline void _I_div_magA_minus_aaT_div_magA3(const double* a, const double inv_magA, double* aaT) const
    {
        const double inv_magA3 = inv_magA * inv_magA * inv_magA;
        aaT[0] = inv_magA - a[0]*a[0] * inv_magA3;
        aaT[1] = -a[1]*a[0] * inv_magA3;
        aaT[2] = -a[2]*a[0] * inv_magA3;
        aaT[3] = aaT[1];
        aaT[4] = inv_magA - a[1]*a[1] * inv_magA3;
        aaT[5] = -a[2]*a[1] * inv_magA3;
        aaT[6] = aaT[2];
        aaT[7] = aaT[5];
        aaT[8] = inv_magA - a[2]*a[2] * inv_magA3;
    }

    inline void _skew(const double* vec, double* mat) const
    {
        // assume mat is to be column-major
        mat[0] = 0;
        mat[1] = vec[2];
        mat[2] = -vec[1];
        mat[3] = -vec[2];
        mat[4] = 0;
        mat[5] = vec[0];
        mat[6] = vec[1];
        mat[7] = -vec[0];
        mat[8] = 0;
    }

    // from https://stackoverflow.com/questions/25512037/how-to-determine-if-a-point-lies-over-a-triangle-in-3d
    inline bool _pointOverTriangle(const double* p, const double* a, const double* b, const double* c) const
    {
        double ba[3], cb[3], ac[3], px[3], n[3], nx[3];

        _sub3(b, a, ba);
        _sub3(c, b, cb);
        _sub3(a, c, ac);

        _cross3(ac, ba, n);  // Same as n = ba X ca

        _sub3(p, a, px);
        _cross3(ba, px, nx);
        if (_dot3(nx, n) < 0) return 0;

        _sub3(p, b, px);
        _cross3(cb, px, nx);
        if (_dot3(nx, n) < 0) return 0;

        _sub3(p, c, px);
        _cross3(ac, px, nx);
        if (_dot3(nx, n) < 0) return 0;

        return 1;
    }

    /** Helper method for cross product between two 3-vectors v1 and v2, store the result in v3 */
    inline void _cross3(const double* v1, const double* v2, double* v3) const
    {
        v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
        v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    }

    inline double _dot3(const double* v1, const double* v2) const
    {
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    }

    inline void _sub3 (const double* v1, const double* v2, double* v3) const
    {
        v3[0] = v1[0] - v2[0];
        v3[1] = v1[1] - v2[1];
        v3[2] = v1[2] - v2[2];
    }

    /** Helper method for 1x3 vector times 3x3 matrix, resulting in 1x3 vector. */
    inline void _vec_mat_mul3(const double* vec, const double* mat, double* res) const
    {
        res[0] = vec[0]*mat[0] + vec[1]*mat[1] + vec[2]*mat[2];
        res[1] = vec[0]*mat[3] + vec[1]*mat[4] + vec[2]*mat[5];
        res[2] = vec[0]*mat[6] + vec[1]*mat[7] + vec[2]*mat[8];
    }
};
    
}

#endif // COLLISION_CONSTRAINT_HPP
#include "mesh/MeshProperties.h"

#include <cmath>

namespace destruct {

// For a tetrahedron with vertex 0 at the origin and the other three at
// positions a, b, c, the following identities hold (density = 1):
//
//   V             = (a . (b x c)) / 6
//   integral x    = V * (a_x + b_x + c_x) / 4
//   integral x^2  = V * ( (a_x + b_x + c_x)^2 + (a_x^2 + b_x^2 + c_x^2) ) / 20
//   integral x y  = V * ( (a_x + b_x + c_x)(a_y + b_y + c_y)
//                       + (a_x a_y + b_x b_y + c_x c_y) ) / 20
//
// The mesh integrals are obtained by summing these tetra integrals over
// every triangle (the sign of V takes care of "inside vs outside"
// concave regions automatically).
//
// After accumulating world-frame integrals, we use the parallel-axis
// theorem to translate the inertia tensor from the origin to the CoM.

MassProperties computeMassProperties(const Mesh& mesh) {
    MassProperties r;
    if (mesh.empty()) return r;

    double V = 0.0;
    double mx = 0.0, my = 0.0, mz = 0.0;          // integral x, y, z dV (times density=1)
    double ixx = 0.0, iyy = 0.0, izz = 0.0;       // integral x^2, y^2, z^2 dV
    double ixy = 0.0, iyz = 0.0, ixz = 0.0;       // integral x*y, y*z, x*z dV

    for (const Tri& t : mesh.triangles) {
        const glm::vec3& a = mesh.vertices[t.a];
        const glm::vec3& b = mesh.vertices[t.b];
        const glm::vec3& c = mesh.vertices[t.c];

        // Signed volume of (origin, a, b, c) times 6.
        double sixV = glm::dot(a, glm::cross(b, c));
        double Vt   = sixV / 6.0;

        // Sums of vertex coords over the tetra's non-origin vertices.
        double sx  = a.x + b.x + c.x;
        double sy  = a.y + b.y + c.y;
        double sz  = a.z + b.z + c.z;
        // Sums of squares.
        double sxx = a.x*a.x + b.x*b.x + c.x*c.x;
        double syy = a.y*a.y + b.y*b.y + c.y*c.y;
        double szz = a.z*a.z + b.z*b.z + c.z*c.z;
        double sxy_ = a.x*a.y + b.x*b.y + c.x*c.y;
        double syz_ = a.y*a.z + b.y*b.z + c.y*c.z;
        double sxz_ = a.x*a.z + b.x*b.z + c.x*c.z;

        V  += Vt;
        mx += Vt * sx / 4.0;
        my += Vt * sy / 4.0;
        mz += Vt * sz / 4.0;

        ixx += Vt * (sx*sx + sxx) / 20.0;
        iyy += Vt * (sy*sy + syy) / 20.0;
        izz += Vt * (sz*sz + szz) / 20.0;
        ixy += Vt * (sx*sy + sxy_) / 20.0;
        iyz += Vt * (sy*sz + syz_) / 20.0;
        ixz += Vt * (sx*sz + sxz_) / 20.0;
    }

    // If the mesh came out inside-out (negative signed volume), flip the sign
    // of all accumulators. This lets us handle arbitrary-orientation meshes
    // gracefully; fragments from clipping are usually fine, but user OBJs
    // sometimes aren't.
    if (V < 0.0) {
        V = -V;
        mx = -mx; my = -my; mz = -mz;
        ixx = -ixx; iyy = -iyy; izz = -izz;
        ixy = -ixy; iyz = -iyz; ixz = -ixz;
    }
    if (V <= 1e-12) return r;

    r.volume = static_cast<float>(V);
    r.centerOfMass = glm::vec3(
        static_cast<float>(mx / V),
        static_cast<float>(my / V),
        static_cast<float>(mz / V));

    // Inertia tensor about the origin, expanded:
    //   I = integral ( (r.r) I_3 - r r^T ) dV
    // Diagonal: integral (y^2 + z^2), (x^2 + z^2), (x^2 + y^2).
    // Off-diagonal (i, j): - integral (r_i r_j).
    glm::mat3 Iorigin{0.0f};
    Iorigin[0][0] = static_cast<float>(iyy + izz);
    Iorigin[1][1] = static_cast<float>(ixx + izz);
    Iorigin[2][2] = static_cast<float>(ixx + iyy);
    Iorigin[1][0] = Iorigin[0][1] = static_cast<float>(-ixy);
    Iorigin[2][0] = Iorigin[0][2] = static_cast<float>(-ixz);
    Iorigin[2][1] = Iorigin[1][2] = static_cast<float>(-iyz);

    // Parallel axis theorem: I_com = I_origin - V * ( (d.d) I - d d^T )
    // where d = centerOfMass (the translation from CoM to origin is -d, but
    // the formula has only second-order terms in d, so sign doesn't matter).
    const glm::vec3& d = r.centerOfMass;
    float dd = glm::dot(d, d);
    glm::mat3 shift{0.0f};
    shift[0][0] = r.volume * (dd - d.x * d.x);
    shift[1][1] = r.volume * (dd - d.y * d.y);
    shift[2][2] = r.volume * (dd - d.z * d.z);
    shift[1][0] = shift[0][1] = r.volume * (-d.x * d.y);
    shift[2][0] = shift[0][2] = r.volume * (-d.x * d.z);
    shift[2][1] = shift[1][2] = r.volume * (-d.y * d.z);

    r.inertiaAboutCoM = Iorigin - shift;
    return r;
}

} // namespace destruct

using Unity.Mathematics;

// the error quadric of Surface Simplification Using Quadric Error Metrics (Garland and Heckbert, sections 4 and 5), in
// the letters of Controlling Quadric Error Simplification with Line Quadrics (Liu, Rahimzadeh and Zordan, section 3):
// Q = [A b; bT c], and the error of a point x is E(x) = xT A x + 2 bT x + c. The quadric of a plane gives the squared
// distance to the plane, quadrics add, and the error of a sum is the sum of the errors.
//
// It is kept in doubles. An error is what is left when terms the size of the squared coordinates cancel. On a ball in
// a grid of 128^3 cells the midpoint of an edge is about a hundredth of a cell from the planes around it, and a quadric
// kept in floats is off by as much. One kept in doubles is off by a millionth of a cell
public struct BurstQuadric {

    // how small a pivot of A may be against its largest for the quadric to have a point of least error (see minimize).
    // The largest pivot over the smallest is about the condition number of A, and rounding moves the solution of
    // A x = -b by about that number times 1e-16 of the size of the coordinates: at this limit, a millionth of the grid
    public const double PivotLimit = 1e-10;

    // A, of which only the upper triangle is kept, b and c
    public double  a00, a01, a02, a11, a12, a22;
    public double3 b;
    public double  c;

    // the squared distance to the plane through the point with the unit normal. Equation 4 of Liu et al., and the
    // fundamental error quadric of Garland and Heckbert's section 5 with d = -n . p
    public static BurstQuadric plane( double3 normal, double3 point ) {
        var d = -math.dot( normal, point );

        return new BurstQuadric {
            a00 = normal.x * normal.x,
            a01 = normal.x * normal.y,
            a02 = normal.x * normal.z,
            a11 = normal.y * normal.y,
            a12 = normal.y * normal.z,
            a22 = normal.z * normal.z,
            b   = normal * d,
            c   = d * d
        };
    }

    // the plane of a triangle, and the area of the triangle, which is what its plane weighs in the quadric of a vertex
    // (Liu et al., equation 8). A triangle without area has no plane and gives false
    public static bool triangle( double3 v0, double3 v1, double3 v2, out BurstQuadric quadric, out double area ) {
        var normal = math.cross( v1 - v0, v2 - v0 );
        var length = math.length( normal );

        if( !( length > 0.0 ) ) {
            quadric = default;
            area    = 0.0;

            return false;
        }

        quadric = plane( normal / length, v0 );
        area    = 0.5 * length;

        return true;
    }

    // the plane through an edge that is square to one of the edge's triangles, which Garland and Heckbert add for every
    // triangle of an edge that is to keep its shape (section 6, preserving boundaries). The triangle is the edge and the
    // vertex opposite to it. An edge without length or a triangle without area gives false
    public static bool boundary( double3 start, double3 end, double3 opposite, out BurstQuadric quadric ) {
        var along  = end - start;
        var across = opposite - start;

        // the part of the way to the opposite vertex that is square to the edge, times the squared length of the edge.
        // It lies in the plane of the triangle, so the plane it is the normal of holds the edge and the normal of the
        // triangle. It has no length when the edge has none or the opposite vertex is on the line of the edge
        var normal = ( across * math.dot( along, along ) ) - ( along * math.dot( across, along ) );
        var length = math.length( normal );

        if( !( length > 0.0 ) ) {
            quadric = default;

            return false;
        }

        quadric = plane( normal / length, start );

        return true;
    }

    // the squared distance to the line through the point along the normal (Liu et al., section 4.1). Equations 13 to 15
    // add the quadrics of two planes through the line that are square to each other, with unit normals x and y. For any
    // such pair x xT + y yT = I - n nT, which is used here and needs no choice of x. A normal without length gives false
    public static bool line( double3 point, double3 normal, out BurstQuadric quadric ) {
        var length = math.length( normal );

        if( !( length > 0.0 ) ) {
            quadric = default;

            return false;
        }

        normal /= length;

        // A p, the part of the point that is square to the line
        var square = point - ( normal * math.dot( normal, point ) );

        quadric = new BurstQuadric {
            a00 = 1.0 - ( normal.x * normal.x ),
            a01 =     - ( normal.x * normal.y ),
            a02 =     - ( normal.x * normal.z ),
            a11 = 1.0 - ( normal.y * normal.y ),
            a12 =     - ( normal.y * normal.z ),
            a22 = 1.0 - ( normal.z * normal.z ),
            b   = -square,
            c   = math.dot( point, square )
        };

        return true;
    }

    // the quadric whose error is the sum of the two errors (Garland and Heckbert, section 4)
    public static BurstQuadric operator +( in BurstQuadric quadric, in BurstQuadric other ) {
        return new BurstQuadric {
            a00 = quadric.a00 + other.a00,
            a01 = quadric.a01 + other.a01,
            a02 = quadric.a02 + other.a02,
            a11 = quadric.a11 + other.a11,
            a12 = quadric.a12 + other.a12,
            a22 = quadric.a22 + other.a22,
            b   = quadric.b   + other.b,
            c   = quadric.c   + other.c
        };
    }

    // the quadric whose error is the weight times the error
    public static BurstQuadric operator *( in BurstQuadric quadric, double weight ) {
        return new BurstQuadric {
            a00 = quadric.a00 * weight,
            a01 = quadric.a01 * weight,
            a02 = quadric.a02 * weight,
            a11 = quadric.a11 * weight,
            a12 = quadric.a12 * weight,
            a22 = quadric.a22 * weight,
            b   = quadric.b   * weight,
            c   = quadric.c   * weight
        };
    }

    // E(x) = xT A x + 2 bT x + c. It is a sum of squares, which rounding can leave just below zero
    public readonly double evaluate( double3 x ) {
        var error = math.dot( x, this.multiply( x ) ) + ( 2.0 * math.dot( this.b, x ) ) + this.c;

        return math.max( error, 0.0 );
    }

    // the point of least error, where the gradient of E is zero: A x = -b (Garland and Heckbert, equation 1; Liu et al.,
    // equation 11). Planes that do not meet in a single point have none, and give false.
    //
    // A is symmetric and no error is below zero, so the system is solved as a Cholesky decomposition solves it, which
    // Liu et al. name for it: by eliminating one coordinate after another, starting with the one that has the largest
    // entry on the diagonal. What is on the diagonal for a coordinate when its turn comes is its pivot. Three planes
    // that meet in a point leave three pivots of similar size. Planes along a line leave nothing for the third but
    // rounding, and parallel planes nothing for the second
    public readonly bool minimize( out double3 x ) {
        x = default;

        // p is the coordinate with the largest entry on the diagonal, q and r are the other two
        var p = this.a00 >= this.a11 ? ( this.a00 >= this.a22 ? 0 : 2 ) : ( this.a11 >= this.a22 ? 1 : 2 );
        var q = ( p + 1 ) % 3;
        var r = ( p + 2 ) % 3;

        var first = this.diagonal( p );

        if( !( first > 0.0 ) ) {
            return false;
        }

        // what is left of the rows of q and r when p is eliminated from them
        var factorQ = this.offDiagonal( p, q ) / first;
        var factorR = this.offDiagonal( p, r ) / first;

        var leftQQ = this.diagonal( q )       - ( factorQ * this.offDiagonal( p, q ) );
        var leftRR = this.diagonal( r )       - ( factorR * this.offDiagonal( p, r ) );
        var leftQR = this.offDiagonal( q, r ) - ( factorQ * this.offDiagonal( p, r ) );

        var second = leftQQ;

        if( !( second > PivotLimit * first ) ) {
            return false;
        }

        var factor = leftQR / second;
        var third  = leftRR - ( factor * leftQR );

        if( !( third > PivotLimit * first ) ) {
            return false;
        }

        // the right hand side goes through the same eliminations, and the coordinates come out last to first
        var rightP = -this.b[p];
        var rightQ = -this.b[q] - ( factorQ * rightP );
        var rightR = -this.b[r] - ( factorR * rightP ) - ( factor * rightQ );

        var xr = rightR / third;
        var xq = ( rightQ / second ) - ( factor * xr );
        var xp = ( rightP / first ) - ( factorQ * xq ) - ( factorR * xr );

        x[p] = xp;
        x[q] = xq;
        x[r] = xr;

        return true;
    }

    // the entry of A on the diagonal
    private readonly double diagonal( int index ) {
        return index == 0 ? this.a00 : index == 1 ? this.a11 : this.a22;
    }

    // the entry of A off the diagonal at two different indices, in either order
    private readonly double offDiagonal( int index, int other ) {
        return index + other == 1 ? this.a01 : index + other == 2 ? this.a02 : this.a12;
    }

    // A x
    private readonly double3 multiply( double3 x ) {
        return new double3(
            ( this.a00 * x.x ) + ( this.a01 * x.y ) + ( this.a02 * x.z ),
            ( this.a01 * x.x ) + ( this.a11 * x.y ) + ( this.a12 * x.z ),
            ( this.a02 * x.x ) + ( this.a12 * x.y ) + ( this.a22 * x.z )
        );
    }

}

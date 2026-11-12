using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// how Occupancy-Based Dual Contouring splits the polygon around an edge into triangles (section 5.4, appendix A.1.5),
// on polygons made by hand. The edge runs along z through the middle of the polygon, from its solid end below to its
// empty end above, and the polygon goes around it so that its first three vertices face up
public class OccupancyPolygonTest {

    private static readonly float3 solidEnd = new( 0.0f, 0.0f, -0.5f );
    private static readonly float3 emptyEnd = new( 0.0f, 0.0f,  0.5f );

    // a flat square around the edge
    private static readonly float3[] square = {
        new( -0.5f, -0.5f, 0.0f ),
        new(  0.5f, -0.5f, 0.0f ),
        new(  0.5f,  0.5f, 0.0f ),
        new( -0.5f,  0.5f, 0.0f )
    };

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SplitJob : IJob {

        public float3 vertex0;
        public float3 vertex1;
        public float3 vertex2;
        public float3 vertex3;
        public float3 solidEnd;
        public float3 emptyEnd;
        public float3 edgePoint;

        public NativeArray<int>  split;
        public NativeArray<bool> concave;
        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            this.split[0] = BurstOccupancy.splitOf( this.vertex0, this.vertex1, this.vertex2, this.vertex3, this.solidEnd, this.emptyEnd, this.edgePoint, 1e-6f );

            // each vertex against the diagonal between its neighbors
            this.concave[0] = BurstOccupancy.isConcave( this.vertex0, this.vertex3, this.vertex1, this.solidEnd, this.emptyEnd );
            this.concave[1] = BurstOccupancy.isConcave( this.vertex1, this.vertex0, this.vertex2, this.solidEnd, this.emptyEnd );
            this.concave[2] = BurstOccupancy.isConcave( this.vertex2, this.vertex1, this.vertex3, this.solidEnd, this.emptyEnd );
            this.concave[3] = BurstOccupancy.isConcave( this.vertex3, this.vertex2, this.vertex0, this.solidEnd, this.emptyEnd );
        }

    }

    private static ( int split, bool[] concave ) splitOf( float3[] polygon, float3 edgePoint ) {
        using var split   = new NativeArray<int>( 1, Allocator.TempJob );
        using var concave = new NativeArray<bool>( 4, Allocator.TempJob );
        using var managed = new NativeArray<bool>( 1, Allocator.TempJob );

        new SplitJob {
            vertex0   = polygon[0],
            vertex1   = polygon[1],
            vertex2   = polygon[2],
            vertex3   = polygon[3],
            solidEnd  = solidEnd,
            emptyEnd  = emptyEnd,
            edgePoint = edgePoint,
            split     = split,
            concave   = concave,
            managed   = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        return ( split[0], concave.ToArray( ) );
    }

    private static float3[] with( int index, float3 vertex ) {
        var polygon = ( float3[] )square.Clone( );

        polygon[index] = vertex;

        return polygon;
    }

    // a flat polygon fits along either diagonal, and both pass as near the edge. It is split as the other algorithms
    // split it
    [Test]
    public void aFlatPolygonIsSplitAlongTheSecondDiagonal( ) {
        var ( split, concave ) = splitOf( square, float3.zero );

        CollectionAssert.AreEqual( new[] { false, false, false, false }, concave, "concave vertices" );

        Assert.AreEqual( BurstOccupancy.SecondDiagonal, split );
    }

    // a vertex pushed in past the diagonal between its neighbors is concave: that diagonal runs outside the polygon,
    // and the polygon is split along the other one, which ends in the vertex
    [Test]
    public void aPolygonIsSplitAlongTheDiagonalThatEndsInItsConcaveVertex( ) {
        var ( split, concave ) = splitOf( with( 1, new float3( -0.2f, 0.2f, 0.0f ) ), float3.zero );

        CollectionAssert.AreEqual( new[] { false, true, false, false }, concave, "concave vertices with the second pushed in" );

        Assert.AreEqual( BurstOccupancy.SecondDiagonal, split, "with the second vertex pushed in" );

        ( split, concave ) = splitOf( with( 0, new float3( 0.2f, 0.2f, 0.0f ) ), float3.zero );

        CollectionAssert.AreEqual( new[] { true, false, false, false }, concave, "concave vertices with the first pushed in" );

        Assert.AreEqual( BurstOccupancy.FirstDiagonal, split, "with the first vertex pushed in" );
    }

    // equations 3 and 4 look at the ends of the edge: a vertex is also concave when the triangle of its neighbors and
    // an end of the edge is on the wrong side of it. With the second and fourth vertices far above the empty end, the
    // diagonal between them passes over the edge instead of through it, which makes the first and third concave
    [Test]
    public void aVertexIsConcaveWhereTheDiagonalOfItsNeighborsPassesBeyondTheEdge( ) {
        var polygon = ( float3[] )square.Clone( );

        polygon[1].z = 2.0f;
        polygon[3].z = 2.0f;

        var ( split, concave ) = splitOf( polygon, float3.zero );

        CollectionAssert.AreEqual( new[] { true, false, true, false }, concave, "concave vertices" );

        Assert.AreEqual( BurstOccupancy.FirstDiagonal, split );
    }

    // with a concave vertex at either end of each diagonal neither fits, and the polygon becomes four triangles around
    // the 1D point of the edge
    [Test]
    public void aPolygonNeitherDiagonalFitsBecomesAFan( ) {
        var polygon = ( float3[] )square.Clone( );

        polygon[1] = new float3( -0.2f, 0.2f, 2.0f );
        polygon[3].z = 2.0f;

        var ( split, concave ) = splitOf( polygon, float3.zero );

        Assert.IsTrue( concave[1], "the second vertex, past the first diagonal" );
        Assert.IsTrue( concave[0] || concave[2], "a vertex at an end of the first diagonal, with the second passing beyond the edge" );

        Assert.AreEqual( BurstOccupancy.Fan, split );
    }

    // where both diagonals fit, the polygon is split along the one whose triangles pass nearer the 1D point of the edge,
    // which is on the surface. Here the second and fourth vertices are raised a little: the first diagonal passes
    // through the edge at the height of the first and third, the second at the height of the other two
    [Test]
    public void ofTwoDiagonalsThatFitTheOneNearerThePointOnTheEdgeIsTaken( ) {
        var polygon = ( float3[] )square.Clone( );

        polygon[1].z = 0.2f;
        polygon[3].z = 0.2f;

        var ( split, concave ) = splitOf( polygon, new float3( 0.0f, 0.0f, 0.0f ) );

        CollectionAssert.AreEqual( new[] { false, false, false, false }, concave, "concave vertices" );

        Assert.AreEqual( BurstOccupancy.FirstDiagonal, split, "with the point on the edge at the height of the first diagonal" );

        ( split, _ ) = splitOf( polygon, new float3( 0.0f, 0.0f, 0.2f ) );

        Assert.AreEqual( BurstOccupancy.SecondDiagonal, split, "with the point on the edge at the height of the second diagonal" );
    }

}

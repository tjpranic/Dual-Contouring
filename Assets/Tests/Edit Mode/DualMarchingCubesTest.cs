using System.Linq;
using NUnit.Framework;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Manifold Dual Contouring, section III: "each vertex is associated with a set of edges and each edge is associated with
// exactly one vertex", with a vertex for every piece of the contour in a cell
public class DualMarchingCubesTest {

    private const MaterialIndex _ = MaterialIndex.Void;
    private const MaterialIndex A = MaterialIndex.Material0;
    private const MaterialIndex B = MaterialIndex.Material1;

    /*
    corner layout:          edges:
         4--------------7        +------3-------+
        /|             /|       /|             /|
       / |            / |     10 |           11 |
      /  |           /  |     /  6           /  7
     5--------------6   |    +------2-------+   |
     |   |          |   |    |   |          |   |
     |   |          |   |    |   |          |   |
     |   3----------|---2    4   +------1---5---+
     |  /           |  /     |  /           |  /
     | /            | /      | 8            | 9
     |/             |/       |/             |/
     0--------------1        +------0-------+
    */
    private static readonly int[,] edgeCorners = {
        { 0, 1 }, { 3, 2 }, { 5, 6 }, { 4, 7 },
        { 0, 5 }, { 1, 6 }, { 3, 4 }, { 2, 7 },
        { 0, 3 }, { 1, 2 }, { 5, 4 }, { 6, 7 }
    };

    // the edges of each vertex in a cell with the given corners, as "0 4 8 | 1 7 9": vertices in order of their first edge
    private static string vertexEdges( params MaterialIndex[] corners ) {
        return string.Join(
            " | ",
            DualMarchingCubes.lookupVertexEdges( corners ).Select(
                ( edges ) => edges.OrderBy( ( edge ) => edge ).ToArray( )
            ).OrderBy(
                ( edges ) => edges[0]
            ).Select(
                ( edges ) => string.Join( " ", edges )
            )
        );
    }

    [Test]
    public void cellWithoutContourHasNoVertices( ) {
        Assert.AreEqual( "", vertexEdges( _, _, _, _, _, _, _, _ ), "empty" );
        Assert.AreEqual( "", vertexEdges( A, A, A, A, A, A, A, A ), "solid" );
        Assert.AreEqual( "", vertexEdges( A, B, A, B, A, B, A, B ), "solid, of two materials" );
    }

    [Test]
    public void cellCutByOneSheetHasOneVertex( ) {
        Assert.AreEqual( "0 4 8",   vertexEdges( A, _, _, _, _, _, _, _ ), "one solid corner" );
        Assert.AreEqual( "4 5 8 9", vertexEdges( A, A, _, _, _, _, _, _ ), "two solid corners along an edge" );
        Assert.AreEqual( "4 5 6 7", vertexEdges( A, A, A, A, _, _, _, _ ), "solid bottom face" );
        Assert.AreEqual( "3 7 11",  vertexEdges( A, A, A, A, A, A, A, _ ), "one empty corner" );
    }

    // solid corners that only meet diagonally are not connected, the contour cuts off each of them by itself
    [Test]
    public void solidCornersAcrossADiagonalHaveAVertexEach( ) {
        Assert.AreEqual( "0 4 8 | 1 7 9",  vertexEdges( A, _, A, _, _, _, _, _ ), "diagonally across a face" );
        Assert.AreEqual( "0 4 8 | 3 7 11", vertexEdges( A, _, _, _, _, _, _, A ), "diagonally across the cell" );

        Assert.AreEqual( "0 4 8 | 1 7 9 | 2 5 11 | 3 6 10", vertexEdges( A, _, A, _, A, _, A, _ ), "four corners that are all diagonal to each other" );
        Assert.AreEqual( "0 5 9 | 1 6 8 | 2 4 10 | 3 7 11", vertexEdges( _, A, _, A, _, A, _, A ), "the other four" );
    }

    // empty corners diagonally across a face are connected, through the gap between the solid corners of that face.
    // Which is the same face seen from the cell on the other side of it, so neighboring cells agree on where the
    // contour crosses the face
    [Test]
    public void emptyCornersAcrossAFaceDiagonalShareAVertex( ) {
        Assert.AreEqual( "0 1 4 7 8 9", vertexEdges( _, A, _, A, A, A, A, A ) );
    }

    // there is no face between corners diagonally across the cell, and nothing that tells whether the contour runs
    // around both or around each. It runs around each, as it does for solid corners
    [Test]
    public void emptyCornersAcrossTheCellHaveAVertexEach( ) {
        Assert.AreEqual( "0 4 8 | 3 7 11", vertexEdges( _, A, A, A, A, A, A, _ ) );
    }

    [Test]
    public void piecesOfTheContourAreToldApartBySolidAndEmptyOnly( ) {
        Assert.AreEqual( "4 5 8 9",       vertexEdges( A, B, _, _, _, _, _, _ ), "two materials along an edge" );
        Assert.AreEqual( "0 4 8 | 1 7 9", vertexEdges( A, _, B, _, _, _, _, _ ), "two materials diagonally across a face" );
    }

    [Test]
    public void everyIntersectedEdgeBelongsToExactlyOneVertex( ) {
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var corners = Enumerable.Range( 0, 8 ).Select( ( corner ) => ( solidCorners & ( 1 << corner ) ) != 0 ? A : _ ).ToArray( );

            var vertices = DualMarchingCubes.lookupVertexEdges( corners );

            for( var edge = 0; edge < edgeCorners.GetLength( 0 ); ++edge ) {
                var intersected = corners[edgeCorners[edge, 0]] != corners[edgeCorners[edge, 1]];

                Assert.AreEqual( intersected ? 1 : 0, vertices.Count( ( edges ) => edges.Contains( edge ) ), $"vertices of edge {edge} with solid corners {solidCorners}" );
            }

            // a piece of the contour is a polygon around at least three edges, and there are twelve edges to go around
            Assert.LessOrEqual( vertices.Length, 4, $"vertices with solid corners {solidCorners}" );

            foreach( var edges in vertices ) {
                Assert.GreaterOrEqual( edges.Length, 3, $"edges of a vertex with solid corners {solidCorners}" );
            }
        }
    }

}

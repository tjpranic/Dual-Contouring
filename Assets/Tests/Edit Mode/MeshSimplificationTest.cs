using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;

// BurstMeshSimplifier.simplify on the contours of every algorithm and implementation. There is no second simplifier to
// compare with, so the meshes are held to answers known in closed form, to what a simplification may not change about
// a mesh, and to the shapes they are contours of
public class MeshSimplificationTest : TopologyTest {

    // an algorithm with one of its implementations, and with or without the simplification of its own
    public enum Contour {
        UniformManaged,
        UniformBurst,
        UniformGPU,
        AdaptiveManaged,
        AdaptiveBurst,
        AdaptiveSimplified,
        ManifoldManaged,
        ManifoldBurst,
        ManifoldClustered,
        OccupancyBurst,
        OccupancyGPU
    }

    public enum Surface {
        Sphere,
        ThreeBalls,
        CubesAndSphere,
        SphereCutaway,
        TwoSpheres,
        SlabWithBand,
        TurnedShapes,
        NewShapes,
        HollowsAcrossFaces,
        BallAcrossBounds,
        PlateAcrossBounds
    }

    // surfaces that leave the grid, which spans -0.5 to 0.5: a ball through one side, and a plate with a band of a
    // second material that leaves with it, so that the lines between the materials run into the open edges
    private static readonly Shape[] ballAcrossBounds = { sphere( new Vector3( 0.3f, 0.07f, -0.04f ), 0.35f ) };

    private static readonly Shape[] plateAcrossBounds = {
        cuboid( Vector3.zero,                     new Vector3( 0.7f,  0.1f, 0.7f ) ),
        cuboid( new Vector3( 0.02f, 0.0f, 0.0f ), new Vector3( 0.15f, 0.1f, 0.7f ), Combination.Union, MaterialIndex.Material1 )
    };

    private static Shape[] shapesOf( Surface surface ) {
        return surface switch {
            Surface.Sphere             => shapesOf( Model.Sphere ),
            Surface.ThreeBalls         => shapesOf( Model.ThreeBalls ),
            Surface.CubesAndSphere     => shapesOf( Model.CubesAndSphere ),
            Surface.SphereCutaway      => shapesOf( Model.SphereCutaway ),
            Surface.TwoSpheres         => twoSpheres,
            Surface.SlabWithBand       => slabWithBand,
            Surface.TurnedShapes       => turnedShapes,
            Surface.NewShapes          => newShapes,
            Surface.HollowsAcrossFaces => hollowsAcrossFaces,
            Surface.BallAcrossBounds   => ballAcrossBounds,
            Surface.PlateAcrossBounds  => plateAcrossBounds,
            _                          => throw new Exception( "Unknown surface specified" )
        };
    }

    private static bool isManaged( Contour contour ) {
        return contour == Contour.UniformManaged || contour == Contour.AdaptiveManaged || contour == Contour.ManifoldManaged;
    }

    // the error threshold of the scenes whose octrees simplify
    private const float errorThreshold = 2e-4f;

    private readonly List<Object> meshes = new( );

    [TearDown]
    public void destroyMeshes( ) {
        foreach( var mesh in this.meshes ) {
            Object.DestroyImmediate( mesh );
        }
        this.meshes.Clear( );
    }

    // a voxelizer of 2^cellsExponent cells or leaves across
    private Voxelizer voxelizerOf( Contour contour, IEnumerable<Shape> shapes, int cellsExponent ) {
        var implementation = contour switch {
            Contour.UniformManaged or Contour.AdaptiveManaged or Contour.ManifoldManaged => ImplementationType.Managed,
            Contour.UniformGPU or Contour.OccupancyGPU                                   => ImplementationType.GPU,
            _                                                                            => ImplementationType.Burst
        };

        Voxelizer voxelizer;

        switch( contour ) {
            case Contour.UniformManaged:
            case Contour.UniformBurst:
            case Contour.UniformGPU: {
                voxelizer = this.createUniformVoxelizer( cellsExponent, implementation, QEFSolverType.SVD, shapes );
                break;
            }
            case Contour.AdaptiveManaged:
            case Contour.AdaptiveBurst:
            case Contour.AdaptiveSimplified: {
                var adaptive = this.createVoxelizer<AdaptiveDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );

                adaptive.simplification = contour == Contour.AdaptiveSimplified;
                adaptive.errorThreshold = errorThreshold;
                adaptive.topologySafe   = true;

                voxelizer = adaptive;
                break;
            }
            case Contour.ManifoldManaged:
            case Contour.ManifoldBurst:
            case Contour.ManifoldClustered: {
                var manifold = this.createVoxelizer<ManifoldDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );

                manifold.errorThreshold  = contour == Contour.ManifoldClustered ? errorThreshold : 0.0f;
                manifold.enforceManifold = true;

                voxelizer = manifold;
                break;
            }
            default: {
                voxelizer = this.createVoxelizer<OccupancyDualContouring>( cellsExponent, QEFSolverType.SVD, shapes );
                break;
            }
        }

        voxelizer.implementationType   = implementation;
        voxelizer.vertexNormals        = VertexNormals.Shared;
        voxelizer.keepDebugInformation = false;

        return voxelizer;
    }

    private Mesh simplify( Mesh mesh, Voxelizer voxelizer, double errorInCells ) {
        var simplified = BurstMeshSimplifier.simplify( mesh, errorInCells * voxelizer.cellSize, out _ );

        this.meshes.Add( simplified );

        return simplified;
    }

    private static int triangleCountOf( Mesh mesh ) {
        return Enumerable.Range( 0, mesh.subMeshCount ).Sum( ( subMesh ) => ( int )mesh.GetIndexCount( subMesh ) / 3 );
    }

    // the smallest area of the triangles of a mesh
    private static double smallestAreaOf( Mesh mesh ) {
        var positions = mesh.vertices;
        var indices   = mesh.triangles;
        var smallest  = double.PositiveInfinity;

        for( var index = 0; index < indices.Length; index += 3 ) {
            smallest = Math.Min( smallest, 0.5 * Vector3.Cross( positions[indices[index + 1]] - positions[indices[index]], positions[indices[index + 2]] - positions[indices[index]] ).magnitude );
        }
        return smallest;
    }

    // ---- shapes with known answers ----

    public enum Exact {
        Uniform,
        Adaptive,
        Manifold
    }

    private static Contour contourOf( Exact algorithm ) {
        return algorithm switch {
            Exact.Uniform  => Contour.UniformBurst,
            Exact.Adaptive => Contour.AdaptiveBurst,
            _              => Contour.ManifoldBurst
        };
    }

    // how far a corner of a simplified cuboid may be from the true corner, in cells. The corners of the contours
    // themselves are within 0.02 cells
    private const float cornerTolerance = 0.05f;

    private void assertVerticesAreTheCorners( Mesh mesh, Volume volume, Vector3[] corners, float cell, float tolerance, string what ) {
        var vertices = mesh.vertices;

        Assert.AreEqual( corners.Length, vertices.Length, $"{what}: vertices" );

        foreach( var corner in corners ) {
            var world   = ShapeContourTest.toWorld( volume, corner );
            var nearest = vertices.Min( ( vertex ) => Vector3.Distance( vertex, world ) );

            Assert.LessOrEqual( nearest / cell, tolerance, $"{what}: distance in cells from the corner at {world} to the nearest vertex" );
        }
    }

    // a shape with flat faces is the planes of its faces, and its simplest mesh has its corners for vertices and two
    // triangles less than twice as many triangles. The three algorithms that take their vertices from the planes of
    // their hermite data put a vertex at every corner, and every other vertex on a face or an edge, where a collapse
    // costs next to nothing.
    //
    // The corners are not kept exactly. A corner's vertex stands for little of the third face it is on while the edge
    // next to it has been collapsed into a vertex that stands for much of the other two, and the two can then collapse
    // into a point a little way down the edge within the error. Step 3 of the plan measured a corner 0.29 cells from
    // its place at an error of 0.1, on the edge it belongs to
    [Test]
    public void polyhedronBecomesItsCornersAndFaces(
        [Values] Exact algorithm,
        [Values( DensityFunction.Type.Pyramid, DensityFunction.Type.Octahedron, DensityFunction.Type.HexagonalPrism )] DensityFunction.Type type,
        [Values( 5, 6 )] int cellsExponent
    ) {
        var extents   = ShapeContourTest.extentsOf( type );
        var shape     = new Shape( type, ShapeContourTest.center, extents, Combination.Union, MaterialIndex.Material0 );
        var voxelizer = this.voxelizerOf( contourOf( algorithm ), new[] { shape }, cellsExponent );

        var simplified = this.simplify( this.voxelize( voxelizer ).mesh, voxelizer, 0.1 );
        var corners    = ShapeDistanceTest.polyhedronOf( type, extents ).corners;

        this.assertVerticesAreTheCorners( simplified, voxelizer.GetComponentsInChildren<Volume>( )[0], corners, voxelizer.cellSize, ( float )( distanceFactor * 0.1 ), $"{type}" );

        // and every vertex is on the shape, as near as the vertices of the contour are
        var volume = voxelizer.GetComponentsInChildren<Volume>( )[0];

        foreach( var vertex in simplified.vertices ) {
            Assert.LessOrEqual( Mathf.Abs( volume.sample( vertex ) ) / voxelizer.cellSize, cornerTolerance, $"distance in cells of the vertex at {vertex} from the shape" );
        }

        // a closed surface of triangles has two triangles less than twice its vertices
        Assert.AreEqual( ( corners.Length * 2 ) - 4, triangleCountOf( simplified ), "triangles" );
    }

    // an upright cuboid is exact on every grid and with every algorithm, occupancy-based dual contouring among them:
    // eight vertices and twelve triangles at any error
    [Test]
    public void cuboidBecomesEightVerticesAndTwelveTriangles(
        [Values( Contour.UniformBurst, Contour.AdaptiveBurst, Contour.ManifoldBurst, Contour.OccupancyBurst, Contour.UniformGPU, Contour.OccupancyGPU, Contour.UniformManaged )] Contour contour,
        [Values( 0.01, 0.1 )] double error
    ) {
        var extents   = new Vector3( 0.25f, 0.2f, 0.3f );
        var shape     = cuboid( ShapeContourTest.center, extents );
        var voxelizer = this.voxelizerOf( contour, new[] { shape }, isManaged( contour ) ? 4 : 5 );

        var simplified = this.simplify( this.voxelize( voxelizer ).mesh, voxelizer, error );

        var corners = Enumerable.Range( 0, 8 ).Select(
            ( corner ) => new Vector3( ( corner & 1 ) == 0 ? -extents.x : extents.x, ( corner & 2 ) == 0 ? -extents.y : extents.y, ( corner & 4 ) == 0 ? -extents.z : extents.z )
        ).ToArray( );

        this.assertVerticesAreTheCorners( simplified, voxelizer.GetComponentsInChildren<Volume>( )[0], corners, voxelizer.cellSize, cornerTolerance, "cuboid" );

        Assert.AreEqual( 12, triangleCountOf( simplified ), "triangles" );
    }

    // a cuboid with a band of a second material across it, flush with it: the two lines between the materials run
    // around it, straight on each face. Its simplest mesh has the eight corners and the eight points where the lines
    // cross the edges of the cuboid, and the lines stay where the band begins and ends
    [Test]
    public void cuboidOfTwoMaterialsKeepsTheLinesBetweenThem( [Values] Exact algorithm, [Values( 5, 6 )] int cellsExponent ) {
        var voxelizer  = this.voxelizerOf( contourOf( algorithm ), slabWithBand, cellsExponent );
        var simplified = this.simplify( this.voxelize( voxelizer ).mesh, voxelizer, 0.1 );
        var cell       = voxelizer.cellSize;

        Assert.AreEqual( 16, simplified.vertexCount,          "vertices" );
        Assert.AreEqual( 28, triangleCountOf( simplified ),   "triangles" );
        Assert.AreEqual( 2,  simplified.subMeshCount,         "sub-meshes" );

        var vertices = simplified.vertices;

        // the band is the sub-mesh that spans less of x. Its vertices are at the two planes where it begins and ends
        var spans = Enumerable.Range( 0, 2 ).Select( ( subMesh ) => simplified.GetTriangles( subMesh ).Distinct( ).Select( ( index ) => vertices[index].x ).ToArray( ) ).OrderBy( ( span ) => span.Max( ) - span.Min( ) ).ToArray( );

        foreach( var x in spans[0] ) {
            Assert.LessOrEqual( Mathf.Min( Mathf.Abs( x - bandMinimum ), Mathf.Abs( x - bandMaximum ) ) / cell, cornerTolerance, $"distance in cells of a vertex of the band at x = {x} from where the band begins or ends" );
        }

        Assert.AreEqual( 8, spans[0].Length, "vertices of the band" );
    }

    // a cylinder of a second material set into a slab, flush with it above and below: the line between the two
    // materials is a circle on either flat face, of a radius of 0.2
    private static readonly Vector3 discCenter = new( 0.02f, 0.0f, 0.03f );

    private static readonly Shape[] slabWithDisc = {
        cuboid( Vector3.zero, new Vector3( 0.4f, 0.1f, 0.4f ) ),
        new Shape( DensityFunction.Type.Cylinder, discCenter, new Vector3( 0.2f, 0.1f, 0.2f ), Combination.Union, MaterialIndex.Material1 )
    };

    private static float distanceToCircles( Vector3 point ) {
        var radial = new Vector2( point.x - discCenter.x, point.z - discCenter.z ).magnitude - 0.2f;
        var height = Mathf.Abs( point.y ) - 0.1f;

        return Mathf.Sqrt( ( radial * radial ) + ( height * height ) );
    }

    // the edges that triangles of two sub-meshes share, as their two vertices
    private static List<( int, int )> edgesBetweenSubMeshes( Mesh mesh ) {
        var edges = Enumerable.Range( 0, 2 ).Select( ( subMesh ) => {
            var indices = mesh.GetTriangles( subMesh );
            var set     = new HashSet<( int, int )>( );

            for( var index = 0; index < indices.Length; index += 3 ) {
                for( var corner = 0; corner < 3; ++corner ) {
                    var a = indices[index + corner];
                    var b = indices[index + ( ( corner + 1 ) % 3 )];

                    set.Add( ( Math.Min( a, b ), Math.Max( a, b ) ) );
                }
            }
            return set;
        } ).ToArray( );

        return edges[0].Where( ( edge ) => edges[1].Contains( edge ) ).ToList( );
    }

    // a flat face has no curvature to hold a round line on it: every collapse of the face costs nothing, and a vertex
    // of the line comes to stand for far more of the face than of the line. The error of the line is held to the
    // bound by itself for that reason (BurstMeshSimplifier.QuadricsJob), and the circle stays a circle: its vertices
    // and the middles of its edges are within the same few errors of it as vertices are of a curved surface
    [Test]
    public void roundLineOnAFlatFaceStaysRound( [Values( Contour.UniformBurst, Contour.ManifoldBurst )] Contour contour, [Values( 5, 6 )] int cellsExponent, [Values( 0.05, 0.25 )] double error ) {
        var voxelizer  = this.voxelizerOf( contour, slabWithDisc, cellsExponent );
        var mesh       = this.voxelize( voxelizer ).mesh;
        var simplified = this.simplify( mesh, voxelizer, error );
        var cell       = voxelizer.cellSize;

        var line      = edgesBetweenSubMeshes( simplified );
        var positions = simplified.vertices;

        Assert.Less( line.Count, edgesBetweenSubMeshes( mesh ).Count, "edges of the line, against the contour's" );

        foreach( var ( a, b ) in line ) {
            Assert.LessOrEqual( distanceToCircles( positions[a] ) / cell,                            distanceFactor * error, $"distance in cells of a vertex of the line from the circle, with {line.Count} edges" );
            Assert.LessOrEqual( distanceToCircles( ( positions[a] + positions[b] ) / 2.0f ) / cell, distanceFactor * error, $"distance in cells of the middle of an edge of the line from the circle, with {line.Count} edges" );
        }

        // the slab itself is flat all over and comes down to its corners and the two circles
        Assert.Less( triangleCountOf( simplified ), line.Count * 6, "triangles, against six for every edge of the line" );
    }

    // ---- what simplification keeps ----

    private static readonly double[] errors = { 0.02, 0.1, 0.5 };

    // the simplified mesh of every contour has the pieces, the handles and the lines of the contour: BurstMeshSimplifierTest
    // .shapeOf says what those are. It has no triangle without area, uses every vertex it has, and a larger error
    // leaves no more triangles than a smaller one
    [Test]
    public void simplifiedContourHasTheShapeOfTheContour( [Values] Contour contour, [Values] Surface surface ) {
        var voxelizer = this.voxelizerOf( contour, shapesOf( surface ), isManaged( contour ) ? 4 : 5 );
        var mesh      = this.voxelize( voxelizer ).mesh;

        if( mesh.vertexCount == 0 ) {
            Assert.Ignore( "the contour is empty" );
        }

        var shape    = BurstMeshSimplifierTest.shapeOf( mesh );
        var previous = triangleCountOf( mesh );

        foreach( var error in errors ) {
            var simplified = this.simplify( mesh, voxelizer, error );
            var triangles  = triangleCountOf( simplified );
            var what       = $"at an error of {error}, {mesh.vertexCount} vertices and {triangleCountOf( mesh )} triangles became {simplified.vertexCount} and {triangles}";

            Assert.AreEqual( shape, BurstMeshSimplifierTest.shapeOf( simplified ), $"{what}: shape" );

            Assert.Greater( smallestAreaOf( simplified ), 0.0, $"{what}: smallest triangle" );

            Assert.AreEqual( simplified.vertexCount, simplified.triangles.Distinct( ).Count( ), $"{what}: vertices that triangles use" );

            Assert.AreEqual( mesh.subMeshCount, simplified.subMeshCount, $"{what}: sub-meshes" );

            Assert.LessOrEqual( triangles, previous, $"{what}: triangles, against those at the error before" );

            previous = triangles;
        }
    }

    // every edge of a flat face along the grid costs exactly nothing. Of edges that cost the same the one whose ends
    // stand for less area is collapsed first, so the vertices of the face take their neighbours in turn. Going by the
    // order of the edges alone, one vertex takes its neighbours one after another and has much of the face around it
    // for most of the run, with every one of its edges worked out again at each collapse
    [Test]
    public void flatFaceIsNotGatheredAroundOneVertex( [Values( 5, 6 )] int cellsExponent ) {
        var voxelizer = this.voxelizerOf( Contour.UniformBurst, new[] { cube( ShapeContourTest.center, 0.3f ) }, cellsExponent );
        var mesh      = this.voxelize( voxelizer ).mesh;

        var simplified = BurstMeshSimplifier.simplify( mesh, 0.05 * voxelizer.cellSize, out var statistics );

        this.meshes.Add( simplified );

        Assert.AreEqual( 8, simplified.vertexCount, "vertices" );

        var each = ( double )statistics.evaluations / statistics.collapses;

        Debug.Log( $"{1 << cellsExponent}: {mesh.vertexCount} vertices, {statistics.collapses} collapses, {each:F1} edges worked out again for each" );

        Assert.LessOrEqual( each, 12.0, "edges worked out again for each collapse" );
    }

    // the same mesh simplifies the same way every time
    [Test]
    public void simplifyingTwiceGivesTheSameMesh( [Values( Contour.UniformBurst, Contour.OccupancyBurst, Contour.ManifoldClustered )] Contour contour, [Values( Surface.ThreeBalls, Surface.CubesAndSphere, Surface.NewShapes )] Surface surface ) {
        var voxelizer = this.voxelizerOf( contour, shapesOf( surface ), 5 );
        var mesh      = this.voxelize( voxelizer ).mesh;

        var first  = this.simplify( mesh, voxelizer, 0.1 );
        var second = this.simplify( mesh, voxelizer, 0.1 );

        Assert.Less( first.vertexCount, mesh.vertexCount, "vertices" );

        CollectionAssert.AreEqual( first.vertices, second.vertices, "vertices" );
        CollectionAssert.AreEqual( first.normals,  second.normals,  "normals" );

        for( var subMesh = 0; subMesh < first.subMeshCount; ++subMesh ) {
            CollectionAssert.AreEqual( first.GetTriangles( subMesh ), second.GetTriangles( subMesh ), $"triangles of sub-mesh {subMesh}" );
        }
    }

    // ---- how far it leaves the surface ----

    // the shapes whose density is their distance, the ellipsoid as a ball
    public static readonly DensityFunction.Type[] types = {
        DensityFunction.Type.Cuboid,
        DensityFunction.Type.Ellipsoid,
        DensityFunction.Type.Cylinder,
        DensityFunction.Type.Cone,
        DensityFunction.Type.Capsule,
        DensityFunction.Type.Torus,
        DensityFunction.Type.Pyramid,
        DensityFunction.Type.Octahedron,
        DensityFunction.Type.HexagonalPrism
    };

    // the error bound is on a root mean square distance to the planes of the surface a vertex stands for. The distance
    // of a vertex to the shape itself is another thing, and step 3 of the plan measured it: at most this many times
    // the error, or as far as the vertices of the contour are already
    private const double distanceFactor = 4.0;

    [Test]
    public void simplifiedVerticesStayNearTheShape(
        [Values( Contour.UniformBurst, Contour.ManifoldBurst, Contour.OccupancyBurst )] Contour contour,
        [ValueSource( nameof( types ) )] DensityFunction.Type type,
        [Values] ShapeContourTest.Turn turn
    ) {
        var extents   = type == DensityFunction.Type.Ellipsoid ? Vector3.one * 0.3f : ShapeContourTest.extentsOf( type );
        var shape     = new Shape( type, ShapeContourTest.center, ShapeContourTest.rotationOf( turn ), extents, Combination.Union, MaterialIndex.Material0 );
        var voxelizer = this.voxelizerOf( contour, new[] { shape }, 5 );
        var volume    = voxelizer.GetComponentsInChildren<Volume>( )[0];
        var mesh      = this.voxelize( voxelizer ).mesh;
        var cell      = voxelizer.cellSize;

        double furthest( Mesh of ) {
            return of.vertices.Max( ( vertex ) => Math.Abs( volume.sample( vertex ) ) ) / cell;
        }

        var contourDistance = furthest( mesh );

        foreach( var error in new[] { 0.05, 0.1, 0.25 } ) {
            var simplified = this.simplify( mesh, voxelizer, error );
            var distance   = furthest( simplified );

            Assert.LessOrEqual(
                distance, Math.Max( contourDistance * 1.001, distanceFactor * error ),
                $"at an error of {error}: the furthest vertex is {distance:G3} cells from the shape, {distance / error:F1} times the error; the furthest of the contour is {contourDistance:G3}"
            );
        }
    }

}

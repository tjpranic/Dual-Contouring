using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// each shape of the article other than the box and the ellipsoid, upright and turned, contoured by every algorithm, which
// run for each implementation in the fixtures at the end. The surface has to be closed, its vertices on the shape, and
// the sharp corners and edges of the shape have to be in the mesh, where Unity shows the shape
public abstract class ShapeContourTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    public enum Turn {
        Upright,
        Turned
    }

    public static readonly DensityFunction.Type[] types = ShapeDistanceTest.types;

    // the shapes with corners where faces meet in a point, and the cone's tip
    public static readonly DensityFunction.Type[] withCorners = {
        DensityFunction.Type.Cone,
        DensityFunction.Type.Pyramid,
        DensityFunction.Type.Octahedron,
        DensityFunction.Type.HexagonalPrism
    };

    // the shapes with edges where faces meet along a line, and the rims of the cylinder and the cone
    public static readonly DensityFunction.Type[] withEdges = {
        DensityFunction.Type.Cylinder,
        DensityFunction.Type.Cone,
        DensityFunction.Type.Pyramid,
        DensityFunction.Type.Octahedron,
        DensityFunction.Type.HexagonalPrism
    };

    private static readonly Vector3 center = new( 0.013f, -0.021f, 0.017f );

    // turned, every shape reaches at most 0.41 from its center, which keeps it inside the grid
    private static Vector3 extentsOf( DensityFunction.Type type ) {
        return type switch {
            DensityFunction.Type.Capsule => new Vector3( 0.15f, 0.3f, 0.15f ),
            DensityFunction.Type.Torus   => new Vector3( 0.3f,  0.1f, 0.3f  ),
            _                            => new Vector3( 0.25f, 0.2f, 0.25f )
        };
    }

    private static Quaternion rotationOf( Turn turn ) {
        return turn == Turn.Turned ? Quaternion.Euler( 30.0f, -60.0f, 75.0f ) : Quaternion.identity;
    }

    // the error threshold of the "Simplification" scenes
    private const float errorThreshold = 2e-4f;

    // the size of a cell
    private const float cell = 1.0f / 16.0f;

    // a binary search of six steps along an edge of a cell, 1 / 16 long, finds every intersection to within 1 / 1024 of
    // the surface. A vertex where up to three flat faces meet lies at most sqrt(3) / 1024 = 0.0017 from the shape
    private const float flatTolerance = 0.002f;

    // the smallest radius of curvature of the shape near a point in its own frame, infinite on a flat face. The cone's
    // side is curved across only, and less the farther it is from the tip: there the radius is the distance from the
    // axis over how far the side's normal leans out from it
    private static float radiusOfCurvatureAt( DensityFunction.Type type, Vector3 extents, Vector3 local ) {
        var across = Mathf.Min( extents.x, extents.z );

        switch( type ) {
            case DensityFunction.Type.Cylinder: {
                return across;
            }
            case DensityFunction.Type.Capsule: {
                return Mathf.Min( across, extents.y );
            }
            case DensityFunction.Type.Torus: {
                return Mathf.Min( extents.y, across );
            }
            case DensityFunction.Type.Cone: {
                var height = 2.0f * extents.y;
                var lean   = height / Mathf.Sqrt( ( height * height ) + ( across * across ) );

                return Mathf.Sqrt( ( local.x * local.x ) + ( local.z * local.z ) ) / lean;
            }
            default: {
                return float.PositiveInfinity;
            }
        }
    }

    // a vertex goes where the tangent planes of the intersections in its cell come closest. On a curved surface that is
    // off the surface by up to the sag of a circle of the radius of curvature across the cell's diagonal, 3 h^2 / 8 R for
    // a cell of size h. Near the cone's tip the radius shrinks to nothing, and so the tolerance grows without bound: the
    // tip's own vertex is held by everyCornerHasAVertex instead
    private static float vertexToleranceAt( DensityFunction.Type type, Vector3 extents, Vector3 local ) {
        return flatTolerance + ( 3.0f * cell * cell / ( 8.0f * radiusOfCurvatureAt( type, extents, local ) ) );
    }

    // corners where more than three faces meet: the octahedron's and the pyramid's tip. A cell next to one can see only
    // two of the faces, which meet nowhere but at the corner. Its vertex is then anywhere on the line where the planes
    // of the two cross, through the corner, and off the shape. Vertices that close to such a corner are left out, and
    // everyCornerHasAVertex holds the corner itself
    private static Vector3[] cornersOfMoreThanThreeFacesOf( DensityFunction.Type type, Vector3 extents ) {
        return type switch {
            DensityFunction.Type.Octahedron => cornersOf( type, extents ),
            DensityFunction.Type.Pyramid    => cornersOf( type, extents ).Take( 1 ).ToArray( ),
            _                               => Array.Empty<Vector3>( )
        };
    }

    private const float nearACorner = 2.0f * cell;

    // 16^3 cells or leaves either way. Simplified as the "Simplification" scenes are, or not at all, apart from Manifold
    // Dual Contouring clustering what has no error, which keeps flat faces flat
    private Voxelizer create( DensityFunction.Type type, Algorithm algorithm, Turn turn, bool simplified ) {
        var shapes = new[] { new Shape( type, center, extentsOf( type ), DensityFunction.Combination.Union, SurfaceExtractor.MaterialIndex.Material0 ).rotated( rotationOf( turn ) ) };

        switch( algorithm ) {
            case Algorithm.Uniform: {
                return this.createUniformVoxelizer( 4, this.implementation, QEFSolverType.SVD, shapes );
            }
            case Algorithm.Adaptive: {
                var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes );

                voxelizer.simplification = simplified;
                voxelizer.errorThreshold = errorThreshold;
                voxelizer.topologySafe   = true;

                return voxelizer;
            }
            case Algorithm.Manifold: {
                var voxelizer = this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, shapes );

                voxelizer.errorThreshold  = simplified ? errorThreshold : 0.0f;
                voxelizer.enforceManifold = true;

                return voxelizer;
            }
            default: {
                throw new Exception( "Unknown algorithm specified" );
            }
        }
    }

    // a point in the shape's own frame where Unity shows it: its transform takes the point, unscaled, to the world. The
    // voxelizer is at the origin, unturned and of unit scale, so its object space is world space
    private static Vector3 toWorld( Volume volume, Vector3 local ) {
        return volume.transform.TransformPoint( local.divide( volume.transform.localScale ) );
    }

    private static Vector3 toLocal( Volume volume, Vector3 world ) {
        return volume.transform.InverseTransformPoint( world ).multiply( volume.transform.localScale );
    }

    private static Vector3 onCircle( float radius, float height, float angle ) {
        return new Vector3( radius * Mathf.Cos( angle ), height, radius * Mathf.Sin( angle ) );
    }

    // the corners of the shape in its own frame: the cone's tip, and the corners of the shapes with flat faces
    private static Vector3[] cornersOf( DensityFunction.Type type, Vector3 extents ) {
        return type switch {
            DensityFunction.Type.Cone           => new[] { new Vector3( 0.0f, extents.y, 0.0f ) },
            DensityFunction.Type.Pyramid        => ShapeDistanceTest.polyhedronOf( type, extents ).corners,
            DensityFunction.Type.Octahedron     => ShapeDistanceTest.polyhedronOf( type, extents ).corners,
            DensityFunction.Type.HexagonalPrism => ShapeDistanceTest.polyhedronOf( type, extents ).corners,
            _                                   => Array.Empty<Vector3>( )
        };
    }

    // points along the edges and rims of the shape in its own frame: 33 along each straight edge, from end to end, and 64
    // around each rim
    private static Vector3[] edgePointsOf( DensityFunction.Type type, Vector3 extents ) {
        var across = Mathf.Min( extents.x, extents.z );
        var y      = extents.y;

        var points = new List<Vector3>( );

        void rim( float height ) {
            for( var step = 0; step < 64; ++step ) {
                points.Add( onCircle( across, height, step * 2.0f * Mathf.PI / 64.0f ) );
            }
        }

        switch( type ) {
            case DensityFunction.Type.Cylinder: {
                rim( -y );
                rim( y );
                break;
            }
            case DensityFunction.Type.Cone: {
                rim( -y );
                break;
            }
            default: {
                foreach( var ( start, end ) in ShapeDistanceTest.edgesOf( type, extents ) ) {
                    for( var step = 0; step <= 32; ++step ) {
                        points.Add( Vector3.Lerp( start, end, step / 32.0f ) );
                    }
                }
                break;
            }
        }

        return points.ToArray( );
    }

    // Uniform and Adaptive Dual Contouring give a cell one vertex, also a cell whose corners show several pieces of
    // surface, which an edge of a turned shape can leave (finding 6 of Shape Rotation). There the vertex is off the shape
    // and the pieces meet in it. Manifold Dual Contouring gives such a cell a vertex for each piece
    private static bool oneVertexPerCell( Algorithm algorithm ) {
        return algorithm != Algorithm.Manifold;
    }

    private ( Volume volume, Vector3[] vertices, int[] triangles, SurfaceExtractor.Voxel[] split ) contourWithCells( DensityFunction.Type type, Algorithm algorithm, Turn turn ) {
        var voxelizer    = this.create( type, algorithm, turn, false );
        var voxelization = this.voxelize( voxelizer );

        var split = oneVertexPerCell( algorithm )
            ? voxelization.voxels.Where( ( voxel ) => voxel.type != SurfaceExtractor.Voxel.Type.Internal && voxel.hasFeaturePoint && showsSeveralPieces( voxel ) ).ToArray( )
            : Array.Empty<SurfaceExtractor.Voxel>( );

        return ( voxelizer.GetComponentInChildren<Volume>( ), voxelization.mesh.vertices, voxelization.mesh.triangles, split );
    }

    [Test]
    public void surfaceIsClosed( [ValueSource( nameof( types ) )] DensityFunction.Type type, [Values] Algorithm algorithm, [Values] Turn turn, [Values] bool simplified ) {
        var topology = this.contour( this.create( type, algorithm, turn, simplified ) );

        if( oneVertexPerCell( algorithm ) ) {
            assertClosed( topology );
            return;
        }

        // one surface, without handles but for the torus' one
        assertManifold( topology );

        Assert.AreEqual( 1,                                                topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( type == DensityFunction.Type.Torus ? 0 : 2,       topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

    [Test]
    public void everyVertexLiesOnTheShape( [ValueSource( nameof( types ) )] DensityFunction.Type type, [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( volume, vertices, triangles, split ) = this.contourWithCells( type, algorithm, turn );

        var extents = extentsOf( type );
        var corners = cornersOfMoreThanThreeFacesOf( type, extents ).Select( ( corner ) => toWorld( volume, corner ) ).ToArray( );

        var used = triangles.Distinct( ).Select( ( index ) => vertices[index] ).Where(
            ( vertex ) => !split.Any( ( voxel ) => voxel.vertex == vertex ) && !corners.Any( ( corner ) => Vector3.Distance( vertex, corner ) <= nearACorner )
        ).ToArray( );

        Assert.Greater( used.Length, 0, "the mesh is empty" );

        // every shape is an exact signed distance, so its density is how far a point is from it. Each vertex is measured
        // against its own tolerance, and the one farthest out of it is reported
        var worst = used.Select( ( vertex ) => {
            var local     = toLocal( volume, vertex );
            var distance  = Mathf.Abs( volume.sample( vertex ) );
            var tolerance = vertexToleranceAt( type, extents, local );

            return ( local, distance, tolerance, share: distance / tolerance );
        } ).OrderByDescending( ( item ) => item.share ).First( );

        Assert.That( worst.share, Is.LessThanOrEqualTo( 1.0f ), $"the vertex at {worst.local:F4} in the shape's frame is {worst.distance} from it, against {worst.tolerance}, the farthest out of {used.Length}" );
    }

    // the cell around a corner sees the faces that meet there, and the vertex goes where they meet. All the tangent
    // planes of a cone pass through its tip
    [Test]
    public void everyCornerHasAVertex( [ValueSource( nameof( withCorners ) )] DensityFunction.Type type, [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( volume, vertices, triangles, _ ) = this.contourWithCells( type, algorithm, turn );

        var used = triangles.Distinct( ).Select( ( index ) => vertices[index] ).ToArray( );

        var nearest = cornersOf( type, extentsOf( type ) ).Select(
            ( corner ) => ( corner, distance: used.Min( ( vertex ) => Vector3.Distance( vertex, toWorld( volume, corner ) ) ) )
        ).OrderByDescending( ( item ) => item.distance ).First( );

        Assert.That( nearest.distance, Is.LessThanOrEqualTo( flatTolerance ), $"the nearest vertex to the corner at {nearest.corner}, in the shape's frame, the farthest of its corners" );
    }

    // a cell an edge passes through sees the two faces that meet there, and its vertex goes onto the edge. The mesh only
    // keeps away from an edge around a cell whose corners show several pieces of surface, as far as the triangles of its
    // vertex reach: into the cells next to it
    [Test]
    public void theMeshRunsAlongEveryEdge( [ValueSource( nameof( withEdges ) )] DensityFunction.Type type, [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( volume, vertices, triangles, split ) = this.contourWithCells( type, algorithm, turn );

        // the rims are curved, and the mesh runs along them from vertex to vertex, a cell or less apart. In between it
        // leaves a rim by up to the rim's sag across a cell, h^2 / 8 R
        var extents       = extentsOf( type );
        var points        = edgePointsOf( type, extents );
        var tolerance     = type == DensityFunction.Type.Cylinder || type == DensityFunction.Type.Cone
            ? flatTolerance + ( cell * cell / ( 8.0f * Mathf.Min( extents.x, extents.z ) ) )
            : flatTolerance;
        var checkedPoints = 0;
        var worst         = 0.0f;
        var worstAt       = Vector3.zero;

        foreach( var local in points ) {
            var point = toWorld( volume, local );

            if( split.Any( ( voxel ) => new Bounds( voxel.center, voxel.size * 3.0f ).Contains( point ) ) ) {
                continue;
            }

            ++checkedPoints;

            var distance = distanceToMesh( point, vertices, triangles );

            if( !( distance <= worst ) ) {
                worst   = distance;
                worstAt = local;
            }
        }

        Assert.Greater( checkedPoints, points.Length / 2, "most of the edges are left out" );
        Assert.That( worst, Is.LessThanOrEqualTo( tolerance ), $"how far the mesh is from the edge at {worstAt}, in the shape's frame, the farthest of {checkedPoints} points" );
    }

}

public class ManagedShapeContourTest : ShapeContourTest {

    public ManagedShapeContourTest( ) {
        this.implementation = ImplementationType.Managed;
    }

}

public class BurstShapeContourTest : ShapeContourTest {

    public BurstShapeContourTest( ) {
        this.implementation = ImplementationType.Burst;
    }

}

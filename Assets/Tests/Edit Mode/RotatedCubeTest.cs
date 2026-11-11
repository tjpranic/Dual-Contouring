using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// a cube turned about one axis and about a diagonal, contoured by every algorithm, which run for each implementation in
// the fixtures at the end. Unlike the cubes of the other tests, its faces cross the grid at an angle and its edges and
// corners lie anywhere in their cells. The SVD solver still has to put the vertices on its faces, edges and corners,
// where Unity shows the turned cube
public abstract class RotatedCubeTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    public enum Turn {
        AboutAnAxis,
        AboutADiagonal
    }

    // the turned cube reaches at most sqrt(3) * 0.25 = 0.433 from its center, which keeps it inside the grid
    private const float extent = 0.25f;

    private static readonly Vector3 center = new( 0.013f, -0.021f, 0.017f );

    // a binary search of six steps along an edge of a cell, 1 / 16 long, finds every intersection to within 1 / 1024 of
    // the surface. A vertex lies where up to three faces of the cube meet, each moved by up to that much, so at most
    // sqrt(3) / 1024 = 0.0017 from the cube
    private const float tolerance = 0.002f;

    // the error threshold of the "Simplification" scenes, at which flat parts of the surface collapse
    private const float errorThreshold = 2e-4f;

    private static Quaternion rotationOf( Turn turn ) {
        return turn switch {
            Turn.AboutAnAxis    => Quaternion.AngleAxis( 45.0f, Vector3.up  ),
            Turn.AboutADiagonal => Quaternion.AngleAxis( 45.0f, Vector3.one ),
            _                   => throw new Exception( "Unknown turn specified" )
        };
    }

    // 16^3 cells or leaves either way, simplifying as the "Simplification" scenes do
    private Voxelizer create( Algorithm algorithm, Turn turn ) {
        var shapes = new[] { cube( center, extent ).rotated( rotationOf( turn ) ) };

        switch( algorithm ) {
            case Algorithm.Uniform: {
                return this.createUniformVoxelizer( 4, this.implementation, QEFSolverType.SVD, shapes );
            }
            case Algorithm.Adaptive: {
                var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes );

                voxelizer.simplification = true;
                voxelizer.errorThreshold = errorThreshold;
                voxelizer.topologySafe   = true;

                return voxelizer;
            }
            case Algorithm.Manifold: {
                var voxelizer = this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, shapes );

                voxelizer.errorThreshold  = errorThreshold;
                voxelizer.enforceManifold = true;

                return voxelizer;
            }
            default: {
                throw new Exception( "Unknown algorithm specified" );
            }
        }
    }

    // the cube as Unity shows it: its transform takes the corners of a unit cube to where it draws them. The voxelizer is
    // at the origin, unturned and of unit scale, so its object space is world space
    private static Vector3[] cornersOf( Voxelizer voxelizer ) {
        var cube = voxelizer.GetComponentInChildren<Volume>( ).transform;

        return Enumerable.Range( 0, 8 ).Select(
            ( corner ) => cube.TransformPoint( new Vector3( ( corner & 1 ) == 0 ? -0.5f : 0.5f, ( corner & 2 ) == 0 ? -0.5f : 0.5f, ( corner & 4 ) == 0 ? -0.5f : 0.5f ) )
        ).ToArray( );
    }

    // how far a point is from the surface of the cube, measured in the cube's own frame, where it is the unit cube scaled
    // by twice its extent
    private static float distanceToCube( Voxelizer voxelizer, Vector3 point ) {
        var cube = voxelizer.GetComponentInChildren<Volume>( ).transform;

        return Mathf.Abs( Cuboid.sample( cube.InverseTransformPoint( point ), Vector3.one * 0.5f ) ) * extent * 2.0f;
    }

    // the closest point of a triangle to a point, from Real-Time Collision Detection by Ericson, section 5.1.5
    private static Vector3 closestPointOnTriangle( Vector3 point, Vector3 a, Vector3 b, Vector3 c ) {
        var ab = b - a;
        var ac = c - a;
        var ap = point - a;

        var d1 = Vector3.Dot( ab, ap );
        var d2 = Vector3.Dot( ac, ap );
        if( d1 <= 0.0f && d2 <= 0.0f ) {
            return a;
        }

        var bp = point - b;
        var d3 = Vector3.Dot( ab, bp );
        var d4 = Vector3.Dot( ac, bp );
        if( d3 >= 0.0f && d4 <= d3 ) {
            return b;
        }

        var vc = ( d1 * d4 ) - ( d3 * d2 );
        if( vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f ) {
            return a + ( d1 / ( d1 - d3 ) * ab );
        }

        var cp = point - c;
        var d5 = Vector3.Dot( ab, cp );
        var d6 = Vector3.Dot( ac, cp );
        if( d6 >= 0.0f && d5 <= d6 ) {
            return c;
        }

        var vb = ( d5 * d2 ) - ( d1 * d6 );
        if( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f ) {
            return a + ( d2 / ( d2 - d6 ) * ac );
        }

        var va = ( d3 * d6 ) - ( d5 * d4 );
        if( va <= 0.0f && ( d4 - d3 ) >= 0.0f && ( d5 - d6 ) >= 0.0f ) {
            return b + ( ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) ) * ( c - b ) );
        }

        var denominator = 1.0f / ( va + vb + vc );

        return a + ( ab * ( vb * denominator ) ) + ( ac * ( vc * denominator ) );
    }

    private static float distanceToMesh( Vector3 point, Vector3[] vertices, int[] triangles ) {
        var nearest = float.PositiveInfinity;

        for( var index = 0; index < triangles.Length; index += 3 ) {
            var closest = closestPointOnTriangle( point, vertices[triangles[index]], vertices[triangles[index + 1]], vertices[triangles[index + 2]] );

            nearest = Mathf.Min( nearest, Vector3.Distance( point, closest ) );
        }

        return nearest;
    }

    // whether the corners of a cell show more than one piece of surface: its inside corners, or its outside ones, fall
    // into separate groups that no edge of the cell connects. An edge of the turned cube that crosses a face of a cell
    // along its diagonal leaves such a cell
    private static bool showsSeveralPieces( SurfaceExtractor.Voxel voxel ) {
        bool fallsApart( bool inside ) {
            var group = Enumerable.Range( 0, SurfaceExtractor.Voxel.CornerCount ).ToArray( );

            int find( int corner ) {
                while( group[corner] != corner ) {
                    corner = group[corner];
                }
                return corner;
            }

            foreach( var edge in voxel.edges ) {
                var start = Array.IndexOf( voxel.corners, edge.corners[0] );
                var end   = Array.IndexOf( voxel.corners, edge.corners[1] );

                if( ( edge.corners[0].density < 0.0f ) == inside && ( edge.corners[1].density < 0.0f ) == inside ) {
                    group[find( start )] = find( end );
                }
            }

            return Enumerable.Range( 0, SurfaceExtractor.Voxel.CornerCount ).Where(
                ( corner ) => ( voxel.corners[corner].density < 0.0f ) == inside
            ).Select( find ).Distinct( ).Count( ) > 1;
        }

        return fallsApart( true ) || fallsApart( false );
    }

    // Uniform and Adaptive Dual Contouring give a cell one vertex, also a cell whose corners show several pieces of
    // surface. There the vertex is where the planes of all of them come closest, off the cube, and the pieces meet in it,
    // so the surface stays closed but is not manifold there. Manifold Dual Contouring gives such a cell a vertex for each
    // piece, and only its surface has to be manifold and on the cube everywhere
    private static bool oneVertexPerCell( Algorithm algorithm ) {
        return algorithm != Algorithm.Manifold;
    }

    private ( Voxelizer voxelizer, Vector3[] vertices, int[] triangles, SurfaceExtractor.Voxel[] split ) contourWithCells( Algorithm algorithm, Turn turn ) {
        var voxelizer    = this.create( algorithm, turn );
        var voxelization = this.voxelize( voxelizer );

        // the cells left out where one vertex per cell is all there is to expect
        var split = oneVertexPerCell( algorithm )
            ? voxelization.voxels.Where( ( voxel ) => voxel.type != SurfaceExtractor.Voxel.Type.Internal && voxel.hasFeaturePoint && showsSeveralPieces( voxel ) ).ToArray( )
            : Array.Empty<SurfaceExtractor.Voxel>( );

        return ( voxelizer, voxelization.mesh.vertices, voxelization.mesh.triangles, split );
    }

    [Test]
    public void surfaceIsClosed( [Values] Algorithm algorithm, [Values] Turn turn ) {
        var topology = this.contour( this.create( algorithm, turn ) );

        if( oneVertexPerCell( algorithm ) ) {
            assertClosed( topology );
        }
        else {
            assertSingleSphere( topology );
        }
    }

    [Test]
    public void everyVertexLiesOnTheCube( [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( voxelizer, vertices, triangles, split ) = this.contourWithCells( algorithm, turn );

        var used = triangles.Distinct( ).Select( ( index ) => vertices[index] ).Where(
            ( vertex ) => !split.Any( ( voxel ) => voxel.vertex == vertex )
        ).ToArray( );

        Assert.Greater( used.Length, 0, "the mesh is empty" );

        var distances = used.Select( ( vertex ) => distanceToCube( voxelizer, vertex ) ).ToArray( );
        var off       = distances.Count( ( distance ) => !( distance <= tolerance ) );

        Assert.AreEqual( 0, off, $"{off} of {distances.Length} vertices lie more than {tolerance} from the cube, the farthest {distances.Max( )}" );
    }

    // the cell around a corner sees the three faces that meet there, and the vertex goes where they meet
    [Test]
    public void everyCornerOfTheCubeHasAVertex( [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( voxelizer, vertices, triangles, _ ) = this.contourWithCells( algorithm, turn );

        var used = triangles.Distinct( ).Select( ( index ) => vertices[index] ).ToArray( );

        foreach( var corner in cornersOf( voxelizer ) ) {
            var nearest = used.Min( ( vertex ) => Vector3.Distance( vertex, corner ) );

            Assert.That( nearest, Is.LessThanOrEqualTo( tolerance ), $"the nearest vertex to the corner at {corner}" );
        }
    }

    // a cell an edge passes through sees the two faces that meet there, and its vertex goes onto the edge. Simplification
    // collapses cells along an edge into larger ones, down to a single edge of the mesh from corner to corner, which runs
    // along the edge all the same. The mesh only keeps away from an edge around a cell whose corners show several pieces
    // of surface, as far as the triangles of its vertex reach: into the cells next to it
    [Test]
    public void theMeshRunsAlongEveryEdgeOfTheCube( [Values] Algorithm algorithm, [Values] Turn turn ) {
        var ( voxelizer, vertices, triangles, split ) = this.contourWithCells( algorithm, turn );

        var corners = cornersOf( voxelizer );

        // corners whose indices differ in one bit share an edge
        var edges = Enumerable.Range( 0, 8 ).SelectMany(
            ( corner ) => new[] { 1, 2, 4 }.Where( ( bit ) => ( corner & bit ) == 0 ).Select( ( bit ) => ( corners[corner], corners[corner | bit] ) )
        ).ToArray( );

        Assert.AreEqual( 12, edges.Length );

        var checkedPoints = 0;

        foreach( var ( start, end ) in edges ) {
            for( var step = 0; step <= 32; ++step ) {
                var point = Vector3.Lerp( start, end, step / 32.0f );

                if( split.Any( ( voxel ) => new Bounds( voxel.center, voxel.size * 3.0f ).Contains( point ) ) ) {
                    continue;
                }

                ++checkedPoints;

                var distance = distanceToMesh( point, vertices, triangles );

                Assert.That( distance, Is.LessThanOrEqualTo( tolerance ), $"how far the mesh is from {point}, on the edge from {start} to {end}" );
            }
        }

        Assert.Greater( checkedPoints, 12 * 32 / 2, "most of the edges are left out" );
    }

}

public class ManagedRotatedCubeTest : RotatedCubeTest {

    public ManagedRotatedCubeTest( ) {
        this.implementation = ImplementationType.Managed;
    }

}

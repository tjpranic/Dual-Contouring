using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// shared models and assertions for tests that look at how the triangles of a contoured mesh connect
public abstract class TopologyTest : ContouringTest {

    // each of these contours to a single closed surface without handles
    public enum Model {
        Sphere,
        ThreeBalls,
        CubesAndSphere,
        SphereCutaway
    }

    // the cube [0, 0.25]^3. On a grid of 8^3 cells the cell [0, 0.125]^3 sees the faces x = 0, y = 0 and z = 0 through
    // three axis aligned hermite samples, so the minimizer of its QEF is the corner of the cube: exactly (0, 0, 0)
    protected static readonly Shape[] cubeWithCornerAtOrigin = { cube( new Vector3( 0.125f, 0.125f, 0.125f ), 0.125f ) };

    // two spheres 0.04 apart along the diagonal of the cell [0, 0.0625]^2 in x and y of a grid of 16^3. Each sphere
    // covers one of two diagonally opposite corners of that cell and neither covers the other two, so the cell holds
    // two separate pieces of surface
    protected static readonly Shape[] twoSpheres = {
        sphere( new Vector3( 0.03125f - 0.0849f, 0.03125f - 0.0849f, 0.003f ), 0.1f ),
        sphere( new Vector3( 0.03125f + 0.0849f, 0.03125f + 0.0849f, 0.003f ), 0.1f )
    };

    // a slab with a band of a second material across it, flush with the slab on every side. The band reaches from
    // x = -0.13 to x = 0.17, and neither of those lies on a plane of a grid of 16^3 cells
    protected const float bandMinimum = -0.13f;
    protected const float bandMaximum =  0.17f;

    protected static readonly Shape[] slabWithBand = {
        cuboid( Vector3.zero,                    new Vector3( 0.4f,  0.1f, 0.4f ) ),
        cuboid( new Vector3( 0.02f, 0.0f, 0.0f ), new Vector3( 0.15f, 0.1f, 0.4f ), Combination.Union, MaterialIndex.Material1 )
    };

    // turned shapes: a cube turned about a diagonal, an ellipsoid turned about all three axes cut out of it, and a box of
    // a second material turned about one axis set into it. Faces, edges and the boundary between the materials all cross
    // the grid at an angle
    protected static readonly Shape[] turnedShapes = {
        cube      ( new Vector3( -0.05f, 0.013f, -0.021f ), 0.25f ).rotated( Quaternion.AngleAxis( 45.0f, Vector3.one ) ),
        ellipsoid ( new Vector3(  0.15f, 0.1f,    0.05f  ), new Vector3( 0.2f,  0.12f, 0.08f ), Combination.Subtraction ).rotated( Quaternion.Euler( 30.0f, -60.0f, 75.0f ) ),
        cuboid    ( new Vector3(  0.1f, -0.2f,    0.1f   ), new Vector3( 0.25f, 0.06f, 0.1f  ), Combination.Union, MaterialIndex.Material1 ).rotated( Quaternion.AngleAxis( 30.0f, Vector3.up ) )
    };

    // the shapes of the article other than the box and the ellipsoid, each turned: a cylinder and a cone that meet, a
    // torus of a second material around them, a capsule with an octahedron cut out of it, a pyramid and a hexagonal
    // prism. Their curved surfaces, rims, tips and corners all lie across the grid at angles
    protected static readonly Shape[] newShapes = {
        new Shape( DensityFunction.Type.Cylinder,       new Vector3( -0.2f,  0.15f,  0.0f  ), Quaternion.Euler( 20.0f,  0.0f,  30.0f  ), new Vector3( 0.12f, 0.18f, 0.12f ), Combination.Union,       MaterialIndex.Material0 ),
        new Shape( DensityFunction.Type.Cone,           new Vector3(  0.2f,  0.15f,  0.05f ), Quaternion.Euler( -30.0f, 40.0f, 10.0f  ), new Vector3( 0.14f, 0.18f, 0.14f ), Combination.Union,       MaterialIndex.Material0 ),
        new Shape( DensityFunction.Type.Torus,          new Vector3(  0.0f,  0.1f,   0.0f  ), Quaternion.Euler( 70.0f,  10.0f, 0.0f   ), new Vector3( 0.32f, 0.08f, 0.32f ), Combination.Union,       MaterialIndex.Material1 ),
        new Shape( DensityFunction.Type.Capsule,        new Vector3(  0.0f, -0.22f, -0.15f ), Quaternion.AngleAxis( 80.0f, new Vector3( 1.0f, 0.0f, 1.0f ) ), new Vector3( 0.1f, 0.22f, 0.1f ), Combination.Union, MaterialIndex.Material0 ),
        new Shape( DensityFunction.Type.Octahedron,     new Vector3(  0.05f, -0.22f, -0.15f ), Quaternion.Euler( 10.0f,  20.0f, 30.0f  ), new Vector3( 0.1f,  0.1f,  0.1f  ), Combination.Subtraction, MaterialIndex.Material0 ),
        new Shape( DensityFunction.Type.Pyramid,        new Vector3( -0.22f, -0.2f,   0.2f  ), Quaternion.Euler( 0.0f,   30.0f, 15.0f  ), new Vector3( 0.13f, 0.12f, 0.13f ), Combination.Union,       MaterialIndex.Material0 ),
        new Shape( DensityFunction.Type.HexagonalPrism, new Vector3(  0.22f, -0.2f,   0.22f ), Quaternion.Euler( 40.0f,  0.0f,  -20.0f ), new Vector3( 0.13f, 0.1f,  0.12f ), Combination.Union,       MaterialIndex.Material0 )
    };

    // a block with six small hollows in it, each around one corner of a grid of 16^3 cells and no other. They come in
    // three pairs, around corners that lie diagonally across a face of a cell: once for each direction a face can
    // face. The two cells on either side of such a face hold both hollows of a pair and are otherwise solid, which
    // makes that face a joined face of both (DualMarchingCubes.lookupJoinedFace). A grid corner n is at n / 16 - 0.5
    protected static readonly Shape[] hollowsAcrossFaces = {
        cube( Vector3.zero, 0.4f ),
        hollowAt(  9,  9,  9 ), hollowAt( 10, 10,  9 ), // across a face that faces along z
        hollowAt(  5,  5,  5 ), hollowAt(  5,  6,  6 ), // along x
        hollowAt( 12,  5, 11 ), hollowAt( 13,  5, 12 )  // along y
    };

    private static Shape hollowAt( int x, int y, int z ) {
        return cube( ( new Vector3( x, y, z ) / 16.0f ) - ( Vector3.one * 0.5f ), 0.02f, Combination.Subtraction );
    }

    protected static Shape[] shapesOf( Model model ) {
        return model switch {
            // sits off the grid lines so no corner or edge of the grid lands exactly on the surface
            Model.Sphere => new[] {
                sphere( new Vector3( 0.013f, -0.021f, 0.017f ), 0.3f )
            },
            // the "Simplification" scenes
            Model.ThreeBalls => new[] {
                sphere( new Vector3( -0.2f, 0.0f, 0.0f ), 0.3f ),
                sphere( new Vector3(  0.2f, 0.0f, 0.0f ), 0.3f ),
                sphere( new Vector3(  0.0f, 0.2f, 0.0f ), 0.3f )
            },
            // the "SVD Solve" scenes
            Model.CubesAndSphere => new[] {
                cube   ( new Vector3( -0.2f, 0.0f, 0.0f ), 0.2f ),
                sphere ( Vector3.zero,                     0.3f ),
                cube   ( new Vector3(  0.2f, 0.0f, 0.0f ), 0.2f )
            },
            // the "Aggresive Simplification" scene: half of the outer sphere is cut away to show an inner sphere of
            // another material
            Model.SphereCutaway => new[] {
                sphere ( Vector3.zero,                    0.5f ),
                cube   ( new Vector3( 0.5f, 0.0f, 0.0f ), 0.5f, Combination.Subtraction ),
                sphere ( Vector3.zero,                    0.4f, Combination.Union, MaterialIndex.Material1 )
            },
            _ => throw new Exception( "Unknown model specified" )
        };
    }

    protected MeshTopology contour( Voxelizer voxelizer ) {
        return MeshTopology.of( this.voxelize( voxelizer ).mesh );
    }

    // in how many separate pieces each material shows on the surface, from the material with the fewest pieces up
    protected int[] contourRegions( Voxelizer voxelizer ) {
        var mesh = this.voxelize( voxelizer ).mesh;

        return Enumerable.Range( 0, mesh.subMeshCount ).Select( ( subMesh ) => MeshTopology.of( mesh, subMesh ).components ).OrderBy( ( pieces ) => pieces ).ToArray( );
    }

    // Dual Contouring of Hermite Data, section 2.3: with several materials, "edges that exhibit index changes are also
    // tagged by exact intersection points and normals", which go into the QEF of a cell like those of the surface.
    // That puts the vertices of the cells a material boundary passes through on that boundary: the part of the
    // surface that shows the band begins and ends where the band does, not where the grid happens to put it
    protected void assertBandFollowsMaterialBoundary( Voxelizer voxelizer ) {
        // a binary search of six steps finds an intersection to within a sixty-fourth of an edge of a cell
        const float tolerance = 0.002f;

        var mesh = this.voxelize( voxelizer ).mesh;

        Assert.AreEqual( 2, mesh.subMeshCount, "materials on the surface" );

        var vertices = mesh.vertices;

        // where along x the vertices of each material lie, the band is the one that spans less
        var spans = Enumerable.Range( 0, mesh.subMeshCount ).Select(
            ( subMesh ) => mesh.GetTriangles( subMesh ).Distinct( ).Select( ( index ) => vertices[index].x ).ToArray( )
        ).OrderBy(
            ( span ) => span.Max( ) - span.Min( )
        ).ToArray( );

        var band = spans[0];
        var slab = spans[1];

        Assert.AreEqual( bandMinimum, band.Min( ), tolerance, "where the band begins" );
        Assert.AreEqual( bandMaximum, band.Max( ), tolerance, "where the band ends" );

        var overlapping = slab.Count( ( x ) => x > bandMinimum + tolerance && x < bandMaximum - tolerance );

        Assert.AreEqual( 0, overlapping, "vertices of the slab that lie inside the band" );
    }

    // the closest point of a triangle to a point, from Real-Time Collision Detection by Ericson, section 5.1.5
    protected static Vector3 closestPointOnTriangle( Vector3 point, Vector3 a, Vector3 b, Vector3 c ) {
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

    protected static float distanceToMesh( Vector3 point, Vector3[] vertices, int[] triangles ) {
        var nearest = float.PositiveInfinity;

        for( var index = 0; index < triangles.Length; index += 3 ) {
            var closest = closestPointOnTriangle( point, vertices[triangles[index]], vertices[triangles[index + 1]], vertices[triangles[index + 2]] );

            nearest = Mathf.Min( nearest, Vector3.Distance( point, closest ) );
        }

        return nearest;
    }

    // whether the corners of a cell show more than one piece of surface: its inside corners, or its outside ones, fall
    // into separate groups that no edge of the cell connects. An edge of a turned shape that crosses a face of a cell
    // along its diagonal leaves such a cell
    protected static bool showsSeveralPieces( SurfaceExtractor.Voxel voxel ) {
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

    // every edge is shared by an even number of triangles
    protected static void assertClosed( MeshTopology topology ) {
        Assert.Greater( topology.triangles, 0, "the mesh is empty" );

        Assert.AreEqual( 0, topology.oddEdges, $"edges shared by an odd number of triangles in: {topology}" );
    }

    // every edge is shared by exactly two triangles that agree on which side faces out, and the triangles around every
    // vertex form a single fan
    protected static void assertManifold( MeshTopology topology ) {
        assertClosed( topology );

        Assert.AreEqual( 0, topology.nonManifoldEdges, $"edges shared by more than two triangles in: {topology}" );
        Assert.AreEqual( 0, topology.windingConflicts, $"neighboring triangles facing opposite ways in: {topology}" );
        Assert.AreEqual( 0, topology.pinchedVertices,  $"vertices with more than one fan of triangles in: {topology}" );
    }

    // two closed manifold surfaces without handles, apart from each other
    protected static void assertTwoSpheres( MeshTopology topology ) {
        assertManifold( topology );

        Assert.AreEqual( 2, topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( 4, topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

    // the block and each of its six hollows, every one a manifold surface without handles and apart from the others
    protected static void assertBlockWithSixHollows( MeshTopology topology ) {
        assertManifold( topology );

        Assert.AreEqual( 7,  topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( 14, topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

    // one manifold surface without handles, which is what every Model is
    protected static void assertSingleSphere( MeshTopology topology ) {
        assertManifold( topology );

        Assert.AreEqual( 1, topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( 2, topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

}

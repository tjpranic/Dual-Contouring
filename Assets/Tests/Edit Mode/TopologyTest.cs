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

    // one manifold surface without handles, which is what every Model is
    protected static void assertSingleSphere( MeshTopology topology ) {
        assertManifold( topology );

        Assert.AreEqual( 1, topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( 2, topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

}

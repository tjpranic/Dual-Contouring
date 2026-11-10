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

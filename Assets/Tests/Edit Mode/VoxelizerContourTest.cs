using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;

// Voxelizer.contour, which builds and shows the mesh of the volumes and can be called again and again, the way the
// interactive editor calls it. None of these voxelizers is started: contour has to work without Start
public class VoxelizerContourTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    // an algorithm with one of its implementations
    public readonly struct Variant {

        public readonly Algorithm          algorithm;
        public readonly ImplementationType implementation;

        public Variant( Algorithm algorithm, ImplementationType implementation ) {
            this.algorithm      = algorithm;
            this.implementation = implementation;
        }

        public override string ToString( ) {
            return $"{this.algorithm} {this.implementation}";
        }

    }

    private static readonly Variant[] variants = {
        new( Algorithm.Uniform,  ImplementationType.Managed ),
        new( Algorithm.Uniform,  ImplementationType.Burst ),
        new( Algorithm.Uniform,  ImplementationType.GPU ),
        new( Algorithm.Adaptive, ImplementationType.Managed ),
        new( Algorithm.Adaptive, ImplementationType.Burst ),
        new( Algorithm.Manifold, ImplementationType.Managed ),
        new( Algorithm.Manifold, ImplementationType.Burst )
    };

    // 16^3 cells for Uniform, 16^3 leaves for the octrees
    private const int resolution = 4;

    private Voxelizer create( Variant variant, IEnumerable<Shape> shapes, VertexNormals normals = VertexNormals.Shared ) {
        Voxelizer voxelizer = variant.algorithm switch {
            Algorithm.Uniform  => this.createVoxelizer<UniformDualContouring>  ( resolution,     QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive => this.createVoxelizer<AdaptiveDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold => this.createVoxelizer<ManifoldDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            _                  => throw new Exception( "Unknown algorithm specified" )
        };

        voxelizer.implementationType = variant.implementation;
        voxelizer.vertexNormals      = normals;

        // contour only looks for debug data when the debug view wants it
        voxelizer.keepDebugInformation = false;

        return voxelizer;
    }

    private static Mesh shown( Voxelizer voxelizer ) {
        return voxelizer.GetComponent<MeshFilter>( ).sharedMesh;
    }

    private static void assertSameTriangles( Mesh expected, Mesh actual ) {
        MeshComparison.assertSameTriangles( expected, actual );
    }

    private static int triangleCountOf( Mesh mesh ) {
        return Enumerable.Range( 0, mesh.subMeshCount ).Sum( ( subMesh ) => ( int )mesh.GetIndexCount( subMesh ) / 3 );
    }

    [Test]
    public void contourShowsAMeshOfTheVolumes( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ) );

        voxelizer.contour( drawFromGPU: false );

        var mesh = shown( voxelizer );

        Assert.IsNotNull( mesh );
        Assert.Greater( mesh.vertexCount, 0 );

        assertSameTriangles( this.voxelize( voxelizer ).mesh, mesh );
    }

    [Test]
    public void secondContourShowsTheSameMesh( [ValueSource( nameof( variants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, shapesOf( Model.SphereCutaway ), normals );

        voxelizer.contour( drawFromGPU: false );

        var first = Object.Instantiate( shown( voxelizer ) );

        try {
            voxelizer.contour( drawFromGPU: false );

            assertSameTriangles( first, shown( voxelizer ) );
        }
        finally {
            Object.DestroyImmediate( first );
        }
    }

    [Test]
    public void contourKeepsOneMeshFilter( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.Sphere ) );

        voxelizer.contour( drawFromGPU: false );
        voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( 1, voxelizer.GetComponents<MeshFilter>( ).Length );
    }

    [Test]
    public void contourDestroysTheMeshItReplaces( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.Sphere ) );

        voxelizer.contour( drawFromGPU: false );

        var first = shown( voxelizer );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsTrue( first == null, "the first mesh is still there" );
        Assert.IsTrue( shown( voxelizer ) != null );
    }

    // a mesh shown in edit mode would otherwise be written into the scene file. It may still be unloaded once nothing
    // uses it, so a mesh whose voxelizer is gone does not stay in memory for the rest of the session
    [Test]
    public void contourMeshIsNotSavedWithTheScene( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.Sphere ) );

        voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild, shown( voxelizer ).hideFlags );
    }

    [Test]
    public void contourReportsItsMesh( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.CubesAndSphere ) );

        var statistics = voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( shown( voxelizer ).vertexCount,       statistics.vertexCount );
        Assert.AreEqual( triangleCountOf( shown( voxelizer ) ), statistics.triangleCount );
        Assert.Greater( statistics.milliseconds, 0.0 );
    }

    // reordering the vertex buffer only moves vertices around, so it can be left out where time matters more
    [Test]
    public void contourWithoutReorderingShowsTheSameTriangles( [ValueSource( nameof( variants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ), normals );

        voxelizer.contour( drawFromGPU: false, reorderVertices: false );

        var unordered = Object.Instantiate( shown( voxelizer ) );

        try {
            voxelizer.contour( drawFromGPU: false, reorderVertices: true );

            assertSameTriangles( unordered, shown( voxelizer ) );
        }
        finally {
            Object.DestroyImmediate( unordered );
        }
    }

    // the interactive editor lets every shape be removed
    [Test]
    public void contourOfNothingShowsAnEmptyMesh( [ValueSource( nameof( variants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, new Shape[0], normals );

        var statistics = voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( 0, shown( voxelizer ).vertexCount );
        Assert.AreEqual( 0, statistics.vertexCount );
        Assert.AreEqual( 0, statistics.triangleCount );
    }

    // the density functions to contour can be given, in place of the volumes below the voxelizer
    [Test]
    public void contourOfGivenDensityFunctionsIgnoresTheVolumes( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ) );
        var reference = this.create( variant, shapesOf( Model.Sphere ) );

        voxelizer.contour( reference.GetComponentsInChildren<Volume>( ), drawFromGPU: false );
        reference.contour( drawFromGPU: false );

        assertSameTriangles( shown( reference ), shown( voxelizer ) );
    }

    [Test]
    public void clearRemovesTheMesh( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.Sphere ) );

        voxelizer.contour( drawFromGPU: false );

        var mesh = shown( voxelizer );

        voxelizer.clear( );

        Assert.IsTrue( shown( voxelizer ) == null, "the mesh filter still shows a mesh" );
        Assert.IsTrue( mesh == null, "the mesh is still there" );
    }

    // the interactive editor keeps a voxelizer of each algorithm on one object, sharing one mesh filter
    [Test]
    public void contourReplacesTheMeshOfAnotherVoxelizerOnTheSameObject( ) {
        var uniform  = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.Sphere ) );
        var adaptive = uniform.gameObject.AddComponent<AdaptiveDualContouring>( );

        adaptive.resolution         = resolution + 1;
        adaptive.implementationType = ImplementationType.Burst;

        uniform.contour( drawFromGPU: false );

        var first = shown( uniform );

        adaptive.contour( drawFromGPU: false );

        Assert.IsTrue( first == null, "the mesh of the first voxelizer is still there" );
        Assert.AreEqual( 1, uniform.GetComponents<MeshFilter>( ).Length );
        Assert.IsTrue( shown( adaptive ) != null );
    }

    [Test]
    public void uniformContoursWithoutStart( [Values( ImplementationType.Managed, ImplementationType.Burst, ImplementationType.GPU )] ImplementationType implementation ) {
        var voxelizer = this.create( new( Algorithm.Uniform, implementation ), shapesOf( Model.Sphere ) );

        Assert.Greater( this.voxelize( voxelizer ).mesh.vertexCount, 0 );
    }

    // reading back only the mesh gives what reading back everything gives
    [Test]
    public void gpuMeshReadBackMatchesConvert( [Values] Model model, [Values] VertexNormals normals ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.GPU ), shapesOf( model ), normals );

        voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ).visit(
            ( cpu ) => { Assert.Fail( "the GPU implementation gave a CPU voxelization" ); return 0; },
            ( gpu ) => {
                var mesh      = voxelizer.readMesh( gpu );
                var converted = voxelizer.convert( gpu ).mesh;

                try {
                    CollectionAssert.AreEqual( converted.vertices, mesh.vertices, "vertices" );
                    CollectionAssert.AreEqual( converted.normals,  mesh.normals,  "normals" );

                    Assert.AreEqual( converted.subMeshCount, mesh.subMeshCount, "sub-meshes" );
                    Assert.AreEqual( converted.indexFormat,  mesh.indexFormat,  "index format" );

                    for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
                        CollectionAssert.AreEqual( converted.GetTriangles( subMesh ), mesh.GetTriangles( subMesh ), $"sub-mesh {subMesh}" );
                    }
                }
                finally {
                    Object.DestroyImmediate( mesh );
                    Object.DestroyImmediate( converted );
                }
                return 0;
            }
        );
    }

    // the GPU implementation keeps its buffers from one contour to the next. Whatever came before, a contour gives
    // what a voxelizer that never contoured anything gives
    [Test]
    public void gpuContoursInARowMatchFreshContours( ) {
        var sequence = new ( Shape[] shapes, int resolution )[] {
            ( shapesOf( Model.ThreeBalls ),     4 ),
            ( shapesOf( Model.CubesAndSphere ), 4 ), // same resolution, same number of shapes
            ( shapesOf( Model.Sphere ),         4 ), // fewer shapes
            ( shapesOf( Model.SphereCutaway ),  5 ), // larger grid
            ( shapesOf( Model.ThreeBalls ),     3 ), // smaller grid
            ( new Shape[0],                     3 ), // nothing
            ( shapesOf( Model.ThreeBalls ),     4 )  // back to the start
        };

        var reused = this.create( new( Algorithm.Uniform, ImplementationType.GPU ), new Shape[0] );

        foreach( var ( shapes, cellsExponent ) in sequence ) {
            var fresh = this.create( new( Algorithm.Uniform, ImplementationType.GPU ), shapes );

            fresh.resolution  = cellsExponent;
            reused.resolution = cellsExponent;

            reused.contour( fresh.GetComponentsInChildren<Volume>( ), drawFromGPU: false );
            fresh.contour( drawFromGPU: false );

            assertSameTriangles( shown( fresh ), shown( reused ) );

            // a fresh voxelizer only stays fresh for one contour
            fresh.OnDestroy( );
        }
    }

    // once its buffers are released, as when it is disabled or destroyed, the GPU implementation can still contour
    [Test]
    public void gpuContoursAfterReleasingItsBuffers( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.GPU ), shapesOf( Model.ThreeBalls ) );

        voxelizer.contour( drawFromGPU: false );

        var first = Object.Instantiate( shown( voxelizer ) );

        try {
            voxelizer.OnDestroy( );
            voxelizer.contour( drawFromGPU: false );

            assertSameTriangles( first, shown( voxelizer ) );
        }
        finally {
            Object.DestroyImmediate( first );
        }
    }

}

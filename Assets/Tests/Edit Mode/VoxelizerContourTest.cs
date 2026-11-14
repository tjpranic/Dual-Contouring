using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using ContourVertices    = SurfaceExtractor.ContourVertices;
using SmoothNormals      = SurfaceExtractor.SmoothNormals;
using QEFSolverType      = QEFSolver.Type;

// Voxelizer.contour, which builds and shows the mesh of the volumes and can be called again and again, the way the
// interactive editor calls it. None of these voxelizers is started: contour has to work without Start
public class VoxelizerContourTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold,
        Occupancy
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
        new( Algorithm.Manifold, ImplementationType.Burst ),
        new( Algorithm.Occupancy, ImplementationType.Burst ),
        new( Algorithm.Occupancy, ImplementationType.GPU )
    };

    // 16^3 cells for the uniform grids, 16^3 leaves for the octrees
    private const int resolution = 4;

    private Voxelizer create( Variant variant, IEnumerable<Shape> shapes, VertexNormals normals = VertexNormals.Creased ) {
        Voxelizer voxelizer = variant.algorithm switch {
            Algorithm.Uniform  => this.createVoxelizer<UniformDualContouring>  ( resolution,     QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive => this.createVoxelizer<AdaptiveDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold => this.createVoxelizer<ManifoldDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Occupancy => this.createVoxelizer<OccupancyDualContouring>( resolution,     QEFSolverType.SVD, shapes ),
            _                  => throw new Exception( "Unknown algorithm specified" )
        };

        voxelizer.implementationType = variant.implementation;
        voxelizer.vertexNormals      = normals;

        // creased as a voxelizer starts out
        voxelizer.creaseAngle = 40.0f;

        // contour only looks for debug data when the debug view wants it
        voxelizer.keepDebugInformation = false;

        return voxelizer;
    }

    private static Mesh shown( Voxelizer voxelizer ) {
        return voxelizer.GetComponent<MeshFilter>( ).sharedMesh;
    }

    // the meshes asShown made, destroyed with the test
    private readonly List<Object> finished = new( );

    [TearDown]
    public void destroyFinished( ) {
        foreach( var mesh in this.finished ) {
            Object.DestroyImmediate( mesh );
        }
        this.finished.Clear( );
    }

    // a mesh that voxelize gave, or a simplification of it, as contour shows it under the voxelizer's vertex normals:
    // creased (BurstCreases), or split. The GPU splits and creases by itself when it is not asked for shared vertices,
    // and what it gives is shown as it is. The mesh given is left as it is
    private Mesh asShown( Voxelizer voxelizer, Mesh mesh ) {
        if( voxelizer.implementationType == ImplementationType.GPU && voxelizer.contourVertices != ContourVertices.Shared ) {
            return mesh;
        }

        Mesh result;

        if( voxelizer.vertexNormals == VertexNormals.Split ) {
            result = Object.Instantiate( mesh );

            Voxelizer.splitVertices( result );
        }
        else {
            result = BurstCreases.crease( mesh, voxelizer.creaseAngle, voxelizer.smoothNormals, out _ );
        }

        this.finished.Add( result );

        return result;
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

        assertSameTriangles( this.asShown( voxelizer, this.voxelize( voxelizer ).mesh ), mesh );
    }

    // creased vertex normals are the contour's shared vertices creased under the voxelizer's angle and smooth normals,
    // and the mesh the crease was made from is not left behind
    [Test]
    public void contourCreasesUnderTheVoxelizersSettings( [ValueSource( nameof( variants ) )] Variant variant, [Values( 0.0f, 25.0f, 180.0f )] float angle, [Values] SmoothNormals smoothNormals ) {
        var voxelizer = this.create( variant, shapesOf( Model.CubesAndSphere ) );

        voxelizer.creaseAngle   = angle;
        voxelizer.smoothNormals = smoothNormals;

        var meshesBefore = Resources.FindObjectsOfTypeAll<Mesh>( ).Length;
        var statistics   = voxelizer.contour( drawFromGPU: false );
        var mesh         = shown( voxelizer );

        Assert.AreEqual( meshesBefore + 1, Resources.FindObjectsOfTypeAll<Mesh>( ).Length, "meshes, against before the contour" );

        var expected = this.asShown( voxelizer, this.voxelize( voxelizer ).mesh );

        assertSameTriangles( expected, mesh );

        Assert.AreEqual( mesh.vertexCount, statistics.vertexCount, "vertices reported" );
        Assert.AreEqual( HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild, mesh.hideFlags, "the mesh is not saved with the scene" );
    }

    // at 180 degrees under Contour nothing is creased: the mesh shown is the contour's, shared vertices and normals,
    // which is what the Shared vertex normals that Creased replaced gave
    [Test]
    public void fullCreaseAngleUnderContourShowsTheContoursOwnVertices( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.SphereCutaway ) );

        voxelizer.creaseAngle   = 180.0f;
        voxelizer.smoothNormals = SmoothNormals.Contour;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var contour    = this.voxelize( voxelizer ).mesh;

        assertSameTriangles( contour, shown( voxelizer ) );

        Assert.AreEqual( contour.vertexCount, statistics.vertexCount, "vertices" );
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
    public void gpuMeshReadBackMatchesConvert( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] Model model, [Values] VertexNormals normals ) {
        var voxelizer = this.create( new( algorithm, ImplementationType.GPU ), shapesOf( model ), normals );

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
    public void gpuContoursInARowMatchFreshContours( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm ) {
        var sequence = new ( Shape[] shapes, int resolution )[] {
            ( shapesOf( Model.ThreeBalls ),     4 ),
            ( shapesOf( Model.CubesAndSphere ), 4 ), // same resolution, same number of shapes
            ( shapesOf( Model.Sphere ),         4 ), // fewer shapes
            ( shapesOf( Model.SphereCutaway ),  5 ), // larger grid
            ( shapesOf( Model.ThreeBalls ),     3 ), // smaller grid
            ( new Shape[0],                     3 ), // nothing
            ( shapesOf( Model.ThreeBalls ),     4 )  // back to the start
        };

        var reused = this.create( new( algorithm, ImplementationType.GPU ), new Shape[0] );

        foreach( var ( shapes, cellsExponent ) in sequence ) {
            var fresh = this.create( new( algorithm, ImplementationType.GPU ), shapes );

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
    public void gpuContoursAfterReleasingItsBuffers( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm ) {
        var voxelizer = this.create( new( algorithm, ImplementationType.GPU ), shapesOf( Model.ThreeBalls ) );

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


    // ---- mesh simplification ----

    // how far a collapse may leave the surface in these tests, in cells: far enough for every contour to lose most of
    // its triangles
    private const float simplificationError = 0.25f;

    private Voxelizer createSimplifying( Variant variant, IEnumerable<Shape> shapes, VertexNormals normals = VertexNormals.Creased ) {
        var voxelizer = this.create( variant, shapes, normals );

        voxelizer.meshSimplification  = true;
        voxelizer.simplificationError = simplificationError;

        return voxelizer;
    }

    // with the setting on, the mesh that is shown is the contour, simplified
    [Test]
    public void contourWithSimplificationShowsTheSimplifiedContour( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.createSimplifying( variant, shapesOf( Model.ThreeBalls ) );

        var statistics = voxelizer.contour( drawFromGPU: false );
        var mesh       = shown( voxelizer );

        // voxelize gives what contour simplifies
        var contour    = this.voxelize( voxelizer ).mesh;
        var simplified = BurstMeshSimplifier.simplify( contour, simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight, out var simplification );

        try {
            Assert.Greater( simplification.collapses, contour.vertexCount / 2, "collapses" );

            Assert.AreEqual( mesh.vertexCount,        statistics.vertexCount,   "vertices reported" );
            Assert.AreEqual( triangleCountOf( mesh ), statistics.triangleCount, "triangles reported" );

            // of the mesh as it was before it was creased
            var welded = MeshComparison.welded( mesh );

            this.finished.Add( welded );

            Assert.AreEqual( BurstMeshSimplifierTest.shapeOf( contour ), BurstMeshSimplifierTest.shapeOf( welded ), "shape" );

            // the GPU hands its vertices over in a different order every time, and the order decides between edges that
            // cost the same, so its simplified meshes are alike and not the same
            if( variant.implementation == ImplementationType.GPU ) {
                Assert.Less( triangleCountOf( mesh ), triangleCountOf( contour ) / 2, "triangles" );
            }
            else {
                assertSameTriangles( this.asShown( voxelizer, simplified ), mesh );
            }

            Assert.AreEqual( HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild, mesh.hideFlags, "the mesh is not saved with the scene" );
        }
        finally {
            Object.DestroyImmediate( simplified );
        }
    }

    // ---- the record of a simplification ----

    private static readonly Variant[] cpuVariants = variants.Where( ( variant ) => variant.implementation != ImplementationType.GPU ).ToArray( );

    // takes the volumes of a voxelizer away, so that contouring it again would show nothing
    private static void removeVolumes( Voxelizer voxelizer ) {
        foreach( var volume in voxelizer.GetComponentsInChildren<Volume>( ) ) {
            volume.gameObject.SetActive( false );
        }
    }

    // a voxelizer keeps its last contour as it was before it was simplified, with the collapses that simplify it.
    // simplify shows that contour again under the simplification settings as they are now: at another detail, or
    // under another error or line weight. It shows what contouring afresh under those settings shows, and it does not
    // contour: the volumes are gone by the time it is called
    [Test]
    public void simplifyShowsTheLastContourUnderTheSettingsAsTheyAreNow( [ValueSource( nameof( cpuVariants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.createSimplifying( variant, shapesOf( Model.ThreeBalls ), normals );
        var first     = voxelizer.contour( drawFromGPU: false );
        var simplest  = MeshComparison.trianglesOf( shown( voxelizer ) );

        removeVolumes( voxelizer );

        int assertShowsWhatAFreshContourShows( string what ) {
            var statistics = voxelizer.simplify( );

            Assert.IsTrue( statistics.HasValue, $"{what}: there is a contour to simplify" );

            var fresh = this.createSimplifying( variant, shapesOf( Model.ThreeBalls ), normals );

            fresh.simplificationError  = voxelizer.simplificationError;
            fresh.lineQuadricWeight    = voxelizer.lineQuadricWeight;
            fresh.simplificationDetail = voxelizer.simplificationDetail;

            var expected = fresh.contour( drawFromGPU: false );
            var mesh     = shown( voxelizer );

            assertSameTriangles( shown( fresh ), mesh );

            Assert.AreEqual( mesh.vertexCount,        statistics.Value.vertexCount,   $"{what}: vertices reported" );
            Assert.AreEqual( triangleCountOf( mesh ), statistics.Value.triangleCount, $"{what}: triangles reported" );
            Assert.AreEqual( expected.triangleCount,  statistics.Value.triangleCount, $"{what}: triangles, against those of a fresh contour" );

            Assert.AreEqual( statistics.Value.milliseconds, statistics.Value.simplificationMilliseconds, $"{what}: time simplifying, which is all of it" );

            Assert.AreEqual( first.unsimplifiedVertexCount,   statistics.Value.unsimplifiedVertexCount,   $"{what}: vertices before simplification" );
            Assert.AreEqual( first.unsimplifiedTriangleCount, statistics.Value.unsimplifiedTriangleCount, $"{what}: triangles before simplification" );

            Assert.AreEqual( HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild, mesh.hideFlags, $"{what}: the mesh is not saved with the scene" );

            if( normals == VertexNormals.Split ) {
                assertFlatShaded( mesh );
            }

            return triangleCountOf( mesh );
        }

        voxelizer.simplificationDetail = 0.5f;

        var half = assertShowsWhatAFreshContourShows( "at a detail of 0.5" );

        Assert.Greater( half, first.triangleCount.Value, "triangles at a detail of 0.5, against the simplest" );

        voxelizer.simplificationError = 0.05f;

        var finer = assertShowsWhatAFreshContourShows( "at an error of 0.05" );

        Assert.Greater( finer, half, "triangles at a fifth of the error" );

        voxelizer.simplificationDetail = 0.0f;
        voxelizer.lineQuadricWeight    = 0.05f;

        var heavy = assertShowsWhatAFreshContourShows( "at a line weight of 0.05" );

        Assert.Less( heavy, finer, "triangles at a detail of 0" );

        // and back where it began
        voxelizer.simplificationError = simplificationError;
        voxelizer.lineQuadricWeight   = 1e-5f;

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "there is a contour to simplify" );

        CollectionAssert.AreEqual( simplest, MeshComparison.trianglesOf( shown( voxelizer ) ), "triangles under the first settings again" );
    }

    // a contour that was read back from the GPU is kept like any other
    [Test]
    public void simplifyShowsAContourOfTheGPUAgain( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] VertexNormals normals ) {
        var voxelizer = this.createSimplifying( new( algorithm, ImplementationType.GPU ), shapesOf( Model.ThreeBalls ), normals );
        var first     = voxelizer.contour( drawFromGPU: true );
        var simplest  = MeshComparison.trianglesOf( shown( voxelizer ) );

        removeVolumes( voxelizer );

        voxelizer.simplificationDetail = 1.0f;

        var whole = voxelizer.simplify( );

        Assert.IsTrue( whole.HasValue, "there is a contour to simplify" );

        Assert.AreEqual( first.unsimplifiedTriangleCount, whole.Value.triangleCount,             "triangles at a detail of 1, against the contour's" );
        Assert.AreEqual( first.unsimplifiedTriangleCount, triangleCountOf( shown( voxelizer ) ), "triangles shown at a detail of 1" );

        Assert.Greater( first.unsimplifiedTriangleCount.Value, first.triangleCount.Value * 2, "triangles of the contour, against the simplest" );

        if( normals == VertexNormals.Split ) {
            assertFlatShaded( shown( voxelizer ) );
        }

        voxelizer.simplificationDetail = 0.0f;

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "there is a contour to simplify" );

        CollectionAssert.AreEqual( simplest, MeshComparison.trianglesOf( shown( voxelizer ) ), "triangles at a detail of 0 again" );
    }

    // contour shows the mesh at the detail the voxelizer is set to, and reports the size the contour came with
    [Test]
    public void contourShowsTheMeshAtTheDetail( [ValueSource( nameof( cpuVariants ) )] Variant variant ) {
        var voxelizer = this.createSimplifying( variant, shapesOf( Model.CubesAndSphere ) );

        voxelizer.simplificationDetail = 0.5f;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var contour    = this.voxelize( voxelizer ).mesh;
        var record     = MeshSimplification.of( contour, simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight );
        var expected   = record.meshAt( record.levelOf( 0.5f ) );

        try {
            assertSameTriangles( this.asShown( voxelizer, expected ), shown( voxelizer ) );

            Assert.AreEqual( contour.vertexCount,        statistics.unsimplifiedVertexCount,   "vertices before simplification" );
            Assert.AreEqual( triangleCountOf( contour ), statistics.unsimplifiedTriangleCount, "triangles before simplification" );

            // simplifying is part of the contour, and is timed by itself
            Assert.Greater( statistics.simplificationMilliseconds.Value, 0.0,                  "time simplifying" );
            Assert.Less( statistics.simplificationMilliseconds.Value, statistics.milliseconds, "time simplifying, against the whole contour" );

            Assert.Greater( statistics.triangleCount.Value,        triangleCountOf( contour ) / 4, "triangles at a detail of 0.5" );
            Assert.Less( statistics.triangleCount.Value, triangleCountOf( contour ) * 3 / 4,       "triangles at a detail of 0.5" );
        }
        finally {
            Object.DestroyImmediate( expected );
        }

        // at a detail of 1 it is the contour
        voxelizer.simplificationDetail = 1.0f;

        voxelizer.contour( drawFromGPU: false );

        assertSameTriangles( this.asShown( voxelizer, contour ), shown( voxelizer ) );
    }

    // simplify has nothing to show, and leaves what is shown alone, without a contour that was simplified: before the
    // first contour, with simplification off, after a contour made with it off, after clear, and after a contour of
    // nothing
    [Test]
    public void simplifyGivesNothingWithoutASimplifiedContour( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.ThreeBalls ) );

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "before the first contour" );

        var plain = voxelizer.contour( drawFromGPU: false );
        var mesh  = shown( voxelizer );

        Assert.IsNull( plain.unsimplifiedVertexCount,    "vertices before simplification, of a contour that was not simplified" );
        Assert.IsNull( plain.unsimplifiedTriangleCount,  "triangles before simplification, of a contour that was not simplified" );
        Assert.IsNull( plain.simplificationMilliseconds, "time simplifying, of a contour that was not simplified" );

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "with simplification off" );

        voxelizer.meshSimplification = true;

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "after a contour that was not simplified" );
        Assert.AreSame( mesh, shown( voxelizer ), "the mesh shown" );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "after a contour that was simplified" );

        mesh = shown( voxelizer );

        voxelizer.meshSimplification = false;

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "with simplification off again" );
        Assert.AreSame( mesh, shown( voxelizer ), "the mesh shown" );

        voxelizer.meshSimplification = true;

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "with simplification on again" );

        voxelizer.clear( );

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "after clear" );
        Assert.IsNull( shown( voxelizer ), "the mesh shown after clear" );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "after another contour" );

        removeVolumes( voxelizer );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsFalse( voxelizer.simplify( ).HasValue, "after a contour of nothing" );
        Assert.AreEqual( 0, shown( voxelizer ).vertexCount, "vertices shown after a contour of nothing" );
    }

    // the error is in the cells of the contour that is kept. A resolution that was changed since has not been
    // contoured yet, and says nothing about that contour
    [Test]
    public void simplifyKeepsToTheCellsOfTheContour( ) {
        var voxelizer = this.createSimplifying( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.Sphere ) );

        voxelizer.contour( drawFromGPU: false );

        var simplest = MeshComparison.trianglesOf( shown( voxelizer ) );

        voxelizer.resolution += 1;

        // under another line weight and back, which simplifies the contour again both times
        voxelizer.lineQuadricWeight = 0.05f;

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "there is a contour to simplify" );

        CollectionAssert.AreNotEqual( simplest, MeshComparison.trianglesOf( shown( voxelizer ) ), "triangles under another line weight" );

        voxelizer.lineQuadricWeight = 1e-5f;

        Assert.IsTrue( voxelizer.simplify( ).HasValue, "there is a contour to simplify" );

        CollectionAssert.AreEqual( simplest, MeshComparison.trianglesOf( shown( voxelizer ) ), "triangles under the first line weight again" );
    }

    // the weight of the line quadrics is the voxelizer's: with one far above what it starts out with, the mesh is the
    // one the simplifier gives at that weight and not the one it gives otherwise
    [Test]
    public void lineQuadricWeightIsTheVoxelizers( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm ) {
        var voxelizer = this.createSimplifying( new( algorithm, ImplementationType.Burst ), shapesOf( Model.CubesAndSphere ) );
        var usual     = voxelizer.lineQuadricWeight;

        voxelizer.lineQuadricWeight = 0.05f;

        voxelizer.contour( drawFromGPU: false );

        var contour  = this.voxelize( voxelizer ).mesh;
        var expected = BurstMeshSimplifier.simplify( contour, simplificationError * voxelizer.cellSize, 0.05f, out _ );
        var other    = BurstMeshSimplifier.simplify( contour, simplificationError * voxelizer.cellSize, usual, out _ );

        try {
            assertSameTriangles( this.asShown( voxelizer, expected ), shown( voxelizer ) );

            Assert.AreNotEqual( triangleCountOf( other ), triangleCountOf( expected ), "triangles at the two weights" );
        }
        finally {
            Object.DestroyImmediate( expected );
            Object.DestroyImmediate( other );
        }
    }

    // with the setting off, and whatever the error is set to, the mesh is the contour
    [Test]
    public void contourWithoutSimplificationShowsTheContour( [ValueSource( nameof( variants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, shapesOf( Model.SphereCutaway ), normals );

        voxelizer.simplificationError = 10.0f;

        voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( normals == VertexNormals.Split ? ContourVertices.Split : ContourVertices.Creased, voxelizer.contourVertices, "the vertices asked of the implementation" );

        // the GPU splits and creases vertices itself, and the meshes of the others are split or creased by contour
        var expected = this.asShown( voxelizer, this.voxelize( voxelizer ).mesh );

        assertSameTriangles( expected, shown( voxelizer ) );
    }

    // a mesh drawn straight from the compute buffers never reaches the CPU, where it is simplified. With the setting on
    // it is read back, whatever contour is asked to draw from
    [Test]
    public void simplifiedContourIsReadBackFromTheGPU( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] VertexNormals normals ) {
        var voxelizer = this.create( new( algorithm, ImplementationType.GPU ), shapesOf( Model.ThreeBalls ), normals );

        // drawing straight from the GPU reads the colours of the renderer's materials, and this renderer has none
        voxelizer.GetComponent<MeshRenderer>( ).sharedMaterials = new Material[0];

        var direct = voxelizer.contour( drawFromGPU: true );

        Assert.IsNull( direct.triangleCount, "triangles reported when drawing straight from the GPU" );

        voxelizer.meshSimplification  = true;
        voxelizer.simplificationError = simplificationError;

        var statistics = voxelizer.contour( drawFromGPU: true );
        var mesh       = shown( voxelizer );

        Assert.IsNotNull( mesh, "the mesh that is shown" );
        Assert.Greater( mesh.vertexCount, 0, "vertices" );

        Assert.AreEqual( triangleCountOf( mesh ), statistics.triangleCount, "triangles reported" );

        // it is asked for with shared vertices, whatever the voxelizer's setting
        Assert.AreEqual( ContourVertices.Shared, voxelizer.contourVertices, "the vertices asked of the implementation" );

        voxelizer.meshSimplification = false;

        var contour = voxelizer.contour( drawFromGPU: false );

        Assert.Less( statistics.triangleCount.Value, contour.triangleCount.Value / 2, "triangles, against the contour's" );
    }

    private static void assertFlatShaded( Mesh mesh ) {
        var vertices  = mesh.vertices;
        var normals   = mesh.normals;
        var triangles = Enumerable.Range( 0, mesh.subMeshCount ).SelectMany( ( subMesh ) => mesh.GetTriangles( subMesh ) ).ToArray( );

        Assert.AreEqual( triangles.Length, mesh.vertexCount, "a vertex for every corner of every triangle" );

        CollectionAssert.AreEquivalent( Enumerable.Range( 0, triangles.Length ), triangles, "every vertex is used once" );

        for( var index = 0; index < triangles.Length; index += 3 ) {
            var a = vertices[triangles[index]];
            var b = vertices[triangles[index + 1]];
            var c = vertices[triangles[index + 2]];

            // Vector3.normalized gives no direction to a vector shorter than 1e-5, which the cross product of two
            // sides of a triangle of a fine grid is
            var cross  = Vector3.Cross( b - a, c - a );
            var normal = cross / cross.magnitude;

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.LessOrEqual( ( normals[triangles[index + corner]] - normal ).magnitude, 1e-3f, $"normal of corner {corner} of triangle {index / 3} against the normal of its face" );
            }
        }
    }

    // flat shading takes vertices that no two triangles share, and simplification takes vertices that they do. The mesh
    // is simplified with shared vertices and split or creased afterwards, for the GPU as for the CPU
    [Test]
    public void simplifiedContourWithSplitNormalsIsSplitAfterwards( [ValueSource( nameof( variants ) )] Variant variant ) {
        var creased = this.createSimplifying( variant, shapesOf( Model.CubesAndSphere ), VertexNormals.Creased );
        var split   = this.createSimplifying( variant, shapesOf( Model.CubesAndSphere ), VertexNormals.Split );

        var creasedStatistics = creased.contour( drawFromGPU: false );
        var splitStatistics   = split.contour( drawFromGPU: false );

        assertFlatShaded( shown( split ) );

        Assert.Less( creasedStatistics.vertexCount.Value, creasedStatistics.triangleCount.Value * 3, "vertices of the mesh with creased normals, against three for every triangle" );

        // the same simplification either way. The GPU's differs from one contour to the next
        if( variant.implementation != ImplementationType.GPU ) {
            Assert.AreEqual( creasedStatistics.triangleCount, splitStatistics.triangleCount, "triangles" );

            CollectionAssert.AreEquivalent(
                MeshComparison.trianglesOf( shown( creased ) ).Select( withoutNormals ),
                MeshComparison.trianglesOf( shown( split ) ).Select( withoutNormals ),
                "the triangles, by the positions of their corners"
            );
        }
    }

    // a triangle as MeshComparison describes it, a position and a normal for every corner, without the normals
    private static string withoutNormals( string triangle ) {
        return string.Join( " ", triangle.Split( ' ' ).Select( ( corner ) => corner.Split( '/' )[0] ) );
    }

    // split, a mesh has three vertices for every triangle, which passes 65,535 at 21,845 triangles. The three balls on
    // 128^3 cells keep more than that at a small error
    [Test]
    public void simplifiedContourOfManyTrianglesIsSplitWithThirtyTwoBitIndices( ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( 7, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ) );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.vertexNormals        = VertexNormals.Split;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = true;
        voxelizer.simplificationError  = 0.01f;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var mesh       = shown( voxelizer );

        Assert.Greater( statistics.triangleCount.Value, 21845, "triangles" );
        Assert.Less( statistics.triangleCount.Value, 104500 / 2, "triangles, against half of the contour's" );

        Assert.AreEqual( UnityEngine.Rendering.IndexFormat.UInt32, mesh.indexFormat, "index format" );
        Assert.AreEqual( statistics.triangleCount.Value * 3, mesh.vertexCount, "vertices" );

        assertFlatShaded( mesh );
    }

    [Test]
    public void contourOfNothingWithSimplificationShowsAnEmptyMesh( [ValueSource( nameof( variants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.createSimplifying( variant, new Shape[0], normals );

        var statistics = voxelizer.contour( drawFromGPU: false );

        Assert.AreEqual( 0, shown( voxelizer ).vertexCount );
        Assert.AreEqual( 0, statistics.vertexCount );
        Assert.AreEqual( 0, statistics.triangleCount );
    }

    // the error is in cells, and a cell is what the grid is divided into: an octree of a depth has half as many leaves
    // across as a grid of that resolution has cells
    [Test]
    public void cellSizeIsOneOverTheCellsAcross( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.Sphere ) );

        Assert.AreEqual( 1.0f / 16.0f, voxelizer.cellSize, "the size of a cell of 16 across" );

        voxelizer.resolution += 1;

        Assert.AreEqual( 1.0f / 32.0f, voxelizer.cellSize, "the size of a cell of 32 across" );
    }

    // the error is measured in cells: the same shapes on a grid twice as fine, simplified at twice the error in cells,
    // come out with about as many triangles
    [Test]
    public void simplificationErrorIsInCells( [Values( Algorithm.Uniform, Algorithm.Manifold, Algorithm.Occupancy )] Algorithm algorithm ) {
        int trianglesAt( int cellsExponent, float error ) {
            var voxelizer = this.createSimplifying( new( algorithm, ImplementationType.Burst ), shapesOf( Model.Sphere ) );

            voxelizer.resolution         += cellsExponent - 4;
            voxelizer.simplificationError = error;

            return voxelizer.contour( drawFromGPU: false ).triangleCount.Value;
        }

        var coarse = trianglesAt( 5, 0.1f );
        var fine   = trianglesAt( 6, 0.2f );
        var finer  = trianglesAt( 6, 0.1f );

        Assert.AreEqual( coarse, fine, coarse * 0.25, "triangles on 64^3 cells at 0.2 cells, against those on 32^3 at 0.1" );

        Assert.Greater( finer, fine * 1.5, "triangles on 64^3 cells at 0.1 cells, against those at 0.2" );
    }

}

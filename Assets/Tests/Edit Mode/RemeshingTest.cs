using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using SmoothNormals      = SurfaceExtractor.SmoothNormals;
using ContourVertices    = SurfaceExtractor.ContourVertices;
using QEFSolverType      = QEFSolver.Type;

// the frame the remeshing techniques go into (Documents/Plans/Remeshing.md, step 1), through Voxelizer.contour,
// simplify and remesh: which meshes are kept for remeshing, that a remesh is what a fresh contour under the same
// settings shows, and that the flags of the faces follow the mesh through creasing, splitting and reordering, for every
// algorithm and implementation. Field Aligned stands for the techniques where any will do, since it makes quads
public class RemeshingTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold,
        Occupancy
    }

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
        new( Algorithm.Uniform,   ImplementationType.Managed ),
        new( Algorithm.Uniform,   ImplementationType.Burst ),
        new( Algorithm.Uniform,   ImplementationType.GPU ),
        new( Algorithm.Adaptive,  ImplementationType.Managed ),
        new( Algorithm.Adaptive,  ImplementationType.Burst ),
        new( Algorithm.Manifold,  ImplementationType.Managed ),
        new( Algorithm.Manifold,  ImplementationType.Burst ),
        new( Algorithm.Occupancy, ImplementationType.Burst ),
        new( Algorithm.Occupancy, ImplementationType.GPU )
    };

    private static readonly Variant[] cpuVariants = variants.Where( ( variant ) => variant.implementation != ImplementationType.GPU ).ToArray( );

    private static readonly Variant[] gpuVariants = variants.Where( ( variant ) => variant.implementation == ImplementationType.GPU ).ToArray( );

    // models with flat faces, along the grid and turned, of one material and of two
    public enum Surface {
        CubesAndSphere,
        SlabWithBand,
        TurnedShapes
    }

    private static Shape[] shapesOf( Surface surface ) {
        return surface switch {
            Surface.CubesAndSphere => shapesOf( Model.CubesAndSphere ),
            Surface.SlabWithBand   => slabWithBand,
            Surface.TurnedShapes   => turnedShapes,
            _                      => throw new ArgumentException( surface.ToString( ) )
        };
    }

    // 16^3 cells for the uniform grids, 16^3 leaves for the octrees
    private const int resolution = 4;

    private Voxelizer create( Variant variant, IEnumerable<Shape> shapes, VertexNormals normals = VertexNormals.Creased, Remeshing.Technique technique = Remeshing.Technique.FieldAligned ) {
        Voxelizer voxelizer = variant.algorithm switch {
            Algorithm.Uniform   => this.createVoxelizer<UniformDualContouring>  ( resolution,     QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive  => this.createVoxelizer<AdaptiveDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold  => this.createVoxelizer<ManifoldDualContouring> ( resolution + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Occupancy => this.createVoxelizer<OccupancyDualContouring>( resolution,     QEFSolverType.SVD, shapes ),
            _                   => throw new ArgumentException( variant.ToString( ) )
        };

        voxelizer.implementationType   = variant.implementation;
        voxelizer.vertexNormals        = normals;
        voxelizer.creaseAngle          = 40.0f;
        voxelizer.keepDebugInformation = false;
        voxelizer.remeshing            = technique;

        return voxelizer;
    }

    private readonly List<Object> made = new( );

    [TearDown]
    public void destroyMade( ) {
        foreach( var item in this.made ) {
            Object.DestroyImmediate( item );
        }
        this.made.Clear( );
    }

    private static Mesh shown( Voxelizer voxelizer ) {
        var meshFilter = voxelizer.GetComponent<MeshFilter>( );

        return meshFilter == null ? null : meshFilter.sharedMesh;
    }

    private static int triangleCountOf( Mesh mesh ) {
        return Enumerable.Range( 0, mesh.subMeshCount ).Sum( ( subMesh ) => ( int )mesh.GetIndexCount( subMesh ) / 3 );
    }

    // takes the volumes of a voxelizer away, so that contouring it again would show nothing
    private static void removeVolumes( Voxelizer voxelizer ) {
        foreach( var volume in voxelizer.GetComponentsInChildren<Volume>( ) ) {
            volume.gameObject.SetActive( false );
        }
    }

    // the faces the flags of a mesh make, counted as a remesh counts its own (BurstRemeshing.count), with the vertices at
    // one position made one: a creased or split mesh has several there. Where the flags follow the triangles through
    // creasing, splitting and reordering, these are the faces the remesh says it made
    private static BurstRemeshing.FaceCounts countsOf( Mesh mesh, IReadOnlyList<byte> flags ) {
        var vertices = mesh.vertices;
        var places   = new Dictionary<Vector3, int>( );
        var built    = new List<BurstMesh.Triangle>( );

        int place( int index ) {
            if( !places.TryGetValue( vertices[index], out var at ) ) {
                places.Add( vertices[index], at = places.Count );
            }
            return at;
        }

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices = mesh.GetTriangles( subMesh );

            for( var index = 0; index < indices.Length; index += 3 ) {
                built.Add( new BurstMesh.Triangle( place( indices[index] ), place( indices[index + 1] ), place( indices[index + 2] ), subMesh ) );
            }
        }

        Assert.AreEqual( built.Count, flags.Count, "a flag for every triangle" );

        using var triangles = new Unity.Collections.NativeArray<BurstMesh.Triangle>( built.ToArray( ), Unity.Collections.Allocator.TempJob );
        using var native    = new Unity.Collections.NativeArray<byte>( flags.ToArray( ), Unity.Collections.Allocator.TempJob );

        BurstRemeshing.count( triangles, native, out _, out var faces );

        return faces;
    }

    // the faces the flags make of a mesh, in the order of its index buffer: every edge a triangle leaves out is left out
    // by the one other triangle on it too, of the same sub-mesh, and the triangles joined across such edges are a face
    // of as many sides as they draw edges. A creased or split mesh has several vertices at a position, so edges are found
    // by the positions of their ends
    private static ( int triangles, int quads, int larger ) facesOf( Mesh mesh, IReadOnlyList<byte> flags ) {
        var vertices  = mesh.vertices;
        var triangles = new List<( int a, int b, int c, int subMesh )>( );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices = mesh.GetTriangles( subMesh );

            for( var index = 0; index < indices.Length; index += 3 ) {
                triangles.Add( ( indices[index], indices[index + 1], indices[index + 2], subMesh ) );
            }
        }

        Assert.AreEqual( triangles.Count, flags.Count, "a flag for every triangle" );

        var parents = Enumerable.Range( 0, triangles.Count ).ToArray( );

        int find( int item ) {
            while( parents[item] != item ) {
                item = parents[item] = parents[parents[item]];
            }
            return item;
        }

        var leftOut = new Dictionary<( Vector3, Vector3 ), List<int>>( );
        var sides   = new int[triangles.Count];

        for( var triangle = 0; triangle < triangles.Count; ++triangle ) {
            var corners = new[] { triangles[triangle].a, triangles[triangle].b, triangles[triangle].c };

            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    ++sides[triangle];
                    continue;
                }

                var a   = vertices[corners[k]];
                var b   = vertices[corners[( k + 1 ) % 3]];
                var key = a.x < b.x || ( a.x == b.x && ( a.y < b.y || ( a.y == b.y && a.z < b.z ) ) ) ? ( a, b ) : ( b, a );

                if( !leftOut.TryGetValue( key, out var list ) ) {
                    leftOut.Add( key, list = new List<int>( ) );
                }
                list.Add( triangle );
            }
        }

        foreach( var entry in leftOut ) {
            Assert.AreEqual( 2, entry.Value.Count, $"triangles that leave out the edge {entry.Key}" );

            var first  = entry.Value[0];
            var second = entry.Value[1];

            Assert.AreEqual( triangles[first].subMesh, triangles[second].subMesh, $"sub-meshes of the triangles across {entry.Key}" );

            parents[find( first )] = find( second );
        }

        var faceSides = new Dictionary<int, int>( );

        for( var triangle = 0; triangle < triangles.Count; ++triangle ) {
            var root = find( triangle );

            faceSides[root] = ( faceSides.TryGetValue( root, out var count ) ? count : 0 ) + sides[triangle];
        }

        Assert.IsTrue( faceSides.Values.All( ( count ) => count >= 3 ), "every face has three sides or more" );

        return ( faceSides.Values.Count( ( count ) => count == 3 ), faceSides.Values.Count( ( count ) => count == 4 ), faceSides.Values.Count( ( count ) => count > 4 ) );
    }

    // ---- with remeshing off ----

    [Test]
    public void withRemeshingOffTheContourHasNoFlags( [ValueSource( nameof( variants ) )] Variant variant ) {
        var voxelizer  = this.create( variant, shapesOf( Model.CubesAndSphere ), technique: Remeshing.Technique.Off );
        var statistics = voxelizer.contour( drawFromGPU: false );

        Assert.IsNull( statistics.remeshing,    "what a remesh did" );
        Assert.IsNull( voxelizer.shownEdgeFlags, "the flags of the mesh shown" );

        // the GPU still creases its own vertices
        Assert.AreEqual( ContourVertices.Creased, voxelizer.contourVertices, "what the implementation is asked for" );
    }

    // ---- a technique through the frame ----

    // Field Aligned's faces through the frame: the flags of the mesh shown make the faces the remesh says it made, as
    // the mesh is shown, creased or split, its vertices reordered or not, of one material or two. A GPU contour is read
    // back to be remeshed. Its faces are quads, with triangles and pentagons among them
    [Test]
    public void aRemeshsFacesGoThroughTheFrame(
        [ValueSource( nameof( variants ) )] Variant variant, [Values] Surface surface, [Values] VertexNormals normals, [Values] bool reorderVertices
    ) {
        var voxelizer  = this.create( variant, shapesOf( surface ), normals );
        var statistics = voxelizer.contour( drawFromGPU: true, reorderVertices: reorderVertices );
        var mesh       = shown( voxelizer );

        Assert.IsNotNull( mesh, "a mesh, read back from the GPU too" );
        Assert.IsTrue( statistics.remeshing.HasValue, "what the remesh did" );

        var remeshing = statistics.remeshing.Value;

        Assert.IsNull( remeshing.gaveUp, "gave up" );
        Assert.AreEqual( Remeshing.Technique.FieldAligned, remeshing.technique );
        Assert.IsTrue( remeshing.fieldAligned.HasValue, "what Field Aligned did" );

        var faces = countsOf( mesh, voxelizer.shownEdgeFlags );

        Assert.AreEqual( remeshing.triangles,   faces.triangles, "triangles" );
        Assert.AreEqual( remeshing.quads,       faces.quads,     "quads" );
        Assert.AreEqual( remeshing.largerFaces, faces.larger,    "larger faces" );
        Assert.Greater( faces.quads, 4 * ( faces.triangles + faces.larger ), "mostly quads" );

        // where the remesh is a manifold, as on these models but the turned shapes, where sheets meet, every edge a
        // triangle leaves out is left out by the one other triangle on it, of its sub-mesh (finding 7)
        if( surface != Surface.TurnedShapes ) {
            Assert.AreEqual( 0, faces.inconsistent, "flags that make no face" );
            Assert.AreEqual( ( faces.triangles, faces.quads, faces.larger ), facesOf( mesh, voxelizer.shownEdgeFlags ), "faces, by the edges left out" );
        }

        // the GPU hands its vertices over in a different order every time, so only the CPU's contour can be had again
        // to work out the length from
        if( variant.implementation == ImplementationType.GPU ) {
            Assert.Greater( remeshing.edgeLength, 0.5, "edge length in cells, of a contour of about a cell" );
            Assert.Less( remeshing.edgeLength, 2.0,    "edge length in cells, of a contour of about a cell" );
        }
        else {
            // quads have a vertex for every l^2 of the area: at 1 as many as the contour has
            var contour = this.voxelize( voxelizer ).mesh;

            this.made.Add( contour );

            var vertices  = contour.vertices;
            var triangles = Enumerable.Range( 0, contour.subMeshCount ).SelectMany( ( subMesh ) => contour.GetTriangles( subMesh ) ).ToArray( );
            var area      = 0.0;

            for( var index = 0; index < triangles.Length; index += 3 ) {
                area += 0.5 * Vector3.Cross( vertices[triangles[index + 1]] - vertices[triangles[index]], vertices[triangles[index + 2]] - vertices[triangles[index]] ).magnitude;
            }

            var used = triangles.Distinct( ).Count( );

            Assert.AreEqual( Math.Sqrt( area / used ) / voxelizer.cellSize, remeshing.edgeLength, remeshing.edgeLength * 1e-4, "edge length in cells" );
            Assert.AreEqual( triangles.Length / 3, remeshing.inputTriangleCount, "triangles given" );
        }
    }

    // a voxelizer keeps the mesh it gave the remeshing pass, remeshing on or off. remesh shows it again as the settings
    // are now, and shows what a fresh contour under them shows, without contouring: the volumes are gone by then
    [Test]
    public void remeshShowsTheKeptMeshUnderTheSettingsAsTheyAreNow( [ValueSource( nameof( cpuVariants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, shapesOf( Model.CubesAndSphere ), normals, Remeshing.Technique.Off );

        voxelizer.contour( drawFromGPU: false );

        var plain = MeshComparison.trianglesOf( shown( voxelizer ) );

        removeVolumes( voxelizer );

        void assertShowsWhatAFreshContourShows( string what ) {
            var statistics = voxelizer.remesh( );

            Assert.IsTrue( statistics.HasValue, $"{what}: there is a mesh to remesh" );

            var fresh = this.create( variant, shapesOf( Model.CubesAndSphere ), normals, voxelizer.remeshing );

            fresh.remeshingEdgeLength = voxelizer.remeshingEdgeLength;

            var expected = fresh.contour( drawFromGPU: false );

            MeshComparison.assertSameTriangles( shown( fresh ), shown( voxelizer ) );

            CollectionAssert.AreEqual( fresh.shownEdgeFlags, voxelizer.shownEdgeFlags, $"{what}: flags" );

            Assert.AreEqual( expected.triangleCount, statistics.Value.triangleCount, $"{what}: triangles reported" );
            Assert.AreEqual( expected.vertexCount,   statistics.Value.vertexCount,   $"{what}: vertices reported" );

            Assert.AreEqual( expected.remeshing.HasValue, statistics.Value.remeshing.HasValue, $"{what}: whether it was remeshed" );

            if( expected.remeshing.HasValue ) {
                Assert.AreEqual( expected.remeshing.Value.quads,      statistics.Value.remeshing.Value.quads,      $"{what}: quads" );
                Assert.AreEqual( expected.remeshing.Value.edgeLength, statistics.Value.remeshing.Value.edgeLength, 1e-9, $"{what}: edge length" );
            }

            Assert.AreEqual( HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild, shown( voxelizer ).hideFlags, $"{what}: the mesh is not saved with the scene" );
        }

        voxelizer.remeshing = Remeshing.Technique.FieldAligned;

        assertShowsWhatAFreshContourShows( "turned on" );

        var length = voxelizer.remesh( ).Value.remeshing.Value.edgeLength;

        voxelizer.remeshingEdgeLength = 2.0f;

        assertShowsWhatAFreshContourShows( "at twice the length" );

        Assert.AreEqual( 2.0 * length, voxelizer.remesh( ).Value.remeshing.Value.edgeLength, 1e-9, "edge length at twice the setting" );

        voxelizer.remeshing = Remeshing.Technique.Off;

        assertShowsWhatAFreshContourShows( "turned off" );

        Assert.IsNull( voxelizer.shownEdgeFlags, "flags with remeshing off" );

        CollectionAssert.AreEqual( plain, MeshComparison.trianglesOf( shown( voxelizer ) ), "the contour as it was shown first" );
    }

    // remesh has nothing to show, and leaves what is shown alone, without a kept mesh: before the first contour, after
    // clear, and after a contour of nothing
    [Test]
    public void remeshGivesNothingWithoutAKeptMesh( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.ThreeBalls ) );

        Assert.IsFalse( voxelizer.remesh( ).HasValue, "before the first contour" );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsTrue( voxelizer.remesh( ).HasValue, "after a contour" );

        voxelizer.clear( );

        Assert.IsFalse( voxelizer.remesh( ).HasValue, "after clear" );
        Assert.IsNull( shown( voxelizer ), "the mesh shown after clear" );
        Assert.IsNull( voxelizer.shownEdgeFlags, "the flags after clear" );

        voxelizer.contour( drawFromGPU: false );

        removeVolumes( voxelizer );

        var statistics = voxelizer.contour( drawFromGPU: false );

        Assert.IsNull( statistics.remeshing, "what a remesh of nothing did" );
        Assert.IsFalse( voxelizer.remesh( ).HasValue, "after a contour of nothing" );
        Assert.AreEqual( 0, shown( voxelizer ).vertexCount, "vertices shown after a contour of nothing" );
    }

    // the GPU splits or creases its own vertices when it is not asked for shared ones, and draws straight from its
    // buffers when it can. Neither keeps a mesh to remesh. With remeshing on it is asked for shared vertices, and its
    // contour is read back and kept
    [Test]
    public void aGPUContourIsKeptOnlyWithSharedVertices( [ValueSource( nameof( gpuVariants ) )] Variant variant, [Values] VertexNormals normals ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ), normals, Remeshing.Technique.Off );

        // drawing straight from the GPU takes the colour of the renderer's material
        var material = Object.Instantiate( Resources.Load<Material>( "Materials/unlit_white" ) );

        material.hideFlags = HideFlags.HideAndDontSave;
        voxelizer.GetComponent<MeshRenderer>( ).sharedMaterial = material;
        this.made.Add( material );

        Assert.AreEqual( normals == VertexNormals.Split ? ContourVertices.Split : ContourVertices.Creased, voxelizer.contourVertices, "asked for with remeshing off" );

        voxelizer.contour( drawFromGPU: false );

        Assert.IsFalse( voxelizer.remesh( ).HasValue, "after a contour read back, split or creased by the GPU" );

        voxelizer.contour( drawFromGPU: true );

        Assert.IsNull( shown( voxelizer ), "drawn straight from the GPU" );
        Assert.IsFalse( voxelizer.remesh( ).HasValue, "after a contour drawn straight from the GPU" );

        voxelizer.remeshing = Remeshing.Technique.FieldAligned;

        Assert.AreEqual( ContourVertices.Shared, voxelizer.contourVertices, "asked for with remeshing on" );

        var statistics = voxelizer.contour( drawFromGPU: true );

        Assert.IsNotNull( shown( voxelizer ), "read back to be remeshed" );
        Assert.IsTrue( statistics.remeshing.HasValue, "remeshed" );
        Assert.IsTrue( voxelizer.remesh( ).HasValue, "kept" );
    }

    // the pass comes after the simplifier, and is given what the simplifier shows: at the detail, and again after
    // simplify shows another
    [Test]
    public void aSimplifiedContourIsRemeshed( [ValueSource( nameof( cpuVariants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.CubesAndSphere ) );

        voxelizer.meshSimplification   = true;
        voxelizer.simplificationError  = 0.25f;
        voxelizer.simplificationDetail = 0.0f;

        var contour   = this.voxelize( voxelizer ).mesh;
        var record    = MeshSimplification.of( contour, voxelizer.simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight );
        var simplest  = record.meshAt( record.levelOf( 0.0f ) );
        var halfway   = record.meshAt( record.levelOf( 0.5f ) );

        try {
            var statistics = voxelizer.contour( drawFromGPU: false );

            Assert.AreEqual( triangleCountOf( simplest ), statistics.remeshing.Value.inputTriangleCount, "triangles given, at a detail of 0" );
            Assert.AreEqual( simplest.vertexCount,        statistics.remeshing.Value.inputVertexCount,   "vertices given, at a detail of 0" );
            Assert.AreEqual( facesOf( shown( voxelizer ), voxelizer.shownEdgeFlags ).quads, statistics.remeshing.Value.quads, "quads" );

            voxelizer.simplificationDetail = 0.5f;

            var again = voxelizer.simplify( );

            Assert.IsTrue( again.HasValue, "simplified again" );
            Assert.AreEqual( triangleCountOf( halfway ), again.Value.remeshing.Value.inputTriangleCount, "triangles given, at a detail of 0.5" );
            Assert.AreEqual( facesOf( shown( voxelizer ), voxelizer.shownEdgeFlags ).quads, again.Value.remeshing.Value.quads, "quads at a detail of 0.5" );

            // and remesh starts from the mesh at that detail
            voxelizer.remeshing = Remeshing.Technique.Off;

            Assert.AreEqual( triangleCountOf( halfway ), voxelizer.remesh( ).Value.triangleCount, "triangles at a detail of 0.5, remeshing off" );
        }
        finally {
            Object.DestroyImmediate( contour );
            Object.DestroyImmediate( simplest );
            Object.DestroyImmediate( halfway );
        }
    }

    // the length is reported in the cells of the contour that is kept, whatever the resolution has been set to since,
    // as the simplification's error is
    [Test]
    public void theEdgeLengthIsInTheCellsOfTheKeptContour( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.CubesAndSphere ) );

        voxelizer.meshSimplification = true;

        var length = voxelizer.contour( drawFromGPU: false ).remeshing.Value.edgeLength;

        voxelizer.resolution += 1;

        Assert.AreEqual( length, voxelizer.simplify( ).Value.remeshing.Value.edgeLength, 1e-9, "after simplify" );
        Assert.AreEqual( length, voxelizer.remesh( ).Value.remeshing.Value.edgeLength,   1e-9, "after remesh" );
    }

    // split, a mesh has three vertices for every triangle, which passes 65,535 at 21,845 triangles. The flags still
    // follow its triangles
    [Test]
    public void aRemeshedContourOfManyTrianglesIsSplitWithThirtyTwoBitIndices( [Values( Remeshing.Technique.FieldAligned, Remeshing.Technique.Isotropic )] Remeshing.Technique technique ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( 6, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ) );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.vertexNormals        = VertexNormals.Split;
        voxelizer.keepDebugInformation = false;
        voxelizer.remeshing            = technique;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var mesh       = shown( voxelizer );

        Assert.Greater( statistics.triangleCount.Value, 21845, "triangles" );
        Assert.AreEqual( UnityEngine.Rendering.IndexFormat.UInt32, mesh.indexFormat, "index format" );
        Assert.AreEqual( statistics.remeshing.Value.quads, voxelizer.shownEdgeFlags == null ? 0 : countsOf( mesh, voxelizer.shownEdgeFlags ).quads, "quads" );
        Assert.AreEqual( 3 * statistics.triangleCount.Value, mesh.vertexCount, "three vertices for every triangle" );
    }

    // ---- Isotropic ----

    // the kept mesh as it came, and remeshed by a technique, shown with shared vertices: creased at 180 degrees, every
    // triangle around a vertex is one group. A GPU contour is only kept while remeshing is on, so it contours remeshed
    private ( Mesh input, Mesh output, Remeshing.Statistics statistics ) remeshedOf( Voxelizer voxelizer, Remeshing.Technique technique ) {
        voxelizer.vertexNormals = VertexNormals.Creased;
        voxelizer.creaseAngle   = 180.0f;
        voxelizer.remeshing     = technique;

        voxelizer.contour( drawFromGPU: false );

        voxelizer.remeshing = Remeshing.Technique.Off;
        voxelizer.remesh( );

        var input = Object.Instantiate( shown( voxelizer ) );

        this.made.Add( input );

        voxelizer.remeshing = technique;

        var statistics = voxelizer.remesh( ).Value.remeshing.Value;

        return ( input, shown( voxelizer ), statistics );
    }

    private ( Mesh input, Mesh output, Remeshing.Statistics statistics ) isotropicOf( Voxelizer voxelizer ) {
        return this.remeshedOf( voxelizer, Remeshing.Technique.Isotropic );
    }

    private static BurstMesh.Triangle[] trianglesWithSubMeshesOf( Mesh mesh ) {
        return Enumerable.Range( 0, mesh.subMeshCount ).SelectMany(
            ( subMesh ) => {
                var indices = mesh.GetTriangles( subMesh );

                return Enumerable.Range( 0, indices.Length / 3 ).Select( ( triangle ) => new BurstMesh.Triangle( indices[triangle * 3], indices[( triangle * 3 ) + 1], indices[( triangle * 3 ) + 2], subMesh ) );
            }
        ).ToArray( );
    }

    private static BurstRemeshing.Topology linesOf( Mesh mesh ) {
        using var triangles = new Unity.Collections.NativeArray<BurstMesh.Triangle>( trianglesWithSubMeshesOf( mesh ), Unity.Collections.Allocator.TempJob );
        using var every     = new Unity.Collections.NativeArray<byte>( 0, Unity.Collections.Allocator.TempJob );

        BurstRemeshing.count( triangles, every, out var topology, out _ );

        return topology;
    }

    // the nearest point of a mesh to a point, facing either way, and the normal of the triangle it is on
    private static double nearestOn( SurfaceGrid grid, Vector3 point, double reach, out double3 normal ) {
        var from     = ( double3 )( float3 )point;
        var best     = double.MaxValue;

        normal = double3.zero;

        foreach( var facing in new[] { new double3( 1, 0, 0 ), new double3( -1, 0, 0 ), new double3( 0, 1, 0 ), new double3( 0, -1, 0 ), new double3( 0, 0, 1 ), new double3( 0, 0, -1 ) } ) {
            if( grid.nearestOnSurface( from, SurfaceGrid.anySubMesh, facing, reach, out var nearest, out var triangle ) && math.distance( from, nearest ) < best ) {
                best   = math.distance( from, nearest );
                normal = grid.normals[triangle];
            }
        }

        return best;
    }

    // what the isotropic remesh keeps of the mesh it is given, for every algorithm and implementation, contoured and
    // simplified: the topology of the surface and of every material, its lines of open edges and of edges of more than
    // two triangles, its pinched vertices; every vertex on that mesh, every triangle facing the way the mesh does there;
    // about as many vertices; and the same remesh every time
    [Test]
    public void theIsotropicRemeshKeepsWhatTheMeshIs( [ValueSource( nameof( variants ) )] Variant variant, [Values] Surface surface, [Values] bool simplified ) {
        var voxelizer = this.create( variant, shapesOf( surface ), VertexNormals.Creased, Remeshing.Technique.Off );

        voxelizer.meshSimplification = simplified;

        var ( input, output, statistics ) = this.isotropicOf( voxelizer );
        var cellSize                       = ( double )voxelizer.cellSize;
        var length                         = statistics.edgeLength * cellSize;

        Debug.Log( $"{variant} {surface} {( simplified ? "simplified" : "contour" )}: {output.vertexCount} vertices from {input.vertexCount}, ratio {output.vertexCount / ( double )input.vertexCount:F3}, edges of {statistics.edgeLength:F2} cells" );

        Assert.IsNull( statistics.gaveUp, "gave up" );
        Assert.AreEqual( 0, statistics.newOpenLines,        "new lines of open edges" );
        Assert.AreEqual( 0, statistics.newNonManifoldLines, "new lines of edges of more than two triangles" );
        Assert.AreEqual( 0, statistics.isotropic.Value.unplaced, "vertices without a point of the mesh within reach" );

        // the topology
        var was = MeshTopology.of( input );
        var now = MeshTopology.of( output );

        Assert.AreEqual( was.components,          now.components,          "pieces" );
        Assert.AreEqual( was.eulerCharacteristic, now.eulerCharacteristic, "euler characteristic" );
        Assert.AreEqual( was.pinchedVertices,     now.pinchedVertices,     "pinched vertices" );
        Assert.LessOrEqual( now.windingConflicts, was.windingConflicts,   "edges two triangles run along the same way" );
        Assert.AreEqual( 0,                       now.degenerateTriangles, "triangles that use a vertex twice" );

        Assert.AreEqual( input.subMeshCount, output.subMeshCount, "sub-meshes" );

        for( var subMesh = 0; subMesh < input.subMeshCount; ++subMesh ) {
            var wasOf = MeshTopology.of( input, subMesh );
            var nowOf = MeshTopology.of( output, subMesh );

            Assert.AreEqual( ( wasOf.components, wasOf.eulerCharacteristic ), ( nowOf.components, nowOf.eulerCharacteristic ), $"pieces and euler characteristic of sub-mesh {subMesh}" );
        }

        var wasLines = linesOf( input );
        var nowLines = linesOf( output );

        Assert.AreEqual( ( wasLines.openLines, wasLines.nonManifoldLines ), ( nowLines.openLines, nowLines.nonManifoldLines ), "lines of open edges and of edges of more than two triangles" );

        // on the mesh it was given, and facing its way
        var built = new HandBuiltMesh( );

        built.positions.AddRange( input.vertices );
        built.triangles.AddRange( trianglesWithSubMeshesOf( input ) );

        using var features = new FeatureMeshRun( built, 40.0 );
        using var grid     = SurfaceGrid.of( features.mesh, 2.0 * cellSize, Unity.Collections.Allocator.TempJob );

        var positions = output.vertices;

        foreach( var position in positions ) {
            Assert.LessOrEqual( nearestOn( grid, position, cellSize, out _ ) / cellSize, 1e-5, $"the vertex at {position} is on the mesh, in cells" );
        }

        foreach( var triangle in trianglesWithSubMeshesOf( output ) ) {
            var a      = ( double3 )( float3 )positions[triangle.a];
            var b      = ( double3 )( float3 )positions[triangle.b];
            var c      = ( double3 )( float3 )positions[triangle.c];
            var facing = math.cross( b - a, c - a );
            var middle = ( a + b + c ) / 3.0;

            Assert.Greater( math.length( facing ), 0.0, $"the triangle at {middle} has an area" );

            // within an edge length of the mesh: corners by angle closer than that become one, and cut the shape
            var off = nearestOn( grid, ( float3 )middle, 2.0 * length, out _ );

            Assert.LessOrEqual( off, length, $"the middle of the triangle at {middle / cellSize} in cells is within an edge length of the mesh" );

            // and where it lies on the mesh, it does not face away from the mesh at its corners and its middle alike,
            // but where the mesh itself faces its way within half a cell, as where it folds or crumples: a contour or
            // its simplified mesh may, and a fold stays one, the remesh turning no triangle over from how it was
            // (step 3 of the plan)
            if( off < 0.25 * cellSize ) {
                var away = new[] { a, b, c, middle }.All( ( point ) => {
                    nearestOn( grid, ( float3 )point, cellSize, out var there );

                    return !( math.dot( facing, there ) > 0.0 );
                } );

                Assert.IsTrue(
                    !away || grid.nearestOnSurface( middle, SurfaceGrid.anySubMesh, facing, 0.5 * cellSize, out _, out _ ),
                    $"the triangle {a / cellSize}, {b / cellSize}, {c / cellSize} in cells, of normal {math.normalize( facing )}, faces away from the mesh, which does not fold there"
                );
            }
        }

        // about as many vertices, on meshes of 300 vertices and more: from 1.06 to 1.18 as many at this grid when the plan
        // measured it, where the lines are a larger share of the mesh than at 64^3, which gave 0.95 to 1.07 (step 3).
        // The simplified meshes here are all smaller
        if( input.vertexCount >= 300 ) {
            var ratio = output.vertexCount / ( double )input.vertexCount;

            Assert.That( ratio, Is.InRange( 0.9, 1.25 ), $"vertices, against the {input.vertexCount} of the mesh given" );
        }

        // and the same again
        var first = MeshComparison.trianglesOf( output );

        voxelizer.remesh( );

        CollectionAssert.AreEqual( first, MeshComparison.trianglesOf( shown( voxelizer ) ), "the second remesh" );
    }

    // every vertex is on the mesh it was given at 64^3 too, on the turned shapes, whose triangles bridge their creases
    // and fold most, and where smoothing holds the most moves back
    [Test]
    public void everyVertexIsOnTheMeshAtSixtyFourCubed( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] bool simplified, [Values( 1.0f, 2.0f )] float edgeLength ) {
        Voxelizer voxelizer = algorithm == Algorithm.Uniform
            ? this.createVoxelizer<UniformDualContouring>( 6, QEFSolverType.SVD, shapesOf( Surface.TurnedShapes ) )
            : this.createVoxelizer<OccupancyDualContouring>( 6, QEFSolverType.SVD, shapesOf( Surface.TurnedShapes ) );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = simplified;
        voxelizer.remeshingEdgeLength  = edgeLength;

        var ( input, output, statistics ) = this.isotropicOf( voxelizer );
        var cellSize                       = ( double )voxelizer.cellSize;

        var built = new HandBuiltMesh( );

        built.positions.AddRange( input.vertices );
        built.triangles.AddRange( trianglesWithSubMeshesOf( input ) );

        using var features = new FeatureMeshRun( built, 40.0 );
        using var grid     = SurfaceGrid.of( features.mesh, 2.0 * cellSize, Unity.Collections.Allocator.TempJob );

        var furthest = output.vertices.Max( ( position ) => nearestOn( grid, position, cellSize, out _ ) ) / cellSize;

        Debug.Log( $"{algorithm} {( simplified ? "simplified" : "contour" )} at {edgeLength}: {output.vertexCount} vertices, the furthest {furthest:G3} cells from the mesh, {statistics.isotropic.Value.held} moves held, {statistics.isotropic.Value.refusedSplits} splits refused" );

        Assert.LessOrEqual( furthest, 1e-5, "the furthest vertex from the mesh, in cells" );
    }

    // a box turned about an axis or a diagonal, whose contour's edges cross the grid with the noise of findings 17 and
    // 19, keeps its twelve edges as lines and its eight corners when remeshed, with no other feature, read at the
    // scale the remesh finds its features at (step 3b): every algorithm, contoured and simplified, at 32^3 and 64^3,
    // turned about an axis; and turned about a diagonal by Manifold, and by Occupancy simplified at 64^3, which are
    // where it holds (Measurements). The others keep the points where their sheets touch, which are hard, and Occupancy's
    // contours strips that bridge the creases wider than half a cell. The lines are within a cell of the box's edges,
    // and the corners of its corners, which Occupancy's simplified box turned about a diagonal only holds to at 64^3:
    // at 32^3 two of its corners are 1.1 and 1.7 cells off. Simplified, the box turned about an axis is its twelve lines and eight corners
    // by each edge's own angle too, before and after, as at step 3
    public enum Turn {
        AboutAnAxis,
        AboutADiagonal
    }

    private static IEnumerable<TestCaseData> turnedBoxes( ) {
        foreach( var cellsExponent in new[] { 5, 6 } )
        foreach( var simplified in new[] { false, true } ) {
            foreach( var algorithm in Enum.GetValues( typeof( Algorithm ) ).Cast<Algorithm>( ) ) {
                yield return new TestCaseData( Turn.AboutAnAxis, algorithm, simplified, cellsExponent );
            }

            yield return new TestCaseData( Turn.AboutADiagonal, Algorithm.Manifold, simplified, cellsExponent );

            if( simplified && cellsExponent == 6 ) {
                yield return new TestCaseData( Turn.AboutADiagonal, Algorithm.Occupancy, simplified, cellsExponent );
            }
        }
    }

    [TestCaseSource( nameof( turnedBoxes ) )]
    public void aTurnedBoxKeepsItsTwelveLinesAndEightCorners( Turn turn, Algorithm algorithm, bool simplified, int cellsExponent ) {
        var centre   = new Vector3( 0.013f, -0.021f, 0.017f );
        var rotation = turn == Turn.AboutAnAxis ? Quaternion.AngleAxis( 45.0f, Vector3.up ) : Quaternion.AngleAxis( 45.0f, Vector3.one );
        var shapes   = new[] { cube( centre, 0.25f ).rotated( rotation ) };

        Voxelizer voxelizer = algorithm switch {
            Algorithm.Uniform   => this.createVoxelizer<UniformDualContouring>  ( cellsExponent,     QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive  => this.createVoxelizer<AdaptiveDualContouring> ( cellsExponent + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold  => this.createVoxelizer<ManifoldDualContouring> ( cellsExponent + 1, QEFSolverType.SVD, shapes ),
            Algorithm.Occupancy => this.createVoxelizer<OccupancyDualContouring>( cellsExponent,     QEFSolverType.SVD, shapes ),
            _                   => throw new ArgumentException( algorithm.ToString( ) )
        };

        if( voxelizer is AdaptiveDualContouring adaptive ) {
            adaptive.simplification = false;
        }
        if( voxelizer is ManifoldDualContouring manifold ) {
            manifold.errorThreshold  = 0.0f;
            manifold.enforceManifold = true;
        }

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = simplified;

        var ( input, output, statistics ) = this.isotropicOf( voxelizer );

        Assert.IsNull( statistics.gaveUp, "gave up" );

        // the box's corners, and its edges as their two ends
        var corners = Enumerable.Range( 0, 8 ).Select( ( k ) => ( double3 )( float3 )( centre + ( rotation * new Vector3( ( k & 1 ) != 0 ? 0.25f : -0.25f, ( k & 2 ) != 0 ? 0.25f : -0.25f, ( k & 4 ) != 0 ? 0.25f : -0.25f ) ) ) ).ToArray( );
        var edges   = Enumerable.Range( 0, 8 ).SelectMany( ( k ) => Enumerable.Range( 0, 3 ).Where( ( bit ) => ( k & ( 1 << bit ) ) == 0 ).Select( ( bit ) => ( corners[k], corners[k | ( 1 << bit )] ) ) ).ToArray( );
        var cell    = ( double )voxelizer.cellSize;

        double offTheEdges( double3 point ) {
            return edges.Min( ( edge ) => {
                var along = math.clamp( math.dot( point - edge.Item1, edge.Item2 - edge.Item1 ) / math.lengthsq( edge.Item2 - edge.Item1 ), 0.0, 1.0 );

                return math.distance( point, edge.Item1 + ( ( edge.Item2 - edge.Item1 ) * along ) );
            } );
        }

        // the scale the remesh found its features at
        var featureLength = math.min( statistics.edgeLength, 1.0 ) * cell;

        var readings = new List<( Mesh mesh, string which, double length )> { ( output, "remeshed", featureLength ) };

        if( turn == Turn.AboutAnAxis ) {
            readings.Add( ( input, "as given", featureLength ) );
        }
        if( turn == Turn.AboutAnAxis && simplified ) {
            readings.Add( ( input, "as given, by each edge's own angle", 0.0 ) );
            readings.Add( ( output, "remeshed, by each edge's own angle", 0.0 ) );
        }

        foreach( var ( mesh, which, length ) in readings ) {
            var built = new HandBuiltMesh( );

            built.positions.AddRange( mesh.vertices );
            built.triangles.AddRange( trianglesWithSubMeshesOf( mesh ) );

            using var features = new FeatureMeshRun( built, 40.0, length );

            var found = features.cornerVertices( );

            Assert.AreEqual( ( 12, 8 ), ( features.lineCount, found.Length ), $"lines and corners, {which}, of {mesh.vertexCount} vertices" );

            foreach( var corner in found ) {
                Assert.Less( corners.Min( ( c ) => math.distance( c, features.positionOf( corner ) ) ), cell, $"{which}: corner {corner} at {features.positionOf( corner )} is at a corner of the box" );
            }

            foreach( var edge in features.edges( ).Where( ( edge ) => features.mesh.edgeLines[edge.edge] != -1 ) ) {
                Assert.Less( math.max( offTheEdges( features.positionOf( edge.a ) ), offTheEdges( features.positionOf( edge.b ) ) ), cell, $"{which}: feature edge {edge.a}-{edge.b} is along an edge of the box" );
            }
        }
    }

    // a remesh finds the features by angle of a simplified model of flat faces as each edge's own angle has them: at
    // the scale of its edge length, but of no more than a cell (decision 12). The slab's sides are 6 cells wide at 32^3,
    // and the simplified slab is remeshed at edges of 12, at which they would be merged into its top and bottom
    [Test]
    public void aRemeshFindsTheFeaturesOfASimplifiedSlabAtNoMoreThanACell( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm ) {
        Voxelizer voxelizer = algorithm == Algorithm.Uniform
            ? this.createVoxelizer<UniformDualContouring>( 5, QEFSolverType.SVD, slabWithBand )
            : this.createVoxelizer<OccupancyDualContouring>( 5, QEFSolverType.SVD, slabWithBand );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = true;

        var ( input, _, statistics ) = this.isotropicOf( voxelizer );

        Assert.Greater( statistics.edgeLength, 8.0, "the edge length, in cells" );

        var kept   = Remeshing.of( input, voxelizer.cellSize );
        var length = kept.lengthOf( Remeshing.Technique.Isotropic, 1.0f );

        Assert.AreEqual( voxelizer.cellSize, kept.featureLengthOf( length ), 1e-6, "the length the features are found at" );
        Assert.AreEqual( 0.5 * voxelizer.cellSize, kept.featureLengthOf( 0.5 * voxelizer.cellSize ), 1e-9, "a length under a cell, as it is" );

        var built = new HandBuiltMesh( );

        built.positions.AddRange( input.vertices );
        built.triangles.AddRange( trianglesWithSubMeshesOf( input ) );

        using var each    = new FeatureMeshRun( built, 40.0 );
        using var atScale = new FeatureMeshRun( built, 40.0, kept.featureLengthOf( length ) );

        Assert.AreEqual( each.lineCount, atScale.lineCount, "lines" );
        Assert.AreEqual( each.featureEdgeCount, atScale.featureEdgeCount, "feature edges" );
        CollectionAssert.AreEqual( each.kinds, atScale.kinds, "what every vertex is" );
    }

    // a box along the grid shades flat on its faces when remeshed, contoured or simplified, with either kind of normals
    // out: creased at 40 degrees, every corner of every triangle has the triangle's own normal. Its contour's vertices on
    // its edges have normals between its faces, which a remesh that took them over as they are spread into the faces,
    // where the creasing kept them (finding 21)
    [Test]
    public void aRemeshedBoxShadesFlatOnItsFaces( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] bool simplified, [Values] Remeshing.Normals normals ) {
        var shapes = new[] { cube( new Vector3( 0.013f, -0.021f, 0.017f ), 0.28f ) };

        Voxelizer voxelizer = algorithm == Algorithm.Uniform
            ? this.createVoxelizer<UniformDualContouring>( 5, QEFSolverType.SVD, shapes )
            : this.createVoxelizer<OccupancyDualContouring>( 5, QEFSolverType.SVD, shapes );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = simplified;
        voxelizer.vertexNormals        = VertexNormals.Creased;
        voxelizer.creaseAngle          = 40.0f;
        voxelizer.smoothNormals        = SmoothNormals.Contour;
        voxelizer.remeshing            = Remeshing.Technique.Isotropic;
        voxelizer.remeshingNormals     = normals;

        voxelizer.contour( drawFromGPU: false );

        var mesh          = shown( voxelizer );
        var positions     = mesh.vertices;
        var vertexNormals = mesh.normals;
        var worst         = 0.0f;

        foreach( var triangle in trianglesWithSubMeshesOf( mesh ) ) {
            var face = Vector3.Cross( positions[triangle.b] - positions[triangle.a], positions[triangle.c] - positions[triangle.a] ).normalized;

            foreach( var corner in new[] { triangle.a, triangle.b, triangle.c } ) {
                worst = Mathf.Max( worst, Vector3.Angle( face, vertexNormals[corner] ) );
            }
        }

        Assert.LessOrEqual( worst, 1.0f, "the most a corner's normal is off its triangle's, in degrees" );
    }

    // at a crease angle of 180 the creasing changes nothing, so what the remesh gives is shown: with Input, the
    // contour's normals as the input shows them at 180, where the vertices lie, which on a box lean between its faces
    // and reach into them near its edges, where a vertex's triangles lie flat (finding 21); with Faces, the normal of each
    // vertex's own triangles, weighted by their areas
    [Test]
    public void theRemeshsNormalsAreTheInputsOrTheFaces( [Values] Remeshing.Normals normals ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( 5, QEFSolverType.SVD, new[] { cube( new Vector3( 0.013f, -0.021f, 0.017f ), 0.28f ) } );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.vertexNormals        = VertexNormals.Creased;
        voxelizer.creaseAngle          = 180.0f;
        voxelizer.remeshing            = Remeshing.Technique.Isotropic;
        voxelizer.remeshingNormals     = normals;

        voxelizer.contour( drawFromGPU: false );

        var mesh      = shown( voxelizer );
        var positions = mesh.vertices;
        var sums      = new double3[positions.Length];
        var faces     = new List<double3>[positions.Length];

        foreach( var triangle in trianglesWithSubMeshesOf( mesh ) ) {
            var a = ( double3 )( float3 )positions[triangle.a];
            var n = math.cross( ( double3 )( float3 )positions[triangle.b] - a, ( double3 )( float3 )positions[triangle.c] - a );

            foreach( var corner in new[] { triangle.a, triangle.b, triangle.c } ) {
                sums[corner] += n;
                ( faces[corner] ??= new List<double3>( ) ).Add( math.normalize( n ) );
            }
        }

        var offOwn = Enumerable.Range( 0, positions.Length ).Select( ( vertex ) => Vector3.Angle( mesh.normals[vertex], ( Vector3 )( float3 )math.normalize( sums[vertex] ) ) ).ToArray( );
        var flat   = Enumerable.Range( 0, positions.Length ).Where( ( vertex ) => faces[vertex].All( ( face ) => math.dot( face, math.normalize( sums[vertex] ) ) > math.cos( math.radians( 1.0 ) ) ) ).ToArray( );

        if( normals == Remeshing.Normals.Faces ) {
            Assert.LessOrEqual( offOwn.Max( ), 0.01f, "the most a vertex's normal is off its own triangles', in degrees" );
        }
        else {
            Assert.Greater( flat.Max( ( vertex ) => offOwn[vertex] ), 5.0f, "the most the normal of a vertex whose triangles lie flat is off them, in degrees" );
        }
    }

    // a voxelizer that splits its vertices asks the remesh for the normals of the faces, which cost nothing: splitting
    // gives every vertex its triangle's normal whatever the remesh gave it
    [Test]
    public void splitVerticesAskTheRemeshForTheNormalsOfTheFaces( ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( resolution, QEFSolverType.SVD, shapesOf( Model.Sphere ) );

        voxelizer.remeshingNormals = Remeshing.Normals.Input;
        voxelizer.vertexNormals    = VertexNormals.Creased;

        Assert.AreEqual( Remeshing.Normals.Input, voxelizer.remeshingSettings.normals, "creased" );

        voxelizer.vertexNormals = VertexNormals.Split;

        Assert.AreEqual( Remeshing.Normals.Faces, voxelizer.remeshingSettings.normals, "split" );
    }

    // shorter edges give more vertices, and longer ones fewer, by the square of the length (decision 9)
    [Test]
    public void theEdgeLengthSetsHowManyVerticesThereAre( [ValueSource( nameof( cpuVariants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ), VertexNormals.Creased, Remeshing.Technique.Off );
        var counts    = new Dictionary<float, int>( );

        voxelizer.creaseAngle = 180.0f;
        voxelizer.remeshing   = Remeshing.Technique.Isotropic;

        voxelizer.contour( drawFromGPU: false );

        foreach( var length in new[] { 0.5f, 1.0f, 2.0f } ) {
            voxelizer.remeshingEdgeLength = length;

            counts[length] = voxelizer.remesh( ).Value.vertexCount.Value;
        }

        Debug.Log( $"{variant}: {string.Join( ", ", counts.Select( ( pair ) => $"{pair.Value} at {pair.Key}" ) )}" );

        Assert.That( counts[0.5f] / ( double )counts[1.0f], Is.InRange( 3.5, 4.5 ),   "at half the length" );
        Assert.That( counts[2.0f] / ( double )counts[1.0f], Is.InRange( 0.2, 0.33 ),  "at twice the length" );
    }

    // remesh after a change of any of Isotropic's settings shows what a fresh contour under them shows
    [Test]
    public void remeshShowsWhatAFreshIsotropicContourShows( [ValueSource( nameof( cpuVariants ) )] Variant variant ) {
        var voxelizer = this.create( variant, shapesOf( Model.CubesAndSphere ), VertexNormals.Creased, Remeshing.Technique.Isotropic );

        voxelizer.contour( drawFromGPU: false );

        removeVolumes( voxelizer );

        foreach( var ( what, change ) in new ( string, Action )[] {
            ( "at first",                ( ) => { } ),
            ( "at another feature angle", ( ) => voxelizer.remeshingFeatureAngle = 60.0f ),
            ( "with fewer iterations",    ( ) => voxelizer.remeshingIterations = 2 ),
            ( "with more area rounds",    ( ) => voxelizer.remeshingAreaIterations = 8 ),
            ( "at another edge length",   ( ) => voxelizer.remeshingEdgeLength = 1.5f )
        } ) {
            change( );

            var statistics = voxelizer.remesh( ).Value;
            var fresh      = this.create( variant, shapesOf( Model.CubesAndSphere ), VertexNormals.Creased, Remeshing.Technique.Isotropic );

            fresh.remeshingEdgeLength     = voxelizer.remeshingEdgeLength;
            fresh.remeshingFeatureAngle   = voxelizer.remeshingFeatureAngle;
            fresh.remeshingIterations     = voxelizer.remeshingIterations;
            fresh.remeshingAreaIterations = voxelizer.remeshingAreaIterations;

            var expected = fresh.contour( drawFromGPU: false );

            MeshComparison.assertSameTriangles( shown( fresh ), shown( voxelizer ) );

            Assert.AreEqual( expected.vertexCount, statistics.vertexCount, $"{what}: vertices reported" );
            Assert.IsNull( voxelizer.shownEdgeFlags, $"{what}: a mesh of triangles draws every edge" );
        }
    }

    // the feature angle, the iterations and the area rounds each change what the remesh makes, through the voxelizer
    [Test]
    public void eachOfIsotropicsSettingsChangesTheRemesh( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.CubesAndSphere ), VertexNormals.Creased, Remeshing.Technique.Isotropic );

        voxelizer.contour( drawFromGPU: false );

        var defaults = MeshComparison.trianglesOf( shown( voxelizer ) );

        foreach( var ( what, change, back ) in new ( string, Action, Action )[] {
            ( "a feature angle above the cubes' right angles", ( ) => voxelizer.remeshingFeatureAngle = 100.0f, ( ) => voxelizer.remeshingFeatureAngle = 40.0f ),
            ( "two iterations",                                ( ) => voxelizer.remeshingIterations = 2,        ( ) => voxelizer.remeshingIterations = 5 ),
            ( "no area rounds",                                ( ) => voxelizer.remeshingAreaIterations = 0,    ( ) => voxelizer.remeshingAreaIterations = 5 ),
            ( "the normals of the faces",                      ( ) => voxelizer.remeshingNormals = Remeshing.Normals.Faces, ( ) => voxelizer.remeshingNormals = Remeshing.Normals.Input )
        } ) {
            change( );
            voxelizer.remesh( );

            CollectionAssert.AreNotEqual( defaults, MeshComparison.trianglesOf( shown( voxelizer ) ), $"with {what}" );

            back( );
            voxelizer.remesh( );

            CollectionAssert.AreEqual( defaults, MeshComparison.trianglesOf( shown( voxelizer ) ), $"back from {what}" );
        }
    }

    // ---- Field Aligned ----

    private static double areaOf( Mesh mesh, int subMesh ) {
        var vertices = mesh.vertices;
        var indices  = mesh.GetTriangles( subMesh );
        var area     = 0.0;

        for( var index = 0; index < indices.Length; index += 3 ) {
            area += 0.5 * Vector3.Cross( vertices[indices[index + 1]] - vertices[indices[index]], vertices[indices[index + 2]] - vertices[indices[index]] ).magnitude;
        }

        return area;
    }

    // what Field Aligned keeps of the mesh it is given, for every algorithm and implementation, contoured and
    // simplified, as quads with triangles and pentagons among them: every vertex on that mesh, the area of every
    // material to within a tenth or so, about as many vertices, mostly quads, and the same remesh every time. Its
    // topology is counted and not held to the input's (finding 7)
    [Test]
    public void theFieldAlignedRemeshKeepsTheSurfaceAndItsAreas( [ValueSource( nameof( variants ) )] Variant variant, [Values] Surface surface, [Values] bool simplified ) {
        var voxelizer = this.create( variant, shapesOf( surface ), VertexNormals.Creased, Remeshing.Technique.Off );

        voxelizer.meshSimplification = simplified;

        var ( input, output, statistics ) = this.remeshedOf( voxelizer, Remeshing.Technique.FieldAligned );
        var cellSize                       = ( double )voxelizer.cellSize;

        Debug.Log( $"{variant} {surface} {( simplified ? "simplified" : "contour" )}: {output.vertexCount} vertices from {input.vertexCount}, {statistics.quads} quads, {statistics.triangles} triangles, {statistics.largerFaces} larger, {statistics.newOpenLines} new open lines, {statistics.newNonManifoldLines} new lines of more than two triangles" );

        Assert.IsNull( statistics.gaveUp, "gave up" );
        Assert.AreEqual( 0, statistics.fieldAligned.Value.offInput, "vertices without a point of the mesh within reach" );
        Assert.Greater( statistics.quads, statistics.triangles + statistics.largerFaces, "mostly quads" );

        // on the mesh it was given
        var built = new HandBuiltMesh( );

        built.positions.AddRange( input.vertices );
        built.triangles.AddRange( trianglesWithSubMeshesOf( input ) );

        using var features = new FeatureMeshRun( built, 40.0 );
        using var grid     = SurfaceGrid.of( features.mesh, 2.0 * cellSize, Unity.Collections.Allocator.TempJob );

        foreach( var position in output.vertices ) {
            Assert.LessOrEqual( nearestOn( grid, position, cellSize, out _ ) / cellSize, 1e-5, $"the vertex at {position} is on the mesh, in cells" );
        }

        // every material about as large. A face takes the material most of its area has, so a line between two moves by
        // up to half an edge, which on a small simplified mesh of long edges is much of a small material's area:
        // measured 0.92 to 1.01 of the input's on contours, and down to 0.86 on the turned shapes simplified
        Assert.AreEqual( input.subMeshCount, output.subMeshCount, "sub-meshes" );

        for( var subMesh = 0; subMesh < input.subMeshCount; ++subMesh ) {
            Assert.That( areaOf( output, subMesh ) / areaOf( input, subMesh ), Is.InRange( simplified ? 0.8 : 0.9, 1.05 ), $"the area of sub-mesh {subMesh} against the input's" );
        }

        // about as many vertices, on meshes of 300 vertices and more: measured 0.96 to 1.07
        if( input.vertexCount >= 300 ) {
            Assert.That( output.vertexCount / ( double )input.vertexCount, Is.InRange( 0.9, 1.15 ), $"vertices, against the {input.vertexCount} of the mesh given" );
        }

        // and the same again
        var first = MeshComparison.trianglesOf( output );
        var flags = voxelizer.shownEdgeFlags.ToArray( );

        voxelizer.remesh( );

        CollectionAssert.AreEqual( first, MeshComparison.trianglesOf( shown( voxelizer ) ), "the second remesh" );
        CollectionAssert.AreEqual( flags, voxelizer.shownEdgeFlags, "the second remesh's flags" );
    }

    // Field Aligned's triangles and its quads alone, which are split from faces of twice the length, are on the mesh too
    [Test]
    public void everyVertexOfEachKindOfFaceIsOnTheMesh( [Values( Algorithm.Uniform, Algorithm.Occupancy )] Algorithm algorithm, [Values] bool simplified, [Values( BurstFieldAligned.Faces.Triangles, BurstFieldAligned.Faces.Quads )] BurstFieldAligned.Faces faces ) {
        var voxelizer = this.create( new( algorithm, ImplementationType.Burst ), shapesOf( Model.CubesAndSphere ), VertexNormals.Creased, Remeshing.Technique.Off );

        voxelizer.meshSimplification = simplified;
        voxelizer.remeshingFaces     = faces;

        var ( input, output, statistics ) = this.remeshedOf( voxelizer, Remeshing.Technique.FieldAligned );
        var cellSize                       = ( double )voxelizer.cellSize;

        Assert.IsNull( statistics.gaveUp, "gave up" );

        var built = new HandBuiltMesh( );

        built.positions.AddRange( input.vertices );
        built.triangles.AddRange( trianglesWithSubMeshesOf( input ) );

        using var features = new FeatureMeshRun( built, 40.0 );
        using var grid     = SurfaceGrid.of( features.mesh, 2.0 * cellSize, Unity.Collections.Allocator.TempJob );

        foreach( var position in output.vertices ) {
            Assert.LessOrEqual( nearestOn( grid, position, cellSize, out _ ) / cellSize, 1e-5, $"the vertex at {position} is on the mesh, in cells" );
        }
    }

    // the edge length sets how many vertices Field Aligned makes, in each of its faces: a fourth as many at twice the
    // length and four times at half
    [Test]
    public void theEdgeLengthSetsHowManyVerticesFieldAlignedMakes( [ValueSource( nameof( cpuVariants ) )] Variant variant, [Values] BurstFieldAligned.Faces faces ) {
        var voxelizer = this.create( variant, shapesOf( Model.ThreeBalls ), VertexNormals.Creased, Remeshing.Technique.Off );
        var counts    = new Dictionary<float, int>( );

        voxelizer.creaseAngle    = 180.0f;
        voxelizer.remeshing      = Remeshing.Technique.FieldAligned;
        voxelizer.remeshingFaces = faces;

        voxelizer.contour( drawFromGPU: false );

        foreach( var length in new[] { 0.5f, 1.0f, 2.0f } ) {
            voxelizer.remeshingEdgeLength = length;

            var statistics = voxelizer.remesh( ).Value;

            Assert.IsNull( statistics.remeshing.Value.gaveUp, $"gave up at {length}" );

            counts[length] = statistics.vertexCount.Value;
        }

        Debug.Log( $"{variant} {faces}: {string.Join( ", ", counts.Select( ( pair ) => $"{pair.Value} at {pair.Key}" ) )}" );

        Assert.That( counts[0.5f] / ( double )counts[1.0f], Is.InRange( 3.5, 4.5 ),  "at half the length" );
        Assert.That( counts[2.0f] / ( double )counts[1.0f], Is.InRange( 0.2, 0.33 ), "at twice the length" );
    }

    // each of Field Aligned's settings changes what it makes, and those it does not take, Isotropic's, change nothing
    [Test]
    public void eachOfFieldAlignedsSettingsChangesTheRemesh( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.CubesAndSphere ), VertexNormals.Creased, Remeshing.Technique.FieldAligned );

        voxelizer.contour( drawFromGPU: false );

        var defaults = MeshComparison.trianglesOf( shown( voxelizer ) );
        var flags    = voxelizer.shownEdgeFlags.ToArray( );

        foreach( var ( what, change, back ) in new ( string, Action, Action )[] {
            ( "triangles",                                     ( ) => voxelizer.remeshingFaces = BurstFieldAligned.Faces.Triangles, ( ) => voxelizer.remeshingFaces = BurstFieldAligned.Faces.QuadDominant ),
            ( "quads only",                                    ( ) => voxelizer.remeshingFaces = BurstFieldAligned.Faces.Quads,     ( ) => voxelizer.remeshingFaces = BurstFieldAligned.Faces.QuadDominant ),
            ( "a feature angle above the cubes' right angles", ( ) => voxelizer.remeshingFeatureAngle = 100.0f, ( ) => voxelizer.remeshingFeatureAngle = 40.0f ),
            ( "the normals of the faces",                      ( ) => voxelizer.remeshingNormals = Remeshing.Normals.Faces, ( ) => voxelizer.remeshingNormals = Remeshing.Normals.Input )
        } ) {
            change( );
            voxelizer.remesh( );

            CollectionAssert.AreNotEqual( defaults, MeshComparison.trianglesOf( shown( voxelizer ) ), $"with {what}" );

            back( );
            voxelizer.remesh( );

            CollectionAssert.AreEqual( defaults, MeshComparison.trianglesOf( shown( voxelizer ) ), $"back from {what}" );
        }

        // Isotropic's
        voxelizer.remeshingIterations     = 2;
        voxelizer.remeshingAreaIterations = 0;
        voxelizer.remesh( );

        CollectionAssert.AreEqual( defaults, MeshComparison.trianglesOf( shown( voxelizer ) ), "with Isotropic's settings changed" );
        CollectionAssert.AreEqual( flags, voxelizer.shownEdgeFlags, "the flags with Isotropic's settings changed" );
    }

    // a length that would make more vertices than mostVertices gives up, and the mesh is shown as it came
    [Test]
    public void anEdgeLengthOfTooManyVerticesGivesUp( ) {
        var voxelizer = this.create( new( Algorithm.Uniform, ImplementationType.Burst ), shapesOf( Model.ThreeBalls ), VertexNormals.Creased, Remeshing.Technique.Off );

        voxelizer.creaseAngle = 180.0f;
        voxelizer.contour( drawFromGPU: false );

        var plain = MeshComparison.trianglesOf( shown( voxelizer ) );

        voxelizer.remeshing           = Remeshing.Technique.Isotropic;
        voxelizer.remeshingEdgeLength = 0.01f;

        var statistics = voxelizer.remesh( ).Value.remeshing.Value;

        StringAssert.Contains( "would make about", statistics.gaveUp, "why it gave up" );
        Assert.IsNull( statistics.isotropic, "what Isotropic did" );

        CollectionAssert.AreEqual( plain, MeshComparison.trianglesOf( shown( voxelizer ) ), "the mesh as it came" );
    }

}

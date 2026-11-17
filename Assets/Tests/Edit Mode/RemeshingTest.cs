using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using ContourVertices    = SurfaceExtractor.ContourVertices;
using QEFSolverType      = QEFSolver.Type;

// the frame the remeshing techniques go into (Documents/Plans/Remeshing.md, step 1), through Voxelizer.contour,
// simplify and remesh, with the stand-in technique: which meshes are kept for remeshing, that a remesh is what a fresh
// contour under the same settings shows, and that the flags of the faces follow the mesh through creasing, splitting
// and reordering, for every algorithm and implementation
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

    private Voxelizer create( Variant variant, IEnumerable<Shape> shapes, VertexNormals normals = VertexNormals.Creased, Remeshing.Technique technique = Remeshing.Technique.FlatPairs ) {
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

    // the flags make faces of the mesh, in the order of its index buffer: every edge a triangle leaves out is left out
    // by the one other triangle on it too, of the same sub-mesh, and the two lie flat together, as the stand-in pairs
    // them. A creased or split mesh has several vertices at a position, so edges are found by the positions of their
    // ends. Gives the number of pairs
    private static int pairsOf( Mesh mesh, IReadOnlyList<byte> flags ) {
        var vertices  = mesh.vertices;
        var triangles = new List<( int a, int b, int c, int subMesh )>( );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices = mesh.GetTriangles( subMesh );

            for( var index = 0; index < indices.Length; index += 3 ) {
                triangles.Add( ( indices[index], indices[index + 1], indices[index + 2], subMesh ) );
            }
        }

        Assert.AreEqual( triangles.Count, flags.Count, "a flag for every triangle" );

        var leftOut = new Dictionary<( Vector3, Vector3 ), List<int>>( );

        for( var triangle = 0; triangle < triangles.Count; ++triangle ) {
            var corners = new[] { triangles[triangle].a, triangles[triangle].b, triangles[triangle].c };
            var count   = 0;

            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    continue;
                }

                ++count;

                var a   = vertices[corners[k]];
                var b   = vertices[corners[( k + 1 ) % 3]];
                var key = a.x < b.x || ( a.x == b.x && ( a.y < b.y || ( a.y == b.y && a.z < b.z ) ) ) ? ( a, b ) : ( b, a );

                if( !leftOut.TryGetValue( key, out var list ) ) {
                    leftOut.Add( key, list = new List<int>( ) );
                }
                list.Add( triangle );
            }

            Assert.LessOrEqual( count, 1, $"edges triangle {triangle} leaves out" );
        }

        Vector3 normalOf( int triangle ) {
            var t = triangles[triangle];
            return Vector3.Cross( vertices[t.b] - vertices[t.a], vertices[t.c] - vertices[t.a] ).normalized;
        }

        foreach( var entry in leftOut ) {
            Assert.AreEqual( 2, entry.Value.Count, $"triangles that leave out the edge {entry.Key}" );

            var first  = entry.Value[0];
            var second = entry.Value[1];

            Assert.AreEqual( triangles[first].subMesh, triangles[second].subMesh, $"sub-meshes of the triangles across {entry.Key}" );
            Assert.Less( Vector3.Angle( normalOf( first ), normalOf( second ) ), 1.01f, $"angle between the triangles across {entry.Key}" );
        }

        return leftOut.Count;
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

    // ---- the stand-in through the frame ----

    // the stand-in moves nothing, so the mesh shown is the contour as remeshing off shows it, with flags that pair its
    // flat triangles. The flags are those of the mesh as it is shown: creased or split, its vertices reordered or not,
    // of one material or two. A GPU contour is read back to be remeshed
    [Test]
    public void theStandInPairsTheFlatTrianglesOfTheContour(
        [ValueSource( nameof( variants ) )] Variant variant, [Values] Surface surface, [Values] VertexNormals normals, [Values] bool reorderVertices
    ) {
        var voxelizer  = this.create( variant, shapesOf( surface ), normals );
        var statistics = voxelizer.contour( drawFromGPU: true, reorderVertices: reorderVertices );
        var mesh       = shown( voxelizer );

        Assert.IsNotNull( mesh, "a mesh, read back from the GPU too" );
        Assert.IsTrue( statistics.remeshing.HasValue, "what the remesh did" );

        var remeshing = statistics.remeshing.Value;
        var pairs     = pairsOf( mesh, voxelizer.shownEdgeFlags );

        Assert.IsNull( remeshing.gaveUp, "gave up" );
        Assert.Greater( pairs, triangleCountOf( mesh ) / 8, "pairs, on models with flat faces" );

        Assert.AreEqual( pairs,                                    remeshing.quads,              "quads" );
        Assert.AreEqual( triangleCountOf( mesh ) - ( 2 * pairs ), remeshing.triangles,          "triangles" );
        Assert.AreEqual( 0,                                        remeshing.largerFaces,        "larger faces" );
        Assert.AreEqual( 0,                                        remeshing.newOpenEdges,       "new open edges" );
        Assert.AreEqual( 0,                                        remeshing.newNonManifoldEdges, "new edges of more than two triangles" );
        Assert.AreEqual( triangleCountOf( mesh ),                  remeshing.inputTriangleCount, "triangles given" );

        Assert.AreEqual( Remeshing.Technique.FlatPairs, remeshing.technique );

        // the GPU hands its vertices over in a different order every time, so only the CPU's mesh is the same as the one
        // remeshing off shows, and only its contour can be had again to work out the length from
        if( variant.implementation == ImplementationType.GPU ) {
            Assert.Greater( remeshing.edgeLength, 0.5, "edge length in cells, of a contour of about a cell" );
            Assert.Less( remeshing.edgeLength, 2.0,    "edge length in cells, of a contour of about a cell" );
        }
        else {
            // the stand-in makes quads, which have a vertex for every l^2 of the area: at 1 as many as the contour has
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

            var plain = this.create( variant, shapesOf( surface ), normals, Remeshing.Technique.Off );

            plain.contour( drawFromGPU: false, reorderVertices: reorderVertices );

            MeshComparison.assertSameTriangles( shown( plain ), mesh );
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

        voxelizer.remeshing = Remeshing.Technique.FlatPairs;

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

        voxelizer.remeshing = Remeshing.Technique.FlatPairs;

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
            Assert.AreEqual( pairsOf( shown( voxelizer ), voxelizer.shownEdgeFlags ), statistics.remeshing.Value.quads, "quads" );

            voxelizer.simplificationDetail = 0.5f;

            var again = voxelizer.simplify( );

            Assert.IsTrue( again.HasValue, "simplified again" );
            Assert.AreEqual( triangleCountOf( halfway ), again.Value.remeshing.Value.inputTriangleCount, "triangles given, at a detail of 0.5" );
            Assert.AreEqual( pairsOf( shown( voxelizer ), voxelizer.shownEdgeFlags ), again.Value.remeshing.Value.quads, "quads at a detail of 0.5" );

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
    public void aRemeshedContourOfManyTrianglesIsSplitWithThirtyTwoBitIndices( ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( 6, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ) );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.vertexNormals        = VertexNormals.Split;
        voxelizer.keepDebugInformation = false;
        voxelizer.remeshing            = Remeshing.Technique.FlatPairs;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var mesh       = shown( voxelizer );

        Assert.Greater( statistics.triangleCount.Value, 21845, "triangles" );
        Assert.AreEqual( UnityEngine.Rendering.IndexFormat.UInt32, mesh.indexFormat, "index format" );
        Assert.AreEqual( statistics.remeshing.Value.quads, pairsOf( mesh, voxelizer.shownEdgeFlags ), "quads" );
    }

}

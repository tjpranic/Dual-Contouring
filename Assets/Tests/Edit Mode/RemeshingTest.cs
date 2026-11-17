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
        Assert.AreEqual( 0,                                        remeshing.newOpenLines,       "new open lines" );
        Assert.AreEqual( 0,                                        remeshing.newNonManifoldLines, "new lines of edges of more than two triangles" );
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
    public void aRemeshedContourOfManyTrianglesIsSplitWithThirtyTwoBitIndices( [Values( Remeshing.Technique.FlatPairs, Remeshing.Technique.Isotropic )] Remeshing.Technique technique ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( 6, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ) );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.vertexNormals        = VertexNormals.Split;
        voxelizer.keepDebugInformation = false;
        voxelizer.remeshing            = technique;

        var statistics = voxelizer.contour( drawFromGPU: false );
        var mesh       = shown( voxelizer );

        Assert.Greater( statistics.triangleCount.Value, 21845, "triangles" );
        Assert.AreEqual( UnityEngine.Rendering.IndexFormat.UInt32, mesh.indexFormat, "index format" );
        Assert.AreEqual( statistics.remeshing.Value.quads, voxelizer.shownEdgeFlags == null ? 0 : pairsOf( mesh, voxelizer.shownEdgeFlags ), "quads" );
        Assert.AreEqual( 3 * statistics.triangleCount.Value, mesh.vertexCount, "three vertices for every triangle" );
    }

    // ---- Isotropic ----

    // the kept mesh as it came, and remeshed by Isotropic, shown with shared vertices: creased at 180 degrees, every
    // triangle around a vertex is one group. A GPU contour is only kept while remeshing is on, so it contours remeshed
    private ( Mesh input, Mesh output, Remeshing.Statistics statistics ) isotropicOf( Voxelizer voxelizer ) {
        voxelizer.vertexNormals = VertexNormals.Creased;
        voxelizer.creaseAngle   = 180.0f;
        voxelizer.remeshing     = Remeshing.Technique.Isotropic;

        voxelizer.contour( drawFromGPU: false );

        voxelizer.remeshing = Remeshing.Technique.Off;
        voxelizer.remesh( );

        var input = Object.Instantiate( shown( voxelizer ) );

        this.made.Add( input );

        voxelizer.remeshing = Remeshing.Technique.Isotropic;

        var statistics = voxelizer.remesh( ).Value.remeshing.Value;

        return ( input, shown( voxelizer ), statistics );
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

    // a box turned about an axis, simplified down to its eight corners, keeps its twelve edges as lines and its eight
    // corners, by every algorithm, as the plan measured it at 32^3 and 64^3 (step 3). Contoured, and turned about a
    // diagonal, its edges keep the steps of the grid, whose lines come and go (finding 19)
    [Test]
    public void aSimplifiedBoxTurnedAboutAnAxisKeepsItsTwelveLinesAndEightCorners( [Values] Algorithm algorithm ) {
        var shapes = new[] { cube( new Vector3( 0.013f, -0.021f, 0.017f ), 0.25f ).rotated( Quaternion.AngleAxis( 45.0f, Vector3.up ) ) };

        Voxelizer voxelizer = algorithm switch {
            Algorithm.Uniform   => this.createVoxelizer<UniformDualContouring>  ( 5, QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive  => this.createVoxelizer<AdaptiveDualContouring> ( 6, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold  => this.createVoxelizer<ManifoldDualContouring> ( 6, QEFSolverType.SVD, shapes ),
            Algorithm.Occupancy => this.createVoxelizer<OccupancyDualContouring>( 5, QEFSolverType.SVD, shapes ),
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
        voxelizer.meshSimplification   = true;

        var ( input, output, statistics ) = this.isotropicOf( voxelizer );

        Assert.IsNull( statistics.gaveUp, "gave up" );

        foreach( var ( mesh, which ) in new[] { ( input, "simplified" ), ( output, "remeshed" ) } ) {
            var built = new HandBuiltMesh( );

            built.positions.AddRange( mesh.vertices );
            built.triangles.AddRange( trianglesWithSubMeshesOf( mesh ) );

            using var features = new FeatureMeshRun( built, 40.0 );

            Assert.AreEqual( ( 12, 8 ), ( features.lineCount, features.kinds.Count( ( kind ) => kind == BurstIsotropic.Kind.Corner ) ), $"lines and corners, {which}, of {mesh.vertexCount} vertices" );
        }
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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

using Object        = UnityEngine.Object;
using SmoothNormals = SurfaceExtractor.SmoothNormals;
using Builder       = CreaseShapes.Builder;

// the GPU's crease (GPUCreases, CreaseNormals.compute) against BurstCreases, on the meshes built by hand that
// BurstCreasesTest holds to answers known in closed form, run through the kernels from buffers of the test's own, and on
// the contours of both GPU implementations. The groups must be the same. Metal has no doubles, so the normals are held
// to Burst's within a tolerance
public class GPUCreasesTest : ContourVariantsTest {

    // how far a normal of the GPU may be from Burst's, in degrees. The farthest on the contours was 0.001
    private const double tolerance = 0.01;

    // the meshes a test made beyond the contours, which ContouringTest destroys
    private readonly List<Object> made = new( );

    [TearDown]
    public void destroyMade( ) {
        foreach( var item in this.made ) {
            Object.DestroyImmediate( item );
        }
        this.made.Clear( );
    }

    // what a crease gave: its vertices with their normals, and its triangles in the order they came in
    private struct Result {

        public float3[]             positions;
        public float3[]             normals;
        public BurstMesh.Triangle[] triangles;

    }

    private static Result burstCrease( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, float angle, SmoothNormals smoothNormals ) {
        using var positionsIn = new NativeArray<float3>( positions, Allocator.TempJob );
        using var normalsIn   = new NativeArray<float3>( normals, Allocator.TempJob );
        using var trianglesIn = new NativeArray<BurstMesh.Triangle>( triangles, Allocator.TempJob );

        BurstCreases.crease( positionsIn, normalsIn, trianglesIn, angle, smoothNormals, Allocator.TempJob, out var creasedPositions, out var creasedNormals, out var creasedTriangles, out _ );

        try {
            return new Result { positions = creasedPositions.ToArray( ), normals = creasedNormals.ToArray( ), triangles = creasedTriangles.ToArray( ) };
        }
        finally {
            creasedPositions.Dispose( );
            creasedNormals.Dispose( );
            creasedTriangles.Dispose( );
        }
    }

    // a mesh's triangles two to a quad, as the GPU implementations hold them, through the GPU's crease. The quads come
    // back with their indices turned to the creased vertices, the triangles in the order they went in
    private static Result gpuCrease( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, float angle, SmoothNormals smoothNormals ) {
        Assert.AreEqual( 0, triangles.Length % 2, "triangles, two to a quad" );

        var quadCount = triangles.Length / 2;
        var quadsData = new UniformDualContouring.Quad.Data[quadCount];

        for( var quad = 0; quad < quadCount; ++quad ) {
            var first  = triangles[2 * quad];
            var second = triangles[( 2 * quad ) + 1];

            Assert.AreEqual( first.subMeshIndex, second.subMeshIndex, $"sub-meshes of the triangles of quad {quad}" );

            quadsData[quad] = new UniformDualContouring.Quad.Data( new[] { first.a, first.b, first.c, second.a, second.b, second.c }, first.subMeshIndex );
        }

        var vertices = new ComputeBuffer( Math.Max( 1, positions.Length ), Marshal.SizeOf( typeof( Vector3 ) ) );
        var normalsIn = new ComputeBuffer( Math.Max( 1, positions.Length ), Marshal.SizeOf( typeof( Vector3 ) ) );
        var quads    = new ComputeBuffer( Math.Max( 1, quadCount ), Marshal.SizeOf( typeof( UniformDualContouring.Quad.Data ) ) );
        var counts   = new ComputeBuffer( 2, sizeof( int ) );
        var creases  = new GPUCreases( );

        try {
            vertices.SetData( positions );
            normalsIn.SetData( normals );
            quads.SetData( quadsData );
            counts.SetData( new[] { positions.Length, quadCount } );

            creases.crease( vertices, normalsIn, quads, counts, positions.Length, quadCount, angle, smoothNormals, out var creasedVertices, out var creasedNormals, out var creasedCounts );

            var countsData = new int[2];

            creasedCounts.GetData( countsData );

            Assert.AreEqual( quadCount, countsData[1], "quads of the creased contour" );

            var result = new Result {
                positions = new float3[countsData[0]],
                normals   = new float3[countsData[0]],
                triangles = new BurstMesh.Triangle[triangles.Length]
            };

            creasedVertices.GetData( result.positions, 0, 0, countsData[0] );
            creasedNormals.GetData( result.normals, 0, 0, countsData[0] );
            quads.GetData( quadsData );

            for( var quad = 0; quad < quadCount; ++quad ) {
                var indices = new UniformDualContouring.Quad( quadsData[quad] ).indices;

                result.triangles[2 * quad]         = new BurstMesh.Triangle( indices[0], indices[1], indices[2], quadsData[quad].subMeshIndex );
                result.triangles[( 2 * quad ) + 1] = new BurstMesh.Triangle( indices[3], indices[4], indices[5], quadsData[quad].subMeshIndex );
            }

            return result;
        }
        finally {
            creases.release( );
            vertices.Release( );
            normalsIn.Release( );
            quads.Release( );
            counts.Release( );
        }
    }

    private static int indexOf( BurstMesh.Triangle triangle, int corner ) {
        return corner switch { 0 => triangle.a, 1 => triangle.b, _ => triangle.c };
    }

    private static double degreesBetween( float3 a, float3 b ) {
        var x = ( double3 )a;
        var y = ( double3 )b;

        return math.degrees( math.atan2( math.length( math.cross( x, y ) ), math.dot( x, y ) ) );
    }

    // the same crease: as many vertices, every corner at the same position with a normal within the tolerance, and the
    // same corners sharing a vertex. Corners are named by a key that is the same in both
    private static void assertSameCrease( IEnumerable<( string corner, int burst, int gpu )> corners, float3[] burstPositions, float3[] burstNormals, float3[] gpuPositions, float3[] gpuNormals, string what ) {
        Assert.AreEqual( burstPositions.Length, gpuPositions.Length, $"{what}: creased vertices" );

        var gpuOfBurst = new Dictionary<int, int>( );
        var burstOfGPU = new Dictionary<int, int>( );
        var count      = 0;

        foreach( var ( corner, burst, gpu ) in corners ) {
            ++count;

            if( !burstPositions[burst].Equals( gpuPositions[gpu] ) ) {
                Assert.Fail( $"{what}: corner {corner} is at {gpuPositions[gpu]} on the GPU and at {burstPositions[burst]} in Burst" );
            }

            var between = degreesBetween( burstNormals[burst], gpuNormals[gpu] );

            if( !( between <= tolerance ) ) {
                Assert.Fail( $"{what}: the normal of corner {corner} is {between} degrees from Burst's, {gpuNormals[gpu]} against {burstNormals[burst]}" );
            }

            // a corner shares its vertex with the same corners on both
            if( gpuOfBurst.TryGetValue( burst, out var gpuBefore ) && gpuBefore != gpu ) {
                Assert.Fail( $"{what}: corner {corner} shares Burst's vertex {burst} with corners that have another vertex on the GPU" );
            }
            if( burstOfGPU.TryGetValue( gpu, out var burstBefore ) && burstBefore != burst ) {
                Assert.Fail( $"{what}: corner {corner} shares the GPU's vertex {gpu} with corners that have another vertex in Burst" );
            }

            gpuOfBurst[burst] = gpu;
            burstOfGPU[gpu]   = burst;
        }

        Assert.Greater( count, 0, $"{what}: corners" );
    }

    // ---- meshes built by hand ----

    public enum HandBuilt {
        Cube,
        Prism3,
        Prism4,
        Prism8,
        Prism10,
        Prism24,
        FlatPatch,
        Sphere,
        ConeTip,
        TurningTriangles,
        CornersOfTheSameSize,
        CornersOfTheSameSizeReversed,
        PatchWithOddNormals,
        CubeWithFlatTriangle,
        CubesWithALooseVertex,
        BoxesAlongAnEdgeAndAtACorner,
        TinyCube
    }

    private static Builder builderOf( HandBuilt shape ) {
        return shape switch {
            HandBuilt.Cube                         => new Builder( ).box( new float3( -0.5f ), new float3( 1.0f ) ),
            HandBuilt.Prism3                       => CreaseShapes.prism( 3 ),
            HandBuilt.Prism4                       => CreaseShapes.prism( 4 ),
            HandBuilt.Prism8                       => CreaseShapes.prism( 8 ),
            HandBuilt.Prism10                      => CreaseShapes.prism( 10 ),
            HandBuilt.Prism24                      => CreaseShapes.prism( 24 ),
            HandBuilt.FlatPatch                    => CreaseShapes.flatPatch( ),
            HandBuilt.Sphere                       => CreaseShapes.sphere( ),
            HandBuilt.ConeTip                      => CreaseShapes.coneTip( out _ ),
            HandBuilt.TurningTriangles             => CreaseShapes.turningTriangles( ),
            HandBuilt.CornersOfTheSameSize         => CreaseShapes.cornersOfTheSameSize( false ),
            HandBuilt.CornersOfTheSameSizeReversed => CreaseShapes.cornersOfTheSameSize( true ),
            HandBuilt.PatchWithOddNormals          => CreaseShapes.patchWithOddNormals( out _ ),
            HandBuilt.CubeWithFlatTriangle         => CreaseShapes.cubeWithFlatTriangle( ),
            HandBuilt.CubesWithALooseVertex        => CreaseShapes.cubesWithALooseVertex( new float3( 0.6f, 0.0f, 0.8f ) ),
            HandBuilt.BoxesAlongAnEdgeAndAtACorner => CreaseShapes.boxesAlongAnEdgeAndAtACorner( ),
            _                                  => new Builder( ).box( new float3( 0.3f, 0.2f, 0.1f ), new float3( 1e-4f ) )
        };
    }

    // a hand-built mesh through both creases. An odd triangle out is paired with one without area on its own first
    // vertex, which both have the same way: it has no say in that vertex's normal
    private static void assertCreasedAsInBurst( Builder builder, float angle, SmoothNormals smoothNormals, string what ) {
        var triangles = builder.triangles.ToList( );

        if( triangles.Count % 2 == 1 ) {
            var last = triangles[triangles.Count - 1];

            triangles.Add( new BurstMesh.Triangle( last.a, last.a, last.a, last.subMeshIndex ) );
        }

        var positions = builder.positions.ToArray( );
        var normals   = builder.normals.ToArray( );
        var input     = triangles.ToArray( );

        var burst = burstCrease( positions, normals, input, angle, smoothNormals );
        var gpu   = gpuCrease( positions, normals, input, angle, smoothNormals );

        var corners = Enumerable.Range( 0, input.Length ).SelectMany(
            ( triangle ) => Enumerable.Range( 0, 3 ).Select( ( corner ) => ( $"{corner} of triangle {triangle}", indexOf( burst.triangles[triangle], corner ), indexOf( gpu.triangles[triangle], corner ) ) )
        ).ToArray( );

        assertSameCrease( corners, burst.positions, burst.normals, gpu.positions, gpu.normals, what );

        // and the vertices no triangle uses, which are in neither list of corners
        CollectionAssert.AreEquivalent( burst.positions, gpu.positions, $"{what}: the positions of the creased vertices" );
    }

    // every shape but the cone's tip. Its corners are all the same size and its normals alike but for their turn about
    // its axis, so their order is down to rounding, which floats and doubles do differently, and the order decides the
    // groups (coneTipKeepsEveryCornerWithinTheAngleOnTheGPU)
    private static readonly HandBuilt[] shapesWithoutTiesOfRounding = Enum.GetValues( typeof( HandBuilt ) ).Cast<HandBuilt>( ).Where( ( shape ) => shape != HandBuilt.ConeTip ).ToArray( );

    [Test]
    public void handBuiltMeshIsCreasedAsInBurst( [ValueSource( nameof( shapesWithoutTiesOfRounding ) )] HandBuilt shape, [Values( 40.0f, 50.0f, 91.0f, 180.0f )] float angle, [Values] SmoothNormals smoothNormals ) {
        assertCreasedAsInBurst( builderOf( shape ), angle, smoothNormals, $"{shape} at {angle} degrees under {smoothNormals}" );
    }

    // at 0 degrees only triangles that face exactly the same way share a vertex. The GPU's floats and Burst's doubles
    // agree on that where the corners lie on whole numbers in the planes of the axes, where the normals come out exact.
    // Elsewhere the GPU's square root, which is not exact, can leave two triangles in one plane a hair apart
    [Test]
    public void handBuiltMeshAtZeroDegreesIsCreasedAsInBurst( [Values( HandBuilt.Cube, HandBuilt.BoxesAlongAnEdgeAndAtACorner, HandBuilt.CubesWithALooseVertex )] HandBuilt shape, [Values] SmoothNormals smoothNormals ) {
        assertCreasedAsInBurst( builderOf( shape ), 0.0f, smoothNormals, $"{shape} at 0 degrees under {smoothNormals}" );
    }

    // the cone's tip on the GPU: its groups may differ from Burst's, but no corner is further than the angle from its
    // triangle, the tip is split, and the triangles are the cone's
    [Test]
    public void coneTipKeepsEveryCornerWithinTheAngleOnTheGPU( [Values( 40.0f, 50.0f )] float angle, [Values] SmoothNormals smoothNormals ) {
        var builder   = CreaseShapes.coneTip( out var tip );
        var positions = builder.positions.ToArray( );
        var input     = builder.triangles.ToArray( );
        var gpu       = gpuCrease( positions, builder.normals.ToArray( ), input, angle, smoothNormals );

        Assert.GreaterOrEqual( gpu.positions.Count( ( position ) => position.Equals( tip ) ), 3, "groups at the tip" );

        for( var triangle = 0; triangle < input.Length; ++triangle ) {
            var face = BurstCreases.faceOf( gpu.positions[gpu.triangles[triangle].a], gpu.positions[gpu.triangles[triangle].b], gpu.positions[gpu.triangles[triangle].c] );

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.AreEqual( positions[indexOf( input[triangle], corner )], gpu.positions[indexOf( gpu.triangles[triangle], corner )], $"corner {corner} of triangle {triangle}" );

                var normal = gpu.normals[indexOf( gpu.triangles[triangle], corner )];

                Assert.LessOrEqual( degreesBetween( ( float3 )face, normal ), angle + tolerance, $"degrees between the normal of corner {corner} of triangle {triangle} and its triangle's" );
                Assert.AreEqual( 1.0, math.length( ( double3 )normal ), 1e-5, $"length of the normal of corner {corner} of triangle {triangle}" );
            }
        }
    }

    // a dispatch of more than 65,535 groups goes on along y. Only a contour of millions of corners needs that, so the
    // test makes the dispatches go along y after three groups
    [Test]
    public void dispatchesAlongYCreaseAsInBurst( [Values( HandBuilt.Sphere, HandBuilt.Prism24, HandBuilt.CubesWithALooseVertex )] HandBuilt shape ) {
        var across = GPUCreases.groupsAcross;

        try {
            GPUCreases.groupsAcross = 3;

            assertCreasedAsInBurst( builderOf( shape ), 40.0f, SmoothNormals.Faces, $"{shape}, dispatched three groups across" );
        }
        finally {
            GPUCreases.groupsAcross = across;
        }
    }

    // ---- contours ----

    // the corners of a mesh, each named by its sub-mesh, the positions of its triangle's corners and which of them it
    // is, with its vertex. GPU contours made one after the other have their quads in different orders but the same
    // triangles
    private static Dictionary<string, int> cornersOf( Mesh mesh ) {
        var vertices = mesh.vertices;
        var result   = new Dictionary<string, int>( );

        string key( Vector3 v ) {
            return $"{v.x:R},{v.y:R},{v.z:R}";
        }

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices = mesh.GetTriangles( subMesh );

            for( var index = 0; index < indices.Length; index += 3 ) {
                var triangle = $"{subMesh}: {key( vertices[indices[index]] )} {key( vertices[indices[index + 1]] )} {key( vertices[indices[index + 2]] )}";

                for( var corner = 0; corner < 3; ++corner ) {
                    result.Add( $"{corner} of {triangle}", indices[index + corner] );
                }
            }
        }
        return result;
    }

    private static float3[] float3sOf( Vector3[] vectors ) {
        return vectors.Select( ( vector ) => ( float3 )vector ).ToArray( );
    }

    // the mesh of a GPU contour, read back without the debug data, which takes far longer
    private Mesh gpuMeshOf( Voxelizer voxelizer ) {
        var mesh = voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ).visit( ( cpu ) => cpu.mesh, ( gpu ) => voxelizer.readMesh( gpu ) );

        this.made.Add( mesh );

        return mesh;
    }

    // a contour creased on the GPU against BurstCreases on the same contour given with shared vertices, which a
    // simplifying voxelizer asks for
    [Test]
    public void contourCreasedOnTheGPUIsCreasedAsInBurst( [Values( Contour.UniformGPU, Contour.OccupancyGPU )] Contour contour, [Values] Surface surface, [Values] SmoothNormals smoothNormals ) {
        var voxelizer = this.voxelizerOf( contour, shapesOf( surface ), 5 );

        voxelizer.creaseAngle   = 40.0f;
        voxelizer.smoothNormals = smoothNormals;

        var gpu = this.gpuMeshOf( voxelizer );

        voxelizer.meshSimplification = true;

        var shared = this.gpuMeshOf( voxelizer );
        var burst  = BurstCreases.crease( shared, voxelizer.creaseAngle, smoothNormals, out _ );

        this.made.Add( burst );

        Assert.AreEqual( SurfaceExtractor.ContourVertices.Shared, voxelizer.contourVertices, "the vertices asked for the contour Burst creases" );

        var gpuCorners   = cornersOf( gpu );
        var burstCorners = cornersOf( burst );

        // as sets, since NUnit compares collections as bags in time that grows with the square of their size
        Assert.AreEqual( burstCorners.Count, gpuCorners.Count, "corners of the triangles" );
        Assert.IsTrue( new HashSet<string>( burstCorners.Keys ).SetEquals( gpuCorners.Keys ), "the corners of the triangles" );

        var corners = gpuCorners.Select( ( pair ) => ( pair.Key, burstCorners[pair.Key], pair.Value ) );

        assertSameCrease( corners, float3sOf( burst.vertices ), float3sOf( burst.normals ), float3sOf( gpu.vertices ), float3sOf( gpu.normals ), $"{contour} {surface} {smoothNormals}" );
    }

}

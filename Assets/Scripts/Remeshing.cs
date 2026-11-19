using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// the remeshing pass, which rebuilds the mesh of a contour after mesh simplification and before its normals are creased
// or split (Documents/Plans/Remeshing.md), and what a voxelizer keeps of the mesh the pass was last given. From that mesh
// Voxelizer.remesh shows the contour again under other remeshing settings, without contouring or simplifying. It is all
// in managed arrays, as MeshSimplification is, so that a script reload leaves nothing behind
public class Remeshing {

    // how a mesh is remeshed. The techniques of the plan come with their steps
    public enum Technique {
        Off = 0,

        // triangles close to equilateral and all about one size, with the mesh's features and topology kept
        // (BurstIsotropic)
        Isotropic = 1,

        // triangles, or mostly quads, or only quads, in rows that follow the shape's edges and curvature
        // (BurstInstantMeshes)
        InstantMeshes = 2
    }

    // where the normals of a remeshed mesh come from, which its creasing or splitting then starts from (finding 21,
    // decision 11)
    public enum Normals {
        // the normals the mesh it was given would show creased at the crease angle, at the point of it each vertex lies
        // on (BurstRemeshing.cornerNormalsOf)
        Input = 0,

        // the normal of each vertex's own triangles, weighted by their areas
        Faces = 1
    }

    // how a mesh is remeshed: the technique, the edge length against the mesh's own (decision 9), and what the
    // techniques take besides: among it the normals out, the crease angle the input is shown at for Normals.Input, and
    // the faces Instant Meshes makes
    public readonly struct Settings {

        public readonly Technique                technique;
        public readonly float                    edgeLength;
        public readonly float                    featureAngle;   // degrees
        public readonly int                      iterations;
        public readonly int                      areaIterations;
        public readonly Normals                  normals;
        public readonly float                    creaseAngle;    // degrees
        public readonly BurstInstantMeshes.Faces faces;

        public Settings(
            Technique technique, float edgeLength, float featureAngle = 40.0f, int iterations = 5, int areaIterations = 5, Normals normals = Normals.Input, float creaseAngle = 40.0f,
            BurstInstantMeshes.Faces faces = BurstInstantMeshes.Faces.QuadDominant
        ) {
            this.technique      = technique;
            this.edgeLength     = edgeLength;
            this.featureAngle   = featureAngle;
            this.iterations     = iterations;
            this.areaIterations = areaIterations;
            this.normals        = normals;
            this.creaseAngle    = creaseAngle;
            this.faces          = faces;
        }

    }

    // the most vertices a remesh is let make: a shorter edge length gives up rather than run out of memory. A million
    // vertices take about half a gigabyte in the lists
    public const int mostVertices = 1 << 20;

    // the most vertices the mesh Instant Meshes' fields are on may have, which is the mesh given with its edges split
    // to half the edge length: about twelve times the vertices out (finding 24). Two million take about a gigabyte in
    // the lists and the hierarchy
    public const int mostFieldVertices = 1 << 21;

    // what a remesh did
    public readonly struct Statistics {

        public readonly Technique technique;
        public readonly double    milliseconds;

        // the mesh it was given
        public readonly int inputVertexCount;
        public readonly int inputTriangleCount;

        // the length of the edges it made, in the cells of the contour (decision 9)
        public readonly double edgeLength;

        // its faces by the number of their sides
        public readonly int triangles;
        public readonly int quads;
        public readonly int largerFaces;

        // lines of edges of one triangle, and of more than two, that it has beyond those of the mesh it was given
        // (finding 7): a hole torn, or sheets joined. Isotropic leaves none
        public readonly int newOpenLines;
        public readonly int newNonManifoldLines;

        // why it gave the mesh back as it came, or null when it did not
        public readonly string gaveUp;

        // what Isotropic did, stage by stage, where it remeshed
        public readonly BurstIsotropic.Report? isotropic;

        // what Instant Meshes did, stage by stage, where it got as far as solving its fields
        public readonly BurstInstantMeshes.Report? instantMeshes;

        public Statistics(
            Technique technique, double milliseconds, int inputVertexCount, int inputTriangleCount, double edgeLength, int triangles, int quads, int largerFaces, int newOpenLines,
            int newNonManifoldLines, string gaveUp, BurstIsotropic.Report? isotropic = null, BurstInstantMeshes.Report? instantMeshes = null
        ) {
            this.isotropic           = isotropic;
            this.instantMeshes       = instantMeshes;
            this.technique           = technique;
            this.milliseconds        = milliseconds;
            this.inputVertexCount    = inputVertexCount;
            this.inputTriangleCount  = inputTriangleCount;
            this.edgeLength          = edgeLength;
            this.triangles           = triangles;
            this.quads               = quads;
            this.largerFaces         = largerFaces;
            this.newOpenLines        = newOpenLines;
            this.newNonManifoldLines = newNonManifoldLines;
            this.gaveUp              = gaveUp;
        }

    }

    // the mesh the pass was given: vertices its triangles share, their normals, and its triangles with the sub-mesh of
    // each, and the size of the cells of its contour
    public readonly float3[]             positions;
    public readonly float3[]             normals;
    public readonly BurstMesh.Triangle[] triangles;
    public readonly float                cellSize;

    // its area, the vertices its triangles use, and its open and non-manifold edges, worked out when they are first
    // wanted: keeping the mesh is all that a contour does while remeshing is off
    private bool                    measured;
    private double                  area;
    private int                     usedVertexCount;
    private BurstRemeshing.Topology topology;

    private Remeshing( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, float cellSize ) {
        this.positions = positions;
        this.normals   = normals;
        this.triangles = triangles;
        this.cellSize  = cellSize;
    }

    // keeps a mesh of shared vertices, which is left as it is
    public static Remeshing of( Mesh mesh, float cellSize ) {
        BurstMeshSimplifier.read( mesh, Allocator.TempJob, out var positions, out var normals, out var triangles );

        try {
            return new Remeshing( positions.ToArray( ), normals.ToArray( ), triangles.ToArray( ), cellSize );
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );
        }
    }

    // the faces a technique makes, which its edge length is worked out for: Isotropic's triangles, and Instant Meshes'
    // triangles or quads, which quads with a few triangles and pentagons among them count as
    public static BurstRemeshing.Faces facesOf( Technique technique, BurstInstantMeshes.Faces faces = BurstInstantMeshes.Faces.QuadDominant ) {
        return technique == Technique.InstantMeshes && faces != BurstInstantMeshes.Faces.Triangles ? BurstRemeshing.Faces.Quads : BurstRemeshing.Faces.Triangles;
    }

    // the length of the edges a technique makes at an edge length, in the units of the mesh: the setting times the length
    // at which its faces would be as many vertices as the mesh has (decision 9)
    public double lengthOf( Technique technique, float edgeLength, BurstInstantMeshes.Faces faces = BurstInstantMeshes.Faces.QuadDominant ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        this.measure( positions, triangles );

        return edgeLength * BurstRemeshing.lengthOf( this.area, this.usedVertexCount, facesOf( technique, faces ) );
    }

    private void measure( NativeArray<float3> positions, NativeArray<BurstMesh.Triangle> triangles ) {
        if( this.measured ) {
            return;
        }

        using var area        = new NativeArray<double>( 1, Allocator.TempJob );
        using var vertexCount = new NativeArray<int>( 1, Allocator.TempJob );
        using var none        = new NativeArray<byte>( 0, Allocator.TempJob );

        new BurstRemeshing.AreaJob { positions = positions, triangles = triangles, area = area, vertexCount = vertexCount }.Run( );

        BurstRemeshing.count( triangles, none, out this.topology, out _ );

        this.area            = area[0];
        this.usedVertexCount = vertexCount[0];
        this.measured        = true;
    }

    // the mesh as the pass was given it
    public Mesh mesh( ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        return BurstMesh.createMesh( positions, normals, triangles );
    }

    // the mesh remeshed: a new mesh of shared vertices, and for each of its triangles in the order of its index buffer,
    // which of its edges are edges of its faces (BurstRemeshing.allEdges). A technique that cannot do its work gives the
    // mesh as it came, without flags, and says why in the statistics
    public Mesh remesh( in Settings settings, out byte[] flags, out Statistics statistics ) {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );
        var technique = settings.technique;

        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        this.measure( positions, triangles );

        var kind   = facesOf( technique, settings.faces );
        var length = settings.edgeLength * BurstRemeshing.lengthOf( this.area, this.usedVertexCount, kind );

        Mesh                       mesh;
        string                     gaveUp  = null;
        BurstRemeshing.Topology    topology;
        BurstRemeshing.FaceCounts  faces;
        BurstIsotropic.Report?     report  = null;
        BurstInstantMeshes.Report? instant = null;

        // the mesh as it came, every triangle a face, where a technique gives up
        void asItCame( out Mesh given, out byte[] noFlags, out BurstRemeshing.Topology itsTopology, out BurstRemeshing.FaceCounts itsFaces ) {
            using var every = new NativeArray<byte>( 0, Allocator.TempJob );

            given       = BurstMesh.createMesh( positions, normals, triangles );
            noFlags     = null;
            itsTopology = this.topology;

            BurstRemeshing.count( triangles, every, out _, out itsFaces );
        }

        switch( technique ) {
            case Technique.Isotropic: {
                gaveUp = this.tooManyVertices( length, kind );

                if( gaveUp != null ) {
                    asItCame( out mesh, out flags, out topology, out faces );
                    break;
                }

                using var outPositions  = new NativeList<float3>( Allocator.TempJob );
                using var outNormals    = new NativeList<float3>( Allocator.TempJob );
                using var outTriangles  = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
                using var cornerNormals = settings.normals == Normals.Input
                    ? BurstRemeshing.cornerNormalsOf( positions, normals, triangles, settings.creaseAngle, Allocator.TempJob )
                    : new NativeArray<float3>( 0, Allocator.TempJob );

                BurstIsotropic.remesh(
                    positions,
                    normals,
                    triangles,
                    cornerNormals,
                    new BurstIsotropic.Settings {
                        length        = length,
                        featureCosine = math.cos( math.radians( ( double )settings.featureAngle ) ),
                        featureLength = featureLengthOf( length ),
                        iterations    = settings.iterations,
                        areaRounds    = settings.areaIterations,
                        damping       = BurstIsotropic.damping
                    },
                    outPositions,
                    outNormals,
                    outTriangles,
                    out var isotropic
                );

                report = isotropic;
                mesh   = BurstMesh.createMesh( outPositions.AsArray( ), outNormals.AsArray( ), outTriangles.AsArray( ) );
                flags  = null;

                using var none = new NativeArray<byte>( 0, Allocator.TempJob );

                BurstRemeshing.count( outTriangles.AsArray( ), none, out topology, out faces );

                break;
            }
            case Technique.InstantMeshes: {
                gaveUp = this.tooManyVertices( length, kind );

                if( gaveUp != null ) {
                    asItCame( out mesh, out flags, out topology, out faces );
                    break;
                }

                using var outPositions  = new NativeList<float3>( Allocator.TempJob );
                using var outNormals    = new NativeList<float3>( Allocator.TempJob );
                using var outTriangles  = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
                using var outFlags      = new NativeList<byte>( Allocator.TempJob );
                using var cornerNormals = settings.normals == Normals.Input
                    ? BurstRemeshing.cornerNormalsOf( positions, normals, triangles, settings.creaseAngle, Allocator.TempJob )
                    : new NativeArray<float3>( 0, Allocator.TempJob );

                BurstInstantMeshes.remesh(
                    positions,
                    normals,
                    triangles,
                    cornerNormals,
                    new BurstInstantMeshes.Settings {
                        faces         = settings.faces,
                        length        = length,
                        featureCosine = math.cos( math.radians( ( double )settings.featureAngle ) ),
                        featureLength = featureLengthOf( length ),
                        mostVertices  = mostFieldVertices
                    },
                    outPositions,
                    outNormals,
                    outTriangles,
                    outFlags,
                    out var instantMeshes
                );

                instant = instantMeshes;

                if( instantMeshes.fields.tooLarge ) {
                    gaveUp = $"edges of {length / this.cellSize:G3} cells would put the fields on more than {mostFieldVertices:N0} vertices";
                }
                else if( outTriangles.Length == 0 ) {
                    gaveUp = "no faces came out of the fields";
                }

                if( gaveUp != null ) {
                    asItCame( out mesh, out flags, out topology, out faces );
                    break;
                }

                mesh  = BurstMesh.createMesh( outPositions.AsArray( ), outNormals.AsArray( ), outTriangles.AsArray( ) );
                flags = inIndexOrder( outTriangles.AsArray( ), outFlags.AsArray( ) );

                BurstRemeshing.count( outTriangles.AsArray( ), outFlags.AsArray( ), out topology, out faces );

                break;
            }
            default: {
                throw new ArgumentException( $"{technique} is not a technique that remeshes", nameof( technique ) );
            }
        }

        statistics = new Statistics(
            technique,
            stopwatch.Elapsed.TotalMilliseconds,
            this.positions.Length,
            this.triangles.Length,
            length / this.cellSize,
            faces.triangles,
            faces.quads,
            faces.larger,
            Math.Max( 0, topology.openLines - this.topology.openLines ),
            Math.Max( 0, topology.nonManifoldLines - this.topology.nonManifoldLines ),
            gaveUp,
            report,
            instant
        );

        return mesh;
    }

    // the length the features by angle of the mesh are found at, for a remesh at a length (BurstFeatures; step 3b,
    // decision 12): the length, but no more than a cell of the contour. The noise that features by angle are found
    // at the scale of a length to clear is of the contour's cells, whatever the length, and a simplified mesh is
    // remeshed at lengths of many cells, which would take its features smaller than that with it
    public double featureLengthOf( double length ) {
        return Math.Min( length, this.cellSize );
    }

    // why a remesh at a length gives up before it starts: the mesh has no area, or the length would make more
    // vertices than mostVertices. Null when it does not
    private string tooManyVertices( double length, BurstRemeshing.Faces faces ) {
        if( !( length > 0.0 ) ) {
            return "the mesh has no area";
        }

        var perVertex = faces == BurstRemeshing.Faces.Triangles ? math.sqrt( 3.0 ) / 2.0 * length * length : length * length;
        var vertices  = this.area / perVertex;

        return vertices > mostVertices ? $"edges of {length / this.cellSize:G3} cells would make about {vertices:N0} vertices, more than {mostVertices:N0}" : null;
    }

    // the flags of the triangles in the order of the index buffer a mesh of them has: by sub-mesh, and in the order of
    // the triangles within each, as BurstMesh.WriteMeshJob writes them
    public static byte[] inIndexOrder( NativeArray<BurstMesh.Triangle> triangles, NativeArray<byte> flags ) {
        var subMeshCount = 0;
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            subMeshCount = Math.Max( subMeshCount, triangles[triangle].subMeshIndex + 1 );
        }

        var next = new int[subMeshCount + 1];
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            ++next[triangles[triangle].subMeshIndex + 1];
        }
        for( var subMesh = 0; subMesh < subMeshCount; ++subMesh ) {
            next[subMesh + 1] += next[subMesh];
        }

        var result = new byte[triangles.Length];
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            result[next[triangles[triangle].subMeshIndex]++] = flags[triangle];
        }

        return result;
    }

}

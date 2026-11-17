using System;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using FeatureMesh = BurstIsotropic.FeatureMesh;
using Kind        = BurstIsotropic.Kind;
using FlipRefusal = BurstMeshSimplifier.FlipRefusal;
using Refusal     = BurstMeshSimplifier.Refusal;

// something done to a mesh with its features, given by the vertices of an edge, in a FeatureMeshRun
internal struct FeatureOperation {

    public enum Act {
        Split,     // the edge at the position
        CheckFlip, // whether the edge may be flipped
        Flip,      // the edge, where it may be
        Place,     // where the edge would collapse
        Collapse,  // the edge, where it may and to where placeCollapse says
        Longest    // the longest edge a collapse of the edge into the position would leave
    }

    public Act     act;
    public int     a;
    public int     b;
    public double3 position;

    public static FeatureOperation split( int a, int b, double3 position ) {
        return new FeatureOperation { act = Act.Split, a = a, b = b, position = position };
    }

    public static FeatureOperation checkFlip( int a, int b ) {
        return new FeatureOperation { act = Act.CheckFlip, a = a, b = b };
    }

    public static FeatureOperation flip( int a, int b ) {
        return new FeatureOperation { act = Act.Flip, a = a, b = b };
    }

    public static FeatureOperation place( int a, int b ) {
        return new FeatureOperation { act = Act.Place, a = a, b = b };
    }

    public static FeatureOperation collapse( int a, int b ) {
        return new FeatureOperation { act = Act.Collapse, a = a, b = b };
    }

    public static FeatureOperation longest( int a, int b, double3 position ) {
        return new FeatureOperation { act = Act.Longest, a = a, b = b, position = position };
    }

}

// a hand built mesh taken into a FeatureMesh with its features at an angle, the operations done to it in their order,
// and what each gave, all in one job compiled by Burst. Afterwards the kind of every vertex
internal class FeatureMeshRun : IDisposable {

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct RunJob : IJob {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<float3>             normals;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;
        [ReadOnly] public NativeArray<FeatureOperation>   operations;

        public double      cosine;
        public FeatureMesh mesh;

        public NativeArray<int> lineCount;

        // for every operation: the vertex a split made, a collapse kept or a placement goes into; whether the edge was
        // there, and whether placeCollapse allowed it; what checkFlip and check said; and the position of a placement
        public NativeArray<int>         vertices;
        public NativeArray<bool>        found;
        public NativeArray<bool>        allowed;
        public NativeArray<FlipRefusal> flips;
        public NativeArray<Refusal>     refusals;
        public NativeArray<double3>     placed;

        public NativeList<Kind> kinds;

        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            this.mesh.mesh.build( this.positions, this.normals, this.triangles );

            new BurstFeatures.SkeletonJob {
                cosine    = this.cosine,
                mesh      = this.mesh.mesh,
                edgeLines = this.mesh.edgeLines,
                corners   = this.mesh.corners,
                lineCount = this.lineCount
            }.Execute( );

            for( var index = 0; index < this.operations.Length; ++index ) {
                var operation = this.operations[index];
                var edge      = this.mesh.mesh.edgeBetween( operation.a, operation.b );

                this.vertices[index] = -1;
                this.found[index]    = edge != -1;

                if( edge == -1 ) {
                    continue;
                }

                switch( operation.act ) {
                    case FeatureOperation.Act.Split: {
                        this.vertices[index] = this.mesh.split( edge, operation.position );
                        break;
                    }
                    case FeatureOperation.Act.CheckFlip: {
                        this.flips[index] = this.mesh.checkFlip( edge, this.cosine );
                        break;
                    }
                    case FeatureOperation.Act.Flip: {
                        this.flips[index] = this.mesh.checkFlip( edge, this.cosine );

                        if( this.flips[index] == FlipRefusal.None ) {
                            this.mesh.flip( edge );
                        }
                        break;
                    }
                    case FeatureOperation.Act.Place: {
                        this.allowed[index]  = this.mesh.placeCollapse( edge, out var position, out var into );
                        this.placed[index]   = position;
                        this.vertices[index] = into;
                        break;
                    }
                    case FeatureOperation.Act.Collapse: {
                        this.allowed[index] = this.mesh.placeCollapse( edge, out var position, out var into );
                        this.placed[index]  = position;

                        if( !this.allowed[index] ) {
                            break;
                        }

                        this.refusals[index] = this.mesh.mesh.check( edge, position );

                        if( this.refusals[index] == Refusal.None ) {
                            this.vertices[index] = this.mesh.collapse( edge, position, into );
                        }
                        break;
                    }
                    case FeatureOperation.Act.Longest: {
                        this.placed[index] = new double3( this.mesh.longestEdgeAfter( edge, operation.position ), 0.0, 0.0 );
                        break;
                    }
                }
            }

            for( var vertex = 0; vertex < this.mesh.mesh.vertexCount; ++vertex ) {
                this.kinds.Add( this.mesh.mesh.firstCorners[vertex] == -1 ? Kind.Surface : this.mesh.kindOf( vertex ) );
            }
        }

    }

    public FeatureMesh mesh;
    public int         lineCount;

    public readonly int[]         vertices;
    public readonly bool[]        found;
    public readonly bool[]        allowed;
    public readonly FlipRefusal[] flips;
    public readonly Refusal[]     refusals;
    public readonly double3[]     placed;
    public readonly Kind[]        kinds;

    public FeatureMeshRun( HandBuiltMesh built, double angle, params FeatureOperation[] operations ) {
        this.mesh = new FeatureMesh( built.positions.Count, built.triangles.Count, Allocator.TempJob );

        using var positions     = built.nativePositions( );
        using var normals       = built.nativeNormals( );
        using var triangles     = built.nativeTriangles( );
        using var nativeActs    = new NativeArray<FeatureOperation>( operations, Allocator.TempJob );
        using var lineCount     = new NativeArray<int>( 1, Allocator.TempJob );
        using var vertices      = new NativeArray<int>( operations.Length, Allocator.TempJob );
        using var found         = new NativeArray<bool>( operations.Length, Allocator.TempJob );
        using var allowed       = new NativeArray<bool>( operations.Length, Allocator.TempJob );
        using var flips         = new NativeArray<FlipRefusal>( operations.Length, Allocator.TempJob );
        using var refusals      = new NativeArray<Refusal>( operations.Length, Allocator.TempJob );
        using var placed        = new NativeArray<double3>( operations.Length, Allocator.TempJob );
        using var kinds         = new NativeList<Kind>( Allocator.TempJob );
        using var managed       = new NativeArray<bool>( 1, Allocator.TempJob );

        new RunJob {
            positions  = positions,
            normals    = normals,
            triangles  = triangles,
            operations = nativeActs,
            cosine     = math.cos( math.radians( angle ) ),
            mesh       = this.mesh,
            lineCount  = lineCount,
            vertices   = vertices,
            found      = found,
            allowed    = allowed,
            flips      = flips,
            refusals   = refusals,
            placed     = placed,
            kinds      = kinds,
            managed    = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        this.lineCount = lineCount[0];
        this.vertices  = vertices.ToArray( );
        this.found     = found.ToArray( );
        this.allowed   = allowed.ToArray( );
        this.flips     = flips.ToArray( );
        this.refusals  = refusals.ToArray( );
        this.placed    = placed.ToArray( );
        this.kinds     = kinds.AsArray( ).ToArray( );
    }

    public void Dispose( ) {
        this.mesh.Dispose( );
    }

    // ---- the mesh afterwards, read here ----

    public int vertexCount {
        get { return this.mesh.mesh.vertexCount; }
    }

    public bool hasTriangles( int vertex ) {
        return this.mesh.mesh.firstCorners[vertex] != -1;
    }

    public double3 positionOf( int vertex ) {
        return this.mesh.mesh.positions[vertex];
    }

    // the edge between two vertices, or -1
    public int edgeBetween( int a, int b ) {
        for( var end = this.mesh.mesh.firstEdges[a]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
            var ends = this.mesh.mesh.edges[end >> 1];

            if( ( ends.x == a ? ends.y : ends.x ) == b ) {
                return end >> 1;
            }
        }
        return -1;
    }

    public int lineOf( int a, int b ) {
        var edge = this.edgeBetween( a, b );

        return edge == -1 ? -1 : this.mesh.edgeLines[edge];
    }

    // the edges that are left, as their two ends
    public ( int a, int b, int edge )[] edges( ) {
        return Enumerable.Range( 0, this.mesh.mesh.edgeCount )
                         .Where( ( edge ) => this.mesh.mesh.edges[edge].x != -1 )
                         .Select( ( edge ) => ( this.mesh.mesh.edges[edge].x, this.mesh.mesh.edges[edge].y, edge ) )
                         .ToArray( );
    }

    public int featureEdgeCount {
        get { return this.edges( ).Count( ( edge ) => this.mesh.edgeLines[edge.edge] != -1 ); }
    }

    public int[] cornerVertices( ) {
        return Enumerable.Range( 0, this.vertexCount ).Where( ( vertex ) => this.hasTriangles( vertex ) && this.kinds[vertex] == Kind.Corner ).ToArray( );
    }

    // the triangles that are left, with their sub-meshes
    public ( int3 corners, int subMesh )[] triangles( ) {
        return Enumerable.Range( 0, this.mesh.mesh.triangleCount )
                         .Where( ( triangle ) => this.mesh.mesh.subMeshes[triangle] != -1 )
                         .Select( ( triangle ) => ( this.mesh.mesh.triangles[triangle], this.mesh.mesh.subMeshes[triangle] ) )
                         .ToArray( );
    }

}

using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// what the remeshing techniques share (Documents/Plans/Remeshing.md): the length the edges of a remesh are measured
// against, the faces a remeshed mesh is made of, which of a mesh's edges are open or have more than two triangles, and
// the normals a mesh shows at the corners of its triangles, which a remeshed mesh takes its own from (finding 21)
public static class BurstRemeshing {

    // the faces of a remeshed mesh are triangles, each with a byte that says which of its edges are edges of its face.
    // Bit k stands for the edge from corner k to corner k + 1. A triangle that is a face by itself has all three, and
    // each of the two triangles of a quad leaves out the diagonal between them (finding 8)
    public const byte allEdges = 7;

    // the faces a technique makes, which say how many vertices an edge length gives (finding 16): a mesh of equilateral
    // triangles of side l has a vertex for every (sqrt 3 / 2) l^2 of its area, and a mesh of squares one for every l^2
    public enum Faces {
        Triangles,
        Quads
    }

    // the length at which a remesh into such faces has as many vertices as the mesh it is given, which is an edge
    // length of 1 (decision 9). A mesh without area or without vertices has none
    public static double lengthOf( double area, int vertexCount, Faces faces ) {
        if( !( area > 0.0 ) || vertexCount <= 0 ) {
            return 0.0;
        }

        var perVertex = area / vertexCount;

        return faces == Faces.Triangles ? math.sqrt( perVertex / ( math.sqrt( 3.0 ) / 2.0 ) ) : math.sqrt( perVertex );
    }

    // the area of a mesh's triangles, and how many of its vertices they use
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct AreaJob : IJob {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        // one each
        [WriteOnly] public NativeArray<double> area;
        [WriteOnly] public NativeArray<int>    vertexCount;

        public void Execute( ) {
            var used  = new NativeBitArray( math.max( this.positions.Length, 1 ), Allocator.Temp );
            var sum   = 0.0;
            var count = 0;

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                var t = this.triangles[triangle];
                var a = ( double3 )this.positions[t.a];
                var b = ( double3 )this.positions[t.b];
                var c = ( double3 )this.positions[t.c];

                sum += 0.5 * math.length( math.cross( b - a, c - a ) );

                count += mark( ref used, t.a ) + mark( ref used, t.b ) + mark( ref used, t.c );
            }

            this.area[0]        = sum;
            this.vertexCount[0] = count;

            used.Dispose( );
        }

        private static int mark( ref NativeBitArray used, int vertex ) {
            if( used.IsSet( vertex ) ) {
                return 0;
            }
            used.Set( vertex, true );
            return 1;
        }

    }

    // the edges of a mesh's triangles, each as the corner it starts from: corner 3 t + k of triangle t, whose edge runs
    // to corner k + 1. Sorted by the edge's two vertices and then by corner, the sides of each edge come together, in
    // the order of their triangles, which keeps everything made from them the same from run to run
    // the normal at every corner of every triangle, three to a triangle in their order, as the mesh would show them
    // creased at the angle (BurstCreases) with Smooth Normals at Contour: within a face that is the normal the creasing
    // gives the vertex there, and across a crease each side has its own. Under Faces the creasing after a remesh gives
    // every vertex its own triangles' normal whatever the remesh gave it, so Contour is all that is needed here. A remeshed vertex takes its normal from the corners of the triangle it lies
    // on (finding 21), so that it shades as the mesh it was given did there. The vertices of a mesh the voxelizer keeps
    // carry the contour's normals, which on a sharp edge lie between its faces, and which the creasing replaces there;
    // spread over the faces by a remesh, the creasing would keep them. The caller disposes of the array
    public static NativeArray<float3> cornerNormalsOf(
        NativeArray<float3>             positions,
        NativeArray<float3>             normals,
        NativeArray<BurstMesh.Triangle> triangles,
        double                          angle,
        Allocator                       allocator
    ) {
        BurstCreases.crease( positions, normals, triangles, angle, SurfaceExtractor.SmoothNormals.Contour, Allocator.TempJob, out var creasedPositions, out var creasedNormals, out var creasedTriangles, out _ );

        try {
            var corners = new NativeArray<float3>( triangles.Length * 3, allocator, NativeArrayOptions.UninitializedMemory );

            new CornerNormalsJob { normals = creasedNormals, triangles = creasedTriangles, corners = corners }.Schedule( triangles.Length, 256 ).Complete( );

            return corners;
        }
        finally {
            creasedPositions.Dispose( );
            creasedNormals.Dispose( );
            creasedTriangles.Dispose( );
        }
    }

    // the normal the mesh a remesh was given shows at a point of one of its triangles: the normals at the triangle's
    // corners (cornerNormalsOf), weighted by the barycentric coordinates of the point (Ericson, section 3.4). A triangle
    // without area, or corners whose normals cancel, give the normal the caller has otherwise
    public static double3 inputNormalAt( in SurfaceGrid grid, NativeArray<float3> cornerNormals, double3 point, int triangle, double3 otherwise ) {
        var a  = grid.corners[triangle * 3];
        var v0 = grid.corners[( triangle * 3 ) + 1] - a;
        var v1 = grid.corners[( triangle * 3 ) + 2] - a;
        var v2 = point - a;

        var d00 = math.dot( v0, v0 );
        var d01 = math.dot( v0, v1 );
        var d11 = math.dot( v1, v1 );
        var d20 = math.dot( v2, v0 );
        var d21 = math.dot( v2, v1 );

        var denominator = ( d00 * d11 ) - ( d01 * d01 );

        if( !( denominator > 0.0 ) ) {
            return otherwise;
        }

        var v = ( ( d11 * d20 ) - ( d01 * d21 ) ) / denominator;
        var w = ( ( d00 * d21 ) - ( d01 * d20 ) ) / denominator;
        var u = 1.0 - v - w;

        var mixed = ( ( double3 )cornerNormals[triangle * 3] * u ) + ( ( double3 )cornerNormals[( triangle * 3 ) + 1] * v ) + ( ( double3 )cornerNormals[( triangle * 3 ) + 2] * w );

        return math.lengthsq( mixed ) > 0.0 ? math.normalize( mixed ) : otherwise;
    }

    // the normals of a mesh's vertices at the corners of its triangles, three to a triangle
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct CornerNormalsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3>             normals;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> corners;

        public void Execute( int triangle ) {
            var corners = this.triangles[triangle];

            this.corners[triangle * 3]         = this.normals[corners.a];
            this.corners[( triangle * 3 ) + 1] = this.normals[corners.b];
            this.corners[( triangle * 3 ) + 2] = this.normals[corners.c];
        }

    }

    public struct Side : IComparable<Side> {

        // the lower vertex of the edge in the high half, the higher in the low
        public long key;
        public int  corner;

        public int CompareTo( Side other ) {
            return this.key != other.key ? this.key.CompareTo( other.key ) : this.corner.CompareTo( other.corner );
        }

    }

    public static long keyOf( int a, int b ) {
        return a < b ? ( ( long )a << 32 ) | ( uint )b : ( ( long )b << 32 ) | ( uint )a;
    }

    public static int vertexOf( in BurstMesh.Triangle triangle, int corner ) {
        return corner == 0 ? triangle.a : corner == 1 ? triangle.b : triangle.c;
    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SidesJob : IJob {

        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        // three for every triangle
        public NativeArray<Side> sides;

        public void Execute( ) {
            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                var t = this.triangles[triangle];

                for( var k = 0; k < 3; ++k ) {
                    this.sides[( 3 * triangle ) + k] = new Side { key = keyOf( vertexOf( t, k ), vertexOf( t, ( k + 1 ) % 3 ) ), corner = ( 3 * triangle ) + k };
                }
            }

            this.sides.Sort( );
        }

    }

    // how many of a mesh's edges have one triangle, and how many more than two, and how many lines each kind makes,
    // going from edge to edge of the kind at their vertices: what a technique may leave that the mesh it was given did
    // not have (finding 7). A remesh that samples a line again changes how many edges it has and not that it is a line
    public struct Topology {

        public int openEdges;
        public int nonManifoldEdges;
        public int openLines;
        public int nonManifoldLines;

    }

    // the faces of a remeshed mesh, by the number of their sides, from its triangles and their edges. A face is the
    // triangles that are joined across edges that both of their triangles leave out. Flags that do not make faces, an
    // edge left out on one side only or on an edge that does not have two triangles, or a face of fewer than three
    // sides, are counted apart
    public struct FaceCounts {

        public int triangles;
        public int quads;
        public int larger;
        public int inconsistent;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct CountsJob : IJob {

        [ReadOnly] public NativeArray<Side> sides;

        // one byte for every triangle, or none, which is every edge of every triangle
        [ReadOnly] public NativeArray<byte> flags;

        // one each
        [WriteOnly] public NativeArray<Topology>   topology;
        [WriteOnly] public NativeArray<FaceCounts> faces;

        public void Execute( ) {
            var triangleCount = this.sides.Length / 3;
            var parents       = new NativeArray<int>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var sideCounts    = new NativeArray<int>( math.max( triangleCount, 1 ), Allocator.Temp );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                parents[triangle] = triangle;
            }

            var topology     = new Topology( );
            var inconsistent = 0;

            // the lines, as sets of the vertices of their edges
            var vertexCount = 1;

            for( var side = 0; side < this.sides.Length; ++side ) {
                vertexCount = math.max( vertexCount, ( int )( this.sides[side].key & 0xffffffff ) + 1 );
            }

            var openParents        = new NativeArray<int>( vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var nonManifoldParents = new NativeArray<int>( vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var onOpen             = new NativeBitArray( vertexCount, Allocator.Temp );
            var onNonManifold      = new NativeBitArray( vertexCount, Allocator.Temp );

            for( var vertex = 0; vertex < vertexCount; ++vertex ) {
                openParents[vertex]        = vertex;
                nonManifoldParents[vertex] = vertex;
            }

            for( var start = 0; start < this.sides.Length; ) {
                var end = start + 1;
                while( end < this.sides.Length && this.sides[end].key == this.sides[start].key ) {
                    ++end;
                }

                var count = end - start;
                var lower = ( int )( this.sides[start].key >> 32 );
                var upper = ( int )( this.sides[start].key & 0xffffffff );

                if( count == 1 ) {
                    ++topology.openEdges;

                    union( openParents, lower, upper );
                    onOpen.Set( lower, true );
                    onOpen.Set( upper, true );
                }
                else if( count > 2 ) {
                    ++topology.nonManifoldEdges;

                    union( nonManifoldParents, lower, upper );
                    onNonManifold.Set( lower, true );
                    onNonManifold.Set( upper, true );
                }

                // the triangles on both sides of an edge that both leave out are one face. An edge left out anywhere else
                // makes no face
                var leftOut = 0;
                for( var side = start; side < end; ++side ) {
                    if( !this.drawn( this.sides[side].corner ) ) {
                        ++leftOut;
                    }
                }

                if( leftOut == 2 && count == 2 ) {
                    union( parents, this.sides[start].corner / 3, this.sides[start + 1].corner / 3 );
                }
                else if( leftOut > 0 ) {
                    ++inconsistent;
                }

                start = end;
            }

            // a face has as many sides as its triangles draw edges
            for( var corner = 0; corner < this.sides.Length; ++corner ) {
                if( this.drawn( corner ) ) {
                    ++sideCounts[find( parents, corner / 3 )];
                }
            }

            var faces = new FaceCounts { inconsistent = inconsistent };

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                if( find( parents, triangle ) != triangle ) {
                    continue;
                }

                var sides = sideCounts[triangle];

                if( sides < 3 ) {
                    ++faces.inconsistent;
                }
                else if( sides == 3 ) {
                    ++faces.triangles;
                }
                else if( sides == 4 ) {
                    ++faces.quads;
                }
                else {
                    ++faces.larger;
                }
            }

            for( var vertex = 0; vertex < vertexCount; ++vertex ) {
                if( onOpen.IsSet( vertex ) && find( openParents, vertex ) == vertex ) {
                    ++topology.openLines;
                }
                if( onNonManifold.IsSet( vertex ) && find( nonManifoldParents, vertex ) == vertex ) {
                    ++topology.nonManifoldLines;
                }
            }

            this.topology[0] = topology;
            this.faces[0]    = faces;

            parents.Dispose( );
            sideCounts.Dispose( );
            openParents.Dispose( );
            nonManifoldParents.Dispose( );
            onOpen.Dispose( );
            onNonManifold.Dispose( );
        }

        private bool drawn( int corner ) {
            return this.flags.Length == 0 || ( this.flags[corner / 3] & ( 1 << ( corner % 3 ) ) ) != 0;
        }

        private static int find( NativeArray<int> parents, int item ) {
            while( parents[item] != item ) {
                parents[item] = parents[parents[item]];
                item          = parents[item];
            }
            return item;
        }

        private static void union( NativeArray<int> parents, int item, int other ) {
            var a = find( parents, item );
            var b = find( parents, other );

            // the lower triangle is the root, so a face is counted at its first triangle
            if( a < b ) {
                parents[b] = a;
            }
            else if( b < a ) {
                parents[a] = b;
            }
        }

    }

    // the counts of a mesh's edges and faces. No flags, an empty array, is every edge of every triangle
    public static void count( NativeArray<BurstMesh.Triangle> triangles, NativeArray<byte> flags, out Topology topology, out FaceCounts faces ) {
        using var sides       = new NativeArray<Side>( triangles.Length * 3, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var topologyOut = new NativeArray<Topology>( 1, Allocator.TempJob );
        using var facesOut    = new NativeArray<FaceCounts>( 1, Allocator.TempJob );

        new SidesJob { triangles = triangles, sides = sides }.Run( );

        new CountsJob { sides = sides, flags = flags, topology = topologyOut, faces = facesOut }.Run( );

        topology = topologyOut[0];
        faces    = facesOut[0];
    }

}

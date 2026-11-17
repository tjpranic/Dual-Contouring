using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// what the remeshing techniques share (Documents/Plans/Remeshing.md): the length the edges of a remesh are measured
// against, the faces a remeshed mesh is made of, and which of a mesh's edges are open or have more than two triangles.
// And the stand-in technique that the frame around the techniques is tested with until the first of them is built,
// which pairs triangles that lie flat together into quads. The plan takes it out again at step 5
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

    // how many of a mesh's edges have one triangle, and how many more than two: what a technique may leave that the mesh
    // it was given did not have (finding 7)
    public struct Topology {

        public int openEdges;
        public int nonManifoldEdges;

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

            for( var start = 0; start < this.sides.Length; ) {
                var end = start + 1;
                while( end < this.sides.Length && this.sides[end].key == this.sides[start].key ) {
                    ++end;
                }

                var count = end - start;

                if( count == 1 ) {
                    ++topology.openEdges;
                }
                else if( count > 2 ) {
                    ++topology.nonManifoldEdges;
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

            this.topology[0] = topology;
            this.faces[0]    = faces;

            parents.Dispose( );
            sideCounts.Dispose( );
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

    // ---- the stand-in technique ----

    // how far apart the normals of two triangles may be for them to lie flat together: a degree
    public const double flatPairCosine = 0.99984769515639127; // cos 1°

    private struct Candidate : IComparable<Candidate> {

        public double length;
        public int    side;

        // the longest edges first, which are the diagonals of the quads their triangles would make, and then in the
        // order of the sides
        public int CompareTo( Candidate other ) {
            return this.length != other.length ? other.length.CompareTo( this.length ) : this.side.CompareTo( other.side );
        }

    }

    // pairs triangles across an edge into quads where they lie flat together: of the same sub-mesh, wound the same way,
    // their normals within a degree, and the quad they make convex. Each triangle is in one pair at most, and the
    // longest edges are paired first. Nothing moves: what changes is only which edges are drawn
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct FlatPairsJob : IJob {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;
        [ReadOnly] public NativeArray<Side>               sides;

        // one byte for every triangle
        [WriteOnly] public NativeArray<byte> flags;

        // one: how many pairs were made
        [WriteOnly] public NativeArray<int> pairs;

        public void Execute( ) {
            var candidates = new NativeList<Candidate>( Allocator.Temp );

            for( var start = 0; start < this.sides.Length; ) {
                var end = start + 1;
                while( end < this.sides.Length && this.sides[end].key == this.sides[start].key ) {
                    ++end;
                }

                if( end - start == 2 && this.flat( this.sides[start].corner, this.sides[start + 1].corner, out var length ) ) {
                    candidates.Add( new Candidate { length = length, side = start } );
                }

                start = end;
            }

            candidates.Sort( );

            var flags  = new NativeArray<byte>( this.triangles.Length, Allocator.Temp );
            var paired = new NativeBitArray( math.max( this.triangles.Length, 1 ), Allocator.Temp );
            var count  = 0;

            for( var triangle = 0; triangle < flags.Length; ++triangle ) {
                flags[triangle] = allEdges;
            }

            for( var index = 0; index < candidates.Length; ++index ) {
                var candidate = candidates[index];
                var first     = this.sides[candidate.side].corner;
                var second = this.sides[candidate.side + 1].corner;

                if( paired.IsSet( first / 3 ) || paired.IsSet( second / 3 ) ) {
                    continue;
                }

                paired.Set( first / 3, true );
                paired.Set( second / 3, true );

                flags[first / 3]  = ( byte )( allEdges & ~( 1 << ( first % 3 ) ) );
                flags[second / 3] = ( byte )( allEdges & ~( 1 << ( second % 3 ) ) );

                ++count;
            }

            this.flags.CopyFrom( flags );
            this.pairs[0] = count;

            candidates.Dispose( );
            flags.Dispose( );
            paired.Dispose( );
        }

        private double3 at( int triangle, int corner ) {
            return this.positions[vertexOf( this.triangles[triangle], corner % 3 )];
        }

        // the edge from first's corner runs from a to b in its triangle (a, b, c), and from b to a in the other's
        // triangle (b, a, d). Their quad runs a, d, b, c, and is convex where the other diagonal, from c to d, cuts it
        // into two triangles that face the way the first does
        private bool flat( int first, int second, out double length ) {
            length = 0.0;

            var t1 = first / 3;
            var t2 = second / 3;
            var k1 = first % 3;
            var k2 = second % 3;

            if( t1 == t2 || this.triangles[t1].subMeshIndex != this.triangles[t2].subMeshIndex ) {
                return false;
            }

            var a = this.at( t1, k1 );
            var b = this.at( t1, k1 + 1 );
            var c = this.at( t1, k1 + 2 );
            var d = this.at( t2, k2 + 2 );

            // wound the same way: the other runs the edge the other way round
            if( vertexOf( this.triangles[t2], k2 ) != vertexOf( this.triangles[t1], ( k1 + 1 ) % 3 ) ) {
                return false;
            }

            var n1 = math.cross( b - a, c - a );
            var n2 = math.cross( a - b, d - b );

            var l1 = math.length( n1 );
            var l2 = math.length( n2 );

            if( !( l1 > 0.0 ) || !( l2 > 0.0 ) || math.dot( n1, n2 ) < flatPairCosine * l1 * l2 ) {
                return false;
            }

            if( math.dot( math.cross( a - c, d - c ), n1 ) <= 0.0 || math.dot( math.cross( b - d, c - d ), n1 ) <= 0.0 ) {
                return false;
            }

            length = math.length( b - a );

            return true;
        }

    }

}

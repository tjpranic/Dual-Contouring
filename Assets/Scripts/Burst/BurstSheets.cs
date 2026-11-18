using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// a mesh split into sheets (Documents/Plans/Remeshing.md, decision 4 and finding 18): along its edges of more than two
// triangles and at its vertices of several fans, so that what is left is a manifold. The two field techniques work on
// the sheets: QuadriFlow cannot take such edges, and Field Aligned breaks down around them (finding 4).
//
// Round every edge its triangles are paired, and two triangles that are paired share the edge's two vertices in the
// sheets. Every triangle of a vertex has two edges at it, and at most one partner across each, so the triangles of a
// vertex fall into rows from an open edge to an open edge and into rings, and each row or ring is a vertex of the
// sheets. What can still go wrong is a ring that passes through one edge twice at both of that edge's ends: the edge
// then has its four triangles in the sheets too. The pairing of such an edge is changed for its other one, where
// that mends it
public static class BurstSheets {

    // what a split into sheets did
    public struct Report {

        public int edges;           // edges of more than two triangles
        public int acrossSolid;     // of them, paired across the wedges that are inside the solid
        public int oppositeWinding; // paired each triangle with its neighbour of opposite winding: sheets cross there
        public int repaired;        // given their other pairing after a look at their ends
        public int unpaired;        // left with triangles that have no partner, which are open edges of the sheets
        public int addedVertices;   // vertices the sheets have beyond the mesh's

        // edges of more than two triangles the sheets still have, and edges of one triangle they have beyond the mesh's
        public int nonManifoldEdges;
        public int newOpenEdges;

    }

    // a triangle at an edge: where its third vertex lies round the edge, and which way it runs along it
    private struct Around : IComparable<Around> {

        public double angle;
        public int    side;    // corner 3 t + k, whose edge runs to corner k + 1
        public bool   forward; // from the lower vertex of the edge to the higher

        public int CompareTo( Around other ) {
            return this.angle != other.angle ? this.angle.CompareTo( other.angle ) : this.side.CompareTo( other.side );
        }

    }

    // an edge of four triangles: its sides in their order round it, and its two pairings, each as two pairs of them
    private struct Four {

        public int4 sides;
        public int4 pairing;
        public int4 other;
        public bool acrossSolid;

    }

    // splits a mesh into sheets. The sides are the mesh's (BurstRemeshing.SidesJob). A vertex keeps its index for the
    // first of its rows or rings, and the others are added after the mesh's vertices, so a manifold comes back as it
    // was. Triangles that use a vertex twice are left out
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SplitJob : IJob {

        [ReadOnly] public NativeArray<float3>              positions;
        [ReadOnly] public NativeArray<float3>              normals;
        [ReadOnly] public NativeArray<BurstMesh.Triangle>  triangles;
        [ReadOnly] public NativeArray<BurstRemeshing.Side> sides;

        public NativeList<float3>             sheetPositions;
        public NativeList<float3>             sheetNormals;
        public NativeList<BurstMesh.Triangle> sheetTriangles;

        // for every vertex of the sheets, the vertex of the mesh it is
        public NativeList<int> sources;

        // one
        public NativeArray<Report> report;

        // how often the edges of four triangles are gone through again after one of them changed its pairing
        private const int passes = 8;

        public void Execute( ) {
            var report   = new Report( );
            var partners = new NativeArray<int>( math.max( this.sides.Length, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var fours    = new NativeList<Four>( Allocator.Temp );
            var around   = new NativeList<Around>( 8, Allocator.Temp );

            for( var side = 0; side < partners.Length; ++side ) {
                partners[side] = -1;
            }

            for( var start = 0; start < this.sides.Length; ) {
                var end = start + 1;
                while( end < this.sides.Length && this.sides[end].key == this.sides[start].key ) {
                    ++end;
                }

                this.pairRound( start, end, ref around, ref partners, ref fours, ref report );

                start = end;
            }

            // the ends: an edge whose two pairs are in one ring at both its ends still has four triangles
            for( var pass = 0; pass < passes; ++pass ) {
                var changed = false;

                for( var index = 0; index < fours.Length; ++index ) {
                    var four = fours[index];

                    if( !this.meetsAtBothEnds( four, partners ) ) {
                        continue;
                    }

                    set( ref partners, four.sides, four.other );

                    var swapped = new Four { sides = four.sides, pairing = four.other, other = four.pairing, acrossSolid = four.acrossSolid };

                    if( this.meetsAtBothEnds( swapped, partners ) ) {
                        set( ref partners, four.sides, four.pairing );

                        continue;
                    }

                    fours[index] = swapped;
                    changed      = true;

                    ++report.repaired;
                }

                if( !changed ) {
                    break;
                }
            }

            this.write( partners, ref report );

            this.report[0] = report;

            partners.Dispose( );
            fours.Dispose( );
            around.Dispose( );
        }

        private readonly bool degenerate( int triangle ) {
            var t = this.triangles[triangle];

            return t.a == t.b || t.b == t.c || t.a == t.c;
        }

        private readonly int vertexAt( int corner ) {
            return BurstRemeshing.vertexOf( this.triangles[corner / 3], corner % 3 );
        }

        private static int next( int corner ) {
            return ( corner / 3 * 3 ) + ( ( corner + 1 ) % 3 );
        }

        private static int previous( int corner ) {
            return ( corner / 3 * 3 ) + ( ( corner + 2 ) % 3 );
        }

        // pairs the triangles of one edge, whose sides are a run of the sorted sides
        private void pairRound( int start, int end, ref NativeList<Around> around, ref NativeArray<int> partners, ref NativeList<Four> fours, ref Report report ) {
            var lower = ( int )( this.sides[start].key >> 32 );
            var upper = ( int )( this.sides[start].key & 0xffffffff );

            around.Clear( );

            for( var index = start; index < end; ++index ) {
                var side = this.sides[index].corner;

                if( !this.degenerate( side / 3 ) ) {
                    around.Add( new Around { side = side, forward = this.vertexAt( side ) == lower } );
                }
            }

            if( around.Length < 2 ) {
                return;
            }
            if( around.Length == 2 ) {
                partners[around[0].side] = around[1].side;
                partners[around[1].side] = around[0].side;

                return;
            }

            ++report.edges;

            // where each triangle lies round the edge, seen along it from its lower vertex to its higher
            var a         = ( double3 )this.positions[lower];
            var direction = ( double3 )this.positions[upper] - a;
            var length    = math.length( direction );

            if( !( length > 0.0 ) ) {
                ++report.unpaired;

                return;
            }

            direction /= length;

            var x = math.normalize( math.cross( math.abs( direction.x ) < 0.9 ? new double3( 1.0, 0.0, 0.0 ) : new double3( 0.0, 1.0, 0.0 ), direction ) );
            var y = math.cross( direction, x );

            for( var index = 0; index < around.Length; ++index ) {
                var item  = around[index];
                var third = ( double3 )this.positions[this.vertexAt( previous( item.side ) )] - a;

                item.angle = math.atan2( math.dot( third, y ), math.dot( third, x ) );

                around[index] = item;
            }

            around.Sort( );

            var count      = around.Length;
            var alternates = count % 2 == 0;

            for( var index = 0; index < count && alternates; ++index ) {
                alternates = around[index].forward != around[( index + 1 ) % count].forward;
            }

            if( alternates ) {
                // a triangle that runs the edge backwards faces against the way the angle grows, so the solid is
                // between it and the next
                var first = around[0].forward ? 1 : 0;

                for( var index = first; index < count; index += 2 ) {
                    partners[around[index].side]                 = around[( index + 1 ) % count].side;
                    partners[around[( index + 1 ) % count].side] = around[index].side;
                }

                ++report.acrossSolid;

                if( count == 4 ) {
                    fours.Add( new Four {
                        sides       = new int4( around[0].side, around[1].side, around[2].side, around[3].side ),
                        pairing     = new int4( first, first + 1, first + 2, ( first + 3 ) % 4 ),
                        other       = new int4( 1 - first, 2 - first, 3 - first, first == 0 ? 0 : 3 ),
                        acrossSolid = true
                    } );
                }

                return;
            }

            if( count == 4 ) {
                // two sheets cross: the windings run a, b, b, a from some triangle on, and each triangle has one
                // neighbour that runs the other way. The other pairing that keeps to the windings takes each triangle
                // with the one across the edge from it
                for( var first = 0; first < 4; ++first ) {
                    var i1 = ( first + 1 ) % 4;
                    var i2 = ( first + 2 ) % 4;
                    var i3 = ( first + 3 ) % 4;

                    if( around[first].forward == around[i1].forward || around[i1].forward != around[i2].forward || around[i2].forward == around[i3].forward ) {
                        continue;
                    }

                    var four = new Four {
                        sides   = new int4( around[0].side, around[1].side, around[2].side, around[3].side ),
                        pairing = new int4( first, i1, i2, i3 ),
                        other   = new int4( first, i2, i1, i3 )
                    };

                    set( ref partners, four.sides, four.pairing );

                    fours.Add( four );

                    ++report.oppositeWinding;

                    return;
                }
            }

            ++report.unpaired;
        }

        // pairs the sides of an edge of four triangles: the first two of the pairing with each other, and the last two
        private static void set( ref NativeArray<int> partners, int4 sides, int4 pairing ) {
            partners[sides[pairing.x]] = sides[pairing.y];
            partners[sides[pairing.y]] = sides[pairing.x];
            partners[sides[pairing.z]] = sides[pairing.w];
            partners[sides[pairing.w]] = sides[pairing.z];
        }

        // whether the two pairs of an edge are in one ring or row of triangles at both its ends
        private readonly bool meetsAtBothEnds( in Four four, NativeArray<int> partners ) {
            var first  = four.sides[four.pairing.x];
            var mate   = four.sides[four.pairing.y];
            var corner = this.vertexAt( first );
            var other  = this.vertexAt( next( first ) );

            return this.meets( first, mate, corner, four.sides, partners ) && this.meets( first, mate, other, four.sides, partners );
        }

        // whether the pair of two sides of an edge is, at a vertex of the edge, in one ring or row with the edge's other
        // pair: going round the vertex from either of the two, away from the edge, comes to the edge again at a
        // triangle of the other pair
        private readonly bool meets( int first, int mate, int vertex, int4 sides, NativeArray<int> partners ) {
            var arrived = this.walk( first, vertex, sides, partners );

            if( arrived == -1 ) {
                arrived = this.walk( mate, vertex, sides, partners );
            }

            return arrived != -1 && arrived != first && arrived != mate;
        }

        // goes round a vertex from a side of an edge at it, through the triangle and on from triangle to triangle
        // across their partners, until a side of the same edge is come to, which is given, or a side without a
        // partner, which gives -1
        private readonly int walk( int side, int vertex, int4 sides, NativeArray<int> partners ) {
            for( var steps = 0; steps <= this.triangles.Length; ++steps ) {
                // the triangle's other side at the vertex: the side before the one that starts at the vertex, or that one
                var leaving = this.vertexAt( side ) == vertex ? previous( side ) : next( side );

                if( math.any( sides == leaving ) ) {
                    return leaving;
                }

                side = partners[leaving];

                if( side == -1 ) {
                    return -1;
                }
            }
            return -1;
        }

        // the sheets: the corners of two triangles that are partners across an edge are one vertex at both its ends
        private void write( NativeArray<int> partners, ref Report report ) {
            var parents = new NativeArray<int>( math.max( partners.Length, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var ids     = new NativeArray<int>( math.max( partners.Length, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var taken   = new NativeBitArray( math.max( this.positions.Length, 1 ), Allocator.Temp );

            for( var corner = 0; corner < parents.Length; ++corner ) {
                parents[corner] = corner;
                ids[corner]     = -1;
            }

            for( var side = 0; side < partners.Length; ++side ) {
                var partner = partners[side];

                if( partner <= side ) {
                    continue;
                }

                // the side starts at one vertex of the edge and its partner at the same one or at the other
                var sameWay = this.vertexAt( side ) == this.vertexAt( partner );

                union( parents, side, sameWay ? partner : next( partner ) );
                union( parents, next( side ), sameWay ? next( partner ) : partner );
            }

            this.sheetPositions.Clear( );
            this.sheetNormals.Clear( );
            this.sheetTriangles.Clear( );
            this.sources.Clear( );

            this.sheetPositions.AddRange( this.positions );
            this.sheetNormals.AddRange( this.normals );

            for( var vertex = 0; vertex < this.positions.Length; ++vertex ) {
                this.sources.Add( vertex );
            }

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                if( this.degenerate( triangle ) ) {
                    continue;
                }

                var corners = new int3( );

                for( var k = 0; k < 3; ++k ) {
                    var root = find( parents, ( triangle * 3 ) + k );

                    if( ids[root] == -1 ) {
                        var vertex = this.vertexAt( ( triangle * 3 ) + k );

                        if( !taken.IsSet( vertex ) ) {
                            taken.Set( vertex, true );

                            ids[root] = vertex;
                        }
                        else {
                            ids[root] = this.sheetPositions.Length;

                            this.sheetPositions.Add( this.positions[vertex] );
                            this.sheetNormals.Add( this.normals[vertex] );
                            this.sources.Add( vertex );

                            ++report.addedVertices;
                        }
                    }

                    corners[k] = ids[root];
                }

                this.sheetTriangles.Add( new BurstMesh.Triangle( corners.x, corners.y, corners.z, this.triangles[triangle].subMeshIndex ) );
            }

            parents.Dispose( );
            ids.Dispose( );
            taken.Dispose( );
        }

        private static int find( NativeArray<int> parents, int item ) {
            while( parents[item] != item ) {
                parents[item] = parents[parents[item]];
                item          = parents[item];
            }
            return item;
        }

        // the lower corner is the root, so that a vertex of the sheets is found at its first corner
        private static void union( NativeArray<int> parents, int item, int other ) {
            var a = find( parents, item );
            var b = find( parents, other );

            if( a < b ) {
                parents[b] = a;
            }
            else if( b < a ) {
                parents[a] = b;
            }
        }

    }

    // a mesh split into sheets, into lists that are the caller's, and what the split did and left
    public static Report split(
        NativeArray<float3>             positions,
        NativeArray<float3>             normals,
        NativeArray<BurstMesh.Triangle> triangles,
        NativeList<float3>              sheetPositions,
        NativeList<float3>              sheetNormals,
        NativeList<BurstMesh.Triangle>  sheetTriangles,
        NativeList<int>                 sources
    ) {
        using var sides  = new NativeArray<BurstRemeshing.Side>( triangles.Length * 3, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var report = new NativeArray<Report>( 1, Allocator.TempJob );
        using var none   = new NativeArray<byte>( 0, Allocator.TempJob );

        new BurstRemeshing.SidesJob { triangles = triangles, sides = sides }.Run( );

        new SplitJob {
            positions      = positions,
            normals        = normals,
            triangles      = triangles,
            sides          = sides,
            sheetPositions = sheetPositions,
            sheetNormals   = sheetNormals,
            sheetTriangles = sheetTriangles,
            sources        = sources,
            report         = report
        }.Run( );

        var result = report[0];

        if( result.edges > 0 ) {
            BurstRemeshing.count( triangles, none, out var before, out _ );
            BurstRemeshing.count( sheetTriangles.AsArray( ), none, out var after, out _ );

            result.nonManifoldEdges = after.nonManifoldEdges;
            result.newOpenEdges     = math.max( 0, after.openEdges - before.openEdges );
        }

        return result;
    }

}

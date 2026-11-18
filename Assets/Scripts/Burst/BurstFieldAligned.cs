using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using Sharp    = BurstMeshSimplifier.Sharp;
using Symmetry = BurstFields.Symmetry;

// Field Aligned (Documents/Plans/Remeshing.md, step 5): the mesh read off the two fields of Instant Field-Aligned Meshes
// (Jakob, Tarini, Panozzo and Sorkine-Hornung, section 3.8), which BurstFields solves on the mesh a remesh is given.
// Two neighbours of the mesh the fields are on whose lattice points are one point become one vertex, nearest first,
// and two whose lattice points are a step apart are joined by an edge; the faces are walked round the edges, cut down
// to the faces asked for, and smoothed back onto the mesh given.
//
// Where the paper speaks it is followed, and where it is silent its authors' code is (github.com/wjakob/instant-meshes,
// BSD 3-clause; finding 11). From the code, in this file: the snapping of a vertex near an edge onto it and of two
// vertices near each other into one, the diagonals taken out of quads, the walk of faces of three to eight sides,
// triangles first, a larger face cut, holes of up to six sides filled, and two rounds of smoothing. This project's own:
// every vertex put back on the input before its faces are found, faces of triangles and pentagons kept as polygons where
// the code makes fans of them, quads only by splitting each face round its middle, no face from a walk that comes round
// to a vertex twice or round the rim of an open mesh, materials by area, and the normals out as Isotropic gives them
public static class BurstFieldAligned {

    // the faces of the mesh: triangles; quads with triangles and pentagons where rows of faces begin and end; or quads
    // only, from the faces of the second at twice the length, each split into quads round its middle (figure 1)
    public enum Faces : byte {
        Triangles    = 0,
        QuadDominant = 1,
        Quads        = 2
    }

    // the settings of a remesh, in the units of the mesh
    public struct Settings {

        public Faces  faces;
        public double length;        // the length the edges are to have
        public double featureCosine; // of the Feature Angle (BurstFeatures)
        public double featureLength; // the length the features by angle are found at (BurstFeatures), or 0 for each edge's own
        public bool   freeMaterials; // lines between materials not held, for measuring what holding them does
        public int    mostVertices;  // of the mesh the fields are on, or 0 for no limit

    }

    // what a remesh did, and how long each stage took
    public struct Report {

        public BurstFields.Report fields;

        // the graph read off the fields (the paper's G')
        public int  unitLinks;     // edges of the mesh the fields are on whose lattice points are a step apart
        public int  zeroLinks;     // and whose lattice points are one
        public int  collapses;     // of the second, made
        public int  conflicts;     // of the second, not made: the two were already joined by an edge of the graph
        public int  vertices;      // of the graph
        public int  spurious;      // left out of it: too few vertices of the mesh agreed on them (section 3.8)
        public int  snapped;       // vertices moved onto an edge they were near (the code)
        public int  merged;        // vertices merged into a neighbour they were near (the code)
        public int  diagonals;     // edges taken out across quads (the code)
        public bool unsettled;     // the snapping stopped at its most rounds
        public int  offInput;      // vertices with no point of the input within reach, which stay off it

        // the faces walked round the graph's edges, by their sides, and the holes filled after
        public int walkedTriangles;
        public int walkedQuads;
        public int walkedPentagons;
        public int walkedLarger;
        public int holes;
        public int unfilledHoles;  // of more than six sides, or that come round to a vertex twice

        public int moves;          // vertices smoothing moved, each time one was
        public int unplaced;       // vertices smoothing left where they were: no point of the input within reach
        public int unmapped;       // faces no area of the input mapped into, whose material is had from the input at their middle

        public double extractionMilliseconds; // the graph read off the fields
        public double facesMilliseconds;
        public double smoothMilliseconds;     // with the materials
        public double outputMilliseconds;

    }

    // ---- the graph ----

    // the graph read off the fields, which the snapping changes: vertices with their positions and normals, whether
    // smoothing leaves them where they are, whether they were merged into another, and an edge list for each vertex. An
    // entry of a list is a neighbour, or -1 once the edge is taken out, and the next entry of the list or -1. A vertex is
    // in a list once at most
    public struct Graph : IDisposable {

        public NativeList<double3> positions;
        public NativeList<double3> normals;
        public NativeList<bool>    fixedVertices; // on a crease or a held line (the code's crease_out)
        public NativeList<int>     into;          // the vertex it was merged into, or -1
        public NativeList<int>     members;       // the vertices of the mesh the fields are on it was made of

        public NativeList<int>  heads;
        public NativeList<int2> entries;

        public Graph( Allocator allocator ) {
            this.positions     = new NativeList<double3>( allocator );
            this.normals       = new NativeList<double3>( allocator );
            this.fixedVertices = new NativeList<bool>( allocator );
            this.into          = new NativeList<int>( allocator );
            this.members       = new NativeList<int>( allocator );
            this.heads         = new NativeList<int>( allocator );
            this.entries       = new NativeList<int2>( allocator );
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.fixedVertices.Dispose( );
            this.into.Dispose( );
            this.members.Dispose( );
            this.heads.Dispose( );
            this.entries.Dispose( );
        }

        public readonly int vertexCount {
            get { return this.positions.Length; }
        }

        public int add( double3 position, double3 normal, bool held, int members ) {
            this.positions.Add( position );
            this.normals.Add( normal );
            this.fixedVertices.Add( held );
            this.into.Add( -1 );
            this.members.Add( members );
            this.heads.Add( -1 );

            return this.positions.Length - 1;
        }

        public readonly bool removed( int vertex ) {
            return this.into[vertex] != -1;
        }

        // the vertex a vertex ended in, after the merges
        public readonly int resolved( int vertex ) {
            while( vertex != -1 && this.into[vertex] != -1 ) {
                vertex = this.into[vertex];
            }
            return vertex;
        }

        public readonly bool has( int a, int b ) {
            for( var entry = this.heads[a]; entry != -1; entry = this.entries[entry].y ) {
                if( this.entries[entry].x == b ) {
                    return true;
                }
            }
            return false;
        }

        // joins two vertices that are not joined yet
        public void join( int a, int b ) {
            this.entries.Add( new int2( b, this.heads[a] ) );
            this.heads[a] = this.entries.Length - 1;

            this.entries.Add( new int2( a, this.heads[b] ) );
            this.heads[b] = this.entries.Length - 1;
        }

        public void part( int a, int b ) {
            this.cut( a, b );
            this.cut( b, a );
        }

        private void cut( int from, int to ) {
            for( var entry = this.heads[from]; entry != -1; entry = this.entries[entry].y ) {
                if( this.entries[entry].x == to ) {
                    this.entries[entry] = new int2( -1, this.entries[entry].y );
                    return;
                }
            }
        }

        // how many neighbours two vertices have in common, and the lengths of the edges from both to them
        public readonly int common( int a, int b, out double lengths ) {
            var count = 0;

            lengths = 0.0;

            for( var entry = this.heads[a]; entry != -1; entry = this.entries[entry].y ) {
                var k = this.entries[entry].x;

                if( k < 0 || k == b || !this.has( b, k ) ) {
                    continue;
                }

                ++count;
                lengths += math.distance( this.positions[k], this.positions[a] ) + math.distance( this.positions[k], this.positions[b] );
            }

            return count;
        }

    }

    // ---- the extraction ----

    // what an edge of the mesh the fields are on says of its two lattice points (section 3.8): nothing, that they are
    // one point, or that they are a step apart
    public const byte Apart = 0;
    public const byte Same  = 1;
    public const byte Step  = 2;

    // whether lattice steps from one point to another are a step to a neighbour: along an axis for squares, which leaves
    // out the diagonal of a square as the code does, and for triangles along either axis or along the third side of the
    // triangle they make. The code takes a step of one along both axes of a triangular lattice too, which is a point the
    // height of two triangles away (finding 11)
    public static bool isStep( int2 steps, Symmetry symmetry ) {
        var size = math.abs( steps );

        if( size.x + size.y == 1 ) {
            return true;
        }

        return symmetry == Symmetry.Triangles && size.x == 1 && steps.x == -steps.y;
    }

    // every edge of level 0 from a vertex to a higher one, as an edge with lattice points that are one, a step apart,
    // or neither, and how far apart the two points nearest each other are: the directions are turned to match, the
    // lattice points of both that are nearest each other are found round the point between the two, and the steps from
    // one vertex's lattice point to the other's are what is left (the code's extract_graph, extrinsic)
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct ClassifyJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3>           positions;
        [ReadOnly] public NativeArray<float3>           normals;
        [ReadOnly] public NativeArray<float3>           orientations;
        [ReadOnly] public NativeArray<float3>           lattice;
        [ReadOnly] public NativeArray<int>              linkStarts;
        [ReadOnly] public NativeArray<BurstFields.Link> links;

        // one for every link
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<byte>  kinds;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float> distances;

        public float    rho;
        public Symmetry symmetry;

        public void Execute( int vertex ) {
            for( var link = this.linkStarts[vertex]; link < this.linkStarts[vertex + 1]; ++link ) {
                var other = this.links[link].vertex;

                this.kinds[link]     = Apart;
                this.distances[link] = 0.0f;

                if( other <= vertex ) {
                    continue;
                }

                BurstFields.matchOrientations( this.orientations[vertex], this.normals[vertex], this.orientations[other], this.normals[other], this.symmetry, out var a, out var b );

                BurstFields.matchPositions(
                    this.positions[vertex], this.normals[vertex], a, this.lattice[vertex],
                    this.positions[other],  this.normals[other],  b, this.lattice[other],
                    this.rho, this.symmetry, out var own, out var theirs, out var near, out var far
                );

                var steps = own - theirs;

                this.kinds[link]     = math.all( steps == 0 ) ? Same : isStep( steps, this.symmetry ) ? Step : Apart;
                this.distances[link] = math.distancesq( near, far );
            }
        }

    }

    private struct Merge : IComparable<Merge> {

        public float distance;
        public int   a;
        public int   b;

        public int CompareTo( Merge other ) {
            if( this.distance != other.distance ) {
                return this.distance.CompareTo( other.distance );
            }
            return this.a != other.a ? this.a.CompareTo( other.a ) : this.b.CompareTo( other.b );
        }

    }

    // the graph read off the fields (section 3.8). The edges whose lattice points are a step apart join their two
    // vertices; then those whose lattice points are one merge them, nearest first, where the two are not joined by an
    // edge yet, which says the fields disagree whether they are one and keeps them apart. Each group of merged vertices
    // that has an edge is a vertex of the graph, unless fewer than a tenth of the mean number of vertices merged into it,
    // where it is spurious. It is at the mean of their lattice points, weighted by how near each is to its own vertex,
    // with their normals so weighted; and smoothing leaves it where it is where one of them is on a crease or held
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct ClusterJob : IJob {

        [ReadOnly] public NativeArray<float3>           positions;
        [ReadOnly] public NativeArray<float3>           normals;
        [ReadOnly] public NativeArray<float3>           lattice;
        [ReadOnly] public NativeArray<byte>             held;
        [ReadOnly] public NativeArray<int>              linkStarts;
        [ReadOnly] public NativeArray<BurstFields.Link> links;
        [ReadOnly] public NativeArray<byte>             kinds;
        [ReadOnly] public NativeArray<float>            distances;

        // the mesh the fields are on, for its creases
        [ReadOnly] public NativeArray<int>   firstEdges;
        [ReadOnly] public NativeArray<int>   nextEdges;
        [ReadOnly] public NativeArray<Sharp> edgeSharp;
        [ReadOnly] public NativeArray<int>   edgeLines;

        public Graph graph;

        // for every vertex of level 0, the vertex of the graph it is in, or -1
        [WriteOnly] public NativeArray<int> clusters;

        // six: links a step apart, links of one point, merges, conflicts, vertices of the graph, spurious vertices
        public NativeArray<int> counts;

        public int   vertexCount; // of level 0
        public float rho;

        public void Execute( ) {
            var count   = this.vertexCount;
            var parents = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var members = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var heads   = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var tails   = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var lengths = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp );
            var entries = new NativeList<int2>( count * 2, Allocator.Temp );
            var merges  = new NativeList<Merge>( count, Allocator.Temp );

            for( var vertex = 0; vertex < count; ++vertex ) {
                parents[vertex] = vertex;
                members[vertex] = 1;
                heads[vertex]   = -1;
                tails[vertex]   = -1;
            }

            for( var vertex = 0; vertex < count; ++vertex ) {
                for( var link = this.linkStarts[vertex]; link < this.linkStarts[vertex + 1]; ++link ) {
                    var other = this.links[link].vertex;

                    if( this.kinds[link] == Step ) {
                        append( entries, heads, tails, lengths, vertex, other );
                        append( entries, heads, tails, lengths, other, vertex );

                        ++this.counts[0];
                    }
                    else if( this.kinds[link] == Same ) {
                        merges.Add( new Merge { distance = this.distances[link], a = vertex, b = other } );

                        ++this.counts[1];
                    }
                }
            }

            merges.Sort( );

            for( var index = 0; index < merges.Length; ++index ) {
                var a = find( parents, merges[index].a );
                var b = find( parents, merges[index].b );

                if( a == b ) {
                    continue;
                }

                // joined already, which the shorter list says as well as the longer
                var shorter = lengths[a] <= lengths[b] ? a : b;
                var longer  = shorter == a ? b : a;
                var joined  = false;

                for( var entry = heads[shorter]; entry != -1 && !joined; entry = entries[entry].y ) {
                    joined = find( parents, entries[entry].x ) == longer;
                }

                if( joined ) {
                    ++this.counts[3];
                    continue;
                }

                var root  = math.min( a, b );
                var child = math.max( a, b );

                parents[child]  = root;
                members[root]  += members[child];
                lengths[root]  += lengths[child];

                if( heads[root] == -1 ) {
                    heads[root] = heads[child];
                    tails[root] = tails[child];
                }
                else if( heads[child] != -1 ) {
                    entries[tails[root]] = new int2( entries[tails[root]].x, heads[child] );
                    tails[root]          = tails[child];
                }

                ++this.counts[2];
            }

            // the groups with an edge, and the mean of their sizes
            var total  = 0.0;
            var groups = 0;

            for( var vertex = 0; vertex < count; ++vertex ) {
                if( find( parents, vertex ) == vertex && heads[vertex] != -1 ) {
                    total += members[vertex];
                    ++groups;
                }
            }

            var least = groups > 0 ? total / groups / 10.0 : 0.0;
            var ids   = new NativeArray<int>( math.max( count, 1 ), Allocator.Temp );

            for( var vertex = 0; vertex < count; ++vertex ) {
                ids[vertex] = -1;

                if( find( parents, vertex ) != vertex || heads[vertex] == -1 ) {
                    continue;
                }

                if( members[vertex] < least ) {
                    ++this.counts[5];
                    continue;
                }

                ids[vertex] = this.graph.add( double3.zero, double3.zero, false, members[vertex] );
            }

            // where each is: the mean of its lattice points, each weighted by exp( -9 |p - v|^2 / rho^2 ), the paper's
            // beta of rho / 3
            var weights  = new NativeArray<double>( math.max( this.graph.vertexCount, 1 ), Allocator.Temp );
            var inverse  = 1.0 / ( ( double )this.rho * this.rho );

            for( var vertex = 0; vertex < count; ++vertex ) {
                var id = ids[find( parents, vertex )];

                this.clusters[vertex] = id;

                if( id == -1 ) {
                    continue;
                }

                var point  = ( double3 )this.lattice[vertex];
                var weight = math.exp( -9.0 * math.distancesq( point, ( double3 )this.positions[vertex] ) * inverse );

                this.graph.positions[id] += point * weight;
                this.graph.normals[id]   += ( double3 )this.normals[vertex] * weight;
                weights[id]              += weight;

                if( this.held[vertex] != 0 || this.onCrease( vertex ) ) {
                    this.graph.fixedVertices[id] = true;
                }
            }

            for( var id = 0; id < this.graph.vertexCount; ++id ) {
                if( weights[id] > 0.0 ) {
                    this.graph.positions[id] /= weights[id];
                }

                this.graph.normals[id] = math.normalizesafe( this.graph.normals[id] );
            }

            // the edges, between groups that are both vertices
            for( var vertex = 0; vertex < count; ++vertex ) {
                var id = ids[vertex];

                if( id == -1 ) {
                    continue;
                }

                for( var entry = heads[vertex]; entry != -1; entry = entries[entry].y ) {
                    var other = ids[find( parents, entries[entry].x )];

                    if( other != -1 && other != id && !this.graph.has( id, other ) ) {
                        this.graph.join( id, other );
                    }
                }
            }

            this.counts[4] = this.graph.vertexCount;

            parents.Dispose( );
            members.Dispose( );
            heads.Dispose( );
            tails.Dispose( );
            lengths.Dispose( );
            entries.Dispose( );
            merges.Dispose( );
            ids.Dispose( );
            weights.Dispose( );
        }

        private readonly bool onCrease( int vertex ) {
            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                if( this.edgeSharp[end >> 1] == Sharp.None && this.edgeLines[end >> 1] != -1 ) {
                    return true;
                }
            }
            return false;
        }

        private static void append( NativeList<int2> entries, NativeArray<int> heads, NativeArray<int> tails, NativeArray<int> lengths, int vertex, int neighbour ) {
            entries.Add( new int2( neighbour, -1 ) );

            var entry = entries.Length - 1;

            if( heads[vertex] == -1 ) {
                heads[vertex] = entry;
            }
            else {
                entries[tails[vertex]] = new int2( entries[tails[vertex]].x, entry );
            }

            tails[vertex] = entry;
            ++lengths[vertex];
        }

        private static int find( NativeArray<int> parents, int item ) {
            while( parents[item] != item ) {
                parents[item] = parents[parents[item]];
                item          = parents[item];
            }
            return item;
        }

    }

    // how near a vertex is to be to an edge, or to a neighbour, to be snapped onto it, in edge lengths (the code)
    public const double snapWithin = 0.3;

    // the most rounds of snapping. The code goes on until nothing changes, which every mesh measured does within a few;
    // this is only there so that a graph that never settles cannot hang the editor
    public const int mostSnapRounds = 64;

    private struct Snap : IComparable<Snap> {

        public double height;
        public int    i;
        public int    j;
        public int    k;

        public int CompareTo( Snap other ) {
            if( this.height != other.height ) {
                return this.height.CompareTo( other.height );
            }
            if( this.i != other.i ) {
                return this.i.CompareTo( other.i );
            }
            return this.j != other.j ? this.j.CompareTo( other.j ) : this.k.CompareTo( other.k );
        }

    }

    private struct Diagonal : IComparable<Diagonal> {

        public double score;
        public int    i;
        public int    j;

        public int CompareTo( Diagonal other ) {
            if( this.score != other.score ) {
                return this.score.CompareTo( other.score );
            }
            return this.i != other.i ? this.i.CompareTo( other.i ) : this.j.CompareTo( other.j );
        }

    }

    // the height of a triangle of sides a, b and c over side a (Heron)
    public static double heightOver( double a, double b, double c ) {
        var s       = 0.5 * ( a + b + c );
        var squared = s * ( s - a ) * ( s - b ) * ( s - c );

        return a > 0.0 ? 2.0 * math.sqrt( math.max( squared, 0.0 ) ) / a : 0.0;
    }

    // the graph tidied as the code tidies it, which the paper does not mention, until nothing changes:
    // - a vertex i with neighbours j and k, where j and k are joined and the edge from j to k is the longest of the
    //   three, and i is nearer than 0.3 lengths to that edge: merged into j or k where it is that near to either, and
    //   otherwise moved to the middle of the edge, which it then splits. The nearest first;
    // - for quads, an edge with two neighbours in common, which is the diagonal of a quad split into two triangles, taken
    //   out, the nearest to the length a diagonal of its four sides would have first
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SnapJob : IJob {

        public Graph graph;

        // four: snapped onto an edge, merged into a neighbour, diagonals taken out, and 1 where it did not settle
        public NativeArray<int> counts;

        public double rho;
        public bool   quads;

        public void Execute( ) {
            var limit     = snapWithin * this.rho;
            var snaps     = new NativeList<Snap>( Allocator.Temp );
            var diagonals = new NativeList<Diagonal>( Allocator.Temp );
            var rounds    = 0;

            bool changed;

            do {
                changed = false;

                bool snapped;

                do {
                    snapped = false;

                    if( ++rounds > mostSnapRounds ) {
                        this.counts[3] = 1;
                        break;
                    }

                    this.snapsOf( limit, snaps );

                    for( var index = 0; index < snaps.Length; ++index ) {
                        if( this.snap( snaps[index], limit ) ) {
                            snapped = true;
                            changed = true;
                        }
                    }
                } while( snapped );

                if( this.counts[3] != 0 ) {
                    break;
                }

                if( this.quads && this.takeOutDiagonals( diagonals ) ) {
                    changed = true;
                }
            } while( changed );

            snaps.Dispose( );
            diagonals.Dispose( );
        }

        private readonly double3 at( int vertex ) {
            return this.graph.positions[vertex];
        }

        private readonly void snapsOf( double limit, NativeList<Snap> snaps ) {
            snaps.Clear( );

            for( var i = 0; i < this.graph.vertexCount; ++i ) {
                if( this.graph.removed( i ) ) {
                    continue;
                }

                for( var first = this.graph.heads[i]; first != -1; first = this.graph.entries[first].y ) {
                    var j = this.graph.entries[first].x;

                    if( j < 0 ) {
                        continue;
                    }

                    for( var second = this.graph.heads[j]; second != -1; second = this.graph.entries[second].y ) {
                        var k = this.graph.entries[second].x;

                        if( k < 0 || k == i ) {
                            continue;
                        }

                        var a = math.distance( this.at( j ), this.at( k ) );
                        var b = math.distance( this.at( i ), this.at( j ) );
                        var c = math.distance( this.at( i ), this.at( k ) );

                        if( !( a > math.max( b, c ) ) ) {
                            continue;
                        }

                        var height = heightOver( a, b, c );

                        if( height < limit ) {
                            snaps.Add( new Snap { height = height, i = i, j = j, k = k } );
                        }
                    }
                }
            }

            snaps.Sort( );
        }

        // one snap, where the graph is still as it was when it was found
        private bool snap( Snap candidate, double limit ) {
            var i = candidate.i;
            var j = candidate.j;
            var k = candidate.k;

            if( this.graph.removed( i ) || this.graph.removed( j ) || this.graph.removed( k ) || !this.graph.has( i, j ) || !this.graph.has( j, k ) ) {
                return false;
            }

            var a = math.distance( this.at( j ), this.at( k ) );
            var b = math.distance( this.at( i ), this.at( j ) );
            var c = math.distance( this.at( i ), this.at( k ) );

            if( heightOver( a, b, c ) != candidate.height ) {
                return false;
            }

            if( b < limit || c < limit ) {
                var merged = b < limit ? j : k;

                this.graph.positions[i]      = ( this.at( i ) + this.at( merged ) ) * 0.5;
                this.graph.normals[i]        = math.normalizesafe( this.graph.normals[i] + this.graph.normals[merged] );
                this.graph.fixedVertices[i] |= this.graph.fixedVertices[merged];
                this.graph.members[i]       += this.graph.members[merged];

                for( var entry = this.graph.heads[merged]; entry != -1; entry = this.graph.entries[entry].y ) {
                    var neighbour = this.graph.entries[entry].x;

                    if( neighbour < 0 || neighbour == i ) {
                        continue;
                    }

                    this.graph.part( merged, neighbour );

                    if( !this.graph.has( i, neighbour ) ) {
                        this.graph.join( i, neighbour );
                    }
                }

                this.graph.part( merged, i );
                this.graph.into[merged] = i;

                ++this.counts[1];
            }
            else {
                var across = this.graph.has( k, i );

                this.graph.positions[i] = ( this.at( j ) + this.at( k ) ) * 0.5;
                this.graph.normals[i]   = math.normalizesafe( this.graph.normals[j] + this.graph.normals[k] );

                if( this.graph.fixedVertices[j] && this.graph.fixedVertices[k] ) {
                    this.graph.fixedVertices[i] = true;
                }

                this.graph.part( j, k );

                if( !across ) {
                    this.graph.join( i, k );
                }

                ++this.counts[0];
            }

            return true;
        }

        private bool takeOutDiagonals( NativeList<Diagonal> diagonals ) {
            diagonals.Clear( );

            for( var i = 0; i < this.graph.vertexCount; ++i ) {
                for( var entry = this.graph.heads[i]; entry != -1; entry = this.graph.entries[entry].y ) {
                    var j = this.graph.entries[entry].x;

                    if( j <= i || this.graph.common( i, j, out var lengths ) != 2 ) {
                        continue;
                    }

                    // the four sides of the quad, as a square's, give its diagonal
                    var expected = lengths / 4.0 * math.SQRT2;
                    var diagonal = math.distance( this.at( i ), this.at( j ) );
                    var shorter  = math.min( diagonal, expected );

                    diagonals.Add( new Diagonal { score = shorter > 0.0 ? math.abs( diagonal - expected ) / shorter : double.MaxValue, i = i, j = j } );
                }
            }

            diagonals.Sort( );

            var any = false;

            for( var index = 0; index < diagonals.Length; ++index ) {
                var i = diagonals[index].i;
                var j = diagonals[index].j;

                if( this.graph.has( i, j ) && this.graph.common( i, j, out _ ) == 2 ) {
                    this.graph.part( i, j );

                    ++this.counts[2];

                    any = true;
                }
            }

            return any;
        }

    }

    // every vertex of the graph put on the nearest point of the input that faces its way, within reach, before its
    // faces are found. A vertex is at the mean of lattice points, each in its own vertex's tangent plane, which on a
    // curved surface is off it and at a crease can be beyond the crease's end: a vertex there folds its faces over. The
    // code puts vertices back on the input only when it smooths them, and never those on creases. One with no point of
    // the input within reach stays where it is. The vertices from the first given on are placed, so that those a split
    // into quads adds are placed too
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct PlaceJob : IJobParallelFor {

        [NativeDisableParallelForRestriction] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<double3> normals;
        [ReadOnly] public NativeArray<int>     into;

        public SurfaceGrid grid;
        public double      reach;
        public int         first;

        // one for every vertex placed: whether it was
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<bool> placed;

        public void Execute( int index ) {
            var vertex = this.first + index;

            this.placed[index] = false;

            if( this.into[vertex] != -1 ) {
                return;
            }

            if( this.grid.nearestOnSurface( this.positions[vertex], SurfaceGrid.anySubMesh, this.normals[vertex], this.reach, out var nearest, out _ ) ) {
                this.positions[vertex] = nearest;
                this.placed[index]     = true;
            }
        }

    }

    // ---- the faces ----

    // two directions of a tangent plane at right angles, from its normal, s x t = n (BurstFields.tangentsOf)
    public static void tangentsOf( double3 n, out double3 s, out double3 t ) {
        t = math.abs( n.x ) > math.abs( n.y )
            ? new double3( n.z, 0.0, -n.x ) / math.sqrt( ( n.x * n.x ) + ( n.z * n.z ) )
            : new double3( 0.0, n.z, -n.y ) / math.sqrt( ( n.y * n.y ) + ( n.z * n.z ) );
        s = math.cross( t, n );
    }

    // the angle of a point round a vertex, in its tangent plane
    public static double angleOf( double3 point, double3 centre, double3 s, double3 t ) {
        var direction = point - centre;

        return math.atan2( math.dot( t, direction ), math.dot( s, direction ) );
    }

    private struct Around : IComparable<Around> {

        public double angle;
        public int    vertex;

        // the larger angle first, as the code sorts them, which walks every face the way round that its normal says
        public int CompareTo( Around other ) {
            return this.angle != other.angle ? other.angle.CompareTo( this.angle ) : this.vertex.CompareTo( other.vertex );
        }

    }

    // the faces of the graph, as polygons, each the walked face it is part of: the neighbours of every vertex sorted
    // round it in its tangent plane, the faces walked round them, three sides first and then up to eight, each edge from
    // a vertex to its neighbour in one face at most, and the holes the faces leave filled where they have up to six
    // sides. A walk goes along an edge, and at its end turns to the next neighbour round, which keeps the face on one side
    // of every edge it goes along. Then each face is cut down to the faces asked for, and for quads only split round its
    // middle
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct FacesJob : IJob {

        public Graph graph;

        // every vertex's neighbours sorted round it, from rimStarts[v] to rimStarts[v + 1], with their angles, and the
        // walked face that each edge from the vertex to the neighbour is in, or -1
        public NativeList<int>    rimStarts;
        public NativeList<int>    rim;
        public NativeList<double> rimAngles;
        public NativeList<int>    rimFaces;

        // the polygons, from polygonStarts[p] to polygonStarts[p + 1], and the walked face each is part of
        public NativeList<int> polygonStarts;
        public NativeList<int> polygonCorners;
        public NativeList<int> polygonFaces;

        // eight: faces walked of three, four, five and more sides, holes filled and left, walked faces in all
        public NativeArray<int> counts;

        public Faces faces;

        public void Execute( ) {
            var owners = new NativeList<int>( Allocator.Temp );
            var around = new NativeList<Around>( Allocator.Temp );

            this.rimStarts.Clear( );
            this.rim.Clear( );
            this.rimAngles.Clear( );
            this.rimFaces.Clear( );
            this.polygonStarts.Clear( );
            this.polygonCorners.Clear( );
            this.polygonFaces.Clear( );

            this.rimStarts.Add( 0 );
            this.polygonStarts.Add( 0 );

            for( var vertex = 0; vertex < this.graph.vertexCount; ++vertex ) {
                around.Clear( );

                if( !this.graph.removed( vertex ) ) {
                    tangentsOf( this.graph.normals[vertex], out var s, out var t );

                    for( var entry = this.graph.heads[vertex]; entry != -1; entry = this.graph.entries[entry].y ) {
                        var neighbour = this.graph.entries[entry].x;

                        if( neighbour >= 0 ) {
                            around.Add( new Around { angle = angleOf( this.graph.positions[neighbour], this.graph.positions[vertex], s, t ), vertex = neighbour } );
                        }
                    }

                    around.Sort( );
                }

                for( var index = 0; index < around.Length; ++index ) {
                    this.rim.Add( around[index].vertex );
                    this.rimAngles.Add( around[index].angle );
                    this.rimFaces.Add( -1 );
                    owners.Add( vertex );
                }

                this.rimStarts.Add( this.rim.Length );
            }

            var used    = new NativeArray<bool>( math.max( this.rim.Length, 1 ), Allocator.Temp );
            var path    = new NativeList<int>( 16, Allocator.Temp );
            var corners = new NativeList<int>( 16, Allocator.Temp );
            var walked  = 0;

            for( var sides = 3; sides <= 8; ++sides )
            for( var vertex = 0; vertex < this.graph.vertexCount; ++vertex )
            for( var index = 0; index < this.rimStarts[vertex + 1] - this.rimStarts[vertex]; ++index ) {
                if( !walk( this.rimStarts.AsArray( ), this.rim.AsArray( ), used, vertex, index, sides, path ) || !distinct( owners.AsArray( ), path ) || !this.facesItsWay( owners.AsArray( ), path ) ) {
                    continue;
                }

                corners.Clear( );

                for( var step = 0; step < path.Length; ++step ) {
                    used[path[step]]          = true;
                    this.rimFaces[path[step]] = walked;

                    corners.Add( owners[path[step]] );
                }

                ++this.counts[math.min( sides, 6 ) - 3];

                this.emit( corners, walked++ );
            }

            this.fillHoles( owners, used, path, corners, ref walked );

            this.counts[7] = walked;

            if( this.faces == Faces.Quads ) {
                this.splitIntoQuads( );
            }

            owners.Dispose( );
            around.Dispose( );
            used.Dispose( );
            path.Dispose( );
            corners.Dispose( );
        }

        // the holes the faces leave: every edge from a vertex to a neighbour that is in no face, where the edge back
        // is in one, is on the rim of a hole. Walked round those edges alone, a hole of three to six sides is filled
        // (the code)
        private void fillHoles( NativeList<int> owners, NativeArray<bool> used, NativeList<int> path, NativeList<int> corners, ref int walked ) {
            var onRim = new NativeArray<bool>( math.max( this.rim.Length, 1 ), Allocator.Temp );

            for( var entry = 0; entry < this.rim.Length; ++entry ) {
                if( used[entry] ) {
                    continue;
                }

                var back = this.backOf( owners[entry], this.rim[entry] );

                if( back != -1 && used[back] ) {
                    onRim[entry] = true;
                    onRim[back]  = true;
                }
            }

            // the rims of the holes alone, in the same order round each vertex
            var starts     = new NativeList<int>( this.graph.vertexCount + 1, Allocator.Temp );
            var neighbours = new NativeList<int>( Allocator.Temp );
            var entries    = new NativeList<int>( Allocator.Temp );
            var holeOwners = new NativeList<int>( Allocator.Temp );

            starts.Add( 0 );

            for( var vertex = 0; vertex < this.graph.vertexCount; ++vertex ) {
                for( var entry = this.rimStarts[vertex]; entry < this.rimStarts[vertex + 1]; ++entry ) {
                    if( onRim[entry] ) {
                        neighbours.Add( this.rim[entry] );
                        entries.Add( entry );
                        holeOwners.Add( vertex );
                    }
                }

                starts.Add( neighbours.Length );
            }

            var holeUsed = new NativeArray<bool>( math.max( neighbours.Length, 1 ), Allocator.Temp );

            for( var entry = 0; entry < neighbours.Length; ++entry ) {
                holeUsed[entry] = used[entries[entry]];
            }

            for( var vertex = 0; vertex < this.graph.vertexCount; ++vertex )
            for( var index = 0; index < starts[vertex + 1] - starts[vertex]; ++index ) {
                if( !walk( starts.AsArray( ), neighbours.AsArray( ), holeUsed, vertex, index, 0, path ) ) {
                    continue;
                }

                for( var step = 0; step < path.Length; ++step ) {
                    holeUsed[path[step]]   = true;
                    used[entries[path[step]]] = true;
                }

                if( path.Length < 3 || path.Length > 6 || !distinct( holeOwners.AsArray( ), path ) || !this.facesItsWay( holeOwners.AsArray( ), path ) ) {
                    ++this.counts[5];
                    continue;
                }

                corners.Clear( );

                for( var step = 0; step < path.Length; ++step ) {
                    this.rimFaces[entries[path[step]]] = walked;

                    corners.Add( holeOwners[path[step]] );
                }

                ++this.counts[4];

                this.emit( corners, walked++ );
            }

            onRim.Dispose( );
            starts.Dispose( );
            neighbours.Dispose( );
            entries.Dispose( );
            holeOwners.Dispose( );
            holeUsed.Dispose( );
        }

        // whether the polygon a walk went round faces the way its corners do, by its area's normal against the sum of
        // theirs. The rim of an open mesh, walked round from outside, faces the other way and is no face: the code takes
        // it for one where it has up to eight sides, and for a hole where it has up to six
        private readonly bool facesItsWay( NativeArray<int> owners, NativeList<int> path ) {
            var area    = double3.zero;
            var normals = double3.zero;

            for( var step = 0; step < path.Length; ++step ) {
                var vertex = owners[path[step]];

                area    += math.cross( this.graph.positions[vertex], this.graph.positions[owners[path[( step + 1 ) % path.Length]]] );
                normals += this.graph.normals[vertex];
            }

            return math.dot( area, normals ) > 0.0;
        }

        // the entry of the edge from a vertex to a neighbour, in the neighbour's rim, back to the vertex
        private readonly int backOf( int vertex, int neighbour ) {
            for( var entry = this.rimStarts[neighbour]; entry < this.rimStarts[neighbour + 1]; ++entry ) {
                if( this.rim[entry] == vertex ) {
                    return entry;
                }
            }
            return -1;
        }

        // a walked face, cut down to the faces asked for: for triangles into triangles, and otherwise into quads until
        // it has five sides at most, a triangle or a pentagon being kept as it is where the code makes it a fan round a
        // new vertex. Each cut takes off the run of corners whose face has the angles nearest a triangle's or a square's.
        // The code scores a run by the angles of the face before it is cut, one corner along
        private void emit( NativeList<int> corners, int face ) {
            var most   = this.faces == Faces.Triangles ? 3 : 5;
            var run    = this.faces == Faces.Triangles ? 3 : 4;
            var target = this.faces == Faces.Triangles ? 60.0 : 90.0;

            while( corners.Length > most ) {
                var count = corners.Length;
                var best  = double.PositiveInfinity;
                var start = 0;

                for( var first = 0; first < count; ++first ) {
                    var score = 0.0;

                    for( var corner = 0; corner < run; ++corner ) {
                        var before = this.graph.positions[corners[( first + ( ( corner + run - 1 ) % run ) ) % count]];
                        var at     = this.graph.positions[corners[( first + corner ) % count]];
                        var after  = this.graph.positions[corners[( first + ( ( corner + 1 ) % run ) ) % count]];

                        score += math.abs( math.degrees( angleBetween( before - at, after - at ) ) - target );
                    }

                    if( score < best ) {
                        best  = score;
                        start = first;
                    }
                }

                for( var corner = 0; corner < run; ++corner ) {
                    this.polygonCorners.Add( corners[( start + corner ) % count] );
                }

                this.polygonStarts.Add( this.polygonCorners.Length );
                this.polygonFaces.Add( face );

                // the corners inside the run go, one for a triangle and two for a quad, the later place first so that
                // the earlier holds
                var inside = ( start + 1 ) % count;

                if( run == 3 ) {
                    corners.RemoveAt( inside );
                }
                else {
                    var next = ( start + 2 ) % count;

                    corners.RemoveAt( math.max( inside, next ) );
                    corners.RemoveAt( math.min( inside, next ) );
                }
            }

            for( var corner = 0; corner < corners.Length; ++corner ) {
                this.polygonCorners.Add( corners[corner] );
            }

            this.polygonStarts.Add( this.polygonCorners.Length );
            this.polygonFaces.Add( face );
        }

        // every polygon split into quads round its middle: a vertex at its middle, one at the middle of each of its
        // edges that its neighbour across the edge shares, and a quad at each corner (figure 1)
        private void splitIntoQuads( ) {
            var starts    = new NativeList<int>( Allocator.Temp );
            var corners   = new NativeList<int>( Allocator.Temp );
            var faces     = new NativeList<int>( Allocator.Temp );
            var midpoints = new NativeHashMap<long, int>( math.max( this.polygonCorners.Length, 16 ), Allocator.Temp );

            starts.Add( 0 );

            for( var polygon = 0; polygon < this.polygonFaces.Length; ++polygon ) {
                var first = this.polygonStarts[polygon];
                var count = this.polygonStarts[polygon + 1] - first;

                var middle       = double3.zero;
                var middleNormal = double3.zero;
                var held         = true;

                for( var corner = 0; corner < count; ++corner ) {
                    var vertex = this.polygonCorners[first + corner];

                    middle       += this.graph.positions[vertex];
                    middleNormal += this.graph.normals[vertex];
                    held         &= this.graph.fixedVertices[vertex];
                }

                var centre = this.graph.add( middle / count, math.normalizesafe( middleNormal ), held, 0 );

                for( var corner = 0; corner < count; ++corner ) {
                    var before = this.polygonCorners[first + ( ( corner + count - 1 ) % count )];
                    var at     = this.polygonCorners[first + corner];
                    var after  = this.polygonCorners[first + ( ( corner + 1 ) % count )];

                    corners.Add( at );
                    corners.Add( this.midpointOf( midpoints, at, after ) );
                    corners.Add( centre );
                    corners.Add( this.midpointOf( midpoints, before, at ) );

                    starts.Add( corners.Length );
                    faces.Add( this.polygonFaces[polygon] );
                }
            }

            this.polygonStarts.Clear( );
            this.polygonCorners.Clear( );
            this.polygonFaces.Clear( );

            this.polygonStarts.AddRange( starts.AsArray( ) );
            this.polygonCorners.AddRange( corners.AsArray( ) );
            this.polygonFaces.AddRange( faces.AsArray( ) );

            starts.Dispose( );
            corners.Dispose( );
            faces.Dispose( );
            midpoints.Dispose( );
        }

        private int midpointOf( NativeHashMap<long, int> midpoints, int a, int b ) {
            var key = BurstRemeshing.keyOf( a, b );

            if( midpoints.TryGetValue( key, out var vertex ) ) {
                return vertex;
            }

            vertex = this.graph.add(
                ( this.graph.positions[a] + this.graph.positions[b] ) * 0.5,
                math.normalizesafe( this.graph.normals[a] + this.graph.normals[b] ),
                this.graph.fixedVertices[a] && this.graph.fixedVertices[b],
                0
            );

            midpoints.Add( key, vertex );

            return vertex;
        }

    }

    // the angle between two directions, or a right angle where either has no length
    public static double angleBetween( double3 a, double3 b ) {
        var lengths = math.length( a ) * math.length( b );

        return lengths > 0.0 ? math.acos( math.clamp( math.dot( a, b ) / lengths, -1.0, 1.0 ) ) : math.PI / 2.0;
    }

    // a walk round a face from the edge of a vertex to its neighbour at an index of its rim (the code's extract_face):
    // along the edge, and at its end on along the next neighbour round after the vertex it came from, until it comes back
    // to the vertex it started from. It fails where it meets an edge already in a face, comes to a vertex with one
    // neighbour, or passes the number of sides it is to have, and succeeds where it comes back with that number, or any
    // number where it is given none. Gives the entries of the edges it went along
    public static bool walk( NativeArray<int> starts, NativeArray<int> neighbours, NativeArray<bool> used, int start, int index, int sides, NativeList<int> path ) {
        path.Clear( );

        var current = start;
        var at      = index;

        while( true ) {
            var entry = starts[current] + at;

            if( used[entry] || ( sides > 0 && path.Length + 1 > sides ) || path.Length >= neighbours.Length ) {
                return false;
            }

            path.Add( entry );

            var next = neighbours[entry];
            var rank = starts[next + 1] - starts[next];
            var back = -1;

            for( var other = 0; other < rank; ++other ) {
                if( neighbours[starts[next] + other] == current ) {
                    back = other;
                    break;
                }
            }

            if( back == -1 || rank == 1 ) {
                return false;
            }

            current = next;
            at      = ( back + 1 ) % rank;

            if( current == start ) {
                return sides == 0 || path.Length == sides;
            }
        }
    }

    // whether the vertices a walk went from are all different: a walk that came round to a vertex twice is no face
    private static bool distinct( NativeArray<int> owners, NativeList<int> path ) {
        for( var i = 0; i < path.Length; ++i )
        for( var j = i + 1; j < path.Length; ++j ) {
            if( owners[path[i]] == owners[path[j]] ) {
                return false;
            }
        }
        return true;
    }

    // ---- materials ----

    // the sub-mesh of every walked face: the one most of the area of the mesh the fields are on that maps into it has
    // (finding 5). A triangle of that mesh maps to the vertex of the graph that is nearest its middle among those its
    // corners went into, and there into the face whose corner it lies in, by its angle round the vertex. A face nothing
    // maps into takes the sub-mesh of the input at its middle
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct MaterialsJob : IJob {

        public Graph graph;

        [ReadOnly] public NativeArray<int>    rimStarts;
        [ReadOnly] public NativeArray<double> rimAngles;
        [ReadOnly] public NativeArray<int>    rimFaces;
        [ReadOnly] public NativeArray<int>    polygonStarts;
        [ReadOnly] public NativeArray<int>    polygonCorners;
        [ReadOnly] public NativeArray<int>    polygonFaces;

        // the mesh the fields are on, and the vertex of the graph each of its vertices went into
        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     subMeshes;
        [ReadOnly] public NativeArray<int>     clusters;

        public SurfaceGrid grid;

        // one for every walked face
        [WriteOnly] public NativeArray<int> faceSubMeshes;

        // one: faces nothing mapped into
        public NativeArray<int> unmapped;

        public double reach;
        public int    faceCount;

        public void Execute( ) {
            var subMeshCount = 1;
            var lowest       = int.MaxValue;

            for( var triangle = 0; triangle < this.subMeshes.Length; ++triangle ) {
                if( this.subMeshes[triangle] >= 0 ) {
                    subMeshCount = math.max( subMeshCount, this.subMeshes[triangle] + 1 );
                    lowest       = math.min( lowest, this.subMeshes[triangle] );
                }
            }

            var areas = new NativeArray<float>( math.max( this.faceCount * subMeshCount, 1 ), Allocator.Temp );

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                var subMesh = this.subMeshes[triangle];

                if( subMesh < 0 ) {
                    continue;
                }

                var corners = this.triangles[triangle];
                var a       = this.positions[corners.x];
                var b       = this.positions[corners.y];
                var c       = this.positions[corners.z];
                var middle  = ( a + b + c ) / 3.0;
                var vertex  = -1;
                var nearest = double.PositiveInfinity;

                for( var k = 0; k < 3; ++k ) {
                    var candidate = this.graph.resolved( this.clusters[corners[k]] );

                    if( candidate == -1 ) {
                        continue;
                    }

                    var distance = math.distancesq( this.graph.positions[candidate], middle );

                    if( distance < nearest || ( distance == nearest && candidate < vertex ) ) {
                        nearest = distance;
                        vertex  = candidate;
                    }
                }

                if( vertex == -1 || this.rimStarts[vertex + 1] == this.rimStarts[vertex] ) {
                    continue;
                }

                var face = this.faceAround( vertex, middle );

                if( face != -1 ) {
                    areas[( face * subMeshCount ) + subMesh] += ( float )( 0.5 * math.length( math.cross( b - a, c - a ) ) );
                }
            }

            // where nothing maps into a face: the middle of its polygons
            var middles = new NativeArray<double3>( math.max( this.faceCount, 1 ), Allocator.Temp );
            var weights = new NativeArray<int>( math.max( this.faceCount, 1 ), Allocator.Temp );

            for( var polygon = 0; polygon < this.polygonFaces.Length; ++polygon ) {
                for( var corner = this.polygonStarts[polygon]; corner < this.polygonStarts[polygon + 1]; ++corner ) {
                    middles[this.polygonFaces[polygon]] += this.graph.positions[this.polygonCorners[corner]];
                    ++weights[this.polygonFaces[polygon]];
                }
            }

            for( var face = 0; face < this.faceCount; ++face ) {
                var best    = -1;
                var largest = 0.0f;

                for( var subMesh = 0; subMesh < subMeshCount; ++subMesh ) {
                    if( areas[( face * subMeshCount ) + subMesh] > largest ) {
                        largest = areas[( face * subMeshCount ) + subMesh];
                        best    = subMesh;
                    }
                }

                if( best == -1 ) {
                    ++this.unmapped[0];

                    best = lowest == int.MaxValue ? 0 : lowest;

                    if( weights[face] > 0 && this.grid.nearestOnSurface( middles[face] / weights[face], SurfaceGrid.anySubMesh, this.reach, out _, out var triangle ) ) {
                        best = this.grid.subMeshes[triangle];
                    }
                }

                this.faceSubMeshes[face] = best;
            }

            areas.Dispose( );
            middles.Dispose( );
            weights.Dispose( );
        }

        // the walked face whose corner at a vertex a point lies in, by its angle round the vertex: the neighbours are
        // sorted the larger angle first, and the edge to a neighbour is in the face that lies between it and the
        // neighbour before, so the point is in the face of the first neighbour of a smaller angle, or of the first
        // where there is none
        private readonly int faceAround( int vertex, double3 point ) {
            tangentsOf( this.graph.normals[vertex], out var s, out var t );

            var angle = angleOf( point, this.graph.positions[vertex], s, t );
            var start = this.rimStarts[vertex];

            for( var entry = start; entry < this.rimStarts[vertex + 1]; ++entry ) {
                if( this.rimAngles[entry] < angle ) {
                    return this.rimFaces[entry];
                }
            }

            return this.rimFaces[start];
        }

    }

    // ---- smoothing ----

    // the rounds of smoothing after the faces (the code)
    public const int smoothingRounds = 2;

    private struct Pair : IComparable<Pair> {

        public int a;
        public int b;

        public int CompareTo( Pair other ) {
            return this.a != other.a ? this.a.CompareTo( other.a ) : this.b.CompareTo( other.b );
        }

    }

    // the edges of the polygons, as a list of neighbours for every vertex, sorted
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct NeighboursJob : IJob {

        [ReadOnly] public NativeArray<int> polygonStarts;
        [ReadOnly] public NativeArray<int> polygonCorners;

        public int vertexCount;

        public NativeList<int> starts;
        public NativeList<int> neighbours;

        public void Execute( ) {
            var pairs = new NativeList<Pair>( this.polygonCorners.Length * 2, Allocator.Temp );

            for( var polygon = 0; polygon + 1 < this.polygonStarts.Length; ++polygon ) {
                var first = this.polygonStarts[polygon];
                var count = this.polygonStarts[polygon + 1] - first;

                for( var corner = 0; corner < count; ++corner ) {
                    var a = this.polygonCorners[first + corner];
                    var b = this.polygonCorners[first + ( ( corner + 1 ) % count )];

                    pairs.Add( new Pair { a = a, b = b } );
                    pairs.Add( new Pair { a = b, b = a } );
                }
            }

            pairs.Sort( );

            this.starts.Clear( );
            this.neighbours.Clear( );

            var index = 0;

            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                this.starts.Add( this.neighbours.Length );

                var last = -1;

                for( ; index < pairs.Length && pairs[index].a == vertex; ++index ) {
                    if( pairs[index].b != last ) {
                        last = pairs[index].b;
                        this.neighbours.Add( last );
                    }
                }
            }

            this.starts.Add( this.neighbours.Length );

            pairs.Dispose( );
        }

    }

    // where one round of smoothing takes every vertex (the code): one that is not on a crease or a held line, to the
    // middle of its neighbours, and from there to the nearest point of the input that faces its way, within reach. The
    // code casts a ray along the normal of the neighbours' plane and keeps the middle where it hits nothing; here a
    // vertex with no point of the input within reach stays where it is
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SmoothJob : IJobParallelFor {

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<double3> normals;
        [ReadOnly] public NativeArray<bool>    fixedVertices;
        [ReadOnly] public NativeArray<int>     starts;
        [ReadOnly] public NativeArray<int>     neighbours;

        public SurfaceGrid grid;
        public double      reach;

        [WriteOnly] public NativeArray<double3> targets;
        [WriteOnly] public NativeArray<byte>    moves; // 0 stays, 1 moves, 2 has nowhere to go

        public void Execute( int vertex ) {
            var position = this.positions[vertex];
            var first    = this.starts[vertex];
            var count    = this.starts[vertex + 1] - first;

            this.targets[vertex] = position;
            this.moves[vertex]   = 0;

            if( count == 0 || this.fixedVertices[vertex] ) {
                return;
            }

            var middle = double3.zero;

            for( var index = 0; index < count; ++index ) {
                middle += this.positions[this.neighbours[first + index]];
            }

            middle /= count;

            if( this.grid.nearestOnSurface( middle, SurfaceGrid.anySubMesh, this.normals[vertex], this.reach, out var nearest, out _ ) ) {
                this.targets[vertex] = nearest;
                this.moves[vertex]   = 1;
            }
            else {
                this.moves[vertex] = 2;
            }
        }

    }

    // ---- out ----

    // the triangles a polygon is made of: a fan from one of its corners, the one whose triangles lie flattest together,
    // by the least of the cosines between the normals of two that share an edge, and of two as flat the first. For a
    // quad that is the diagonal that leaves its two triangles flatter. A fan with a triangle without area is the worst.
    // Gives the corner
    public static int fanOf( NativeArray<double3> positions, NativeArray<int> corners, int first, int count ) {
        if( count <= 3 ) {
            return 0;
        }

        var best  = 0;
        var worst = double.NegativeInfinity;

        for( var from = 0; from < count; ++from ) {
            var score    = double.PositiveInfinity;
            var previous = double3.zero;
            var origin   = positions[corners[first + from]];

            for( var step = 1; step + 1 < count; ++step ) {
                var b      = positions[corners[first + ( ( from + step ) % count )]];
                var c      = positions[corners[first + ( ( from + step + 1 ) % count )]];
                var normal = math.cross( b - origin, c - origin );
                var length = math.length( normal );

                if( !( length > 0.0 ) ) {
                    score = -2.0;
                    break;
                }

                normal /= length;

                if( step > 1 ) {
                    score = math.min( score, math.dot( previous, normal ) );
                }

                previous = normal;
            }

            if( score > worst ) {
                worst = score;
                best  = from;
            }
        }

        return best;
    }

    // the flags of the triangle of a fan from corner 0 of a polygon of so many sides to corners step and step + 1: the
    // edge from corner step to step + 1 is the polygon's, the edge from corner 0 to step only where step is 1, and the
    // edge from step + 1 back to corner 0 only where step + 1 is the last (BurstRemeshing.allEdges)
    public static byte flagsOf( int step, int count ) {
        var flags = 2;

        if( step == 1 ) {
            flags |= 1;
        }
        if( step + 1 == count - 1 ) {
            flags |= 4;
        }

        return ( byte )flags;
    }

    // the remeshed mesh: the vertices of the polygons, in the order of the graph, and the polygons as fans of triangles
    // with their flags and their walked face's sub-mesh, in the order of the polygons. A polygon with a corner it has
    // twice is left out, which the walk never makes but a split of quads could
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct WriteJob : IJob {

        [ReadOnly] public NativeArray<double3> graphPositions;
        [ReadOnly] public NativeArray<int>     polygonStarts;
        [ReadOnly] public NativeArray<int>     polygonCorners;
        [ReadOnly] public NativeArray<int>     polygonFaces;
        [ReadOnly] public NativeArray<int>     faceSubMeshes;

        public NativeList<double3>            positions;
        public NativeList<double3>            ownNormals; // the sum of the normals of every vertex's triangles, weighted by their areas
        public NativeList<BurstMesh.Triangle> triangles;
        public NativeList<byte>               flags;

        public void Execute( ) {
            var places = new NativeArray<int>( math.max( this.graphPositions.Length, 1 ), Allocator.Temp );

            for( var vertex = 0; vertex < places.Length; ++vertex ) {
                places[vertex] = -1;
            }

            for( var corner = 0; corner < this.polygonCorners.Length; ++corner ) {
                places[this.polygonCorners[corner]] = 0;
            }

            for( var vertex = 0; vertex < this.graphPositions.Length; ++vertex ) {
                if( places[vertex] == 0 ) {
                    places[vertex] = this.positions.Length;
                    this.positions.Add( this.graphPositions[vertex] );
                    this.ownNormals.Add( double3.zero );
                }
            }

            for( var polygon = 0; polygon < this.polygonFaces.Length; ++polygon ) {
                var first   = this.polygonStarts[polygon];
                var count   = this.polygonStarts[polygon + 1] - first;
                var subMesh = this.faceSubMeshes[this.polygonFaces[polygon]];
                var from    = fanOf( this.graphPositions, this.polygonCorners, first, count );

                for( var step = 1; step + 1 < count; ++step ) {
                    var a = places[this.polygonCorners[first + from]];
                    var b = places[this.polygonCorners[first + ( ( from + step ) % count )]];
                    var c = places[this.polygonCorners[first + ( ( from + step + 1 ) % count )]];

                    var normal = math.cross( this.positions[b] - this.positions[a], this.positions[c] - this.positions[a] );

                    this.ownNormals[a] += normal;
                    this.ownNormals[b] += normal;
                    this.ownNormals[c] += normal;

                    this.triangles.Add( new BurstMesh.Triangle( a, b, c, subMesh ) );
                    this.flags.Add( flagsOf( step, count ) );
                }
            }

            places.Dispose( );
        }

    }

    // the normal of every vertex out (finding 21): the input's at the nearest point of it that faces the vertex's way,
    // from the normals at that triangle's corners (BurstRemeshing.cornerNormalsOf), or, where there is none within reach
    // or no corner normals are given, the normal of the vertex's own triangles
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct NormalsJob : IJobParallelFor {

        public SurfaceGrid grid;
        public double      reach;

        [ReadOnly] public NativeArray<float3>  cornerNormals;
        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<double3> ownNormals;

        [WriteOnly] public NativeArray<float3> normals;

        public void Execute( int vertex ) {
            var own = math.normalizesafe( this.ownNormals[vertex] );

            if( this.cornerNormals.Length == 0 || !this.grid.nearestOnSurface( this.positions[vertex], SurfaceGrid.anySubMesh, own, this.reach, out var nearest, out var triangle ) ) {
                this.normals[vertex] = ( float3 )own;
                return;
            }

            this.normals[vertex] = ( float3 )BurstRemeshing.inputNormalAt( this.grid, this.cornerNormals, nearest, triangle, own );
        }

    }

    // ---- the whole ----

    private static readonly ProfilerMarker extractionMarker = new( "Remeshing.FieldAligned.Extraction" );
    private static readonly ProfilerMarker facesMarker      = new( "Remeshing.FieldAligned.Faces" );
    private static readonly ProfilerMarker smoothMarker     = new( "Remeshing.FieldAligned.Smooth" );
    private static readonly ProfilerMarker outputMarker     = new( "Remeshing.FieldAligned.Output" );

    // how many vertices a worker takes at a time
    private const int vertexBatch = 128;

    // how far from a vertex the input is looked for, in edge lengths: smoothing moves a vertex less than an edge, and
    // the surface is nearer than that
    public const double reachInLengths = 1.0;

    // the edge length the fields are solved at: for quads only, twice the length, as each face is split in two each way
    public static double latticeLengthOf( Faces faces, double length ) {
        return faces == Faces.Quads ? 2.0 * length : length;
    }

    // remeshes a mesh of shared vertices into the lists, which are the caller's: the fields solved on it (BurstFields),
    // the graph read off them, its faces, their materials, two rounds of smoothing, and the faces as triangles with their
    // flags (BurstRemeshing.allEdges), in the order of the triangles. The normals out are had from cornerNormals, three
    // to a triangle of the mesh given, or from each vertex's own triangles where it is empty. Gives nothing where the
    // mesh the fields are on would have more than Settings.mostVertices, which the report says
    public static void remesh(
        NativeArray<float3>             positions,
        NativeArray<float3>             normals,
        NativeArray<BurstMesh.Triangle> triangles,
        NativeArray<float3>             cornerNormals,
        in Settings                     settings,
        NativeList<float3>              outPositions,
        NativeList<float3>              outNormals,
        NativeList<BurstMesh.Triangle>  outTriangles,
        NativeList<byte>                outFlags,
        out Report                      report
    ) {
        report = default;

        var quads    = settings.faces != Faces.Triangles;
        var rho      = latticeLengthOf( settings.faces, settings.length );
        var symmetry = quads ? Symmetry.Quads : Symmetry.Triangles;
        var reach    = reachInLengths * settings.length;

        var fieldSettings = new BurstFields.Settings {
            symmetry      = symmetry,
            length        = rho,
            featureCosine = settings.featureCosine,
            featureLength = settings.featureLength,
            creases       = true,
            freeMaterials = settings.freeMaterials,
            iterations    = BurstFields.iterationsALevel,
            seed          = BurstFields.seed,
            mostVertices  = settings.mostVertices
        };

        using var fields = BurstFields.solve( positions, normals, triangles, fieldSettings, Allocator.TempJob, out report.fields );

        if( report.fields.tooLarge ) {
            return;
        }

        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );
        var hierarchy = fields.hierarchy;
        var mesh      = fields.mesh.mesh;
        var count     = mesh.vertexCount;

        using var graph    = new Graph( Allocator.TempJob );
        using var clusters = new NativeArray<int>( math.max( count, 1 ), Allocator.TempJob );
        using var grid     = gridOf( positions, triangles, Allocator.TempJob );

        using( extractionMarker.Auto( ) ) {
            var linkCount = hierarchy.linkStarts[count];

            using var kinds     = new NativeArray<byte>( math.max( linkCount, 1 ), Allocator.TempJob );
            using var distances = new NativeArray<float>( math.max( linkCount, 1 ), Allocator.TempJob );
            using var counts    = new NativeArray<int>( 6, Allocator.TempJob );
            using var snaps     = new NativeArray<int>( 4, Allocator.TempJob );

            new ClassifyJob {
                positions    = hierarchy.positions.AsArray( ),
                normals      = hierarchy.normals.AsArray( ),
                orientations = hierarchy.orientations.AsArray( ),
                lattice      = hierarchy.lattice.AsArray( ),
                linkStarts   = hierarchy.linkStarts.AsArray( ),
                links        = hierarchy.links.AsArray( ),
                rho          = ( float )rho,
                symmetry     = symmetry,
                kinds        = kinds,
                distances    = distances
            }.Schedule( count, vertexBatch ).Complete( );

            new ClusterJob {
                positions   = hierarchy.positions.AsArray( ),
                normals     = hierarchy.normals.AsArray( ),
                lattice     = hierarchy.lattice.AsArray( ),
                held        = hierarchy.held.AsArray( ),
                linkStarts  = hierarchy.linkStarts.AsArray( ),
                links       = hierarchy.links.AsArray( ),
                kinds       = kinds,
                distances   = distances,
                firstEdges  = mesh.firstEdges.AsArray( ),
                nextEdges   = mesh.nextEdges.AsArray( ),
                edgeSharp   = mesh.edgeSharp.AsArray( ),
                edgeLines   = fields.mesh.edgeLines.AsArray( ),
                vertexCount = count,
                rho         = ( float )rho,
                graph       = graph,
                clusters    = clusters,
                counts      = counts
            }.Run( );

            new SnapJob { graph = graph, rho = rho, quads = quads, counts = snaps }.Run( );

            place( graph, grid, rho, 0, ref report );

            report.unitLinks = counts[0];
            report.zeroLinks = counts[1];
            report.collapses = counts[2];
            report.conflicts = counts[3];
            report.vertices  = counts[4];
            report.spurious  = counts[5];
            report.snapped   = snaps[0];
            report.merged    = snaps[1];
            report.diagonals = snaps[2];
            report.unsettled = snaps[3] != 0;
        }

        report.extractionMilliseconds = lap( stopwatch );

        using var rimStarts      = new NativeList<int>( Allocator.TempJob );
        using var rim            = new NativeList<int>( Allocator.TempJob );
        using var rimAngles      = new NativeList<double>( Allocator.TempJob );
        using var rimFaces       = new NativeList<int>( Allocator.TempJob );
        using var polygonStarts  = new NativeList<int>( Allocator.TempJob );
        using var polygonCorners = new NativeList<int>( Allocator.TempJob );
        using var polygonFaces   = new NativeList<int>( Allocator.TempJob );

        var faceCount     = 0;
        var graphVertices = graph.vertexCount;

        using( facesMarker.Auto( ) ) {
            using var counts = new NativeArray<int>( 8, Allocator.TempJob );

            new FacesJob {
                graph          = graph,
                faces          = settings.faces,
                rimStarts      = rimStarts,
                rim            = rim,
                rimAngles      = rimAngles,
                rimFaces       = rimFaces,
                polygonStarts  = polygonStarts,
                polygonCorners = polygonCorners,
                polygonFaces   = polygonFaces,
                counts         = counts
            }.Run( );

            // the vertices the split into quads added
            place( graph, grid, rho, graphVertices, ref report );

            report.walkedTriangles = counts[0];
            report.walkedQuads     = counts[1];
            report.walkedPentagons = counts[2];
            report.walkedLarger    = counts[3];
            report.holes           = counts[4];
            report.unfilledHoles   = counts[5];

            faceCount = counts[7];
        }

        report.facesMilliseconds = lap( stopwatch );

        using var faceSubMeshes = new NativeArray<int>( math.max( faceCount, 1 ), Allocator.TempJob );

        using( smoothMarker.Auto( ) ) {
            using var unmapped = new NativeArray<int>( 1, Allocator.TempJob );

            new MaterialsJob {
                graph          = graph,
                rimStarts      = rimStarts.AsArray( ),
                rimAngles      = rimAngles.AsArray( ),
                rimFaces       = rimFaces.AsArray( ),
                polygonStarts  = polygonStarts.AsArray( ),
                polygonCorners = polygonCorners.AsArray( ),
                polygonFaces   = polygonFaces.AsArray( ),
                positions      = mesh.positions.AsArray( ),
                triangles      = mesh.triangles.AsArray( ),
                subMeshes      = mesh.subMeshes.AsArray( ),
                clusters       = clusters,
                grid           = grid,
                reach          = reach,
                faceCount      = faceCount,
                faceSubMeshes  = faceSubMeshes,
                unmapped       = unmapped
            }.Run( );

            report.unmapped = unmapped[0];

            smooth( graph, polygonStarts, polygonCorners, grid, reach, ref report );
        }

        report.smoothMilliseconds = lap( stopwatch );

        using( outputMarker.Auto( ) ) {
            using var written    = new NativeList<double3>( Allocator.TempJob );
            using var ownNormals = new NativeList<double3>( Allocator.TempJob );

            new WriteJob {
                graphPositions = graph.positions.AsArray( ),
                polygonStarts  = polygonStarts.AsArray( ),
                polygonCorners = polygonCorners.AsArray( ),
                polygonFaces   = polygonFaces.AsArray( ),
                faceSubMeshes  = faceSubMeshes,
                positions      = written,
                ownNormals     = ownNormals,
                triangles      = outTriangles,
                flags          = outFlags
            }.Run( );

            outPositions.ResizeUninitialized( written.Length );
            outNormals.ResizeUninitialized( written.Length );

            for( var vertex = 0; vertex < written.Length; ++vertex ) {
                outPositions[vertex] = ( float3 )written[vertex];
            }

            new NormalsJob {
                grid          = grid,
                reach         = reach,
                cornerNormals = cornerNormals,
                positions     = written.AsArray( ),
                ownNormals    = ownNormals.AsArray( ),
                normals       = outNormals.AsArray( )
            }.Schedule( written.Length, vertexBatch ).Complete( );
        }

        report.outputMilliseconds = lap( stopwatch );
    }

    // the vertices of the graph from the first given on put back on the input (PlaceJob)
    private static void place( Graph graph, SurfaceGrid grid, double reach, int first, ref Report report ) {
        var count = graph.vertexCount - first;

        using var placed = new NativeArray<bool>( math.max( count, 1 ), Allocator.TempJob );

        new PlaceJob {
            positions = graph.positions.AsArray( ),
            normals   = graph.normals.AsArray( ),
            into      = graph.into.AsArray( ),
            grid      = grid,
            reach     = reach,
            first     = first,
            placed    = placed
        }.Schedule( count, vertexBatch ).Complete( );

        for( var index = 0; index < count; ++index ) {
            report.offInput += !graph.removed( first + index ) && !placed[index] ? 1 : 0;
        }
    }

    // the rounds of smoothing over the polygons' edges, the moves of a round worked out from where the round began
    private static void smooth( Graph graph, NativeList<int> polygonStarts, NativeList<int> polygonCorners, SurfaceGrid grid, double reach, ref Report report ) {
        var vertexCount = graph.vertexCount;

        using var starts     = new NativeList<int>( vertexCount + 1, Allocator.TempJob );
        using var neighbours = new NativeList<int>( Allocator.TempJob );
        using var targets    = new NativeArray<double3>( math.max( vertexCount, 1 ), Allocator.TempJob );
        using var moves      = new NativeArray<byte>( math.max( vertexCount, 1 ), Allocator.TempJob );

        new NeighboursJob { polygonStarts = polygonStarts.AsArray( ), polygonCorners = polygonCorners.AsArray( ), vertexCount = vertexCount, starts = starts, neighbours = neighbours }.Run( );

        for( var round = 0; round < smoothingRounds; ++round ) {
            new SmoothJob {
                positions     = graph.positions.AsArray( ),
                normals       = graph.normals.AsArray( ),
                fixedVertices = graph.fixedVertices.AsArray( ),
                starts        = starts.AsArray( ),
                neighbours    = neighbours.AsArray( ),
                grid          = grid,
                reach         = reach,
                targets       = targets,
                moves         = moves
            }.Schedule( vertexCount, vertexBatch ).Complete( );

            for( var vertex = 0; vertex < vertexCount; ++vertex ) {
                graph.positions[vertex] = targets[vertex];

                report.moves    += moves[vertex] == 1 ? 1 : 0;
                report.unplaced += moves[vertex] == 2 ? 1 : 0;
            }
        }
    }

    // the size of the cells of the grid of the input, in its mean edges: a cell then holds a few of its triangles
    // (BurstIsotropic.cellInMeanEdges)
    public static SurfaceGrid gridOf( NativeArray<float3> positions, NativeArray<BurstMesh.Triangle> triangles, Allocator allocator ) {
        var points    = new NativeArray<double3>( positions.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        var corners   = new NativeArray<int3>( triangles.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        var subMeshes = new NativeArray<int>( triangles.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        var noEdges   = new NativeArray<int2>( 0, Allocator.TempJob );
        var noLines   = new NativeArray<int>( 0, Allocator.TempJob );

        try {
            for( var vertex = 0; vertex < positions.Length; ++vertex ) {
                points[vertex] = positions[vertex];
            }

            var total = 0.0;

            for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
                var t = triangles[triangle];

                corners[triangle]   = new int3( t.a, t.b, t.c );
                subMeshes[triangle] = t.subMeshIndex;

                total += math.distance( points[t.a], points[t.b] ) + math.distance( points[t.b], points[t.c] ) + math.distance( points[t.c], points[t.a] );
            }

            var cellSize = triangles.Length > 0 && total > 0.0 ? BurstIsotropic.cellInMeanEdges * total / ( 3.0 * triangles.Length ) : 1.0;

            return SurfaceGrid.of( points, corners, subMeshes, noEdges, noLines, cellSize, allocator );
        }
        finally {
            points.Dispose( );
            corners.Dispose( );
            subMeshes.Dispose( );
            noEdges.Dispose( );
            noLines.Dispose( );
        }
    }

    private static double lap( System.Diagnostics.Stopwatch stopwatch ) {
        var milliseconds = stopwatch.Elapsed.TotalMilliseconds;

        stopwatch.Restart( );

        return milliseconds;
    }

}

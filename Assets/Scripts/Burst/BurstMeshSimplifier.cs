using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;

// simplifies the mesh of a contour by collapsing edges, after Surface Simplification Using Quadric Error Metrics
// (Garland and Heckbert), with the collapses that are allowed taken from Topology Preserving Edge Contraction (Dey,
// Edelsbrunner, Guha and Nekhayev) and Progressive Meshes (Hoppe)
public static class BurstMeshSimplifier {

    // why an edge sets itself apart from the surface around it: it has one triangle, more than two, or triangles of
    // different sub-meshes, which is where two materials meet
    [Flags]
    public enum Sharp : byte {
        None        = 0,
        Open        = 1,
        NonManifold = 2,
        Materials   = 4
    }

    // why an edge is not collapsed
    public enum Refusal {
        None,

        // an end of the edge is a vertex of two triangles on the same three vertices, which the conditions below say
        // nothing about
        Locked,

        // the link conditions of Dey et al., theorem B, for a mesh of triangles that need not be a manifold. The first
        // boundary of the mesh is its edges that do not have exactly two triangles and its vertices that the surface is
        // not a disk around. Condition (i.0), in the order of the simplex that breaks it:
        LinkVertex,       // a vertex next to both ends is not opposite the edge in a triangle
        LinkEdge,         // an edge is opposite both ends in a triangle
        LinkBoundary,     // both ends are in the first boundary and the edge is not
        LinkBoundaryEdge, // a vertex is joined to both ends by edges of the first boundary. This also breaks (i.1)
        // and condition (i.1):
        LinkCorners,      // both ends are of order 2: the first boundary does not pass straight through either

        // the tests of Hoppe, section 4.5, on the sharp edges, in his order
        SharpNeighbour,   // a vertex is joined to both ends by sharp edges
        SharpEnds,        // both ends are on sharp edges and the edge is not sharp
        SharpCorners,     // the edge is sharp and three or more sharp edges meet at both ends
        SharpDart,        // the edge is the only sharp edge of one end, and the other end is not on a line of two

        // the two papers taken together, for a mesh that is not a manifold and has sharp lines as well. Hoppe's meshes
        // are manifolds and those of Dey et al. have no materials, and neither has a test for a vertex of order 2 next
        // to sharp lines that are not its own. A vertex is a point of its own, which no collapse may move, when it is of
        // order 2 or when sharp lines end or meet at it. Refused are an edge between two such points, and an edge that
        // is not sharp from a vertex of order 2 to a vertex of a sharp line
        Points,

        // Garland and Heckbert, section 6: a triangle next to the edge would turn over or lose its area
        Inversion
    }

    // a mesh as lists: for every vertex its triangles and its edges, for every edge its two vertices. That is how
    // Garland and Heckbert keep theirs (section 4.1), and it holds meshes that are not manifolds: an edge may have any
    // number of triangles and a vertex any number of fans of them.
    //
    // The lists are linked through arrays. A triangle has three corners, corner 3 t + k is vertex k of triangle t, and
    // the corners of a vertex follow each other through nextCorners. An edge has two ends, end 2 e + k is vertex k of
    // edge e, and the ends at a vertex follow each other through nextEdges
    public struct Connectivity : IDisposable {

        // vertices. A vertex without triangles has no corners and no edges, and is passed over
        public NativeArray<double3>      positions;
        public NativeArray<double3>      normals;
        public NativeArray<double>       areas;        // a third of the area of each triangle the vertex stands for
        public NativeArray<BurstQuadric> planes;       // the planes of those triangles, each weighted by that third
        public NativeArray<double>       sharpAreas;   // the same thirds, of the triangles of its sharp edges alone
        public NativeArray<BurstQuadric> sharpPlanes;  // the planes that hold those sharp edges in place (QuadricsJob)
        public NativeArray<BurstQuadric> lines;        // the lines of the vertices it stands for (QuadricsJob)
        public NativeArray<int>          firstCorners;
        public NativeArray<int>          firstEdges;
        public NativeArray<byte>         orders;       // the order of Dey et al., section 3: 0, 1 or 2
        public NativeArray<int>          sharpCounts;  // how many sharp edges the vertex has
        public NativeArray<bool>         locked;

        // triangles. A removed triangle has a sub-mesh of -1
        public NativeArray<int3> triangles;
        public NativeArray<int>  subMeshes;
        public NativeArray<int>  nextCorners;

        // edges. A removed edge has vertices of -1
        public NativeList<int2>  edges;
        public NativeList<int>   nextEdges;
        public NativeList<int>   edgeTriangles; // how many triangles the edge has
        public NativeList<Sharp> edgeSharp;

        // the edges the last collapse removed
        public NativeList<int> removedEdges;

        // notes on the vertices around the one being looked at. A note counts while its stamp is the current one. The
        // current stamp is kept where every copy of this struct finds it: a job works on a copy, and a stamp counted
        // up in the copy alone would start over in the next job, among the stamps the first one left
        private NativeArray<int>  stamps;
        private NativeArray<int>  notes;
        private NativeArray<byte> marks;
        private NativeList<int>   scratch;
        private NativeList<int2>  pairs;
        private NativeArray<int>  lastStamp;

        private const byte Opposite = 1;
        private const byte Common   = 2;

        public Connectivity( int vertexCount, int triangleCount, Allocator allocator ) {
            this.positions     = new NativeArray<double3>      ( vertexCount, allocator );
            this.normals       = new NativeArray<double3>      ( vertexCount, allocator );
            this.areas         = new NativeArray<double>       ( vertexCount, allocator );
            this.planes        = new NativeArray<BurstQuadric> ( vertexCount, allocator );
            this.sharpAreas    = new NativeArray<double>       ( vertexCount, allocator );
            this.sharpPlanes   = new NativeArray<BurstQuadric> ( vertexCount, allocator );
            this.lines         = new NativeArray<BurstQuadric> ( vertexCount, allocator );
            this.firstCorners  = new NativeArray<int>          ( vertexCount, allocator );
            this.firstEdges    = new NativeArray<int>          ( vertexCount, allocator );
            this.orders        = new NativeArray<byte>         ( vertexCount, allocator );
            this.sharpCounts   = new NativeArray<int>          ( vertexCount, allocator );
            this.locked        = new NativeArray<bool>         ( vertexCount, allocator );
            this.triangles     = new NativeArray<int3>         ( triangleCount, allocator );
            this.subMeshes     = new NativeArray<int>          ( triangleCount, allocator );
            this.nextCorners   = new NativeArray<int>          ( triangleCount * 3, allocator );
            this.edges         = new NativeList<int2>          ( triangleCount * 3 / 2, allocator );
            this.nextEdges     = new NativeList<int>           ( triangleCount * 3, allocator );
            this.edgeTriangles = new NativeList<int>           ( triangleCount * 3 / 2, allocator );
            this.edgeSharp     = new NativeList<Sharp>         ( triangleCount * 3 / 2, allocator );
            this.removedEdges  = new NativeList<int>           ( 16, allocator );
            this.stamps        = new NativeArray<int>          ( vertexCount, allocator );
            this.notes         = new NativeArray<int>          ( vertexCount, allocator );
            this.marks         = new NativeArray<byte>         ( vertexCount, allocator );
            this.scratch       = new NativeList<int>           ( 64, allocator );
            this.pairs         = new NativeList<int2>          ( 64, allocator );
            this.lastStamp     = new NativeArray<int>          ( 1, allocator );
        }

        // a stamp that no note has yet
        private int nextStamp( ) {
            return ++this.lastStamp[0];
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.areas.Dispose( );
            this.planes.Dispose( );
            this.sharpAreas.Dispose( );
            this.sharpPlanes.Dispose( );
            this.lines.Dispose( );
            this.firstCorners.Dispose( );
            this.firstEdges.Dispose( );
            this.orders.Dispose( );
            this.sharpCounts.Dispose( );
            this.locked.Dispose( );
            this.triangles.Dispose( );
            this.subMeshes.Dispose( );
            this.nextCorners.Dispose( );
            this.edges.Dispose( );
            this.nextEdges.Dispose( );
            this.edgeTriangles.Dispose( );
            this.edgeSharp.Dispose( );
            this.removedEdges.Dispose( );
            this.stamps.Dispose( );
            this.notes.Dispose( );
            this.marks.Dispose( );
            this.scratch.Dispose( );
            this.pairs.Dispose( );
            this.lastStamp.Dispose( );
        }

        public readonly int vertexCount {
            get { return this.positions.Length; }
        }

        public readonly int triangleCount {
            get { return this.triangles.Length; }
        }

        public readonly int edgeCount {
            get { return this.edges.Length; }
        }

        // the vertex at the other end of an edge
        public readonly int across( int edge, int vertex ) {
            var ends = this.edges[edge];

            return ends.x == vertex ? ends.y : ends.x;
        }

        private readonly bool has( int triangle, int vertex ) {
            return math.any( this.triangles[triangle] == vertex );
        }

        // the vertex of a triangle that is neither of two of its vertices
        private readonly int third( int triangle, int vertex, int other ) {
            var corners = this.triangles[triangle];

            return corners.x != vertex && corners.x != other ? corners.x : corners.y != vertex && corners.y != other ? corners.y : corners.z;
        }

        // the edge between two vertices, or -1
        public readonly int edgeBetween( int vertex, int other ) {
            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                if( this.across( end >> 1, vertex ) == other ) {
                    return end >> 1;
                }
            }
            return -1;
        }

        // takes the mesh of a contour. Triangles that use a vertex twice are left out
        public void build( NativeArray<float3> positions, NativeArray<float3> normals, NativeArray<BurstMesh.Triangle> triangles ) {
            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                this.positions[vertex]    = positions[vertex];
                this.normals[vertex]      = normals[vertex];
                this.firstCorners[vertex] = -1;
                this.firstEdges[vertex]   = -1;
            }

            // from the last triangle to the first, which leaves the corners of every vertex in the order of its triangles
            for( var triangle = this.triangleCount - 1; triangle >= 0; --triangle ) {
                var corners = new int3( triangles[triangle].a, triangles[triangle].b, triangles[triangle].c );

                if( corners.x == corners.y || corners.y == corners.z || corners.x == corners.z ) {
                    this.triangles[triangle] = -1;
                    this.subMeshes[triangle] = -1;

                    continue;
                }

                this.triangles[triangle] = corners;
                this.subMeshes[triangle] = triangles[triangle].subMeshIndex;

                for( var corner = 0; corner < 3; ++corner ) {
                    this.nextCorners[( triangle * 3 ) + corner] = this.firstCorners[corners[corner]];
                    this.firstCorners[corners[corner]]          = ( triangle * 3 ) + corner;
                }
            }

            // an edge for every two vertices that share a triangle, made at the lower of the two
            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                var stamp = this.nextStamp( );

                for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                    var corners = this.triangles[corner / 3];

                    for( var other = 0; other < 3; ++other ) {
                        var neighbour = corners[other];

                        if( neighbour <= vertex || this.stamps[neighbour] == stamp ) {
                            continue;
                        }
                        this.stamps[neighbour] = stamp;

                        var edge = this.edges.Length;

                        this.edges.Add( new int2( vertex, neighbour ) );
                        this.edgeTriangles.Add( 0 );
                        this.edgeSharp.Add( Sharp.None );

                        this.nextEdges.Add( this.firstEdges[vertex] );
                        this.nextEdges.Add( this.firstEdges[neighbour] );

                        this.firstEdges[vertex]    = edge * 2;
                        this.firstEdges[neighbour] = ( edge * 2 ) + 1;
                    }
                }
            }

            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                this.refresh( vertex );
            }

            this.lockRepeatedTriangles( );
        }

        // works out again what follows from the triangles of a vertex: for each of its edges how many triangles it has
        // and whether it is sharp, and for the vertex how many sharp edges it has and its order
        private void refresh( int vertex ) {
            var stamp = this.nextStamp( );

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, vertex );

                this.stamps[neighbour] = stamp;
                this.notes[neighbour]  = end >> 1;

                this.edgeTriangles[end >> 1] = 0;
                this.edgeSharp[end >> 1]     = Sharp.None;
            }

            // marks holds the sub-mesh of the first triangle of each edge, to tell when a later one differs from it
            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];
                var subMesh = this.subMeshes[corner / 3];

                for( var other = 0; other < 3; ++other ) {
                    var neighbour = corners[other];

                    if( neighbour == vertex ) {
                        continue;
                    }

                    var edge = this.notes[neighbour];

                    if( this.edgeTriangles[edge] == 0 ) {
                        this.marks[neighbour] = ( byte )subMesh;
                    }
                    else if( this.marks[neighbour] != ( byte )subMesh ) {
                        this.edgeSharp[edge] = Sharp.Materials;
                    }

                    ++this.edgeTriangles[edge];
                }
            }

            var sharp = 0;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var count = this.edgeTriangles[end >> 1];

                if( count == 1 ) {
                    this.edgeSharp[end >> 1] |= Sharp.Open;
                }
                if( count > 2 ) {
                    this.edgeSharp[end >> 1] |= Sharp.NonManifold;
                }
                if( this.edgeSharp[end >> 1] != Sharp.None ) {
                    ++sharp;
                }
            }

            this.sharpCounts[vertex] = sharp;
            this.orders[vertex]      = this.orderOf( vertex );
        }

        // the order of a vertex (Dey et al., section 3), from the triangles around it. Going from triangle to triangle
        // across the edges of the vertex that have exactly two triangles sorts its triangles into sheets.
        // - order 0: the surface around the vertex is a disk. Every edge has two triangles and there is one sheet.
        // - order 1: the surface around it is that around a point of an edge of the first boundary, which is any
        //   number of half disks along a path of two edges. Two edges of the vertex do not have two triangles, and
        //   every sheet runs from one of them to the other.
        // - order 2: anything else, such as two disks that touch in the vertex, or the end of a line of edges of the
        //   first boundary.
        // The triangle counts of the edges of the vertex have to be up to date
        private byte orderOf( int vertex ) {
            var stamp = this.nextStamp( );

            // the vertices at the far ends of the first two edges that do not have two triangles
            var boundary = 0;
            var p        = -1;
            var q        = -1;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, vertex );

                // the first triangle met on the edge, none yet
                this.stamps[neighbour] = stamp;
                this.notes[neighbour]  = -1;
                this.marks[neighbour]  = ( byte )( this.edgeTriangles[end >> 1] == 2 ? 1 : 0 );

                if( this.edgeTriangles[end >> 1] != 2 ) {
                    if( boundary == 0 ) {
                        p = neighbour;
                    }
                    else if( boundary == 1 ) {
                        q = neighbour;
                    }
                    ++boundary;
                }
            }

            // the sheet of each triangle, as the triangle that stands for it, and how often the sheet touches p and q
            this.scratch.Clear( );
            this.pairs.Clear( );

            var index = 0;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner], ++index ) {
                this.scratch.Add( index );
                this.pairs.Add( int2.zero );

                var corners = this.triangles[corner / 3];

                for( var other = 0; other < 3; ++other ) {
                    var neighbour = corners[other];

                    if( neighbour == vertex || this.marks[neighbour] == 0 ) {
                        continue;
                    }
                    if( this.notes[neighbour] == -1 ) {
                        this.notes[neighbour] = index;
                    }
                    else {
                        this.scratch[this.sheetOf( index )] = this.sheetOf( this.notes[neighbour] );
                    }
                }
            }

            if( index == 0 ) {
                return 0;
            }

            index = 0;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner], ++index ) {
                var sheet = this.sheetOf( index );

                this.pairs[sheet] += new int2( this.has( corner / 3, p ) ? 1 : 0, this.has( corner / 3, q ) ? 1 : 0 );
            }

            var sheets = 0;
            var across = true;

            for( var triangle = 0; triangle < index; ++triangle ) {
                if( this.scratch[triangle] == triangle ) {
                    ++sheets;
                    across &= math.all( this.pairs[triangle] == 1 );
                }
            }

            if( boundary == 0 ) {
                return ( byte )( sheets == 1 ? 0 : 2 );
            }

            return ( byte )( boundary == 2 && across ? 1 : 2 );
        }

        private int sheetOf( int triangle ) {
            while( this.scratch[triangle] != triangle ) {
                this.scratch[triangle] = this.scratch[this.scratch[triangle]];

                triangle = this.scratch[triangle];
            }
            return triangle;
        }

        // two triangles on the same three vertices are not part of a simplicial complex, which is what the link
        // conditions are stated for. Their vertices are locked, and no edge at a locked vertex is collapsed, so such
        // triangles leave as they came. Every such set is found at its lowest vertex
        private void lockRepeatedTriangles( ) {
            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                this.pairs.Clear( );

                for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                    var corners = this.triangles[corner / 3];

                    if( math.cmin( corners ) != vertex ) {
                        continue;
                    }

                    var high   = math.cmax( corners );
                    var others = new int2( corners.x + corners.y + corners.z - vertex - high, high );

                    for( var earlier = 0; earlier < this.pairs.Length; ++earlier ) {
                        if( math.all( this.pairs[earlier] == others ) ) {
                            this.locked[vertex]   = true;
                            this.locked[others.x] = true;
                            this.locked[others.y] = true;
                        }
                    }

                    this.pairs.Add( others );
                }
            }
        }

        // the quadrics and the areas of every vertex, one after the other
        public void buildQuadrics( double lineWeight ) {
            var job = this.quadricsJob( lineWeight );

            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                job.Execute( vertex );
            }
        }

        // the job that works out the quadrics and the areas of the vertices
        public QuadricsJob quadricsJob( double lineWeight ) {
            return new QuadricsJob {
                lineWeight   = lineWeight,
                positions    = this.positions,
                triangles    = this.triangles,
                firstCorners = this.firstCorners,
                nextCorners  = this.nextCorners,
                firstEdges   = this.firstEdges,
                nextEdges    = this.nextEdges.AsArray( ),
                edges        = this.edges.AsArray( ),
                edgeSharp    = this.edgeSharp.AsArray( ),
                planes       = this.planes,
                areas        = this.areas,
                sharpPlanes  = this.sharpPlanes,
                sharpAreas   = this.sharpAreas,
                lines        = this.lines
            };
        }

        // whether collapsing an edge into a vertex at the position keeps the mesh what it is: its topology (Dey et al.,
        // theorem B), the topology of the lines its sharp edges form (Hoppe, section 4.5), and the facing of its
        // triangles (Garland and Heckbert, section 6)
        public Refusal check( int edge, double3 position ) {
            var a = this.edges[edge].x;
            var b = this.edges[edge].y;

            if( this.locked[a] || this.locked[b] ) {
                return Refusal.Locked;
            }

            var stamp = this.nextStamp( );

            // the vertices next to a, with the edge to each
            for( var end = this.firstEdges[a]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, a );

                this.stamps[neighbour] = stamp;
                this.notes[neighbour]  = end >> 1;
                this.marks[neighbour]  = 0;
            }

            // the link of the edge: the vertices opposite to it in its triangles
            for( var corner = this.firstCorners[a]; corner != -1; corner = this.nextCorners[corner] ) {
                if( this.has( corner / 3, b ) ) {
                    this.marks[this.third( corner / 3, a, b )] |= Opposite;
                }
            }

            // the vertices in the links of both ends
            var boundaryEdges = false;
            var sharpEdges    = false;

            for( var end = this.firstEdges[b]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, b );

                if( neighbour == a || this.stamps[neighbour] != stamp ) {
                    continue;
                }
                if( ( this.marks[neighbour] & Opposite ) == 0 ) {
                    return Refusal.LinkVertex;
                }

                this.marks[neighbour] |= Common;

                var toA = this.notes[neighbour];
                var toB = end >> 1;

                boundaryEdges |= this.edgeTriangles[toA] != 2 && this.edgeTriangles[toB] != 2;
                sharpEdges    |= this.edgeSharp[toA] != Sharp.None && this.edgeSharp[toB] != Sharp.None;
            }

            // the edges in the links of both ends: a triangle of a and a triangle of b on the same two other vertices.
            // The link of the edge has no edges
            this.pairs.Clear( );

            for( var corner = this.firstCorners[a]; corner != -1; corner = this.nextCorners[corner] ) {
                if( this.oppositeBoth( stamp, corner, a, b, out var others ) ) {
                    this.pairs.Add( others );
                }
            }
            for( var corner = this.firstCorners[b]; corner != -1; corner = this.nextCorners[corner] ) {
                if( !this.oppositeBoth( stamp, corner, b, a, out var others ) ) {
                    continue;
                }
                for( var earlier = 0; earlier < this.pairs.Length; ++earlier ) {
                    if( math.all( this.pairs[earlier] == others ) ) {
                        return Refusal.LinkEdge;
                    }
                }
            }

            // the dummy vertex that Dey et al. join to everything in the first boundary is in the link of a vertex of
            // that boundary, and in the link of the edge when the edge is of that boundary
            if( this.orders[a] >= 1 && this.orders[b] >= 1 && this.edgeTriangles[edge] == 2 ) {
                return Refusal.LinkBoundary;
            }
            if( boundaryEdges ) {
                return Refusal.LinkBoundaryEdge;
            }

            // and it is joined to every vertex of order 2, which puts it in the link of each within the first boundary
            if( this.orders[a] == 2 && this.orders[b] == 2 ) {
                return Refusal.LinkCorners;
            }

            var sharp  = this.edgeSharp[edge] != Sharp.None;
            var countA = this.sharpCounts[a];
            var countB = this.sharpCounts[b];

            if( sharpEdges ) {
                return Refusal.SharpNeighbour;
            }
            if( countA >= 1 && countB >= 1 && !sharp ) {
                return Refusal.SharpEnds;
            }
            if( sharp && countA >= 3 && countB >= 3 ) {
                return Refusal.SharpCorners;
            }
            if( sharp && ( ( countA == 1 && countB != 2 ) || ( countB == 1 && countA != 2 ) ) ) {
                return Refusal.SharpDart;
            }

            // what neither paper tests: see Refusal.Points
            var pointA = this.orders[a] == 2 || countA == 1 || countA >= 3;
            var pointB = this.orders[b] == 2 || countB == 1 || countB >= 3;

            if( pointA && pointB ) {
                return Refusal.Points;
            }
            if( !sharp && ( ( this.orders[a] == 2 && countB >= 1 ) || ( this.orders[b] == 2 && countA >= 1 ) ) ) {
                return Refusal.Points;
            }

            if( this.turnsOver( a, b, position ) || this.turnsOver( b, a, position ) ) {
                return Refusal.Inversion;
            }

            return Refusal.None;
        }

        // whether the triangle of a corner of a vertex stays when the edge to the other vertex is collapsed, with both
        // of its other vertices in the links of both. Gives those two, the lower first
        private readonly bool oppositeBoth( int stamp, int corner, int vertex, int other, out int2 others ) {
            var corners = this.triangles[corner / 3];

            others = default;

            if( math.any( corners == other ) ) {
                return false;
            }

            var x = corners[( ( corner % 3 ) + 1 ) % 3];
            var y = corners[( ( corner % 3 ) + 2 ) % 3];

            if( this.stamps[x] != stamp || this.stamps[y] != stamp || ( this.marks[x] & Common ) == 0 || ( this.marks[y] & Common ) == 0 ) {
                return false;
            }

            others = new int2( math.min( x, y ), math.max( x, y ) );

            return true;
        }

        // whether a triangle of the vertex that stays when the edge to the other vertex is collapsed would face away
        // from where it faces now, or be left without area, with the vertex at the position
        private readonly bool turnsOver( int vertex, int other, double3 position ) {
            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];

                if( math.any( corners == other ) ) {
                    continue;
                }

                var p0 = this.positions[corners.x];
                var p1 = this.positions[corners.y];
                var p2 = this.positions[corners.z];

                var before = math.cross( p1 - p0, p2 - p0 );

                p0 = corners.x == vertex ? position : p0;
                p1 = corners.y == vertex ? position : p1;
                p2 = corners.z == vertex ? position : p2;

                var after = math.cross( p1 - p0, p2 - p0 );

                if( !( math.dot( before, after ) > 0.0 ) ) {
                    return true;
                }
            }
            return false;
        }

        // collapses an edge: the lower of its two vertices is kept, at the position, and stands for both from then on.
        // It takes the triangles and edges of the other, the triangles of the edge go, and of two edges to the same
        // vertex one goes. Gives the vertex kept. The edges that went are in removedEdges
        public int collapse( int edge, double3 position ) {
            var kept    = math.min( this.edges[edge].x, this.edges[edge].y );
            var removed = math.max( this.edges[edge].x, this.edges[edge].y );

            this.removedEdges.Clear( );

            // the triangles of the removed vertex
            var next = -1;

            for( var corner = this.firstCorners[removed]; corner != -1; corner = next ) {
                next = this.nextCorners[corner];

                var triangle = corner / 3;

                if( this.has( triangle, kept ) ) {
                    for( var other = 0; other < 3; ++other ) {
                        if( this.triangles[triangle][other] != removed ) {
                            this.unlinkCorner( this.triangles[triangle][other], ( triangle * 3 ) + other );
                        }
                    }

                    this.triangles[triangle] = -1;
                    this.subMeshes[triangle] = -1;
                }
                else {
                    var corners = this.triangles[triangle];

                    corners[corner % 3] = kept;

                    this.triangles[triangle]  = corners;
                    this.nextCorners[corner]  = this.firstCorners[kept];
                    this.firstCorners[kept]   = corner;
                }
            }

            this.firstCorners[removed] = -1;

            // its edges. What the sharp edges of the kept vertex add to the counts of the vertices around it is taken
            // off first, and put back below when the edges are known again
            var stamp = this.nextStamp( );

            for( var end = this.firstEdges[kept]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, kept );

                this.stamps[neighbour] = stamp;

                if( this.edgeSharp[end >> 1] != Sharp.None ) {
                    --this.sharpCounts[neighbour];
                }
            }

            for( var end = this.firstEdges[removed]; end != -1; end = next ) {
                next = this.nextEdges[end];

                var neighbour = this.across( end >> 1, removed );

                if( neighbour != kept && this.edgeSharp[end >> 1] != Sharp.None ) {
                    --this.sharpCounts[neighbour];
                }

                if( neighbour == kept || this.stamps[neighbour] == stamp ) {
                    // the collapsed edge itself, or an edge to a vertex the kept vertex has an edge to already
                    this.unlinkEdge( neighbour, end >> 1 );

                    this.edges[end >> 1] = -1;
                    this.removedEdges.Add( end >> 1 );
                }
                else {
                    var ends = this.edges[end >> 1];

                    ends[end & 1] = kept;

                    this.edges[end >> 1]  = ends;
                    this.nextEdges[end]   = this.firstEdges[kept];
                    this.firstEdges[kept] = end;
                }
            }

            this.firstEdges[removed] = -1;

            // the new vertex takes the sum of the two normals, each weighted by the area its vertex stands for
            var normal = ( this.normals[kept] * this.areas[kept] ) + ( this.normals[removed] * this.areas[removed] );
            var length = math.length( normal );

            this.positions[kept]  = position;
            this.normals[kept]    = length > 0.0 ? normal / length : double3.zero;
            this.areas[kept]     += this.areas[removed];
            this.planes[kept]    += this.planes[removed];

            this.sharpAreas[kept]  += this.sharpAreas[removed];
            this.sharpPlanes[kept] += this.sharpPlanes[removed];

            this.lines[kept] += this.lines[removed];

            this.sharpCounts[removed] = 0;
            this.orders[removed]      = 0;

            this.refresh( kept );

            for( var end = this.firstEdges[kept]; end != -1; end = this.nextEdges[end] ) {
                if( this.edgeSharp[end >> 1] != Sharp.None ) {
                    ++this.sharpCounts[this.across( end >> 1, kept )];
                }
            }

            return kept;
        }

        private void unlinkCorner( int vertex, int corner ) {
            if( this.firstCorners[vertex] == corner ) {
                this.firstCorners[vertex] = this.nextCorners[corner];

                return;
            }
            for( var before = this.firstCorners[vertex]; before != -1; before = this.nextCorners[before] ) {
                if( this.nextCorners[before] == corner ) {
                    this.nextCorners[before] = this.nextCorners[corner];

                    return;
                }
            }
        }

        private void unlinkEdge( int vertex, int edge ) {
            var end = this.edges[edge].x == vertex ? edge * 2 : ( edge * 2 ) + 1;

            if( this.firstEdges[vertex] == end ) {
                this.firstEdges[vertex] = this.nextEdges[end];

                return;
            }
            for( var before = this.firstEdges[vertex]; before != -1; before = this.nextEdges[before] ) {
                if( this.nextEdges[before] == end ) {
                    this.nextEdges[before] = this.nextEdges[end];

                    return;
                }
            }
        }

        // the mesh as it is now: the vertices that have triangles, in their order, and the triangles, in theirs
        public void write( NativeList<float3> positions, NativeList<float3> normals, NativeList<BurstMesh.Triangle> triangles ) {
            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                if( this.firstCorners[vertex] == -1 ) {
                    continue;
                }

                this.notes[vertex] = positions.Length;

                positions.Add( ( float3 )this.positions[vertex] );
                normals.Add( ( float3 )this.normals[vertex] );
            }

            for( var triangle = 0; triangle < this.triangleCount; ++triangle ) {
                if( this.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.triangles[triangle];

                triangles.Add( new BurstMesh.Triangle( this.notes[corners.x], this.notes[corners.y], this.notes[corners.z], this.subMeshes[triangle] ) );
            }
        }

    }


    // what the planes that hold the sharp edges in place weigh against the planes of the triangles, in where the
    // vertex of a collapsed edge goes and in which edge is collapsed first. How far a vertex may leave either kind of
    // plane is the error bound's to say, for each kind by itself (evaluate), and with that bound the weight changes
    // little: from 0.1 to 100 the lines between materials come out the same to within a few edges
    public const double SharpWeight = 1.0;

    // what the point that stands in for a line weighs (Liu et al., section 4.2), times the area of its vertex
    public const double PointWeight = 1e-6;

    // the three quadrics of a vertex, and the areas the first two are means over.
    // - The planes of its triangles, each weighted by a third of the triangle's area, and the sum of those thirds
    //   (Liu et al., equation 8).
    // - For every sharp edge of the vertex and every triangle of that edge, the plane through the edge that is square
    //   to the triangle (Garland and Heckbert, section 6), weighted by the same third, and the sum of those.
    // - Its line: the squared distance to the line through the vertex along the normal of its triangles, which is
    //   the sum of their normals weighted by area (Liu et al., equations 12 to 15), times the line weight and the
    //   area of the vertex (equation 16). A vertex whose triangles' normals cancel has no line, and gets the squared
    //   distance to itself with a weight that is next to nothing. At a line weight of zero a vertex has neither.
    // The first two are kept apart because an error bound that is a distance has to divide each by its own area. A
    // vertex of a line across a flat face comes to stand for a great deal of the face, where every collapse costs
    // nothing, and for very little line. Divided by all of that area, no error of the line would ever be over the
    // bound. The third is kept apart because it is no distance to the surface at all, and the bound is not for it:
    // on a flat face it grows with every collapse, and a bound on it would stop a cube short of twelve triangles
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct QuadricsJob : IJobParallelFor {

        public double lineWeight;

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     firstCorners;
        [ReadOnly] public NativeArray<int>     nextCorners;
        [ReadOnly] public NativeArray<int>     firstEdges;
        [ReadOnly] public NativeArray<int>     nextEdges;
        [ReadOnly] public NativeArray<int2>    edges;
        [ReadOnly] public NativeArray<Sharp>   edgeSharp;

        [WriteOnly] public NativeArray<BurstQuadric> planes;
        [WriteOnly] public NativeArray<double>       areas;
        [WriteOnly] public NativeArray<BurstQuadric> sharpPlanes;
        [WriteOnly] public NativeArray<double>       sharpAreas;
        [WriteOnly] public NativeArray<BurstQuadric> lines;

        public void Execute( int vertex ) {
            var sum       = default( BurstQuadric );
            var area      = 0.0;
            var sharpSum  = default( BurstQuadric );
            var sharpArea = 0.0;
            var normal    = double3.zero;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];

                if( !BurstQuadric.triangle( this.positions[corners.x], this.positions[corners.y], this.positions[corners.z], out var plane, out var triangleArea ) ) {
                    continue;
                }

                sum  += plane * ( triangleArea / 3.0 );
                area += triangleArea / 3.0;

                // the normal of the triangle times twice its area
                normal += math.cross( this.positions[corners.y] - this.positions[corners.x], this.positions[corners.z] - this.positions[corners.x] );

                // the two edges of the triangle that the vertex is an end of
                for( var side = 1; side <= 2; ++side ) {
                    var end      = corners[( ( corner % 3 ) + side ) % 3];
                    var opposite = corners[( ( corner % 3 ) + 3 - side ) % 3];

                    if( this.sharp( vertex, end ) && BurstQuadric.boundary( this.positions[vertex], this.positions[end], this.positions[opposite], out var square ) ) {
                        sharpSum  += square * ( triangleArea / 3.0 );
                        sharpArea += triangleArea / 3.0;
                    }
                }
            }

            this.planes[vertex]      = sum;
            this.areas[vertex]       = area;
            this.sharpPlanes[vertex] = sharpSum;
            this.sharpAreas[vertex]  = sharpArea;

            var line = default( BurstQuadric );

            if( this.lineWeight > 0.0 ) {
                line = BurstQuadric.line( this.positions[vertex], normal, out var along )
                    ? along * ( this.lineWeight * area )
                    : BurstQuadric.point( this.positions[vertex] ) * ( PointWeight * area );
            }

            this.lines[vertex] = line;
        }

        private readonly bool sharp( int vertex, int other ) {
            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var ends = this.edges[end >> 1];

                if( ends.x == other || ends.y == other ) {
                    return this.edgeSharp[end >> 1] != Sharp.None;
                }
            }
            return false;
        }

    }

    // where the vertex of a collapsed edge goes and what the collapse costs: the point of least error of the quadrics of
    // both ends and the error there. Where there is no such point, the best point of the edge, and where the error
    // does not change along the edge, the best of its middle and its ends (Garland and Heckbert, section 4). The middle
    // is taken unless an end is better by more than the error may change along an edge without having a best point on
    // it (BurstQuadric.minimizeBetween), and the first end unless the second is better by as much. On a flat face that
    // makes it the middle whether rounding leaves the errors exactly equal or not. Gives whether the quadric had a
    // point of least error
    public static bool targetOf( in BurstQuadric quadric, double3 first, double3 second, out double3 position, out double cost ) {
        if( quadric.minimize( out position ) ) {
            cost = quadric.evaluate( position );

            return true;
        }
        if( quadric.minimizeBetween( first, second, out position ) ) {
            cost = quadric.evaluate( position );

            return false;
        }

        position = ( first + second ) * 0.5;
        cost     = quadric.evaluate( position );

        var atFirst   = quadric.evaluate( first );
        var atSecond  = quadric.evaluate( second );
        var tolerance = BurstQuadric.PivotLimit * quadric.trace * math.distancesq( first, second );

        if( atFirst < cost - tolerance ) {
            position = first;
            cost     = atFirst;
        }
        if( atSecond < cost - tolerance ) {
            position = second;
            cost     = atSecond;
        }

        return false;
    }

    // an edge as the loop sees it: where its vertex would go, what collapsing it costs, and whether that is within the
    // error bound. It is given the three quadrics of both ends together, the areas they are means over, and the ends.
    // - The target and the cost are those of the three quadrics together, the planes of the sharp edges times their
    //   weight (targetOf): the lines are part of what a collapse costs and of where its vertex goes, as Liu et al.
    //   have it, and no part of the bound.
    // - The limit is the largest mean squared distance that is allowed. The error of the planes of the triangles at
    //   the target, over the area the two ends stand for, is such a mean, weighted by area: the squared distance to
    //   the surface the new vertex stands for. The error of the planes of the sharp edges over their area is the
    //   squared distance the sharp lines have moved sideways. Each has to be within the limit by itself.
    // Gives whether the quadrics together had a point of least error
    public static bool evaluate(
        in BurstQuadric planes,
        double          area,
        in BurstQuadric sharpPlanes,
        double          sharpArea,
        in BurstQuadric lines,
        double3         first,
        double3         second,
        double          limit,
        out double3     target,
        out double      cost,
        out bool        within
    ) {
        var best = targetOf( planes + ( sharpPlanes * SharpWeight ) + lines, first, second, out target, out cost );

        within = planes.evaluate( target ) <= limit * area && sharpPlanes.evaluate( target ) <= limit * sharpArea;

        return best;
    }

    // the target and the cost of every edge, and whether it is within the error bound
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct CostsJob : IJobParallelFor {

        public double limit;

        [ReadOnly] public NativeArray<double3>      positions;
        [ReadOnly] public NativeArray<BurstQuadric> planes;
        [ReadOnly] public NativeArray<double>       areas;
        [ReadOnly] public NativeArray<BurstQuadric> sharpPlanes;
        [ReadOnly] public NativeArray<double>       sharpAreas;
        [ReadOnly] public NativeArray<BurstQuadric> lines;
        [ReadOnly] public NativeArray<int2>         edges;

        [WriteOnly] public NativeArray<double3> targets;
        [WriteOnly] public NativeArray<double>  costs;
        [WriteOnly] public NativeArray<bool>    within;

        public void Execute( int edge ) {
            var a = this.edges[edge].x;
            var b = this.edges[edge].y;

            evaluate(
                this.planes[a] + this.planes[b], this.areas[a] + this.areas[b], this.sharpPlanes[a] + this.sharpPlanes[b], this.sharpAreas[a] + this.sharpAreas[b],
                this.lines[a] + this.lines[b], this.positions[a], this.positions[b], this.limit, out var target, out var cost, out var within
            );

            this.targets[edge] = target;
            this.costs[edge]   = cost;
            this.within[edge]  = within;
        }

    }

    // what a simplification did
    public struct Statistics {

        public int collapses;

        // collapses whose new vertex is more than two lengths of its edge from the middle of the edge, which the best
        // point of a quadric can be where the surface is almost flat, and edges that were not collapsed with such a point
        public int farCollapses;
        public int farRefusals;

        // how many edges were worked out again, in all: after every collapse, the edges of the vertex it left. And
        // how many of those had no point of least error, so that their vertex went to a point of the edge instead
        public int evaluations;
        public int withoutBestPoint;

    }

    // takes the mesh of a contour into a connectivity
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct BuildJob : IJob {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<float3>             normals;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        public Connectivity mesh;

        public void Execute( ) {
            this.mesh.build( this.positions, this.normals, this.triangles );
        }

    }

    // Garland and Heckbert's loop (section 4.1): the edge that costs least is collapsed, and the costs of the edges of
    // the vertex it leaves are worked out again, until no edge is left to collapse.
    //
    // The edges wait in a heap, the cheapest at the top. Two that cost the same go by area and then by index (before),
    // so that the same mesh is always simplified the same way. An edge is not in the heap while its cost is over the error
    // bound, or while check refuses it. A cost changes only with the quadric or the position of an end of the edge,
    // which is when that end is the vertex a collapse leaves. What check says can change with any collapse that leaves
    // a vertex next to an end of the edge, so the refused edges of those vertices go back into the heap
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct CollapseJob : IJob {

        // the largest mean squared distance that a collapse may leave a vertex at (evaluate)
        public double limit;

        public Connectivity mesh;

        // for every edge, from evaluate
        public NativeArray<double3> targets;
        public NativeArray<double>  costs;
        public NativeArray<bool>    within;

        // the heap, as the edges in it, and for every edge its place in the heap or why it is not in it
        public NativeArray<int> heap;
        public NativeArray<int> places;

        public NativeList<float3>             positions;
        public NativeList<float3>             normals;
        public NativeList<BurstMesh.Triangle> triangles;

        // one statistics, and how often check gave each refusal
        public NativeArray<Statistics> statistics;
        public NativeArray<int>        refusals;

        private int count;

        private const int Out     = -1; // over the error bound, or removed
        private const int Waiting = -2; // refused by check

        public void Execute( ) {
            this.count = 0;

            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                if( this.mesh.edges[edge].x != -1 && this.within[edge] ) {
                    this.heap[this.count] = edge;
                    this.places[edge]     = this.count++;
                }
                else {
                    this.places[edge] = Out;
                }
            }
            for( var place = ( this.count / 2 ) - 1; place >= 0; --place ) {
                this.sinkFrom( place );
            }

            var statistics = default( Statistics );

            while( this.count > 0 ) {
                var edge = this.heap[0];

                this.removeAt( 0 );
                this.places[edge] = Out;

                var ends   = this.mesh.edges[edge];
                var middle = ( this.mesh.positions[ends.x] + this.mesh.positions[ends.y] ) * 0.5;
                var far    = math.distancesq( this.targets[edge], middle ) > 4.0 * math.distancesq( this.mesh.positions[ends.x], this.mesh.positions[ends.y] );

                var refusal = this.mesh.check( edge, this.targets[edge] );

                if( refusal != Refusal.None ) {
                    this.places[edge] = Waiting;

                    ++this.refusals[( int )refusal];

                    if( far ) {
                        ++statistics.farRefusals;
                    }
                    continue;
                }

                // the edges of both ends leave the heap before the collapse changes what they are ordered by, or
                // removes them
                this.takeOutEdgesOf( ends.x );
                this.takeOutEdgesOf( ends.y );

                var kept = this.mesh.collapse( edge, this.targets[edge] );

                ++statistics.collapses;

                if( far ) {
                    ++statistics.farCollapses;
                }

                // the edges of the kept vertex cost what its new quadric and position make them cost
                for( var end = this.mesh.firstEdges[kept]; end != -1; end = this.mesh.nextEdges[end], ++statistics.evaluations ) {
                    var changed = end >> 1;
                    var a       = this.mesh.edges[changed].x;
                    var b       = this.mesh.edges[changed].y;

                    var best = evaluate(
                        this.mesh.planes[a] + this.mesh.planes[b], this.mesh.areas[a] + this.mesh.areas[b], this.mesh.sharpPlanes[a] + this.mesh.sharpPlanes[b], this.mesh.sharpAreas[a] + this.mesh.sharpAreas[b],
                        this.mesh.lines[a] + this.mesh.lines[b], this.mesh.positions[a], this.mesh.positions[b], this.limit, out var target, out var cost, out var within
                    );

                    if( !best ) {
                        ++statistics.withoutBestPoint;
                    }

                    this.targets[changed] = target;
                    this.costs[changed]   = cost;
                    this.within[changed]  = within;

                    if( within ) {
                        this.add( changed );
                    }
                    else {
                        this.places[changed] = Out;
                    }
                }

                // and the edges of the vertices next to it that check refused may no longer be refused
                for( var end = this.mesh.firstEdges[kept]; end != -1; end = this.mesh.nextEdges[end] ) {
                    var neighbour = this.mesh.across( end >> 1, kept );

                    for( var other = this.mesh.firstEdges[neighbour]; other != -1; other = this.mesh.nextEdges[other] ) {
                        if( this.places[other >> 1] == Waiting ) {
                            this.add( other >> 1 );
                        }
                    }
                }
            }

            this.statistics[0] = statistics;

            this.mesh.write( this.positions, this.normals, this.triangles );
        }

        private void takeOutEdgesOf( int vertex ) {
            for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                if( this.places[end >> 1] >= 0 ) {
                    this.removeAt( this.places[end >> 1] );
                }
                this.places[end >> 1] = Out;
            }
        }

        // whether an edge goes before another: the cheaper of the two. Of two that cost the same, the one whose ends
        // stand for less area, and of two that are the same in that too, the lower.
        //
        // On a flat face that lies along the grid every edge costs exactly nothing. Going by the index alone, the
        // vertex of the first edge then takes one neighbour after another, and ends up with every triangle of the
        // face around it: thousands of edges, each of them to be worked out again at every collapse. Going by area
        // first, a vertex that has taken a neighbour waits until the vertices around it have done the same
        private readonly bool before( int edge, int other ) {
            if( this.costs[edge] != this.costs[other] ) {
                return this.costs[edge] < this.costs[other];
            }

            var area      = this.areaOf( edge );
            var otherArea = this.areaOf( other );

            return area < otherArea || ( area == otherArea && edge < other );
        }

        // the area the two ends of an edge stand for
        private readonly double areaOf( int edge ) {
            var ends = this.mesh.edges[edge];

            return this.mesh.areas[ends.x] + this.mesh.areas[ends.y];
        }

        private void put( int place, int edge ) {
            this.heap[place]  = edge;
            this.places[edge] = place;
        }

        private void add( int edge ) {
            this.put( this.count++, edge );
            this.riseFrom( this.count - 1 );
        }

        // takes the edge at a place out of the heap. Its own place is the caller's to set
        private void removeAt( int place ) {
            var last = this.heap[--this.count];

            if( place == this.count ) {
                return;
            }

            this.put( place, last );
            this.riseFrom( place );
            this.sinkFrom( this.places[last] );
        }

        private void riseFrom( int place ) {
            var edge = this.heap[place];

            while( place > 0 ) {
                var parent = ( place - 1 ) / 2;

                if( !this.before( edge, this.heap[parent] ) ) {
                    break;
                }

                this.put( place, this.heap[parent] );

                place = parent;
            }

            this.put( place, edge );
        }

        private void sinkFrom( int place ) {
            var edge = this.heap[place];

            while( true ) {
                var child = ( place * 2 ) + 1;

                if( child >= this.count ) {
                    break;
                }
                if( child + 1 < this.count && this.before( this.heap[child + 1], this.heap[child] ) ) {
                    ++child;
                }
                if( !this.before( this.heap[child], edge ) ) {
                    break;
                }

                this.put( place, this.heap[child] );

                place = child;
            }

            this.put( place, edge );
        }

    }

    // reads a mesh into arrays: its vertices, their normals, and its triangles with the sub-mesh of each
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct ReadMeshJob : IJob {

        [ReadOnly] public Mesh.MeshDataArray meshes;

        public NativeArray<float3>             positions;
        public NativeArray<float3>             normals;
        public NativeArray<BurstMesh.Triangle> triangles;

        public void Execute( ) {
            var mesh = this.meshes[0];

            mesh.GetVertices( this.positions.Reinterpret<Vector3>( ) );
            mesh.GetNormals( this.normals.Reinterpret<Vector3>( ) );

            var triangle = 0;

            for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
                var indices = new NativeArray<int>( mesh.GetSubMesh( subMesh ).indexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );

                mesh.GetIndices( indices, subMesh );

                for( var index = 0; index < indices.Length; index += 3 ) {
                    this.triangles[triangle++] = new BurstMesh.Triangle( indices[index], indices[index + 1], indices[index + 2], subMesh );
                }

                indices.Dispose( );
            }
        }

    }

    private static readonly ProfilerMarker readMeshMarker  = new( "MeshSimplifier.Burst.ReadMesh" );
    private static readonly ProfilerMarker buildMarker     = new( "MeshSimplifier.Burst.Build" );
    private static readonly ProfilerMarker quadricsMarker  = new( "MeshSimplifier.Burst.Quadrics" );
    private static readonly ProfilerMarker costsMarker     = new( "MeshSimplifier.Burst.Costs" );
    private static readonly ProfilerMarker collapseMarker  = new( "MeshSimplifier.Burst.Collapse" );
    private static readonly ProfilerMarker buildMeshMarker = new( "MeshSimplifier.Burst.BuildMesh" );

    // how many vertices and edges a worker takes at a time
    private const int vertexBatch = 256;
    private const int edgeBatch   = 256;

    // the mesh of a contour, simplified: a new mesh with the same sub-meshes, and vertices with normals that its
    // triangles share. The error is how far a collapse may leave a vertex from the planes of the surface it stands
    // for, as a root mean square distance in the units of the mesh. The line weight is that of the lines of the
    // vertices against the planes of their triangles (QuadricsJob), which has no unit
    public static Mesh simplify( Mesh mesh, double error, double lineWeight, out Statistics statistics ) {
        var triangleCount = 0;

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            triangleCount += ( int )mesh.GetIndexCount( subMesh ) / 3;
        }

        using var positions = new NativeArray<float3>( mesh.vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var normals   = new NativeArray<float3>( mesh.vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var triangles = new NativeArray<BurstMesh.Triangle>( triangleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        using( readMeshMarker.Auto( ) ) {
            using var meshes = Mesh.AcquireReadOnlyMeshData( mesh );

            new ReadMeshJob {
                meshes    = meshes,
                positions = positions,
                normals   = normals,
                triangles = triangles
            }.Run( );
        }

        return simplify( positions, normals, triangles, error, lineWeight, out statistics );
    }

    public static Mesh simplify( NativeArray<float3> positions, NativeArray<float3> normals, NativeArray<BurstMesh.Triangle> triangles, double error, double lineWeight, out Statistics statistics ) {
        using var connectivity = new Connectivity( positions.Length, triangles.Length, Allocator.TempJob );

        using( buildMarker.Auto( ) ) {
            new BuildJob {
                positions = positions,
                normals   = normals,
                triangles = triangles,
                mesh      = connectivity
            }.Run( );
        }

        using( quadricsMarker.Auto( ) ) {
            connectivity.quadricsJob( lineWeight ).Schedule( connectivity.vertexCount, vertexBatch ).Complete( );
        }

        using var targets            = new NativeArray<double3>( connectivity.edgeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var costs              = new NativeArray<double>( connectivity.edgeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var within             = new NativeArray<bool>( connectivity.edgeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var heap               = new NativeArray<int>( connectivity.edgeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var places             = new NativeArray<int>( connectivity.edgeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var simplifiedPosition = new NativeList<float3>( Allocator.TempJob );
        using var simplifiedNormals  = new NativeList<float3>( Allocator.TempJob );
        using var simplifiedTriangle = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
        using var statisticsOut      = new NativeArray<Statistics>( 1, Allocator.TempJob );
        using var refusals           = new NativeArray<int>( Enum.GetValues( typeof( Refusal ) ).Length, Allocator.TempJob );

        using( costsMarker.Auto( ) ) {
            new CostsJob {
                limit       = error * error,
                positions   = connectivity.positions,
                planes      = connectivity.planes,
                areas       = connectivity.areas,
                sharpPlanes = connectivity.sharpPlanes,
                sharpAreas  = connectivity.sharpAreas,
                lines       = connectivity.lines,
                edges       = connectivity.edges.AsArray( ),
                targets     = targets,
                costs       = costs,
                within      = within
            }.Schedule( connectivity.edgeCount, edgeBatch ).Complete( );
        }

        using( collapseMarker.Auto( ) ) {
            new CollapseJob {
                limit       = error * error,
                mesh        = connectivity,
                targets     = targets,
                costs       = costs,
                within      = within,
                heap        = heap,
                places     = places,
                positions  = simplifiedPosition,
                normals    = simplifiedNormals,
                triangles  = simplifiedTriangle,
                statistics = statisticsOut,
                refusals   = refusals
            }.Run( );
        }

        statistics = statisticsOut[0];

        using( buildMeshMarker.Auto( ) ) {
            return BurstMesh.createMesh( simplifiedPosition.AsArray( ), simplifiedNormals.AsArray( ), simplifiedTriangle.AsArray( ) );
        }
    }

}

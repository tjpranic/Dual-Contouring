using System;
using Unity.Collections;
using Unity.Mathematics;

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
        public NativeArray<BurstQuadric> planes;       // the planes of those triangles, and of the sharp edges
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

        // notes on the vertices around the one being looked at. A note counts while its stamp is the current one
        private NativeArray<int>  stamps;
        private NativeArray<int>  notes;
        private NativeArray<byte> marks;
        private NativeList<int>   scratch;
        private NativeList<int2>  pairs;
        private int               stamp;

        private const byte Opposite = 1;
        private const byte Common   = 2;

        public Connectivity( int vertexCount, int triangleCount, Allocator allocator ) {
            this.positions     = new NativeArray<double3>      ( vertexCount, allocator );
            this.normals       = new NativeArray<double3>      ( vertexCount, allocator );
            this.areas         = new NativeArray<double>       ( vertexCount, allocator );
            this.planes        = new NativeArray<BurstQuadric> ( vertexCount, allocator );
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
            this.stamp         = 0;
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.areas.Dispose( );
            this.planes.Dispose( );
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
        private readonly int across( int edge, int vertex ) {
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
                ++this.stamp;

                for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                    var corners = this.triangles[corner / 3];

                    for( var other = 0; other < 3; ++other ) {
                        var neighbour = corners[other];

                        if( neighbour <= vertex || this.stamps[neighbour] == this.stamp ) {
                            continue;
                        }
                        this.stamps[neighbour] = this.stamp;

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
            ++this.stamp;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, vertex );

                this.stamps[neighbour] = this.stamp;
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
            ++this.stamp;

            // the vertices at the far ends of the first two edges that do not have two triangles
            var boundary = 0;
            var p        = -1;
            var q        = -1;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, vertex );

                // the first triangle met on the edge, none yet
                this.stamps[neighbour] = this.stamp;
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

        // the quadric of every vertex: the planes of its triangles, each weighted by a third of the triangle's area
        // (Liu et al., equation 8), and for every sharp edge of the vertex and every triangle of that edge the plane
        // through the edge that is square to the triangle (Garland and Heckbert, section 6). That plane counts as the
        // triangle's own does, times the boundary weight
        public void buildQuadrics( double boundaryWeight ) {
            for( var vertex = 0; vertex < this.vertexCount; ++vertex ) {
                this.planes[vertex] = default;
                this.areas[vertex]  = 0.0;
            }

            for( var triangle = 0; triangle < this.triangleCount; ++triangle ) {
                if( this.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.triangles[triangle];

                if( !BurstQuadric.triangle( this.positions[corners.x], this.positions[corners.y], this.positions[corners.z], out var plane, out var area ) ) {
                    continue;
                }

                for( var corner = 0; corner < 3; ++corner ) {
                    this.planes[corners[corner]] += plane * ( area / 3.0 );
                    this.areas[corners[corner]]  += area / 3.0;
                }

                for( var corner = 0; corner < 3; ++corner ) {
                    var start = corners[corner];
                    var end   = corners[( corner + 1 ) % 3];

                    if( this.edgeSharp[this.edgeBetween( start, end )] == Sharp.None ) {
                        continue;
                    }
                    if( BurstQuadric.boundary( this.positions[start], this.positions[end], this.positions[corners[( corner + 2 ) % 3]], out var square ) ) {
                        this.planes[start] += square * ( boundaryWeight * area / 3.0 );
                        this.planes[end]   += square * ( boundaryWeight * area / 3.0 );
                    }
                }
            }
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

            ++this.stamp;

            // the vertices next to a, with the edge to each
            for( var end = this.firstEdges[a]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, a );

                this.stamps[neighbour] = this.stamp;
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

                if( neighbour == a || this.stamps[neighbour] != this.stamp ) {
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
                if( this.oppositeBoth( corner, a, b, out var others ) ) {
                    this.pairs.Add( others );
                }
            }
            for( var corner = this.firstCorners[b]; corner != -1; corner = this.nextCorners[corner] ) {
                if( !this.oppositeBoth( corner, b, a, out var others ) ) {
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
        private readonly bool oppositeBoth( int corner, int vertex, int other, out int2 others ) {
            var corners = this.triangles[corner / 3];

            others = default;

            if( math.any( corners == other ) ) {
                return false;
            }

            var x = corners[( ( corner % 3 ) + 1 ) % 3];
            var y = corners[( ( corner % 3 ) + 2 ) % 3];

            if( this.stamps[x] != this.stamp || this.stamps[y] != this.stamp || ( this.marks[x] & Common ) == 0 || ( this.marks[y] & Common ) == 0 ) {
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
            ++this.stamp;

            for( var end = this.firstEdges[kept]; end != -1; end = this.nextEdges[end] ) {
                var neighbour = this.across( end >> 1, kept );

                this.stamps[neighbour] = this.stamp;

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

                if( neighbour == kept || this.stamps[neighbour] == this.stamp ) {
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

}

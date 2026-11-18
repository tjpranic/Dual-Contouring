using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using Connectivity = BurstMeshSimplifier.Connectivity;
using Sharp        = BurstMeshSimplifier.Sharp;

// the features of a mesh that a remesh keeps (Documents/Plans/Remeshing.md, findings 6, 9 and 17): the edges where the
// surface sets itself apart, the corners where they end, meet or turn, and the lines they make from corner to corner.
// All three techniques find them the same way, on the mesh the pass is given.
//
// Features come in two strengths (finding 9). The simplifier's sharp edges, which are open, of more than two triangles
// or between materials, are hard: its lists hold their topology. The others are found by angle, at the scale of a
// length l that the technique gives, which is its edge length but no more than a cell (Remeshing.featureLengthOf,
// step 3b), since an edge that crosses the grid makes noise of them (findings 17, 19, 25 and 26):
// - the triangles are grouped into patches, joined across every edge that is not hard and does not bend at the scale
//   of l, which goes by how much the edge bends itself and by the band of triangles within a length l of it:
//   - by no more than half the angle, it does not;
//   - by more than the angle, it does, as at the scale of each edge;
//   - in between, it does where the band bends across it by more than the angle, between its triangles on either
//     side: a crease spread over a strip of edges, each of which bends by less. Or where one side of the band weighs
//     next to nothing against the other, as beside a sliver, whose normal says nothing;
// - a patch thinner than a share of l, as twice its area over its perimeter, is merged into the neighbour it lies
//   best against: a sliver along a crease, a strip of triangles bridging one, a fold;
// - an edge by angle is an edge between two patches, or one that bends at the scale of l inside a patch, as it was
//   found, that was not merged away, which is a crease that fades out;
// - a line by angle shorter than l that ends loose is taken out;
// - a line turns where it turns by more than the angle over a length l on either side, and a tip is where the normals
//   of a vertex's triangles, weighted by area, spread by more than the angle.
// At a length of 0 this is the edge's own angle, as it was before step 3b. Hard features are the same at every length
public static class BurstFeatures {

    // how thin a patch is merged into a neighbour, in edge lengths: as twice its area over its perimeter, which for a
    // strip is its width (step 3b, Measurements)
    public const double thinPatches = 0.5;

    // the share of the angle by which an edge has to bend itself to be cut at the scale of l, and how far either side
    // of it the band reaches, in edge lengths (step 3b, Measurements)
    public const double spreadShare = 0.5;
    public const double bandReach   = 1.0;

    // the corners found by angle on the mesh as it came, which a vertex carries through a remesh (finding 6)
    [Flags]
    public enum Corner : byte {
        None = 0,
        Turn = 1, // a line of two feature edges turns here by more than the angle
        Tip  = 2  // no feature edge, and the normals of the triangles are further than the angle from their mean
    }

    // whether a vertex is a corner: where the simplifier keeps a point of its own (Connectivity.isPoint) whatever the
    // angle; where the feature edges do not make one line through it, which is one of them, or three and more; where a
    // line turned when it was found; and where a tip was found and no line has come to it since. Going by the feature
    // edges a vertex has now, a corner of lines a remesh has collapsed away is no corner any more
    public static bool isCorner( bool point, int featureEdges, Corner found ) {
        if( point || ( featureEdges != 0 && featureEdges != 2 ) ) {
            return true;
        }

        return featureEdges == 2 ? ( found & Corner.Turn ) != 0 : ( found & Corner.Tip ) != 0;
    }

    // the normal of a triangle of the lists, times twice its area
    public static double3 normalOf( in Connectivity mesh, int triangle ) {
        var corners = mesh.triangles[triangle];

        return math.cross( mesh.positions[corners.y] - mesh.positions[corners.x], mesh.positions[corners.z] - mesh.positions[corners.x] );
    }

    // whether two triangles meet at more than the angle whose cosine is given. A triangle without area has no normal,
    // and meets nothing at an angle: both sides are then zero
    public static bool bent( double3 normal, double3 other, double cosine ) {
        return math.dot( normal, other ) < cosine * math.length( normal ) * math.length( other );
    }

    // how many feature edges a vertex has: edges whose line is not -1
    public static int featureEdgesOf( in Connectivity mesh, NativeList<int> edgeLines, int vertex ) {
        var count = 0;

        for( var end = mesh.firstEdges[vertex]; end != -1; end = mesh.nextEdges[end] ) {
            if( edgeLines[end >> 1] != -1 ) {
                ++count;
            }
        }
        return count;
    }

    // the features of a mesh as lists, at an angle given by its cosine and at the scale of a length (the header):
    // - for every edge, the line it is on, or -1 where it is no feature edge. A line runs from corner to corner, or is
    //   a loop without one, and the lines are numbered in the order they are found: from the corners in the order of
    //   their vertices, and then the loops;
    // - for every vertex, the corners found by angle;
    // - how many lines there are.
    // The lists are made as long as the connectivity's
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SkeletonJob : IJob {

        public double       cosine;
        public double       length; // l, or 0 for the scale of each edge
        public Connectivity mesh;

        public NativeList<int>    edgeLines;
        public NativeList<Corner> corners;

        // one
        [WriteOnly] public NativeArray<int> lineCount;

        // a feature edge before it has a line
        private const int Unnumbered = -2;

        // what joins the two triangles of an edge into one patch: nothing, as it is hard or bends by more than the angle
        // at the scale it is found at; the edge; or nothing as a triangle of it has no area, which bends against nothing
        private const byte Cut    = 0;
        private const byte Joined = 1;
        private const byte Apart  = 2;

        public void Execute( ) {
            this.edgeLines.Clear( );
            this.corners.Clear( );

            this.edgeLines.Resize( this.mesh.edgeCount, NativeArrayOptions.UninitializedMemory );
            this.corners.Resize( this.mesh.vertexCount, NativeArrayOptions.ClearMemory );

            this.findFeatureEdges( );

            if( this.length > 0.0 ) {
                this.removeShortLooseLines( );
            }

            // corners by angle, at the vertices of one line and at the vertices of none
            for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                if( this.mesh.firstCorners[vertex] == -1 ) {
                    continue;
                }

                var count = featureEdgesOf( this.mesh, this.edgeLines, vertex );

                if( count == 2 && this.turns( vertex ) ) {
                    this.corners[vertex] = Corner.Turn;
                }
                else if( count == 0 && this.spreads( vertex ) ) {
                    this.corners[vertex] = Corner.Tip;
                }
            }

            // the lines from the corners, then the loops. Away from a corner a vertex has exactly two feature edges,
            // so a line goes on through it by the other one
            var lines = 0;

            for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                if( this.mesh.firstCorners[vertex] == -1 || !this.cornerAt( vertex ) ) {
                    continue;
                }

                for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                    if( this.edgeLines[end >> 1] == Unnumbered ) {
                        this.walk( end >> 1, vertex, lines++ );
                    }
                }
            }

            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                if( this.edgeLines[edge] == Unnumbered ) {
                    this.walk( edge, this.mesh.edges[edge].x, lines++ );
                }
            }

            this.lineCount[0] = lines;
        }

        // every edge's feature or none, from the patches (the header): Unnumbered or -1
        private void findFeatureEdges( ) {
            var edgeCount     = this.mesh.edgeCount;
            var triangleCount = this.mesh.triangleCount;

            // the two triangles of every edge that is not hard, and whether they are joined
            var firsts  = new NativeArray<int>( math.max( edgeCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var seconds = new NativeArray<int>( math.max( edgeCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var joins   = new NativeArray<byte>( math.max( edgeCount, 1 ), Allocator.Temp );

            for( var edge = 0; edge < edgeCount; ++edge ) {
                firsts[edge]  = -1;
                seconds[edge] = -1;
            }

            // the three edges of every triangle, in the order of its corners
            var triangleEdges = new NativeArray<int3>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                if( this.mesh.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.mesh.triangles[triangle];
                var edges   = new int3( );

                for( var k = 0; k < 3; ++k ) {
                    var edge = this.mesh.edgeBetween( corners[k], corners[( k + 1 ) % 3] );

                    edges[k] = edge;

                    if( this.mesh.edgeTriangles[edge] != 2 ) {
                        continue;
                    }
                    if( firsts[edge] == -1 ) {
                        firsts[edge] = triangle;
                    }
                    else {
                        seconds[edge] = triangle;
                    }
                }

                triangleEdges[triangle] = edges;
            }

            // the patches, as sets of triangles
            var patches = new NativeArray<int>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                patches[triangle] = triangle;
            }

            // the normal and the middle of every triangle, for the bands
            var normals   = new NativeArray<double3>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var middles   = new NativeArray<double3>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var stamps    = new NativeArray<int>( math.max( triangleCount, 1 ), Allocator.Temp );
            var band      = new NativeList<int>( 64, Allocator.Temp );
            var stamp     = 0;
            var spread    = math.cos( spreadShare * math.acos( math.clamp( this.cosine, -1.0, 1.0 ) ) );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                if( this.mesh.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.mesh.triangles[triangle];

                normals[triangle] = normalOf( this.mesh, triangle );
                middles[triangle] = ( this.mesh.positions[corners.x] + this.mesh.positions[corners.y] + this.mesh.positions[corners.z] ) / 3.0;
            }

            // a triangle without area has no normal and bends against nothing. At the scale of l it joins nothing
            // either, and is a patch of its own, which is thin and merged away: else such triangles on a crease would
            // join its two sides into one patch
            for( var edge = 0; edge < edgeCount; ++edge ) {
                if( this.mesh.edges[edge].x == -1 || this.mesh.edgeSharp[edge] != Sharp.None || seconds[edge] == -1 ) {
                    continue;
                }

                var first  = normals[firsts[edge]];
                var second = normals[seconds[edge]];

                if( this.length > 0.0 ) {
                    if( math.lengthsq( first ) == 0.0 || math.lengthsq( second ) == 0.0 ) {
                        joins[edge] = Apart;

                        continue;
                    }
                    if( bent( first, second, this.cosine ) ||
                        ( bent( first, second, spread ) && this.bentAcross( edge, firsts, seconds, triangleEdges, normals, middles, stamps, band, ref stamp ) ) ) {
                        continue;
                    }
                }
                else if( bent( first, second, this.cosine ) ) {
                    continue;
                }

                joins[edge] = Joined;

                union( patches, firsts[edge], seconds[edge] );
            }

            normals.Dispose( );
            middles.Dispose( );
            stamps.Dispose( );
            band.Dispose( );

            // the patches as found, before any is merged, and whether each was merged away
            var found    = new NativeArray<int>( math.max( triangleCount, 1 ), Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var absorbed = new NativeArray<bool>( math.max( triangleCount, 1 ), Allocator.Temp );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                found[triangle] = find( patches, triangle );
            }

            if( this.length > 0.0 ) {
                this.mergeThinPatches( patches, found, absorbed, firsts, seconds, joins, triangleEdges );
            }

            for( var edge = 0; edge < edgeCount; ++edge ) {
                var feature = false;

                if( this.mesh.edges[edge].x == -1 ) {
                    feature = false;
                }
                else if( this.mesh.edgeSharp[edge] != Sharp.None ) {
                    feature = true;
                }
                else if( seconds[edge] != -1 && joins[edge] == Cut ) {
                    var first  = firsts[edge];
                    var second = seconds[edge];

                    // between two patches, or a crease inside one that it was found in and that was not merged away
                    feature = find( patches, first ) != find( patches, second ) || ( found[first] == found[second] && !absorbed[found[first]] );
                }
                else if( seconds[edge] != -1 && joins[edge] == Apart ) {
                    // an edge of a triangle without area bends against nothing, and is no crease by itself
                    feature = find( patches, firsts[edge] ) != find( patches, seconds[edge] );
                }

                this.edgeLines[edge] = feature ? Unnumbered : -1;
            }

            firsts.Dispose( );
            seconds.Dispose( );
            joins.Dispose( );
            triangleEdges.Dispose( );
            patches.Dispose( );
            found.Dispose( );
            absorbed.Dispose( );
        }

        // whether the band of triangles round an edge, within a length l of it (inBand), taken from it across edges of
        // two triangles, bends by more than the angle across the edge: between its triangles on either side of it, or where
        // one side weighs less than a tenth of the other. Normals are weighted by area, but by no more than the band's
        // along the edge, its length times the reach, so that a large triangle counts for no more of itself than lies
        // in the band. A triangle is on the side of the edge that its middle is, from the plane through the edge along
        // the mean normal of the band, in which a sliver or a fold has too little area to count
        private readonly bool bentAcross(
            int edge, NativeArray<int> firsts, NativeArray<int> seconds, NativeArray<int3> triangleEdges, NativeArray<double3> normals, NativeArray<double3> middles,
            NativeArray<int> stamps, NativeList<int> band, ref int stamp
        ) {
            var ends  = this.mesh.edges[edge];
            var a     = this.mesh.positions[ends.x];
            var b     = this.mesh.positions[ends.y];
            var reach = bandReach * this.length;
            var most  = 2.0 * reach * math.distance( a, b );

            ++stamp;

            band.Clear( );
            band.Add( firsts[edge] );
            band.Add( seconds[edge] );

            stamps[firsts[edge]]  = stamp;
            stamps[seconds[edge]] = stamp;

            var mean = weighted( normals[firsts[edge]], most ) + weighted( normals[seconds[edge]], most );

            for( var index = 0; index < band.Length; ++index ) {
                var edges = triangleEdges[band[index]];

                for( var k = 0; k < 3; ++k ) {
                    var other = firsts[edges[k]] == band[index] ? seconds[edges[k]] : firsts[edges[k]];

                    if( other == -1 || stamps[other] == stamp ) {
                        continue;
                    }

                    stamps[other] = stamp;

                    if( !inBand( middles[other], a, b, reach ) ) {
                        continue;
                    }

                    band.Add( other );

                    mean += weighted( normals[other], most );
                }
            }

            var across  = math.cross( mean, b - a );
            var sides   = new double3x2( );
            var weights = double2.zero;

            for( var index = 0; index < band.Length; ++index ) {
                var normal = weighted( normals[band[index]], most );

                if( math.dot( across, middles[band[index]] - a ) >= 0.0 ) {
                    sides.c0  += normal;
                    weights.x += math.length( normal );
                }
                else {
                    sides.c1  += normal;
                    weights.y += math.length( normal );
                }
            }

            return math.cmin( weights ) < 0.1 * math.cmax( weights ) || bent( sides.c0, sides.c1, this.cosine );
        }

        // a triangle's normal, of twice its area, cut down to a length
        private static double3 weighted( double3 normal, double most ) {
            var length = math.length( normal );

            return length > most ? normal * ( most / length ) : normal;
        }

        // whether a point is in the band round an edge: within a reach of it. The band reaches along the line the edge
        // is on as well as across it, which finds a crease spread over a strip of edges more often on contours, and
        // carries a crease a reach past where it stops bending by the angle, through edges that still bend by more
        // than half of it (step 3b, Measurements)
        private static bool inBand( double3 point, double3 a, double3 b, double reach ) {
            var ab     = b - a;
            var square = math.lengthsq( ab );
            var along  = square > 0.0 ? math.clamp( math.dot( point - a, ab ) / square, 0.0, 1.0 ) : 0.0;

            return math.distance( point, a + ( ab * along ) ) <= reach;
        }

        private readonly double lengthOf( int edge ) {
            var ends = this.mesh.edges[edge];

            return math.distance( this.mesh.positions[ends.x], this.mesh.positions[ends.y] );
        }

        // a neighbour of a patch, and how well the patch lies against it
        private struct Neighbour {

            public int    patch;
            public double score;
            public double shared;

        }

        // every patch thinner than thinPatches lengths, thinnest first, merged into the neighbour across edges that are
        // not hard that it lies best against: by the length of the edges between them, each counted for more the nearer
        // the two triangles' normals are. A patch that has only hard edges round it stays as it is. A patch that grows
        // into one that is still thin waits its turn again
        private void mergeThinPatches(
            NativeArray<int> patches, NativeArray<int> found, NativeArray<bool> absorbed, NativeArray<int> firsts, NativeArray<int> seconds, NativeArray<byte> joins,
            NativeArray<int3> triangleEdges
        ) {
            var triangleCount = this.mesh.triangleCount;

            // the triangles of every patch, as a list through them from its first to its last, and its area and the
            // length round it
            var heads      = new NativeArray<int>( triangleCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var tails      = new NativeArray<int>( triangleCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var nexts      = new NativeArray<int>( triangleCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var areas      = new NativeArray<double>( triangleCount, Allocator.Temp );
            var perimeters = new NativeArray<double>( triangleCount, Allocator.Temp );

            for( var triangle = 0; triangle < triangleCount; ++triangle ) {
                heads[triangle] = -1;
                tails[triangle] = -1;
                nexts[triangle] = -1;
            }

            for( var triangle = triangleCount - 1; triangle >= 0; --triangle ) {
                if( this.mesh.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var patch = found[triangle];

                if( tails[patch] == -1 ) {
                    tails[patch] = triangle;
                }

                nexts[triangle] = heads[patch];
                heads[patch]    = triangle;

                areas[patch] += 0.5 * math.length( normalOf( this.mesh, triangle ) );

                var edges = triangleEdges[triangle];

                for( var k = 0; k < 3; ++k ) {
                    if( joins[edges[k]] != Joined ) {
                        perimeters[patch] += this.lengthOf( edges[k] );
                    }
                }
            }

            var limit      = thinPatches * this.length;
            var heap       = new BurstIsotropic.EdgeHeap( false, Allocator.Temp );
            var neighbours = new NativeList<Neighbour>( 16, Allocator.Temp );

            for( var patch = 0; patch < triangleCount; ++patch ) {
                if( heads[patch] != -1 && thin( areas[patch], perimeters[patch], limit ) ) {
                    heap.push( thicknessOf( areas[patch], perimeters[patch] ), patch );
                }
            }

            while( heap.count > 0 ) {
                var patch = heap.pop( out var thickness );

                if( find( patches, patch ) != patch || thicknessOf( areas[patch], perimeters[patch] ) != thickness ) {
                    continue;
                }

                neighbours.Clear( );

                for( var triangle = heads[patch]; triangle != -1; triangle = nexts[triangle] ) {
                    var edges = triangleEdges[triangle];

                    for( var k = 0; k < 3; ++k ) {
                        var edge = edges[k];

                        if( seconds[edge] == -1 || joins[edge] == Joined || this.mesh.edgeSharp[edge] != Sharp.None ) {
                            continue;
                        }

                        var other = firsts[edge] == triangle ? seconds[edge] : firsts[edge];
                        var into  = find( patches, other );

                        if( into == patch ) {
                            continue;
                        }

                        var own       = normalOf( this.mesh, triangle );
                        var theirs    = normalOf( this.mesh, other );
                        var lengths   = math.length( own ) * math.length( theirs );
                        var alignment = lengths > 0.0 ? 1.0 + ( math.dot( own, theirs ) / lengths ) : 1.0;
                        var length    = this.lengthOf( edge );
                        var index     = 0;

                        while( index < neighbours.Length && neighbours[index].patch != into ) {
                            ++index;
                        }
                        if( index == neighbours.Length ) {
                            neighbours.Add( new Neighbour { patch = into } );
                        }

                        var neighbour = neighbours[index];

                        neighbour.score  += length * alignment;
                        neighbour.shared += length;

                        neighbours[index] = neighbour;
                    }
                }

                if( neighbours.Length == 0 ) {
                    continue;
                }

                var best = 0;

                for( var index = 1; index < neighbours.Length; ++index ) {
                    if( neighbours[index].score > neighbours[best].score || ( neighbours[index].score == neighbours[best].score && neighbours[index].patch < neighbours[best].patch ) ) {
                        best = index;
                    }
                }

                var host = neighbours[best].patch;

                for( var triangle = heads[patch]; triangle != -1; triangle = nexts[triangle] ) {
                    absorbed[found[triangle]] = true;
                }

                patches[patch]  = host;
                nexts[tails[host]] = heads[patch];
                tails[host]     = tails[patch];
                heads[patch]    = -1;

                areas[host]      += areas[patch];
                perimeters[host] += perimeters[patch] - ( 2.0 * neighbours[best].shared );

                if( thin( areas[host], perimeters[host], limit ) ) {
                    heap.push( thicknessOf( areas[host], perimeters[host] ), host );
                }
            }

            heads.Dispose( );
            tails.Dispose( );
            nexts.Dispose( );
            areas.Dispose( );
            perimeters.Dispose( );
            heap.Dispose( );
            neighbours.Dispose( );
        }

        private static double thicknessOf( double area, double perimeter ) {
            return perimeter > 0.0 ? 2.0 * area / perimeter : double.PositiveInfinity;
        }

        private static bool thin( double area, double perimeter, double limit ) {
            return perimeter > 0.0 && 2.0 * area < limit * perimeter;
        }

        private static int find( NativeArray<int> parents, int item ) {
            while( parents[item] != item ) {
                parents[item] = parents[parents[item]];
                item          = parents[item];
            }
            return item;
        }

        // the lower triangle is the root
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

        // every line by angle shorter than l that ends loose, at a vertex of no other feature edge, taken out, from the
        // loose end along vertices of two feature edges to where it meets another line or ends, until none is left. A
        // line that ends at a hard line does not end loose there
        private void removeShortLooseLines( ) {
            var chain   = new NativeList<int>( 64, Allocator.Temp );
            var changed = true;

            while( changed ) {
                changed = false;

                for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                    if( this.mesh.firstCorners[vertex] == -1 || featureEdgesOf( this.mesh, this.edgeLines, vertex ) != 1 ) {
                        continue;
                    }

                    var edge = this.nextFeatureEdge( vertex, -1 );

                    if( this.mesh.edgeSharp[edge] != Sharp.None ) {
                        continue;
                    }

                    chain.Clear( );

                    var total = 0.0;
                    var at    = vertex;

                    while( true ) {
                        chain.Add( edge );

                        total += this.lengthOf( edge );
                        at     = this.mesh.across( edge, at );

                        if( total >= this.length || at == vertex || featureEdgesOf( this.mesh, this.edgeLines, at ) != 2 ) {
                            break;
                        }

                        var next = this.nextFeatureEdge( at, edge );

                        if( this.mesh.edgeSharp[next] != Sharp.None ) {
                            break;
                        }

                        edge = next;
                    }

                    if( total < this.length ) {
                        for( var index = 0; index < chain.Length; ++index ) {
                            this.edgeLines[chain[index]] = -1;
                        }

                        changed = true;
                    }
                }
            }

            chain.Dispose( );
        }

        // the first feature edge of a vertex other than one given
        private readonly int nextFeatureEdge( int vertex, int other ) {
            for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                if( this.edgeLines[end >> 1] != -1 && ( end >> 1 ) != other ) {
                    return end >> 1;
                }
            }
            return -1;
        }

        private readonly bool cornerAt( int vertex ) {
            return isCorner( this.mesh.isPoint( vertex ), featureEdgesOf( this.mesh, this.edgeLines, vertex ), this.corners[vertex] );
        }

        // whether the line through a vertex of two feature edges turns by more than the angle: the two ways along it
        // from the vertex are closer than 180 degrees less the angle. Each way goes along the line for a length l, or to
        // where the line ends or meets another, where both edges are by angle, and one edge where either is hard
        private readonly bool turns( int vertex ) {
            var edges = new int2( -1, -1 );
            var found = 0;
            var hard  = false;

            for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                if( this.edgeLines[end >> 1] != -1 ) {
                    edges[found++]  = end >> 1;
                    hard           |= this.mesh.edgeSharp[end >> 1] != Sharp.None;
                }
            }

            var reach      = hard ? 0.0 : this.length;
            var directions = new double3x2( this.reached( vertex, edges.x, reach ) - this.mesh.positions[vertex], this.reached( vertex, edges.y, reach ) - this.mesh.positions[vertex] );
            var lengths    = math.length( directions.c0 ) * math.length( directions.c1 );

            return lengths > 0.0 && math.dot( directions.c0, directions.c1 ) > -this.cosine * lengths;
        }

        // the point a line comes to from a vertex along one of its feature edges: one edge on, and on through vertices
        // of two feature edges by angle until it has gone a length, or comes back round
        private readonly double3 reached( int vertex, int edge, double reach ) {
            var at    = this.mesh.across( edge, vertex );
            var total = this.lengthOf( edge );

            while( total < reach && at != vertex && featureEdgesOf( this.mesh, this.edgeLines, at ) == 2 ) {
                var next = this.nextFeatureEdge( at, edge );

                if( this.mesh.edgeSharp[next] != Sharp.None ) {
                    break;
                }

                edge   = next;
                at     = this.mesh.across( edge, at );
                total += this.lengthOf( edge );
            }

            return this.mesh.positions[at];
        }

        // whether the normals of the triangles of a vertex spread by more than the angle from the mean of them all,
        // weighted by area, as at the tip of a cone. At the scale of each edge, where the normal of any one triangle
        // does. At the scale of l, where they do on the mean of them all, weighted by area: where the mean of the
        // normals of unit length, weighted so, is shorter than the cosine of the angle. On the tip of a cone the two
        // are the same, and a sliver or a fold that the patches merged away has too little area to make a tip
        private readonly bool spreads( int vertex ) {
            var mean  = double3.zero;
            var total = 0.0;

            for( var corner = this.mesh.firstCorners[vertex]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                var normal = normalOf( this.mesh, corner / 3 );

                mean  += normal;
                total += math.length( normal );
            }

            if( this.length > 0.0 ) {
                return total > 0.0 && math.length( mean ) < this.cosine * total;
            }

            for( var corner = this.mesh.firstCorners[vertex]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                if( bent( normalOf( this.mesh, corner / 3 ), mean, this.cosine ) ) {
                    return true;
                }
            }
            return false;
        }

        // numbers the edges of a line, from an edge and the vertex it starts at, until a corner, or until it comes back
        // round to where it started
        private void walk( int edge, int vertex, int line ) {
            while( true ) {
                this.edgeLines[edge] = line;

                var next = this.mesh.across( edge, vertex );

                if( this.cornerAt( next ) ) {
                    return;
                }

                var following = -1;

                for( var end = this.mesh.firstEdges[next]; end != -1; end = this.mesh.nextEdges[end] ) {
                    if( this.edgeLines[end >> 1] == Unnumbered ) {
                        following = end >> 1;

                        break;
                    }
                }

                if( following == -1 ) {
                    return;
                }

                edge   = following;
                vertex = next;
            }
        }

    }

}

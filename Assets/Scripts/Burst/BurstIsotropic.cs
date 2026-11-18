using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using Connectivity = BurstMeshSimplifier.Connectivity;
using FlipRefusal  = BurstMeshSimplifier.FlipRefusal;
using Refusal      = BurstMeshSimplifier.Refusal;
using Corner       = BurstFeatures.Corner;

// the isotropic remesh of Botsch and Kobbelt (A Remeshing Approach to Multiresolution Modeling, section 4): edges that
// are too long are split, edges that are too short collapsed, edges flipped towards vertices of six edges, and vertices
// smoothed, about five times over, and then vertices smoothed so that their areas come out alike. The paper remeshes a
// smooth surface and leaves out features, which collapses and flips are allowed and staying on the surface. What this
// has of those is the plan's own (Documents/Plans/Remeshing.md, finding 9)
public static class BurstIsotropic {

    // what a vertex is to a remesh, from the features it has now (BurstFeatures.isCorner). A corner stays where it
    // is, a vertex of a line moves along the line, and a vertex of the surface within its tangent plane
    public enum Kind : byte {
        Surface,
        Line,
        Corner
    }

    // the mesh a remesh works on: the simplifier's lists, and beside them the features of BurstFeatures, which the
    // simplifier's own sharp edges are among and which split, flip and collapse carry along. The lists of the features
    // are as long as the connectivity's, edge for edge and vertex for vertex
    public struct FeatureMesh : IDisposable {

        public Connectivity mesh;

        // for every edge, the line of the mesh as it came that the edge is on, or -1
        public NativeList<int> edgeLines;

        // for every vertex, the corners found by angle on the mesh as it came
        public NativeList<Corner> corners;

        // geometric breakpoints of hard polylines, independent of the feature-angle corner classification
        public NativeList<bool> fixedVertices;

        // immutable input-facing anchors, triangle for triangle; splits inherit them and other edits never rotate them
        public NativeList<double3> referenceNormals;

        public FeatureMesh( int vertexCount, int triangleCount, Allocator allocator ) {
            this.mesh             = new Connectivity( vertexCount, triangleCount, allocator );
            this.edgeLines        = new NativeList<int>( triangleCount * 3 / 2, allocator );
            this.corners          = new NativeList<Corner>( vertexCount, allocator );
            this.fixedVertices    = new NativeList<bool>( vertexCount, allocator );
            this.referenceNormals = new NativeList<double3>( triangleCount, allocator );
        }

        public void Dispose( ) {
            this.mesh.Dispose( );
            this.edgeLines.Dispose( );
            this.corners.Dispose( );
            this.fixedVertices.Dispose( );
            this.referenceNormals.Dispose( );
        }

        // Called once on the input, before any edits. A hard polyline may lose a collinear sample, never a bend.
        public void constrain( ) {
            this.fixedVertices.Clear( );
            this.referenceNormals.Clear( );

            for( var triangle = 0; triangle < this.mesh.triangleCount; ++triangle ) {
                this.referenceNormals.Add( this.mesh.subMeshes[triangle] == -1 ? double3.zero : BurstFeatures.normalOf( this.mesh, triangle ) );
            }

            for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                var count      = 0;
                var directions = new double3x2( );

                for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                    if( this.mesh.edgeSharp[end >> 1] == BurstMeshSimplifier.Sharp.None ) {
                        continue;
                    }
                    if( count < 2 ) {
                        directions[count] = this.mesh.positions[this.mesh.across( end >> 1, vertex )] - this.mesh.positions[vertex];
                    }
                    ++count;
                }

                var straight = count == 2 && math.dot( directions.c0, directions.c1 ) < 0.0 && math.lengthsq( math.cross( directions.c0, directions.c1 ) ) == 0.0;

                this.fixedVertices.Add( count > 0 && !straight );
            }
        }

        public readonly int featureEdgesOf( int vertex ) {
            return BurstFeatures.featureEdgesOf( this.mesh, this.edgeLines, vertex );
        }

        public readonly Kind kindOf( int vertex ) {
            return BurstIsotropic.kindOf( this.mesh.isPoint( vertex ), this.featureEdgesOf( vertex ), this.corners[vertex] );
        }

        // splits an edge at a position (Connectivity.split). The two halves of a feature edge are on its line, and the
        // edges across its triangles are on none. Gives the new vertex
        public int split( int edge, double3 position ) {
            var line = this.edgeLines[edge];
            var a    = this.mesh.edges[edge].x;
            var b    = this.mesh.edges[edge].y;

            // Connectivity.split appends the second halves in this order; the first halves keep their indices.
            for( var corner = this.mesh.firstCorners[a]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                if( math.any( this.mesh.triangles[corner / 3] == b ) ) {
                    this.referenceNormals.Add( this.referenceNormals[corner / 3] );
                }
            }

            var middle = this.mesh.split( edge, position );

            // the first edge added is the second half, and the edges across the triangles follow
            this.edgeLines.Add( line );

            while( this.edgeLines.Length < this.mesh.edgeCount ) {
                this.edgeLines.Add( -1 );
            }

            this.corners.Add( Corner.None );
            this.fixedVertices.Add( false );

            return middle;
        }

        // whether an edge may be flipped (Connectivity.checkFlip), at a feature angle given by its cosine: never a
        // feature edge, and never where the two triangles the flip makes would meet at more than the angle, which
        // would make a feature where there was none (finding 9)
        public readonly FlipRefusal checkFlip( int edge, double cosine ) {
            if( this.edgeLines[edge] != -1 && this.mesh.edgeTriangles[edge] == 2 && this.mesh.edgeSharp[edge] == BurstMeshSimplifier.Sharp.None ) {
                return FlipRefusal.Feature;
            }

            var refusal = this.mesh.checkFlip( edge, cosine );

            if( refusal != FlipRefusal.None ) {
                return refusal;
            }

            this.mesh.quadOf( edge, out var a, out var b, out var c, out var d, out var first, out var second );

            var one = math.cross( this.mesh.positions[d] - this.mesh.positions[a], this.mesh.positions[c] - this.mesh.positions[a] );
            var two = math.cross( this.mesh.positions[c] - this.mesh.positions[b], this.mesh.positions[d] - this.mesh.positions[b] );

            return this.facesInput( first, one ) && this.facesInput( second, one ) && this.facesInput( first, two ) && this.facesInput( second, two )
                ? FlipRefusal.None : FlipRefusal.Inversion;
        }

        // flips an edge that checkFlip allows. It is no feature edge, and neither is the edge it becomes
        public void flip( int edge ) {
            this.mesh.flip( edge );
        }

        public readonly bool facesInput( int triangle, double3 normal ) {
            return math.dot( this.referenceNormals[triangle], normal ) > 0.0;
        }

        // where a collapse of an edge goes, by what its two ends are (finding 9):
        // - two vertices of the surface collapse into the middle of the edge, as the paper has it;
        // - a vertex of the surface collapses into the other end, which is on a line or a corner;
        // - along a line, two of its vertices collapse into the middle, and a vertex of the line into a corner;
        // - two corners along a line become one where at most one of them is held: into that one, or else into the one
        //   where more lines meet, and of two alike into the lower. A corner is held where the simplifier keeps a point,
        //   and where it is on a hard line, so that hard lines keep their shape at every length (finding 9): only
        //   corners by angle become one.
        // Hard-polyline breakpoints are held as corners here, without changing their feature classification.
        // An edge between two vertices of lines or corners that is not itself a feature edge does not collapse, nor
        // does one between two held corners. Gives whether the edge may collapse, the position, and the end it goes
        // into, or -1 for the middle
        public readonly bool placeCollapse( int edge, out double3 position, out int into ) {
            var a = this.mesh.edges[edge].x;
            var b = this.mesh.edges[edge].y;

            var kindA = this.fixedVertices[a] ? Kind.Corner : this.kindOf( a );
            var kindB = this.fixedVertices[b] ? Kind.Corner : this.kindOf( b );

            into     = -1;
            position = ( this.mesh.positions[a] + this.mesh.positions[b] ) * 0.5;

            if( kindA == Kind.Surface && kindB == Kind.Surface ) {
                return true;
            }

            if( kindA == Kind.Surface ) {
                into = b;
            }
            else if( kindB == Kind.Surface ) {
                into = a;
            }
            else if( this.edgeLines[edge] == -1 ) {
                return false;
            }
            else if( kindA == Kind.Line && kindB == Kind.Line ) {
                return true;
            }
            else if( kindA == Kind.Line ) {
                into = b;
            }
            else if( kindB == Kind.Line ) {
                into = a;
            }
            else {
                var heldA = this.held( a );
                var heldB = this.held( b );

                if( heldA && heldB ) {
                    return false;
                }

                if( heldA != heldB ) {
                    into = heldA ? a : b;
                }
                else {
                    var countA = this.featureEdgesOf( a );
                    var countB = this.featureEdgesOf( b );

                    into = countA != countB ? ( countA > countB ? a : b ) : math.min( a, b );
                }
            }

            position = this.mesh.positions[into];

            return true;
        }

        // whether a corner or a hard-polyline breakpoint may not move
        public readonly bool held( int vertex ) {
            if( this.fixedVertices[vertex] || this.mesh.isPoint( vertex ) ) {
                return true;
            }

            for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                if( this.mesh.edgeSharp[end >> 1] != BurstMeshSimplifier.Sharp.None ) {
                    return true;
                }
            }
            return false;
        }

        // the longest edge a collapse of an edge into a position would leave, from there to the vertices next to either
        // end. A collapse that leaves one longer than a split allows is not made, as the next split would only undo it
        // (finding 9)
        public readonly double longestEdgeAfter( int edge, double3 position ) {
            var a       = this.mesh.edges[edge].x;
            var b       = this.mesh.edges[edge].y;
            var longest = 0.0;

            for( var end = this.mesh.firstEdges[a]; end != -1; end = this.mesh.nextEdges[end] ) {
                var neighbour = this.mesh.across( end >> 1, a );

                if( neighbour != b ) {
                    longest = math.max( longest, math.distance( position, this.mesh.positions[neighbour] ) );
                }
            }
            for( var end = this.mesh.firstEdges[b]; end != -1; end = this.mesh.nextEdges[end] ) {
                var neighbour = this.mesh.across( end >> 1, b );

                if( neighbour != a ) {
                    longest = math.max( longest, math.distance( position, this.mesh.positions[neighbour] ) );
                }
            }

            return longest;
        }

        // collapses an edge that the connectivity's check allows into the position placeCollapse gave, and into the end
        // it gave (Connectivity.collapse). The kept vertex takes the corners of the end the collapse went into, or of
        // both for the middle. Of two edges that become one, at a vertex opposite the edge, the one that is left is on
        // the line either was on. Gives the vertex kept
        public int collapse( int edge, double3 position, int into ) {
            var a       = this.mesh.edges[edge].x;
            var b       = this.mesh.edges[edge].y;
            var kept    = math.min( a, b );
            var removed = math.max( a, b );

            this.corners[kept]          = into == -1 ? this.corners[a] | this.corners[b] : this.corners[into];
            this.fixedVertices[kept]    = this.fixedVertices[a] || this.fixedVertices[b];
            this.fixedVertices[removed] = false;

            for( var corner = this.mesh.firstCorners[a]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                var triangle = this.mesh.triangles[corner / 3];

                if( !math.any( triangle == b ) ) {
                    continue;
                }

                var opposite  = triangle.x != a && triangle.x != b ? triangle.x : triangle.y != a && triangle.y != b ? triangle.y : triangle.z;
                var staying   = this.mesh.edgeBetween( kept, opposite );
                var going     = this.mesh.edgeBetween( removed, opposite );

                if( this.edgeLines[staying] == -1 ) {
                    this.edgeLines[staying] = this.edgeLines[going];
                }
            }

            var result = this.mesh.collapse( edge, position );

            for( var index = 0; index < this.mesh.removedEdges.Length; ++index ) {
                this.edgeLines[this.mesh.removedEdges[index]] = -1;
            }

            return result;
        }

    }

    // ---- the loop ----

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct ConstraintsJob : IJob {

        public FeatureMesh mesh;

        public void Execute( ) {
            this.mesh.constrain( );
        }

    }

    // the settings of a remesh, in the units of the mesh
    public struct Settings {

        public double length;        // l, the length the edges are to have
        public double featureCosine; // of the Feature Angle (BurstFeatures)
        public double featureLength; // the length the features by angle are found at (BurstFeatures), or 0 for each edge's own
        public int    iterations;    // of the loop
        public int    areaRounds;    // of the smoothing that equalizes areas, after the loop
        public double damping;       // the paper's lambda, for that smoothing

    }

    // what a remesh did, over all iterations and rounds, and how long each stage took
    public struct Report {

        public int splits;
        public int refusedSplits;    // edges longer than 4/3 l not split: no point of the input within reach, or one that turns a triangle over
        public int collapses;
        public int refusedCollapses; // edges shorter than 4/5 l that did not collapse, each time one was tried
        public int flips;
        public int moves;            // vertices smoothing moved, each time one was
        public int unplaced;         // vertices smoothing left where they were: no point of the input within reach
        public int held;             // vertices smoothing left where they were: a triangle would have turned over

        public double buildMilliseconds;    // the lists, the features and the grid
        public double splitMilliseconds;
        public double collapseMilliseconds;
        public double flipMilliseconds;
        public double smoothMilliseconds;   // the loop's smoothing
        public double areaMilliseconds;     // the smoothing that equalizes areas
        public double outputMilliseconds;

    }

    // the paper's limits: edges longer than 4/3 l are split, and edges shorter than 4/5 l collapsed
    public const double splitAbove    = 4.0 / 3.0;
    public const double collapseBelow = 4.0 / 5.0;

    // the paper's lambda, which it gives no value for. Measured on the plan's 15 models at 64^3 (step 3): after five
    // rounds a contour's spread of vertex areas is 0.095 at 0.25 and 0.083 at 1, and a simplified mesh's 0.137 to 0.138
    // at every value. But the larger the value, the more often more rounds widen the spread again: on contours, in none
    // of 120 steps between 0, 2, 5, 10 and 20 rounds at 0.25, 9 at 0.75 and 13 at 1. 0.75 is between the two
    public const double damping = 0.75;

    // how far from where smoothing takes a vertex the nearest point of the input may be, in lengths l: a vertex moves
    // less than l within its tangent plane, and the surface is nearer than that (finding 9)
    public const double reachInLengths = 2.0;

    // the size of the grid's cells, in mean edges of the input: a cell then holds a few of its triangles
    public const double cellInMeanEdges = 2.0;

    // what smoothing does with a vertex
    private const byte Stays    = 0;
    private const byte Moves    = 1;
    private const byte Unplaced = 2;

    // the nearest point of the input to a point of an edge, for a vertex a split or a collapse puts there, so that every
    // vertex is on the input from when it is made: on the line of a feature edge, or else on the surface of the
    // edge's own sub-mesh, facing within 90 degrees of the way its triangles face. Gives whether there is one within
    // reach
    public static bool placeOnInput( in FeatureMesh mesh, in SurfaceGrid grid, int edge, double3 point, double reach, out double3 placed ) {
        var a = mesh.mesh.edges[edge].x;
        var b = mesh.mesh.edges[edge].y;

        if( mesh.mesh.edgeSharp[edge] != BurstMeshSimplifier.Sharp.None ) {
            placed = SurfaceGrid.nearestOnSegment( point, mesh.mesh.positions[a], mesh.mesh.positions[b] );
            return true;
        }

        var line = mesh.edgeLines[edge];

        if( line != -1 ) {
            return grid.nearestOnLines( point, line, line, reach, out placed, out _ );
        }

        var facing  = double3.zero;
        var subMesh = -1;

        for( var corner = mesh.mesh.firstCorners[a]; corner != -1; corner = mesh.mesh.nextCorners[corner] ) {
            if( math.any( mesh.mesh.triangles[corner / 3] == b ) ) {
                facing  += BurstFeatures.normalOf( mesh.mesh, corner / 3 );
                subMesh  = mesh.mesh.subMeshes[corner / 3];
            }
        }

        placed = point;

        return math.lengthsq( facing ) > 0.0 && grid.nearestOnSurface( point, subMesh, math.normalize( facing ), reach, out placed, out _ );
    }

    // The reference at the proposed face's centre is chosen by position and material, never by its proposed normal.
    // Its inherited input normal also has to agree: a nearer opposing sheet, or no reference within reach, refuses
    // the edit rather than letting successive edits turn the face away from the input.
    public static bool facesInput( in FeatureMesh mesh, in SurfaceGrid grid, int triangle, double3 a, double3 b, double3 c, double reach ) {
        var normal = math.cross( b - a, c - a );

        return mesh.facesInput( triangle, normal ) &&
               grid.nearestOnSurface( ( a + b + c ) / 3.0, mesh.mesh.subMeshes[triangle], reach, out _, out var source ) &&
               math.dot( grid.normals[source], mesh.referenceNormals[triangle] ) > 0.0 &&
               math.dot( grid.normals[source], normal ) > 0.0;
    }

    // Check the current faces and fan as well as the immutable input, with earlier moves already applied.
    public static bool turnsOver( in FeatureMesh feature, in SurfaceGrid grid, int vertex, double3 position, double reach ) {
        var mesh   = feature.mesh;
        var normal = double3.zero;

        for( var corner = mesh.firstCorners[vertex]; corner != -1; corner = mesh.nextCorners[corner] ) {
            normal += BurstFeatures.normalOf( mesh, corner / 3 );
        }

        for( var corner = mesh.firstCorners[vertex]; corner != -1; corner = mesh.nextCorners[corner] ) {
            var corners = mesh.triangles[corner / 3];

            var p0 = mesh.positions[corners.x];
            var p1 = mesh.positions[corners.y];
            var p2 = mesh.positions[corners.z];

            var before = math.cross( p1 - p0, p2 - p0 );

            p0 = corners.x == vertex ? position : p0;
            p1 = corners.y == vertex ? position : p1;
            p2 = corners.z == vertex ? position : p2;

            var after = math.cross( p1 - p0, p2 - p0 );

            if( !( math.dot( before, after ) > 0.0 ) || !( math.dot( normal, after ) > 0.0 ) ||
                !facesInput( feature, grid, corner / 3, p0, p1, p2, reach ) ) {
                return true;
            }
        }
        return false;
    }

    public static double lengthOf( in FeatureMesh mesh, int edge ) {
        var ends = mesh.mesh.edges[edge];

        return math.distance( mesh.mesh.positions[ends.x], mesh.mesh.positions[ends.y] );
    }

    // edges by their length, the longest at the top or the shortest, and of two of one length the lower, so that the
    // same mesh is always worked in the same order (finding 9)
    internal struct EdgeHeap : IDisposable {

        private struct Item {

            public double length;
            public int    edge;

        }

        private          NativeList<Item> items;
        private readonly bool             longestFirst;

        public EdgeHeap( bool longestFirst, Allocator allocator ) {
            this.items        = new NativeList<Item>( 256, allocator );
            this.longestFirst = longestFirst;
        }

        public void Dispose( ) {
            this.items.Dispose( );
        }

        public readonly int count {
            get { return this.items.Length; }
        }

        private readonly bool before( in Item item, in Item other ) {
            if( item.length != other.length ) {
                return this.longestFirst ? item.length > other.length : item.length < other.length;
            }
            return item.edge < other.edge;
        }

        public void push( double length, int edge ) {
            var item  = new Item { length = length, edge = edge };
            var place = this.items.Length;

            this.items.Add( item );

            while( place > 0 ) {
                var parent = ( place - 1 ) / 2;

                if( !this.before( item, this.items[parent] ) ) {
                    break;
                }

                this.items[place] = this.items[parent];

                place = parent;
            }

            this.items[place] = item;
        }

        // the edge at the top and the length it had when it was pushed
        public int pop( out double length ) {
            var top  = this.items[0];
            var last = this.items[this.items.Length - 1];

            this.items.ResizeUninitialized( this.items.Length - 1 );

            if( this.items.Length > 0 ) {
                var place = 0;

                while( true ) {
                    var child = ( place * 2 ) + 1;

                    if( child >= this.items.Length ) {
                        break;
                    }
                    if( child + 1 < this.items.Length && this.before( this.items[child + 1], this.items[child] ) ) {
                        ++child;
                    }
                    if( !this.before( this.items[child], last ) ) {
                        break;
                    }

                    this.items[place] = this.items[child];

                    place = child;
                }

                this.items[place] = last;
            }

            length = top.length;

            return top.edge;
        }

    }

    // step 1 of the loop: every edge longer than the limit split, the longest first, the edges the splits make among
    // them, until none is longer. The new vertex goes to the nearest point of the input to the middle of the edge
    // (placeOnInput), so that every vertex is on the input whatever smoothing does after. An edge is not split where
    // there is no such point within reach, or where a triangle the split makes there would face away from the one it
    // halves
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SplitJob : IJob {

        public FeatureMesh mesh;
        public SurfaceGrid grid;
        public double      reach;
        public double      longest;

        // two, added to: splits made, and refused
        public NativeArray<int> splits;

        public void Execute( ) {
            var heap = new EdgeHeap( true, Allocator.Temp );

            for( var edge = 0; edge < this.mesh.mesh.edgeCount; ++edge ) {
                if( this.mesh.mesh.edges[edge].x != -1 && lengthOf( this.mesh, edge ) > this.longest ) {
                    heap.push( lengthOf( this.mesh, edge ), edge );
                }
            }

            while( heap.count > 0 ) {
                var edge = heap.pop( out _ );

                if( this.mesh.mesh.edges[edge].x == -1 || !( lengthOf( this.mesh, edge ) > this.longest ) ) {
                    continue;
                }

                var ends = this.mesh.mesh.edges[edge];

                if( !placeOnInput( this.mesh, this.grid, edge, ( this.mesh.mesh.positions[ends.x] + this.mesh.mesh.positions[ends.y] ) * 0.5, this.reach, out var onInput ) ||
                    this.turnsAHalfOver( edge, onInput ) ) {
                    ++this.splits[1];

                    continue;
                }

                var middle = this.mesh.split( edge, onInput );

                ++this.splits[0];

                // the two halves and the edges across, which are all the new vertex has
                for( var end = this.mesh.mesh.firstEdges[middle]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
                    var length = lengthOf( this.mesh, end >> 1 );

                    if( length > this.longest ) {
                        heap.push( length, end >> 1 );
                    }
                }
            }

            heap.Dispose( );
        }

        // whether a triangle of the edge, halved at the point, would leave a half facing away from it or without area
        private readonly bool turnsAHalfOver( int edge, double3 point ) {
            var a = this.mesh.mesh.edges[edge].x;
            var b = this.mesh.mesh.edges[edge].y;

            for( var corner = this.mesh.mesh.firstCorners[a]; corner != -1; corner = this.mesh.mesh.nextCorners[corner] ) {
                var corners = this.mesh.mesh.triangles[corner / 3];

                if( !math.any( corners == b ) ) {
                    continue;
                }

                var p0 = this.mesh.mesh.positions[corners.x];
                var p1 = this.mesh.mesh.positions[corners.y];
                var p2 = this.mesh.mesh.positions[corners.z];

                var whole = math.cross( p1 - p0, p2 - p0 );

                // the half at the first end, with the point in place of the second, and the half at the second end
                for( var half = 0; half < 2; ++half ) {
                    var replaced = half == 0 ? b : a;

                    var q0 = corners.x == replaced ? point : p0;
                    var q1 = corners.y == replaced ? point : p1;
                    var q2 = corners.z == replaced ? point : p2;

                    var normal = math.cross( q1 - q0, q2 - q0 );

                    if( !( math.dot( whole, normal ) > 0.0 ) || !facesInput( this.mesh, this.grid, corner / 3, q0, q1, q2, this.reach ) ) {
                        return true;
                    }
                }
            }
            return false;
        }

    }

    // step 2: every edge shorter than the limit collapsed where placeCollapse puts it, the shortest first, where the
    // lists' check allows it and no edge longer than the split limit comes of it (finding 9). An edge's length changes
    // when a collapse moves an end, and it waits again by its new length. An edge that is refused waits for the next
    // iteration. A collapse into the middle of an edge goes to the nearest point of the input to it (placeOnInput),
    // where there is one within reach and the lists allow it both there and at the middle
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct CollapseJob : IJob {

        public FeatureMesh mesh;
        public SurfaceGrid grid;
        public double      reach;
        public double      shortest;
        public double      longest;

        // two, added to: collapses made, and collapses refused
        public NativeArray<int> counts;

        public void Execute( ) {
            var heap = new EdgeHeap( false, Allocator.Temp );

            for( var edge = 0; edge < this.mesh.mesh.edgeCount; ++edge ) {
                if( this.mesh.mesh.edges[edge].x != -1 && lengthOf( this.mesh, edge ) < this.shortest ) {
                    heap.push( lengthOf( this.mesh, edge ), edge );
                }
            }

            while( heap.count > 0 ) {
                var edge = heap.pop( out var waited );

                if( this.mesh.mesh.edges[edge].x == -1 ) {
                    continue;
                }

                var length = lengthOf( this.mesh, edge );

                if( length != waited ) {
                    if( length < this.shortest ) {
                        heap.push( length, edge );
                    }
                    continue;
                }

                // the search for the nearest point of the input only for a collapse the lists allow at the middle, which
                // is a few of the edges tried, and then the lists asked again there
                if( !this.mesh.placeCollapse( edge, out var position, out var into ) || this.mesh.mesh.check( edge, position ) != Refusal.None ) {
                    ++this.counts[1];

                    continue;
                }

                if( into == -1 ) {
                    if( !placeOnInput( this.mesh, this.grid, edge, position, this.reach, out var onInput ) || this.mesh.mesh.check( edge, onInput ) != Refusal.None ) {
                        ++this.counts[1];

                        continue;
                    }

                    position = onInput;
                }

                var ends = this.mesh.mesh.edges[edge];

                if( this.mesh.longestEdgeAfter( edge, position ) > this.longest ||
                    !this.facesInputAfterMove( ends.x, ends.y, position ) || !this.facesInputAfterMove( ends.y, ends.x, position ) ) {
                    ++this.counts[1];

                    continue;
                }

                var kept = this.mesh.collapse( edge, position, into );

                ++this.counts[0];

                for( var end = this.mesh.mesh.firstEdges[kept]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
                    var changed = lengthOf( this.mesh, end >> 1 );

                    if( changed < this.shortest ) {
                        heap.push( changed, end >> 1 );
                    }
                }
            }

            heap.Dispose( );
        }

        private readonly bool facesInputAfterMove( int vertex, int removedWith, double3 position ) {
            for( var corner = this.mesh.mesh.firstCorners[vertex]; corner != -1; corner = this.mesh.mesh.nextCorners[corner] ) {
                var corners = this.mesh.mesh.triangles[corner / 3];

                if( math.any( corners == removedWith ) ) {
                    continue;
                }

                var a = corners.x == vertex ? position : this.mesh.mesh.positions[corners.x];
                var b = corners.y == vertex ? position : this.mesh.mesh.positions[corners.y];
                var c = corners.z == vertex ? position : this.mesh.mesh.positions[corners.z];

                if( !facesInput( this.mesh, this.grid, corner / 3, a, b, c, this.reach ) ) {
                    return false;
                }
            }
            return true;
        }

    }

    // step 3: edges flipped where that brings the four vertices of their two triangles nearer six edges each, or four
    // on an open edge, as the paper has it, and checkFlip allows it. Nearer is a smaller sum of the squares of the four
    // differences, which is the plan's own measure, as the paper gives none. A vertex at an edge of more than two
    // triangles, or with open edges that are not one rim through it, has no number it should have, and does not count.
    // The edges are gone through in their order, and then the edges of the four vertices of every flip, in the order they
    // come up, until none is left to look at: every flip makes the sum over the whole mesh smaller, so that comes. A flip
    // changes how many edges its four vertices have and nothing else, which is kept count of here, and never what they
    // should have
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct FlipJob : IJob {

        public FeatureMesh mesh;
        public SurfaceGrid grid;
        public double      reach;
        public double      cosine;

        // one, added to
        public NativeArray<int> flips;

        public void Execute( ) {
            var vertexCount = this.mesh.mesh.vertexCount;
            var valences    = new NativeArray<int>( vertexCount, Allocator.Temp );
            var targets     = new NativeArray<int>( vertexCount, Allocator.Temp );

            for( var vertex = 0; vertex < vertexCount; ++vertex ) {
                this.count( vertex, out var valence, out var target );

                valences[vertex] = valence;
                targets[vertex]  = target;
            }

            // the edges to look at, first to last, and whether each is among them
            var edgeCount = this.mesh.mesh.edgeCount;
            var queue     = new NativeList<int>( edgeCount, Allocator.Temp );
            var queued    = new NativeArray<bool>( edgeCount, Allocator.Temp );

            for( var edge = 0; edge < edgeCount; ++edge ) {
                queue.Add( edge );
                queued[edge] = true;
            }

            for( var next = 0; next < queue.Length; ++next ) {
                var edge = queue[next];

                queued[edge] = false;

                if( this.mesh.mesh.edges[edge].x == -1 || this.mesh.mesh.edgeTriangles[edge] != 2 ) {
                    continue;
                }
                if( !this.mesh.mesh.quadOf( edge, out var a, out var b, out var c, out var d, out var first, out var second ) || c == d ) {
                    continue;
                }

                var before = deviation( valences, targets, a, 0 ) + deviation( valences, targets, b, 0 ) + deviation( valences, targets, c, 0 ) + deviation( valences, targets, d, 0 );
                var after  = deviation( valences, targets, a, -1 ) + deviation( valences, targets, b, -1 ) + deviation( valences, targets, c, 1 ) + deviation( valences, targets, d, 1 );

                if( after >= before || this.mesh.checkFlip( edge, this.cosine ) != BurstMeshSimplifier.FlipRefusal.None ) {
                    continue;
                }
                if( !facesInput( this.mesh, this.grid, first, this.mesh.mesh.positions[a], this.mesh.mesh.positions[d], this.mesh.mesh.positions[c], this.reach ) ||
                    !facesInput( this.mesh, this.grid, second, this.mesh.mesh.positions[b], this.mesh.mesh.positions[c], this.mesh.mesh.positions[d], this.reach ) ) {
                    continue;
                }

                this.mesh.flip( edge );

                --valences[a];
                --valences[b];
                ++valences[c];
                ++valences[d];

                ++this.flips[0];

                // what may flip now is at the four vertices
                this.enqueue( a, queue, queued );
                this.enqueue( b, queue, queued );
                this.enqueue( c, queue, queued );
                this.enqueue( d, queue, queued );
            }

            valences.Dispose( );
            targets.Dispose( );
            queue.Dispose( );
            queued.Dispose( );
        }

        private readonly void enqueue( int vertex, NativeList<int> queue, NativeArray<bool> queued ) {
            for( var end = this.mesh.mesh.firstEdges[vertex]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
                if( !queued[end >> 1] ) {
                    queued[end >> 1] = true;
                    queue.Add( end >> 1 );
                }
            }
        }

        // how many edges a vertex has, and how many it should have: four with two open edges, six with none, and none
        // (-1) where it has edges of more than two triangles or other open edges
        private readonly void count( int vertex, out int valence, out int target ) {
            var open = 0;
            var more = 0;

            valence = 0;

            for( var end = this.mesh.mesh.firstEdges[vertex]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
                var triangles = this.mesh.mesh.edgeTriangles[end >> 1];

                ++valence;

                if( triangles == 1 ) {
                    ++open;
                }
                else if( triangles > 2 ) {
                    ++more;
                }
            }

            target = more > 0 || ( open != 0 && open != 2 ) ? -1 : open == 2 ? 4 : 6;
        }

        // the square of how far a vertex is from the number of edges it should have, with so many more edges
        private static int deviation( NativeArray<int> valences, NativeArray<int> targets, int vertex, int change ) {
            if( targets[vertex] == -1 ) {
                return 0;
            }

            var difference = valences[vertex] + change - targets[vertex];

            return difference * difference;
        }

    }

    // the Voronoi area of a vertex, the paper's A(p) (section 3): from the circumcentres of its triangles, or the
    // middles of their edges where a triangle has an obtuse angle, as Meyer et al. have it
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct AreasJob : IJobParallelFor {

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int>     firstCorners;
        [ReadOnly] public NativeArray<int>     nextCorners;
        [ReadOnly] public NativeArray<int3>    triangles;

        [WriteOnly] public NativeArray<double> areas;

        public void Execute( int vertex ) {
            var area = 0.0;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];
                var k       = corner % 3;

                area += voronoiArea( this.positions[corners[k]], this.positions[corners[( k + 1 ) % 3]], this.positions[corners[( k + 2 ) % 3]] );
            }

            this.areas[vertex] = area;
        }

    }

    // the part of a triangle p q r that is nearer p than the other two, or as near as it can be had where an angle is
    // obtuse: half the triangle for an obtuse angle at p, and a quarter for one at q or r
    public static double voronoiArea( double3 p, double3 q, double3 r ) {
        var cross = math.length( math.cross( q - p, r - p ) );

        if( !( cross > 0.0 ) ) {
            return 0.0;
        }

        var area = 0.5 * cross;

        if( math.dot( q - p, r - p ) < 0.0 ) {
            return area / 2.0;
        }
        if( math.dot( p - q, r - q ) < 0.0 || math.dot( p - r, q - r ) < 0.0 ) {
            return area / 4.0;
        }

        // cot q and cot r are their dot products over the cross product, which is twice the area for every corner
        var cotQ = math.dot( p - q, r - q ) / cross;
        var cotR = math.dot( p - r, q - r ) / cross;

        return ( ( math.distancesq( p, r ) * cotQ ) + ( math.distancesq( p, q ) * cotR ) ) / 8.0;
    }

    // step 4, and the smoothing that equalizes areas after the loop: where each vertex goes, worked out from where the
    // vertices are at the start of the round (finding 9).
    // - A vertex of the surface moves within its tangent plane towards the middle of its neighbours: in the loop the
    //   whole way, and to equalize areas towards the middle weighted by their Voronoi areas, damped (section 4). It
    //   goes on to the nearest point of the input within reach of its own material that faces within 90 degrees of
    //   it.
    // - A vertex of a line moves along it towards the middle of its two neighbours on it, to the nearest point within
    //   reach of the lines its two feature edges are on. Equalizing areas is a matter of the surface, and leaves it.
    // - A corner stays where it is.
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SmoothJob : IJobParallelFor {

        public SurfaceGrid grid;
        public double      reach;

        // with the areas, towards the middle weighted by them, damped by the factor
        public bool   byArea;
        public double damping;

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int>     firstCorners;
        [ReadOnly] public NativeArray<int>     nextCorners;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     subMeshes;
        [ReadOnly] public NativeArray<int>     firstEdges;
        [ReadOnly] public NativeArray<int>     nextEdges;
        [ReadOnly] public NativeArray<int2>    edges;
        [ReadOnly] public NativeArray<int>     edgeLines;
        [ReadOnly] public NativeArray<byte>    orders;
        [ReadOnly] public NativeArray<int>     sharpCounts;
        [ReadOnly] public NativeArray<Corner>  corners;
        [ReadOnly] public NativeArray<bool>    fixedVertices;
        [ReadOnly] public NativeArray<double>  areas;

        [WriteOnly] public NativeArray<double3> targets;
        [WriteOnly] public NativeArray<byte>    moves;

        public void Execute( int vertex ) {
            this.moves[vertex] = Stays;

            if( this.firstCorners[vertex] == -1 || this.fixedVertices[vertex] ) {
                return;
            }

            // the feature edges, their lines and their far ends, the first two
            var count     = 0;
            var lines     = new int2( -1, -1 );
            var neighbour = new int2( -1, -1 );

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                if( this.edgeLines[end >> 1] == -1 ) {
                    continue;
                }
                if( count < 2 ) {
                    lines[count]     = this.edgeLines[end >> 1];
                    neighbour[count] = this.across( end >> 1, vertex );
                }
                ++count;
            }

            var point = this.orders[vertex] == 2 || this.sharpCounts[vertex] == 1 || this.sharpCounts[vertex] >= 3;
            var kind  = kindOf( point, count, this.corners[vertex] );

            if( kind == Kind.Corner || ( this.byArea && kind != Kind.Surface ) ) {
                return;
            }

            var position = this.positions[vertex];
            var normal   = double3.zero;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];

                normal += math.cross( this.positions[corners.y] - this.positions[corners.x], this.positions[corners.z] - this.positions[corners.x] );
            }

            var length = math.length( normal );

            if( !( length > 0.0 ) ) {
                return;
            }

            normal /= length;

            bool    placed;
            double3 nearest;

            if( kind == Kind.Line ) {
                var a = this.positions[neighbour.x];
                var b = this.positions[neighbour.y];

                var along  = math.normalizesafe( b - a );
                var target = position + ( along * math.dot( along, ( ( a + b ) * 0.5 ) - position ) );

                if( this.sharpCounts[vertex] > 0 ) {
                    nearest = SurfaceGrid.nearestOnSegment( target, a, b );
                    placed  = true;
                }
                else {
                    placed = this.grid.nearestOnLines( target, lines.x, lines.y, this.reach, out nearest, out _ );
                }
            }
            else {
                var middle = double3.zero;
                var weight = 0.0;

                for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                    var other = this.across( end >> 1, vertex );
                    var share = this.byArea ? this.areas[other] : 1.0;

                    middle += this.positions[other] * share;
                    weight += share;
                }

                if( !( weight > 0.0 ) ) {
                    return;
                }

                var move = ( middle / weight ) - position;

                move -= normal * math.dot( move, normal );

                var target = position + ( move * ( this.byArea ? this.damping : 1.0 ) );

                placed = this.grid.nearestOnSurface( target, this.subMeshes[this.firstCorners[vertex] / 3], normal, this.reach, out nearest, out _ );
            }

            if( !placed ) {
                this.moves[vertex] = Unplaced;

                return;
            }

            this.targets[vertex] = nearest;
            this.moves[vertex]   = Moves;
        }

        private readonly int across( int edge, int vertex ) {
            var ends = this.edges[edge];

            return ends.x == vertex ? ends.y : ends.x;
        }

    }

    // what a vertex is from whether the lists keep it as a point, how many feature edges it has, and the corners found
    // by angle on the input (BurstFeatures.isCorner)
    public static Kind kindOf( bool point, int featureEdges, Corner found ) {
        if( BurstFeatures.isCorner( point, featureEdges, found ) ) {
            return Kind.Corner;
        }

        return featureEdges == 2 ? Kind.Line : Kind.Surface;
    }

    // the moves of a round, made one vertex after another in their order: each where it turns none of the vertex's
    // triangles over (turnsOver), with its neighbours where they are by then
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct ApplyJob : IJob {

        public FeatureMesh mesh;
        public SurfaceGrid grid;
        public double      reach;

        [ReadOnly] public NativeArray<double3> targets;
        [ReadOnly] public NativeArray<byte>    moves;

        // three, added to: moved, unplaced, held
        public NativeArray<int> counts;

        public void Execute( ) {
            for( var vertex = 0; vertex < this.moves.Length; ++vertex ) {
                if( this.moves[vertex] == Unplaced ) {
                    ++this.counts[1];
                }
                if( this.moves[vertex] != Moves ) {
                    continue;
                }
                if( turnsOver( this.mesh, this.grid, vertex, this.targets[vertex], this.reach ) ) {
                    ++this.counts[2];

                    continue;
                }

                this.mesh.mesh.positions[vertex] = this.targets[vertex];

                ++this.counts[0];
            }
        }

    }

    // the normal of every vertex: the input's at the nearest point of the input that faces the vertex's way, of its own
    // material where it is a vertex of the surface, from the normals at that triangle's corners (cornerNormals, three to
    // an input triangle; BurstRemeshing.cornerNormalsOf). Where there is none within reach, or no corner normals are
    // given, the normal of the vertex's own triangles, weighted by their areas (finding 21)
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct NormalsJob : IJobParallelFor {

        public SurfaceGrid grid;
        public double      reach;

        [ReadOnly] public NativeArray<float3> cornerNormals;

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int>     firstCorners;
        [ReadOnly] public NativeArray<int>     nextCorners;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     subMeshes;
        [ReadOnly] public NativeArray<int>     firstEdges;
        [ReadOnly] public NativeArray<int>     nextEdges;
        [ReadOnly] public NativeArray<int>     edgeLines;
        [ReadOnly] public NativeArray<byte>    orders;
        [ReadOnly] public NativeArray<int>     sharpCounts;
        [ReadOnly] public NativeArray<Corner>  corners;

        [WriteOnly] public NativeArray<double3> normals;

        public void Execute( int vertex ) {
            if( this.firstCorners[vertex] == -1 ) {
                return;
            }

            var position = this.positions[vertex];
            var normal   = double3.zero;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];

                normal += math.cross( this.positions[corners.y] - this.positions[corners.x], this.positions[corners.z] - this.positions[corners.x] );
            }

            normal = math.normalizesafe( normal );

            if( this.cornerNormals.Length == 0 ) {
                this.normals[vertex] = normal;

                return;
            }

            var count = 0;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                if( this.edgeLines[end >> 1] != -1 ) {
                    ++count;
                }
            }

            var point   = this.orders[vertex] == 2 || this.sharpCounts[vertex] == 1 || this.sharpCounts[vertex] >= 3;
            var subMesh = kindOf( point, count, this.corners[vertex] ) == Kind.Surface ? this.subMeshes[this.firstCorners[vertex] / 3] : SurfaceGrid.anySubMesh;

            this.normals[vertex] = this.grid.nearestOnSurface( position, subMesh, normal, this.reach, out var nearest, out var triangle )
                ? BurstRemeshing.inputNormalAt( this.grid, this.cornerNormals, nearest, triangle, normal )
                : normal;
        }

    }

    // the remeshed mesh: the vertices that have triangles, in their order, with their normals, and the triangles, in
    // theirs
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct WriteJob : IJob {

        public FeatureMesh mesh;

        [ReadOnly] public NativeArray<double3> vertexNormals;

        public NativeList<float3>             positions;
        public NativeList<float3>             normals;
        public NativeList<BurstMesh.Triangle> triangles;

        public void Execute( ) {
            var places = new NativeArray<int>( this.mesh.mesh.vertexCount, Allocator.Temp );

            for( var vertex = 0; vertex < this.mesh.mesh.vertexCount; ++vertex ) {
                if( this.mesh.mesh.firstCorners[vertex] == -1 ) {
                    continue;
                }

                places[vertex] = this.positions.Length;

                this.positions.Add( ( float3 )this.mesh.mesh.positions[vertex] );
                this.normals.Add( ( float3 )this.vertexNormals[vertex] );
            }

            for( var triangle = 0; triangle < this.mesh.mesh.triangleCount; ++triangle ) {
                if( this.mesh.mesh.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.mesh.mesh.triangles[triangle];

                this.triangles.Add( new BurstMesh.Triangle( places[corners.x], places[corners.y], places[corners.z], this.mesh.mesh.subMeshes[triangle] ) );
            }

            places.Dispose( );
        }

    }

    private static readonly ProfilerMarker buildMarker    = new( "Remeshing.Isotropic.Build" );
    private static readonly ProfilerMarker splitMarker    = new( "Remeshing.Isotropic.Split" );
    private static readonly ProfilerMarker collapseMarker = new( "Remeshing.Isotropic.Collapse" );
    private static readonly ProfilerMarker flipMarker     = new( "Remeshing.Isotropic.Flip" );
    private static readonly ProfilerMarker smoothMarker   = new( "Remeshing.Isotropic.Smooth" );
    private static readonly ProfilerMarker areaMarker     = new( "Remeshing.Isotropic.Area" );
    private static readonly ProfilerMarker outputMarker   = new( "Remeshing.Isotropic.Output" );

    // how many vertices a worker takes at a time in the smoothing
    private const int vertexBatch = 128;

    // remeshes a mesh of shared vertices into the lists, which are the caller's: the loop of section 4 for the set
    // number of iterations, then the rounds that equalize areas, on the mesh with its features and against the grid of
    // the mesh as it came. The normals out are had from cornerNormals, three to a triangle, or from each vertex's own
    // triangles when it is empty (NormalsJob)
    public static void remesh(
        NativeArray<float3>             positions,
        NativeArray<float3>             normals,
        NativeArray<BurstMesh.Triangle> triangles,
        NativeArray<float3>             cornerNormals,
        in Settings                     settings,
        NativeList<float3>              outPositions,
        NativeList<float3>              outNormals,
        NativeList<BurstMesh.Triangle>  outTriangles,
        out Report                      report
    ) {
        report = default;

        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );
        var reach     = reachInLengths * settings.length;

        using var lineCount = new NativeArray<int>( 1, Allocator.TempJob );
        using var mesh      = new FeatureMesh( positions.Length, triangles.Length, Allocator.TempJob );

        SurfaceGrid grid;

        using( buildMarker.Auto( ) ) {
            new BurstMeshSimplifier.BuildJob { positions = positions, normals = normals, triangles = triangles, mesh = mesh.mesh }.Run( );

            new BurstFeatures.SkeletonJob { cosine = settings.featureCosine, length = settings.featureLength, mesh = mesh.mesh, edgeLines = mesh.edgeLines, corners = mesh.corners, lineCount = lineCount }.Run( );
            new ConstraintsJob { mesh = mesh }.Run( );

            var total = 0.0;
            var count = 0;

            for( var edge = 0; edge < mesh.mesh.edgeCount; ++edge ) {
                total += lengthOf( mesh, edge );
                ++count;
            }

            var cellSize = count > 0 && total > 0.0 ? cellInMeanEdges * total / count : 1.0;

            grid = SurfaceGrid.of( mesh, cellSize, Allocator.TempJob );
        }

        report.buildMilliseconds = lap( stopwatch );

        var counts = new NativeArray<int>( 3, Allocator.TempJob );

        try {
            for( var iteration = 0; iteration < settings.iterations; ++iteration ) {
                using( splitMarker.Auto( ) ) {
                    new SplitJob { mesh = mesh, grid = grid, reach = reach, longest = splitAbove * settings.length, splits = counts }.Run( );

                    report.splits        += counts[0];
                    report.refusedSplits += counts[1];
                    counts[0]             = 0;
                    counts[1]             = 0;
                }

                report.splitMilliseconds += lap( stopwatch );

                using( collapseMarker.Auto( ) ) {
                    new CollapseJob {
                        mesh         = mesh,
                        grid         = grid,
                        reach        = reach,
                        shortest     = collapseBelow * settings.length,
                        longest      = splitAbove * settings.length,
                        counts       = counts
                    }.Run( );

                    report.collapses        += counts[0];
                    report.refusedCollapses += counts[1];
                    counts[0]                = 0;
                    counts[1]                = 0;
                }

                report.collapseMilliseconds += lap( stopwatch );

                using( flipMarker.Auto( ) ) {
                    new FlipJob { mesh = mesh, grid = grid, reach = reach, cosine = settings.featureCosine, flips = counts }.Run( );

                    report.flips += counts[0];
                    counts[0]     = 0;
                }

                report.flipMilliseconds += lap( stopwatch );

                using( smoothMarker.Auto( ) ) {
                    smooth( mesh, grid, reach, false, 0.0, ref report );
                }

                report.smoothMilliseconds += lap( stopwatch );
            }

            using( areaMarker.Auto( ) ) {
                for( var round = 0; round < settings.areaRounds; ++round ) {
                    smooth( mesh, grid, reach, true, settings.damping, ref report );
                }
            }

            report.areaMilliseconds = lap( stopwatch );

            using( outputMarker.Auto( ) ) {
                using var vertexNormals = new NativeArray<double3>( mesh.mesh.vertexCount, Allocator.TempJob );

                new NormalsJob {
                    grid           = grid,
                    reach          = reach,
                    cornerNormals  = cornerNormals,
                    positions      = mesh.mesh.positions.AsArray( ),
                    firstCorners   = mesh.mesh.firstCorners.AsArray( ),
                    nextCorners    = mesh.mesh.nextCorners.AsArray( ),
                    triangles      = mesh.mesh.triangles.AsArray( ),
                    subMeshes      = mesh.mesh.subMeshes.AsArray( ),
                    firstEdges     = mesh.mesh.firstEdges.AsArray( ),
                    nextEdges      = mesh.mesh.nextEdges.AsArray( ),
                    edgeLines      = mesh.edgeLines.AsArray( ),
                    orders         = mesh.mesh.orders.AsArray( ),
                    sharpCounts    = mesh.mesh.sharpCounts.AsArray( ),
                    corners        = mesh.corners.AsArray( ),
                    normals        = vertexNormals
                }.Schedule( mesh.mesh.vertexCount, vertexBatch ).Complete( );

                new WriteJob { mesh = mesh, vertexNormals = vertexNormals, positions = outPositions, normals = outNormals, triangles = outTriangles }.Run( );
            }

            report.outputMilliseconds = lap( stopwatch );
        }
        finally {
            counts.Dispose( );
            grid.Dispose( );
        }
    }

    // one round of smoothing: where every vertex goes, worked out in parallel, and the moves then made
    private static void smooth( FeatureMesh mesh, SurfaceGrid grid, double reach, bool byArea, double damping, ref Report report ) {
        var vertexCount = mesh.mesh.vertexCount;

        using var areas   = new NativeArray<double>( byArea ? vertexCount : 0, Allocator.TempJob );
        using var targets = new NativeArray<double3>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var moves   = new NativeArray<byte>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var counts  = new NativeArray<int>( 3, Allocator.TempJob );

        if( byArea ) {
            new AreasJob {
                positions    = mesh.mesh.positions.AsArray( ),
                firstCorners = mesh.mesh.firstCorners.AsArray( ),
                nextCorners  = mesh.mesh.nextCorners.AsArray( ),
                triangles    = mesh.mesh.triangles.AsArray( ),
                areas        = areas
            }.Schedule( vertexCount, vertexBatch ).Complete( );
        }

        new SmoothJob {
            grid          = grid,
            reach         = reach,
            byArea        = byArea,
            damping       = damping,
            positions     = mesh.mesh.positions.AsArray( ),
            firstCorners  = mesh.mesh.firstCorners.AsArray( ),
            nextCorners   = mesh.mesh.nextCorners.AsArray( ),
            triangles     = mesh.mesh.triangles.AsArray( ),
            subMeshes     = mesh.mesh.subMeshes.AsArray( ),
            firstEdges    = mesh.mesh.firstEdges.AsArray( ),
            nextEdges     = mesh.mesh.nextEdges.AsArray( ),
            edges         = mesh.mesh.edges.AsArray( ),
            edgeLines     = mesh.edgeLines.AsArray( ),
            orders        = mesh.mesh.orders.AsArray( ),
            sharpCounts   = mesh.mesh.sharpCounts.AsArray( ),
            corners       = mesh.corners.AsArray( ),
            fixedVertices = mesh.fixedVertices.AsArray( ),
            areas         = areas,
            targets       = targets,
            moves         = moves
        }.Schedule( vertexCount, vertexBatch ).Complete( );

        new ApplyJob { mesh = mesh, grid = grid, reach = reach, targets = targets, moves = moves, counts = counts }.Run( );

        report.moves    += counts[0];
        report.unplaced += counts[1];
        report.held     += counts[2];
    }

    private static double lap( System.Diagnostics.Stopwatch stopwatch ) {
        var milliseconds = stopwatch.Elapsed.TotalMilliseconds;

        stopwatch.Restart( );

        return milliseconds;
    }

}

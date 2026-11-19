using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using FeatureMesh = BurstIsotropic.FeatureMesh;
using Sharp       = BurstMeshSimplifier.Sharp;

// the two fields of Instant Field-Aligned Meshes (Jakob, Tarini, Panozzo and Sorkine-Hornung, sections 3.1 to 3.5 and
// 3.7), which Field Aligned reads its mesh off and QuadriFlow starts from (Documents/Plans/Remeshing.md): on every
// vertex of the mesh a direction in its tangent plane, up to quarter or sixth turns, and a point of a lattice of the
// edge length in that plane, lined up with the direction. Both are smoothed vertex by vertex against the neighbours,
// each time by whichever turn or lattice step brings a neighbour nearest, measured in space and not within the surface
// (the paper's extrinsic energies), coarse to fine over a hierarchy of merged vertices.
//
// Where the paper speaks it is followed, and where it is silent its authors' code is (github.com/wjakob/instant-meshes,
// BSD 3-clause; finding 11 lists what was taken from where). From the code, in this file: the candidates of the
// searches, the point between two tangent planes with its epsilon, the order of the iteration, the handing down and
// up of the fields, and the constraints along open edges, which here hold lines between materials too
public static class BurstFields {

    // the turns a direction is known up to, which is also the lattice: quarter turns and squares for quads, sixth
    // turns and equilateral triangles for triangles
    public enum Symmetry {
        Quads     = 4,
        Triangles = 6
    }

    // ---- the operators ----

    private const float sin60 = 0.866025403784439f;

    // a length below which a vector has no direction
    private const float tiny = 1e-12f;

    // a tangent turned by 60 degrees about its normal
    public static float3 turn60( float3 q, float3 n ) {
        return ( sin60 * math.cross( n, q ) ) + ( 0.5f * ( q + ( n * math.dot( n, q ) ) ) );
    }

    // a tangent turned about its normal by so many steps of the symmetry (the paper's R)
    public static float3 turned( float3 q, float3 n, int turns, Symmetry symmetry ) {
        if( symmetry == Symmetry.Quads ) {
            var quarter = turns & 3;
            var vector  = ( quarter & 1 ) != 0 ? math.cross( n, q ) : q;

            return quarter < 2 ? vector : -vector;
        }

        switch( ( ( turns % 6 ) + 6 ) % 6 ) {
            case 0:  return q;
            case 1:  return turn60( q, n );
            case 2:  return -turn60( q, -n );
            case 3:  return -q;
            case 4:  return -turn60( q, n );
            default: return turn60( q, -n );
        }
    }

    // the turns of two directions that bring them nearest each other in space (section 3.2, extrinsic), and the two
    // directions so turned. Directions half a turn apart are the same line, so the search is over half the turns of
    // each, and the second is turned over where that leaves the two facing apart
    public static int2 matchOrientations( float3 q0, float3 n0, float3 q1, float3 n1, Symmetry symmetry, out float3 a, out float3 b ) {
        float3x3 first;
        float3x3 second;
        int      count;
        int      lowest;

        if( symmetry == Symmetry.Quads ) {
            first  = new float3x3( q0, math.cross( n0, q0 ), float3.zero );
            second = new float3x3( q1, math.cross( n1, q1 ), float3.zero );
            count  = 2;
            lowest = 0;
        }
        else {
            first  = new float3x3( turn60( q0, -n0 ), q0, turn60( q0, n0 ) );
            second = new float3x3( turn60( q1, -n1 ), q1, turn60( q1, n1 ) );
            count  = 3;
            lowest = -1;
        }

        var best  = float.NegativeInfinity;
        var bestI = 0;
        var bestJ = 0;

        for( var i = 0; i < count; ++i )
        for( var j = 0; j < count; ++j ) {
            var score = math.abs( math.dot( first[i], second[j] ) );

            if( score > best ) {
                best  = score;
                bestI = i;
                bestJ = j;
            }
        }

        var steps = ( int )symmetry;
        var apart = math.dot( first[bestI], second[bestJ] ) < 0.0f;

        a = first[bestI];
        b = apart ? -second[bestJ] : second[bestJ];

        return new int2( ( lowest + bestI + steps ) % steps, ( lowest + bestJ + ( apart ? steps / 2 : 0 ) + steps ) % steps );
    }

    // the point nearest two vertices that lies in the tangent planes of both (the paper's appendix). The epsilon
    // brings it to the middle of the two where the planes are nearly one
    public static float3 middle( float3 v0, float3 n0, float3 v1, float3 n1 ) {
        var n0v0 = math.dot( n0, v0 );
        var n0v1 = math.dot( n0, v1 );
        var n1v0 = math.dot( n1, v0 );
        var n1v1 = math.dot( n1, v1 );
        var n0n1 = math.dot( n0, n1 );

        var scale   = 1.0f / ( 1.0f - ( n0n1 * n0n1 ) + 1e-4f );
        var lambda0 = 2.0f * ( n0v1 - n0v0 - ( n0n1 * ( n1v0 - n1v1 ) ) ) * scale;
        var lambda1 = 2.0f * ( n1v0 - n1v1 - ( n0n1 * ( n0v1 - n0v0 ) ) ) * scale;

        return ( 0.5f * ( v0 + v1 ) ) - ( 0.25f * ( ( n0 * lambda0 ) + ( n1 * lambda1 ) ) );
    }

    // the second axis of a vertex's lattice, a step of the symmetry on from its direction
    public static float3 secondAxis( float3 q, float3 n, Symmetry symmetry ) {
        return symmetry == Symmetry.Quads ? math.cross( n, q ) : turn60( q, n );
    }

    // a point of the lattice through o, so many steps along its two axes
    public static float3 latticePoint( float3 o, float3 q, float3 t, int2 steps, float rho ) {
        return o + ( ( ( q * steps.x ) + ( t * steps.y ) ) * rho );
    }

    // the steps to the corner of the lattice's cell that a point lies in: a square, or the rhombus of two triangles
    public static int2 floorSteps( float3 o, float3 q, float3 t, float3 p, float rho, Symmetry symmetry ) {
        var along = new float2( math.dot( q, p - o ), math.dot( t, p - o ) );

        if( symmetry == Symmetry.Triangles ) {
            along = new float2( ( 4.0f * along.x ) - ( 2.0f * along.y ), ( -2.0f * along.x ) + ( 4.0f * along.y ) ) / 3.0f;
        }

        return ( int2 )math.floor( along / rho );
    }

    // the steps to the point of the lattice through o that is nearest a point (the paper's round). For triangles the
    // appendix rounds the two steps one by one, which is not always the nearest point of a lattice whose axes are not
    // at right angles: the nearest is one of the four corners of the cell, as the code has it
    public static int2 roundSteps( float3 o, float3 q, float3 t, float3 p, float rho, Symmetry symmetry ) {
        if( symmetry == Symmetry.Quads ) {
            return ( int2 )math.floor( ( new float2( math.dot( q, p - o ), math.dot( t, p - o ) ) / rho ) + 0.5f );
        }

        var corner = floorSteps( o, q, t, p, rho, symmetry );
        var best   = float.PositiveInfinity;
        var steps  = corner;

        for( var index = 0; index < 4; ++index ) {
            var candidate = corner + new int2( index & 1, index >> 1 );
            var distance  = math.distancesq( latticePoint( o, q, t, candidate, rho ), p );

            if( distance < best ) {
                best  = distance;
                steps = candidate;
            }
        }

        return steps;
    }

    // the lattice points of a vertex that a search looks at, round a point between it and a neighbour: for quads the
    // four corners of the cell the point lies in, and for triangles the six lattice points nearest it, the paper's
    // "immediate neighbors", four and six to a lattice (section 3.3)
    private struct Candidates {

        public float3x3 points;
        public float3x3 more;
        public int2x3   steps;
        public int2x3   moreSteps;
        public int      count;

        public float3 point( int index ) {
            return index < 3 ? this.points[index] : this.more[index - 3];
        }

        public int2 step( int index ) {
            return index < 3 ? this.steps[index] : this.moreSteps[index - 3];
        }

        public void set( int index, float3 point, int2 steps ) {
            if( index < 3 ) {
                this.points[index] = point;
                this.steps[index]  = steps;
            }
            else {
                this.more[index - 3]      = point;
                this.moreSteps[index - 3] = steps;
            }
        }

    }

    private static Candidates candidatesOf( float3 o, float3 q, float3 t, float3 point, float rho, Symmetry symmetry ) {
        var corner = floorSteps( o, q, t, point, rho, symmetry );
        var result = new Candidates( );

        if( symmetry == Symmetry.Quads ) {
            result.count = 4;

            for( var index = 0; index < 4; ++index ) {
                var steps = corner + new int2( index & 1, index >> 1 );

                result.set( index, latticePoint( o, q, t, steps, rho ), steps );
            }

            return result;
        }

        // the six nearest are among the sixteen points a step round the cell: any other is further than 1.7 lengths
        var distances = new float4x4( );

        for( var i = 0; i < 4; ++i )
        for( var j = 0; j < 4; ++j ) {
            distances[i][j] = math.distancesq( latticePoint( o, q, t, corner + new int2( i - 1, j - 1 ), rho ), point );
        }

        result.count = 6;

        for( var index = 0; index < 6; ++index ) {
            var best  = float.PositiveInfinity;
            var bestI = 0;
            var bestJ = 0;

            for( var i = 0; i < 4; ++i )
            for( var j = 0; j < 4; ++j ) {
                if( distances[i][j] < best ) {
                    best  = distances[i][j];
                    bestI = i;
                    bestJ = j;
                }
            }

            distances[bestI][bestJ] = float.PositiveInfinity;

            var steps = corner + new int2( bestI - 1, bestJ - 1 );

            result.set( index, latticePoint( o, q, t, steps, rho ), steps );
        }

        return result;
    }

    // the lattice points of two vertices that are nearest each other in space, among those round the point between the
    // two (section 3.3, extrinsic): the steps to each from its own lattice point, and the two points
    public static void matchPositions(
        float3 v0, float3 n0, float3 q0, float3 o0, float3 v1, float3 n1, float3 q1, float3 o1, float rho, Symmetry symmetry, out int2 steps0, out int2 steps1, out float3 a, out float3 b
    ) {
        var between = middle( v0, n0, v1, n1 );
        var first   = candidatesOf( o0, q0, secondAxis( q0, n0, symmetry ), between, rho, symmetry );
        var second  = candidatesOf( o1, q1, secondAxis( q1, n1, symmetry ), between, rho, symmetry );

        var best  = float.PositiveInfinity;
        var bestI = 0;
        var bestJ = 0;

        for( var i = 0; i < first.count; ++i ) {
            var point = first.point( i );

            for( var j = 0; j < second.count; ++j ) {
                var distance = math.distancesq( point, second.point( j ) );

                if( distance < best ) {
                    best  = distance;
                    bestI = i;
                    bestJ = j;
                }
            }
        }

        steps0 = first.step( bestI );
        steps1 = second.step( bestJ );
        a      = first.point( bestI );
        b      = second.point( bestJ );
    }

    // two directions of a tangent plane at right angles, from its normal
    public static void tangentsOf( float3 n, out float3 s, out float3 t ) {
        t = math.abs( n.x ) > math.abs( n.y )
            ? new float3( n.z, 0.0f, -n.x ) / math.sqrt( ( n.x * n.x ) + ( n.z * n.z ) )
            : new float3( 0.0f, n.z, -n.y ) / math.sqrt( ( n.y * n.y ) + ( n.z * n.z ) );
        s = math.cross( t, n );
    }

    // ---- the graph and its hierarchy ----

    public struct Link : IComparable<Link> {

        public int   vertex;
        public float weight;

        public int CompareTo( Link other ) {
            return this.vertex.CompareTo( other.vertex );
        }

    }

    // the vertices of the mesh with its edges between them, and above them level on level of vertices merged in pairs
    // (section 3.5), all in one run of arrays: the vertices of level 0 are the mesh's, in their order, and each level
    // follows the one below. A vertex is known by its place in the run
    public struct Hierarchy : IDisposable {

        // where each level starts, and after the last where it ends
        public NativeList<int> levelStarts;

        public NativeList<float3> positions;
        public NativeList<float3> normals;
        public NativeList<float>  areas;

        // the neighbours of vertex v are links[linkStarts[v]] to links[linkStarts[v + 1] - 1], in the order of their
        // vertices. A vertex that is left out of the fields has none
        public NativeList<int>  linkStarts;
        public NativeList<Link> links;

        // the vertex of the level above that a vertex was merged into, or -1 at the top, and the one or two of the
        // level below that a vertex was merged from, the second -1 where it is the first alone
        public NativeList<int>  parents;
        public NativeList<int2> children;

        // vertices whose fields are held (section 3.7): the direction to a line's, and the lattice to pass through a
        // point along that line
        public NativeList<byte>   held;
        public NativeList<float3> heldDirections;
        public NativeList<float3> heldPositions;

        // the vertices of every level sorted by colour, no two of one colour neighbours: colour c of level l, from
        // levelColours[l] up to levelColours[l + 1], is order[colourStarts[c]] to order[colourStarts[c + 1] - 1]
        public NativeList<int> order;
        public NativeList<int> colourStarts;
        public NativeList<int> levelColours;

        // the fields: a unit direction in each vertex's tangent plane, and the lattice point nearest the vertex
        public NativeList<float3> orientations;
        public NativeList<float3> lattice;

        public Hierarchy( Allocator allocator ) {
            this.levelStarts    = new NativeList<int>( 32, allocator );
            this.positions      = new NativeList<float3>( allocator );
            this.normals        = new NativeList<float3>( allocator );
            this.areas          = new NativeList<float>( allocator );
            this.linkStarts     = new NativeList<int>( allocator );
            this.links          = new NativeList<Link>( allocator );
            this.parents        = new NativeList<int>( allocator );
            this.children       = new NativeList<int2>( allocator );
            this.held           = new NativeList<byte>( allocator );
            this.heldDirections = new NativeList<float3>( allocator );
            this.heldPositions  = new NativeList<float3>( allocator );
            this.order          = new NativeList<int>( allocator );
            this.colourStarts   = new NativeList<int>( allocator );
            this.levelColours   = new NativeList<int>( 32, allocator );
            this.orientations   = new NativeList<float3>( allocator );
            this.lattice        = new NativeList<float3>( allocator );
        }

        public void Dispose( ) {
            this.levelStarts.Dispose( );
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.areas.Dispose( );
            this.linkStarts.Dispose( );
            this.links.Dispose( );
            this.parents.Dispose( );
            this.children.Dispose( );
            this.held.Dispose( );
            this.heldDirections.Dispose( );
            this.heldPositions.Dispose( );
            this.order.Dispose( );
            this.colourStarts.Dispose( );
            this.levelColours.Dispose( );
            this.orientations.Dispose( );
            this.lattice.Dispose( );
        }

        public readonly int levelCount {
            get { return this.levelStarts.Length - 1; }
        }

        public readonly int vertexCount {
            get { return this.positions.Length; }
        }

        // a vertex with nothing known of it yet but where it is
        public int add( float3 position, float3 normal, float area, int2 children ) {
            this.positions.Add( position );
            this.normals.Add( normal );
            this.areas.Add( area );
            this.parents.Add( -1 );
            this.children.Add( children );
            this.held.Add( 0 );
            this.heldDirections.Add( float3.zero );
            this.heldPositions.Add( float3.zero );
            this.orientations.Add( float3.zero );
            this.lattice.Add( float3.zero );

            return this.positions.Length - 1;
        }

    }

    // what the smoothing of a vertex reads and writes. The fields are written at one vertex and read at its
    // neighbours, which are of other colours and so not written at the same time
    public struct View {

        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> normals;
        [ReadOnly] public NativeArray<int>    linkStarts;
        [ReadOnly] public NativeArray<Link>   links;
        [ReadOnly] public NativeArray<int2>   children;
        [ReadOnly] public NativeArray<byte>   held;
        [ReadOnly] public NativeArray<float3> heldDirections;
        [ReadOnly] public NativeArray<float3> heldPositions;
        [ReadOnly] public NativeArray<int>    order;
        [ReadOnly] public NativeArray<int>    colourStarts;
        [ReadOnly] public NativeArray<int>    levelColours;
        [ReadOnly] public NativeArray<int>    levelStarts;

        [NativeDisableParallelForRestriction] public NativeArray<float3> orientations;
        [NativeDisableParallelForRestriction] public NativeArray<float3> lattice;

        public View( in Hierarchy hierarchy ) {
            this.positions      = hierarchy.positions.AsArray( );
            this.normals        = hierarchy.normals.AsArray( );
            this.linkStarts     = hierarchy.linkStarts.AsArray( );
            this.links          = hierarchy.links.AsArray( );
            this.children       = hierarchy.children.AsArray( );
            this.held           = hierarchy.held.AsArray( );
            this.heldDirections = hierarchy.heldDirections.AsArray( );
            this.heldPositions  = hierarchy.heldPositions.AsArray( );
            this.order          = hierarchy.order.AsArray( );
            this.colourStarts   = hierarchy.colourStarts.AsArray( );
            this.levelColours   = hierarchy.levelColours.AsArray( );
            this.levelStarts    = hierarchy.levelStarts.AsArray( );
            this.orientations   = hierarchy.orientations.AsArray( );
            this.lattice        = hierarchy.lattice.AsArray( );
        }

    }

    // level 0 of the hierarchy, vertex by vertex, from the mesh a remesh was given, split into sheets and into edges
    // short enough (prepare): where each vertex is, its normal and its area, whether it is left out of the fields, and
    // whether its fields are held.
    // - The normal is the mean of its triangles' normals weighted by their angles at it. With creases, a vertex of a
    //   feature edge by angle takes the normal of one of its triangles instead, the largest (section 3.5).
    // - The area is its Voronoi area (section 3.5; BurstIsotropic.voronoiArea).
    // - A vertex of an edge of more than two triangles is left out, as the code leaves it: no normal means anything
    //   there (finding 11). So is a vertex without triangles.
    // - A vertex of an open edge or of a line between materials is held (section 3.7, finding 11): its direction to
    //   that of one of its hard edges, the flattest in its tangent plane, and its lattice to a line through the vertex
    //   along it. With freeMaterials, lines between materials are not held, which is only for measuring what holding
    //   them does (step 5)
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct VerticesJob : IJobParallelFor {

        public bool creases;
        public bool freeMaterials;

        [ReadOnly] public NativeArray<double3> meshPositions;
        [ReadOnly] public NativeArray<int>     firstCorners;
        [ReadOnly] public NativeArray<int>     nextCorners;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     firstEdges;
        [ReadOnly] public NativeArray<int>     nextEdges;
        [ReadOnly] public NativeArray<int2>    edges;
        [ReadOnly] public NativeArray<int>     edgeTriangles;
        [ReadOnly] public NativeArray<Sharp>   edgeSharp;
        [ReadOnly] public NativeArray<int>     edgeLines;

        [WriteOnly] public NativeArray<float3> positions;
        [WriteOnly] public NativeArray<float3> normals;
        [WriteOnly] public NativeArray<float>  areas;
        [WriteOnly] public NativeArray<bool>   leftOut;
        [WriteOnly] public NativeArray<byte>   held;
        [WriteOnly] public NativeArray<float3> heldDirections;
        [WriteOnly] public NativeArray<float3> heldPositions;
        [WriteOnly] public NativeArray<int>    parents;
        [WriteOnly] public NativeArray<int2>   children;

        public void Execute( int vertex ) {
            var position = this.meshPositions[vertex];

            this.parents[vertex]  = -1;
            this.children[vertex] = -1;

            this.positions[vertex]      = ( float3 )position;
            this.normals[vertex]        = new float3( 1.0f, 0.0f, 0.0f );
            this.areas[vertex]          = 0.0f;
            this.held[vertex]           = 0;
            this.heldDirections[vertex] = float3.zero;
            this.heldPositions[vertex]  = ( float3 )position;
            this.leftOut[vertex]        = true;

            if( this.firstCorners[vertex] == -1 ) {
                return;
            }

            var crease = false;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var edge = end >> 1;

                if( this.edgeTriangles[edge] > 2 ) {
                    return;
                }
                if( this.edgeSharp[edge] == Sharp.None && this.edgeLines[edge] != -1 ) {
                    crease = true;
                }
            }

            var sum     = double3.zero;
            var largest = double3.zero;
            var area    = 0.0;

            for( var corner = this.firstCorners[vertex]; corner != -1; corner = this.nextCorners[corner] ) {
                var corners = this.triangles[corner / 3];
                var k       = corner % 3;
                var next    = this.meshPositions[corners[( k + 1 ) % 3]];
                var before  = this.meshPositions[corners[( k + 2 ) % 3]];
                var normal  = math.cross( next - position, before - position );
                var length  = math.length( normal );

                if( !( length > 0.0 ) ) {
                    continue;
                }

                var lengths = math.length( next - position ) * math.length( before - position );

                sum  += normal / length * math.acos( math.clamp( math.dot( next - position, before - position ) / lengths, -1.0, 1.0 ) );
                area += BurstIsotropic.voronoiArea( position, next, before );

                if( length > math.length( largest ) ) {
                    largest = normal;
                }
            }

            if( !this.creases || !crease || !( math.lengthsq( largest ) > 0.0 ) ) {
                largest = math.lengthsq( sum ) > 0.0 ? sum : largest;
            }

            if( !( math.lengthsq( largest ) > 0.0 ) || !( area > 0.0 ) ) {
                return;
            }

            var unit = math.normalize( largest );

            this.normals[vertex] = ( float3 )unit;
            this.areas[vertex]   = ( float )area;
            this.leftOut[vertex] = false;

            // of its hard edges, the one that lies flattest in its tangent plane, and of two as flat the lower: where a
            // line goes over a crease, the vertex has the normal of one side, and the edge on the other side stands up
            // from its plane
            var hard    = -1;
            var flat    = 0.0;
            var along   = double3.zero;

            for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                var edge = end >> 1;

                if( this.edgeSharp[edge] == Sharp.None || ( this.freeMaterials && this.edgeSharp[edge] == Sharp.Materials ) ) {
                    continue;
                }

                var ends      = this.edges[edge];
                var direction = this.meshPositions[ends.x == vertex ? ends.y : ends.x] - position;
                var inPlane   = direction - ( unit * math.dot( unit, direction ) );
                var flatness  = math.lengthsq( direction ) > 0.0 ? math.lengthsq( inPlane ) / math.lengthsq( direction ) : 0.0;

                if( flatness > flat || ( flatness == flat && flatness > 0.0 && edge < hard ) ) {
                    hard  = edge;
                    flat  = flatness;
                    along = inPlane;
                }
            }

            if( hard != -1 ) {
                this.held[vertex]           = 1;
                this.heldDirections[vertex] = ( float3 )math.normalize( along );
            }
        }

    }

    // the edges of level 0: for every vertex that is not left out, its neighbours that are not either, each at a
    // weight of one (section 3.1, uniform weights), and the level itself
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct LinksJob : IJob {

        [ReadOnly] public NativeArray<int>  firstEdges;
        [ReadOnly] public NativeArray<int>  nextEdges;
        [ReadOnly] public NativeArray<int2> edges;
        [ReadOnly] public NativeArray<bool> leftOut;
        [ReadOnly] public NativeArray<int>  subMeshes;

        public Hierarchy hierarchy;

        // three: the vertices left out, the vertices held, and the triangles of the mesh
        public NativeArray<int> counts;

        public void Execute( ) {
            var neighbours = new NativeList<int>( 16, Allocator.Temp );

            for( var vertex = 0; vertex < this.leftOut.Length; ++vertex ) {
                this.counts[0] += this.leftOut[vertex] ? 1 : 0;
                this.counts[1] += this.hierarchy.held[vertex];
            }

            for( var triangle = 0; triangle < this.subMeshes.Length; ++triangle ) {
                this.counts[2] += this.subMeshes[triangle] != -1 ? 1 : 0;
            }

            this.hierarchy.levelStarts.Clear( );
            this.hierarchy.levelStarts.Add( 0 );
            this.hierarchy.levelStarts.Add( this.leftOut.Length );

            this.hierarchy.linkStarts.Clear( );
            this.hierarchy.links.Clear( );
            this.hierarchy.linkStarts.Add( 0 );

            for( var vertex = 0; vertex < this.leftOut.Length; ++vertex ) {
                neighbours.Clear( );

                if( !this.leftOut[vertex] ) {
                    for( var end = this.firstEdges[vertex]; end != -1; end = this.nextEdges[end] ) {
                        var ends      = this.edges[end >> 1];
                        var neighbour = ends.x == vertex ? ends.y : ends.x;

                        if( !this.leftOut[neighbour] ) {
                            neighbours.Add( neighbour );
                        }
                    }

                    neighbours.Sort( );
                }

                for( var index = 0; index < neighbours.Length; ++index ) {
                    this.hierarchy.links.Add( new Link { vertex = neighbours[index], weight = 1.0f } );
                }

                this.hierarchy.linkStarts.Add( this.hierarchy.links.Length );
            }

            neighbours.Dispose( );
        }

    }

    // the most levels a hierarchy is given. Each level has about half the vertices of the one below, so a mesh of a
    // million vertices has twenty; a graph that merges only a pair a level, as a star does, stops here
    public const int mostLevels = 32;

    private struct Pair : IComparable<Pair> {

        public float score;
        public int   vertex;
        public int   other;

        // the highest score first, and of two the same the lower vertices, so that the same graph always merges alike
        public int CompareTo( Pair pair ) {
            if( this.score != pair.score ) {
                return pair.score.CompareTo( this.score );
            }
            return this.vertex != pair.vertex ? this.vertex.CompareTo( pair.vertex ) : this.other.CompareTo( pair.other );
        }

    }

    // the levels above level 0 (section 3.5). Each level is made from the one below until nothing is left to merge,
    // which is when every piece of the graph is one vertex:
    // - every two neighbours get a score, the dot product of their normals times the larger of the two ratios of their
    //   areas, as the code has it. The paper writes the smaller ratio. With it a vertex that is passed over once stays
    //   small while its neighbours grow, is passed over at every level after, and the levels stop getting smaller
    //   long before the top (finding 22), so the paper's is here only to be measured against;
    // - going down the scores, two neighbours are merged when neither has been yet;
    // - a merged vertex has the sum of the two areas, and their positions and normals weighted by them;
    // - its neighbours are the vertices that its two were neighbours of, at the sum of their weights;
    // - it is held where either of its two is, to that one's line, put in its own tangent plane; and where both are, to
    //   the line of the one with the larger area. The code mixes the two lines, which for two vertices of one straight
    //   line is that line, and for two of different lines, as at a corner, is a line that neither is held to
    //   (finding 23).
    // Then every level is coloured
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct HierarchyJob : IJob {

        public Hierarchy hierarchy;
        public bool      scoreBySmallerRatio;

        public void Execute( ) {
            var h       = this.hierarchy;
            var pairs   = new NativeList<Pair>( Allocator.Temp );
            var scratch = new NativeList<Link>( 64, Allocator.Temp );

            for( var level = 0; level < mostLevels - 1; ++level ) {
                var start = h.levelStarts[level];
                var end   = h.levelStarts[level + 1];

                pairs.Clear( );

                for( var vertex = start; vertex < end; ++vertex )
                for( var link = h.linkStarts[vertex]; link < h.linkStarts[vertex + 1]; ++link ) {
                    var other = h.links[link].vertex;

                    if( other > vertex ) {
                        pairs.Add( new Pair { score = this.scoreOf( h, vertex, other ), vertex = vertex, other = other } );
                    }
                }

                if( pairs.Length == 0 ) {
                    break;
                }

                pairs.Sort( );

                for( var index = 0; index < pairs.Length; ++index ) {
                    var pair = pairs[index];

                    if( h.parents[pair.vertex] != -1 || h.parents[pair.other] != -1 ) {
                        continue;
                    }

                    var merged = this.merge( ref h, pair.vertex, pair.other );

                    h.parents[pair.vertex] = merged;
                    h.parents[pair.other]  = merged;
                }

                for( var vertex = start; vertex < end; ++vertex ) {
                    if( h.parents[vertex] != -1 ) {
                        continue;
                    }

                    var copy = h.add( h.positions[vertex], h.normals[vertex], h.areas[vertex], new int2( vertex, -1 ) );

                    h.held[copy]           = h.held[vertex];
                    h.heldDirections[copy] = h.heldDirections[vertex];
                    h.heldPositions[copy]  = h.heldPositions[vertex];
                    h.parents[vertex]      = copy;
                }

                for( var vertex = end; vertex < h.vertexCount; ++vertex ) {
                    scratch.Clear( );

                    for( var child = 0; child < 2; ++child ) {
                        var below = h.children[vertex][child];

                        if( below == -1 ) {
                            continue;
                        }

                        for( var link = h.linkStarts[below]; link < h.linkStarts[below + 1]; ++link ) {
                            scratch.Add( new Link { vertex = h.parents[h.links[link].vertex], weight = h.links[link].weight } );
                        }
                    }

                    scratch.Sort( );

                    var last = -1;

                    for( var index = 0; index < scratch.Length; ++index ) {
                        var link = scratch[index];

                        if( link.vertex == vertex ) {
                            continue;
                        }
                        if( link.vertex == last ) {
                            var sum = h.links[h.links.Length - 1];

                            sum.weight += link.weight;

                            h.links[h.links.Length - 1] = sum;
                        }
                        else {
                            h.links.Add( link );

                            last = link.vertex;
                        }
                    }

                    h.linkStarts.Add( h.links.Length );
                }

                h.levelStarts.Add( h.vertexCount );
            }

            pairs.Dispose( );
            scratch.Dispose( );

            colour( ref h );
        }

        private readonly float scoreOf( in Hierarchy h, int vertex, int other ) {
            var area      = h.areas[vertex];
            var otherArea = h.areas[other];

            if( !( area > 0.0f ) || !( otherArea > 0.0f ) ) {
                return 0.0f;
            }

            var ratio = area > otherArea ? area / otherArea : otherArea / area;

            return math.dot( h.normals[vertex], h.normals[other] ) * ( this.scoreBySmallerRatio ? 1.0f / ratio : ratio );
        }

        private readonly int merge( ref Hierarchy h, int vertex, int other ) {
            var area      = h.areas[vertex];
            var otherArea = h.areas[other];
            var sum       = area + otherArea;
            var normal    = ( h.normals[vertex] * area ) + ( h.normals[other] * otherArea );
            var length    = math.length( normal );

            var position = sum > tiny
                ? ( ( h.positions[vertex] * area ) + ( h.positions[other] * otherArea ) ) / sum
                : ( h.positions[vertex] + h.positions[other] ) * 0.5f;

            normal = length > tiny ? normal / length : h.normals[vertex];

            var merged = h.add( position, normal, sum, new int2( vertex, other ) );

            var first  = h.held[vertex] != 0;
            var second = h.held[other] != 0;

            if( !first && !second ) {
                return merged;
            }

            // the larger of the two that are held, and of two the same the first
            var from      = first && ( !second || area >= otherArea ) ? vertex : other;
            var direction = h.heldDirections[from];
            var point     = h.heldPositions[from];

            direction -= normal * math.dot( normal, direction );

            if( math.length( direction ) > tiny ) {
                h.held[merged]           = 1;
                h.heldDirections[merged] = math.normalize( direction );
                h.heldPositions[merged]  = point - ( normal * math.dot( normal, point - position ) );
            }

            return merged;
        }

        // every level coloured so that no two neighbours are of one colour (section 4): each vertex in its order takes
        // the lowest colour that none of its neighbours before it has
        private static void colour( ref Hierarchy h ) {
            var colours = new NativeArray<int>( h.vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory );
            var taken   = new NativeList<int>( 16, Allocator.Temp );
            var counts  = new NativeList<int>( 16, Allocator.Temp );

            h.order.ResizeUninitialized( h.vertexCount );
            h.colourStarts.Clear( );
            h.levelColours.Clear( );

            for( var level = 0; level < h.levelCount; ++level ) {
                var start = h.levelStarts[level];
                var end   = h.levelStarts[level + 1];

                taken.Clear( );
                counts.Clear( );

                for( var vertex = start; vertex < end; ++vertex ) {
                    for( var link = h.linkStarts[vertex]; link < h.linkStarts[vertex + 1]; ++link ) {
                        var other = h.links[link].vertex;

                        if( other < vertex ) {
                            taken[colours[other]] = vertex;
                        }
                    }

                    var chosen = 0;

                    while( chosen < taken.Length && taken[chosen] == vertex ) {
                        ++chosen;
                    }

                    if( chosen == taken.Length ) {
                        taken.Add( -1 );
                        counts.Add( 0 );
                    }

                    colours[vertex] = chosen;

                    ++counts[chosen];
                }

                h.levelColours.Add( h.colourStarts.Length );

                var next = start;

                for( var index = 0; index < counts.Length; ++index ) {
                    h.colourStarts.Add( next );

                    next          += counts[index];
                    counts[index]  = h.colourStarts[h.colourStarts.Length - 1];
                }

                for( var vertex = start; vertex < end; ++vertex ) {
                    h.order[counts[colours[vertex]]++] = vertex;
                }
            }

            h.levelColours.Add( h.colourStarts.Length );
            h.colourStarts.Add( h.vertexCount );

            colours.Dispose( );
            taken.Dispose( );
            counts.Dispose( );
        }

    }

    // ---- the smoothing ----

    // the direction of a vertex smoothed against its neighbours' (equations 3 and 4): one neighbour after the other
    // is added to the sum at the turn that brings it nearest the direction so far, which is turned with it; after each
    // the sum is put in the tangent plane and the direction is the sum at unit length. The code keeps the sum at the
    // weights added so far, where the paper's sum keeps its own length. A held vertex ends at the direction it is
    // held to, at whichever turn is nearest
    public static void smoothOrientation( ref View view, int vertex, Symmetry symmetry ) {
        var start = view.linkStarts[vertex];
        var end   = view.linkStarts[vertex + 1];

        if( start == end ) {
            return;
        }

        var normal    = view.normals[vertex];
        var direction = view.orientations[vertex];
        var magnitude = 0.0f;

        for( var index = start; index < end; ++index ) {
            var link = view.links[index];

            matchOrientations( direction, normal, view.orientations[link.vertex], view.normals[link.vertex], symmetry, out var own, out var other );

            var sum = ( own * magnitude ) + ( other * link.weight );

            sum -= normal * math.dot( normal, sum );

            magnitude = math.length( sum );

            if( magnitude > tiny ) {
                direction = sum / magnitude;
            }
        }

        if( view.held[vertex] != 0 ) {
            matchOrientations( direction, normal, view.heldDirections[vertex], normal, symmetry, out _, out direction );
        }

        view.orientations[vertex] = direction;
    }

    // the lattice point of a vertex smoothed against its neighbours' (equations 6 and 7): one neighbour after the other
    // is added to the mean at the lattice steps that bring the two nearest, the mean is put in the tangent plane, and
    // at the end it goes to the point of its lattice that is nearest the vertex. A held vertex's lattice is first
    // moved across the line it is held to, onto it
    public static void smoothPosition( ref View view, int vertex, float rho, Symmetry symmetry ) {
        var start = view.linkStarts[vertex];
        var end   = view.linkStarts[vertex + 1];

        if( start == end ) {
            return;
        }

        var position  = view.positions[vertex];
        var normal    = view.normals[vertex];
        var direction = view.orientations[vertex];
        var point     = view.lattice[vertex];
        var weights   = 0.0f;

        for( var index = start; index < end; ++index ) {
            var link = view.links[index];

            matchPositions(
                position, normal, direction, point,
                view.positions[link.vertex], view.normals[link.vertex], view.orientations[link.vertex], view.lattice[link.vertex],
                rho, symmetry, out _, out _, out var own, out var other
            );

            point    = ( ( own * weights ) + ( other * link.weight ) ) / ( weights + link.weight );
            weights += link.weight;
            point   -= normal * math.dot( normal, point - position );
        }

        if( view.held[vertex] != 0 ) {
            point = ontoHeldLine( point, normal, view.heldDirections[vertex], view.heldPositions[vertex], rho, symmetry );
        }

        var second = secondAxis( direction, normal, symmetry );

        view.lattice[vertex] = latticePoint( point, direction, second, roundSteps( point, direction, second, position, rho, symmetry ), rho );
    }

    // the lattice through a point moved across a line it is held to, by the least that puts one of its lines on it.
    // The lattice has lines along the held line, since the vertex's direction is the line's at some turn, a spacing
    // apart: the edge length for squares, and the height of a triangle for triangles. The code moves the lattice by
    // however far the point is from the line, which for squares is the same lattice. For triangles it is not: their
    // rows are half a step along from each other, so a move of an odd number of rows shifts the lattice half a step
    // along the line, and the vertices of a line do not come to agree (finding 23)
    public static float3 ontoHeldLine( float3 point, float3 normal, float3 along, float3 held, float rho, Symmetry symmetry ) {
        var across   = math.cross( normal, along );
        var spacing  = symmetry == Symmetry.Quads ? rho : sin60 * rho;
        var distance = math.dot( across, held - point );

        return point + ( across * ( distance - ( spacing * math.floor( ( distance / spacing ) + 0.5f ) ) ) );
    }

    // the fields of a vertex handed down to the one or two it was merged from: the direction and the lattice point,
    // each put in the tangent plane of the vertex below
    public static void handDown( ref View view, int vertex, bool positions ) {
        for( var child = 0; child < 2; ++child ) {
            var below = view.children[vertex][child];

            if( below == -1 ) {
                continue;
            }

            var normal = view.normals[below];

            if( positions ) {
                var point = view.lattice[vertex];

                view.lattice[below] = point - ( normal * math.dot( normal, point - view.positions[below] ) );
            }
            else {
                var direction = view.orientations[vertex];

                direction -= normal * math.dot( normal, direction );

                var length = math.length( direction );

                if( length > tiny ) {
                    view.orientations[below] = direction / length;
                }
                else {
                    tangentsOf( normal, out var s, out _ );

                    view.orientations[below] = s;
                }
            }
        }
    }

    // the vertices of one colour of a level, each smoothed once
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct ColourJob : IJobParallelFor {

        public View     view;
        public int      start;     // of the colour, in the order
        public bool     positions; // the lattice points, or else the directions
        public float    rho;
        public Symmetry symmetry;

        public void Execute( int index ) {
            var vertex = this.view.order[this.start + index];

            if( this.positions ) {
                smoothPosition( ref this.view, vertex, this.rho, this.symmetry );
            }
            else {
                smoothOrientation( ref this.view, vertex, this.symmetry );
            }
        }

    }

    // the vertices of a level, each handing its field down
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct HandDownJob : IJobParallelFor {

        public View view;
        public int  start; // of the level
        public bool positions;

        public void Execute( int index ) {
            handDown( ref this.view, this.start + index, this.positions );
        }

    }

    // a run of levels from the higher to the lower, each smoothed so many times, colour by colour, and handed down to
    // the level below it, one vertex after the other: what the two jobs above do for a level too small to be worth
    // sharing out. The vertices of a colour do not read each other, so the order within one changes nothing
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct LevelsJob : IJob {

        public View     view;
        public int      from;
        public int      to;
        public int      iterations;
        public bool     positions;
        public float    rho;
        public Symmetry symmetry;

        public void Execute( ) {
            for( var level = this.from; level >= this.to; --level ) {
                var first = this.view.colourStarts[this.view.levelColours[level]];
                var end   = this.view.colourStarts[this.view.levelColours[level + 1]];

                for( var iteration = 0; iteration < this.iterations; ++iteration )
                for( var index = first; index < end; ++index ) {
                    if( this.positions ) {
                        smoothPosition( ref this.view, this.view.order[index], this.rho, this.symmetry );
                    }
                    else {
                        smoothOrientation( ref this.view, this.view.order[index], this.symmetry );
                    }
                }

                if( level > 0 ) {
                    for( var vertex = this.view.levelStarts[level]; vertex < this.view.levelStarts[level + 1]; ++vertex ) {
                        handDown( ref this.view, vertex, this.positions );
                    }
                }
            }
        }

    }

    // where the fields start, at the top level: a direction at random in each tangent plane, and a point at random
    // within a length of each vertex, from a seed, so that the same mesh gives the same fields (finding 12). The top
    // level is one vertex for every piece of the graph, and every level below takes what the one above hands down
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct StartJob : IJob {

        public View  view;
        public bool  positions;
        public float rho;
        public uint  seed;

        public void Execute( ) {
            var random = new Unity.Mathematics.Random( this.seed );
            var top    = this.view.levelStarts.Length - 2;

            for( var vertex = this.view.levelStarts[top]; vertex < this.view.levelStarts[top + 1]; ++vertex ) {
                tangentsOf( this.view.normals[vertex], out var s, out var t );

                if( this.positions ) {
                    var offset = random.NextFloat2( -1.0f, 1.0f );

                    this.view.lattice[vertex] = this.view.positions[vertex] + ( ( ( s * offset.x ) + ( t * offset.y ) ) * this.rho );
                }
                else {
                    var angle = random.NextFloat( 0.0f, 2.0f * math.PI );

                    this.view.orientations[vertex] = ( s * math.cos( angle ) ) + ( t * math.sin( angle ) );
                }
            }
        }

    }

    // the directions of every level above level 0 made again from the level below, once level 0 is smoothed, so that
    // the lattice points of the coarse levels are solved along the directions the mesh ended with (the code): a merged
    // vertex takes the sum of its two at the turns that bring them nearest, in its own tangent plane
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct HandUpJob : IJob {

        public View     view;
        public Symmetry symmetry;

        public void Execute( ) {
            for( var vertex = this.view.levelStarts[1]; vertex < this.view.positions.Length; ++vertex ) {
                var below     = this.view.children[vertex];
                var direction = this.view.orientations[below.x];

                if( below.y != -1 ) {
                    matchOrientations( direction, this.view.normals[below.x], this.view.orientations[below.y], this.view.normals[below.y], this.symmetry, out var a, out var b );

                    direction = a + b;
                }

                var normal = this.view.normals[vertex];

                direction -= normal * math.dot( normal, direction );

                var length = math.length( direction );

                if( length > tiny ) {
                    this.view.orientations[vertex] = direction / length;
                }
                else {
                    tangentsOf( normal, out var s, out _ );

                    this.view.orientations[vertex] = s;
                }
            }
        }

    }

    // ---- what the fields are like ----

    // how smooth the fields of a level are: the paper's two energies (equations 4 and 6) over the links of the level,
    // each as its mean over them. The angle between two directions is in degrees, and the distance between two lattice
    // points in edge lengths
    public struct Energy {

        public int    links;
        public double orientation;
        public double position;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct EnergyJob : IJob {

        public View     view;
        public int      level;
        public bool     positions; // the lattice points too, once they are solved
        public float    rho;
        public Symmetry symmetry;

        // one
        [WriteOnly] public NativeArray<Energy> energy;

        public void Execute( ) {
            var energy = new Energy( );

            for( var vertex = this.view.levelStarts[this.level]; vertex < this.view.levelStarts[this.level + 1]; ++vertex )
            for( var index = this.view.linkStarts[vertex]; index < this.view.linkStarts[vertex + 1]; ++index ) {
                var other = this.view.links[index].vertex;

                matchOrientations(
                    this.view.orientations[vertex], this.view.normals[vertex], this.view.orientations[other], this.view.normals[other], this.symmetry, out var a, out var b
                );

                var angle = math.degrees( math.acos( math.clamp( ( double )math.dot( a, b ), -1.0, 1.0 ) ) );

                energy.orientation += angle * angle;

                if( this.positions ) {
                    matchPositions(
                        this.view.positions[vertex], this.view.normals[vertex], this.view.orientations[vertex], this.view.lattice[vertex],
                        this.view.positions[other], this.view.normals[other], this.view.orientations[other], this.view.lattice[other],
                        this.rho, this.symmetry, out _, out _, out var p, out var q
                    );

                    energy.position += math.distancesq( ( double3 )p, ( double3 )q ) / ( ( double )this.rho * this.rho );
                }

                ++energy.links;
            }

            if( energy.links > 0 ) {
                energy.orientation /= energy.links;
                energy.position    /= energy.links;
            }

            this.energy[0] = energy;
        }

    }

    // the triangles of the mesh round which a field does not come back to where it started (section 3.6), which are
    // the irregular vertices and faces of the mesh read off it. Round a triangle of the directions, the turns from
    // vertex to vertex do not add up to a whole turn. Round a triangle of the lattice points, where the directions do
    // add up, the lattice steps from vertex to vertex do not add up to none, with the three directions first turned to
    // lie along each other, as the code has it. A triangle of a vertex that is left out of the fields is passed over
    public struct Singularities {

        public int orientation;
        public int position;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SingularitiesJob : IJob {

        public View     view;
        public float    rho;
        public Symmetry symmetry;

        [ReadOnly] public NativeArray<int3> triangles;
        [ReadOnly] public NativeArray<int>  subMeshes;

        // one
        [WriteOnly] public NativeArray<Singularities> singularities;

        public void Execute( ) {
            var result = new Singularities( );
            var steps  = ( int )this.symmetry;

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                if( this.subMeshes[triangle] == -1 ) {
                    continue;
                }

                var corners = this.triangles[triangle];
                var leftOut = false;

                for( var k = 0; k < 3; ++k ) {
                    leftOut |= this.view.linkStarts[corners[k]] == this.view.linkStarts[corners[k] + 1];
                }

                if( leftOut ) {
                    continue;
                }

                var turns = 0;

                for( var k = 0; k < 3; ++k ) {
                    var from = corners[k];
                    var to   = corners[( k + 1 ) % 3];
                    var pair = matchOrientations( this.view.orientations[from], this.view.normals[from], this.view.orientations[to], this.view.normals[to], this.symmetry, out _, out _ );

                    turns += pair.y - pair.x;
                }

                if( ( ( turns % steps ) + steps ) % steps != 0 ) {
                    ++result.orientation;

                    continue;
                }

                // the turns of the three directions that leave the two furthest apart of them nearest each other
                var best   = float.NegativeInfinity;
                var chosen = int3.zero;

                for( var i = 0; i < steps; ++i ) {
                    var first = turned( this.view.orientations[corners.x], this.view.normals[corners.x], i, this.symmetry );

                    for( var j = 0; j < steps; ++j ) {
                        var second = turned( this.view.orientations[corners.y], this.view.normals[corners.y], j, this.symmetry );

                        for( var k = 0; k < steps; ++k ) {
                            var third = turned( this.view.orientations[corners.z], this.view.normals[corners.z], k, this.symmetry );
                            var least = math.min( math.min( math.dot( first, second ), math.dot( second, third ) ), math.dot( third, first ) );

                            if( least > best ) {
                                best   = least;
                                chosen = new int3( i, j, k );
                            }
                        }
                    }
                }

                var sum = int2.zero;

                for( var k = 0; k < 3; ++k ) {
                    var from = corners[k];
                    var to   = corners[( k + 1 ) % 3];

                    matchPositions(
                        this.view.positions[from], this.view.normals[from], turned( this.view.orientations[from], this.view.normals[from], chosen[k], this.symmetry ), this.view.lattice[from],
                        this.view.positions[to], this.view.normals[to], turned( this.view.orientations[to], this.view.normals[to], chosen[( k + 1 ) % 3], this.symmetry ), this.view.lattice[to],
                        this.rho, this.symmetry, out var own, out var other, out _, out _
                    );

                    sum += own - other;
                }

                if( math.any( sum != 0 ) ) {
                    ++result.position;
                }
            }

            this.singularities[0] = result;
        }

    }

    // ---- the whole ----

    // every edge of a mesh longer than a limit split at its middle, the longest first and of two of one length the
    // lower, the edges the splits make among them, until none is longer: how both field techniques prepare the mesh
    // they are given, whose fields live on its vertices (finding 3; the code). It is the first operation of
    // Isotropic's loop, without the search for a point of the input: the middle of an edge is on it. The splits stop
    // where the mesh would pass a number of vertices, if one is given, and say so
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct PrepareJob : IJob {

        public FeatureMesh mesh;
        public double      length; // the edge length the fields are for

        // two, added to: the splits, and 1 where they stopped at the limit
        public NativeArray<int> splits;

        public int mostVertices; // or 0 for no limit

        public void Execute( ) {
            var total = 0.0;

            for( var edge = 0; edge < this.mesh.mesh.edgeCount; ++edge ) {
                total += BurstIsotropic.lengthOf( this.mesh, edge );
            }

            if( !( total > 0.0 ) || !( this.length > 0.0 ) ) {
                return;
            }

            var longest = longestEdge( this.length, total / this.mesh.mesh.edgeCount );
            var heap    = new BurstIsotropic.EdgeHeap( true, Allocator.Temp );

            for( var edge = 0; edge < this.mesh.mesh.edgeCount; ++edge ) {
                if( BurstIsotropic.lengthOf( this.mesh, edge ) > longest ) {
                    heap.push( BurstIsotropic.lengthOf( this.mesh, edge ), edge );
                }
            }

            while( heap.count > 0 ) {
                var edge = heap.pop( out _ );

                if( !( BurstIsotropic.lengthOf( this.mesh, edge ) > longest ) ) {
                    continue;
                }

                if( this.mostVertices > 0 && this.mesh.mesh.vertexCount >= this.mostVertices ) {
                    this.splits[1] = 1;
                    break;
                }

                var ends   = this.mesh.mesh.edges[edge];
                var middle = this.mesh.split( edge, ( this.mesh.mesh.positions[ends.x] + this.mesh.mesh.positions[ends.y] ) * 0.5 );

                ++this.splits[0];

                for( var end = this.mesh.mesh.firstEdges[middle]; end != -1; end = this.mesh.mesh.nextEdges[end] ) {
                    var length = BurstIsotropic.lengthOf( this.mesh, end >> 1 );

                    if( length > longest ) {
                        heap.push( length, end >> 1 );
                    }
                }
            }

            heap.Dispose( );
        }

    }

    // the settings of the fields, in the units of the mesh
    public struct Settings {

        public Symmetry symmetry;
        public double   length;        // rho, the edge length of the lattice
        public double   featureCosine; // of the Feature Angle (BurstFeatures)
        public double   featureLength; // the length the features by angle are found at (BurstFeatures), or 0 for each edge's own
        public bool     creases;       // vertices of feature edges by angle take a triangle's normal (section 3.5)
        public bool     smallerRatio;  // the hierarchy scored by the smaller ratio of two areas, as the paper writes it
        public bool     freeMaterials; // lines between materials not held, for measuring what holding them does
        public int      iterations;    // of the smoothing, a level
        public uint     seed;
        public bool     measure;       // the energies of every level and the singularities, into the report
        public int      mostVertices;  // of the mesh the fields are on, or 0 for no limit: past it there are no fields

    }

    // six iterations a level (section 3.5)
    public const int iterationsALevel = 6;

    // the seed of the fields' random start. Any other gives fields as good and not the same
    public const uint seed = 0x9E3779B9;

    // a level of fewer vertices is smoothed by one worker. Smoothing a vertex is work enough that a few dozen of one
    // colour are worth sharing out: at 64^3 a simplified mesh's fields took half the time at 256 and 4 as at 4,096 and
    // 64, and every remesh came out the same (step 8)
    public const int smallLevel = 256;

    // how many vertices of a colour a worker takes at a time
    private const int vertexBatch = 4;

    // what the fields were made of, and how long each stage took
    public struct Report {

        public BurstSheets.Report sheets;

        public int  inputVertices;
        public int  splits;        // edges split before the fields
        public bool tooLarge;      // the mesh the fields are on would have passed Settings.mostVertices: no fields
        public int  vertices;      // of the mesh the fields are on
        public int triangles;
        public int leftOut;        // vertices the fields leave out
        public int held;           // vertices whose fields are held
        public int levels;
        public int colours;        // of level 0

        // with Settings.measure
        public Singularities singularities;

        public double sheetsMilliseconds;
        public double prepareMilliseconds;   // the lists, the features and the splits
        public double graphMilliseconds;     // level 0
        public double hierarchyMilliseconds; // the levels above it, and the colours
        public double orientationMilliseconds;
        public double positionMilliseconds;

    }

    // what a level was like, with Settings.measure: after its directions were smoothed, and after its lattice points
    public struct LevelReport {

        public int    level;
        public int    vertices;
        public int    colours;
        public Energy afterOrientations;
        public Energy afterPositions;

    }

    // the fields of a mesh, and the mesh they are on, which is the mesh that was given split into sheets and into
    // edges short enough. Vertex v of the mesh is vertex v of level 0. The caller disposes of it
    public struct Fields : IDisposable {

        public FeatureMesh mesh;
        public Hierarchy   hierarchy;

        // for every vertex of the sheets, before the edges were split, the vertex of the mesh that was given
        public NativeList<int> sources;

        public void Dispose( ) {
            this.mesh.Dispose( );
            this.hierarchy.Dispose( );
            this.sources.Dispose( );
        }

    }

    private static readonly ProfilerMarker sheetsMarker      = new( "Remeshing.Fields.Sheets" );
    private static readonly ProfilerMarker prepareMarker     = new( "Remeshing.Fields.Prepare" );
    private static readonly ProfilerMarker graphMarker       = new( "Remeshing.Fields.Graph" );
    private static readonly ProfilerMarker hierarchyMarker   = new( "Remeshing.Fields.Hierarchy" );
    private static readonly ProfilerMarker orientationMarker = new( "Remeshing.Fields.Orientation" );
    private static readonly ProfilerMarker positionMarker    = new( "Remeshing.Fields.Position" );

    // the edges of the mesh the fields are on may be no longer than this: the smaller of half the edge length and
    // twice the mean edge of the mesh (finding 11, the code). A longer limit is quicker and leaves more holes (finding 31)
    public static double longestEdge( double length, double meanEdge ) {
        return math.min( length * 0.5, meanEdge * 2.0 );
    }

    // the mesh the fields are on: the sheets of a mesh of shared vertices, with their features, split into edges
    // short enough for an edge length. The hierarchy is empty yet
    public static Fields prepare(
        NativeArray<float3> positions, NativeArray<float3> normals, NativeArray<BurstMesh.Triangle> triangles, in Settings settings, Allocator allocator, ref Report report
    ) {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );
        var fields    = new Fields { sources = new NativeList<int>( positions.Length, allocator ), hierarchy = new Hierarchy( allocator ) };

        using var sheetPositions = new NativeList<float3>( positions.Length, Allocator.TempJob );
        using var sheetNormals   = new NativeList<float3>( positions.Length, Allocator.TempJob );
        using var sheetTriangles = new NativeList<BurstMesh.Triangle>( triangles.Length, Allocator.TempJob );

        using( sheetsMarker.Auto( ) ) {
            report.sheets        = BurstSheets.split( positions, normals, triangles, sheetPositions, sheetNormals, sheetTriangles, fields.sources );
            report.inputVertices = positions.Length;
        }

        report.sheetsMilliseconds = lap( stopwatch );

        fields.mesh = new FeatureMesh( sheetPositions.Length, sheetTriangles.Length, allocator );

        using( prepareMarker.Auto( ) ) {
            using var lineCount = new NativeArray<int>( 1, Allocator.TempJob );
            using var splits    = new NativeArray<int>( 2, Allocator.TempJob );

            new BurstMeshSimplifier.BuildJob { positions = sheetPositions.AsArray( ), normals = sheetNormals.AsArray( ), triangles = sheetTriangles.AsArray( ), mesh = fields.mesh.mesh }.Run( );

            new BurstFeatures.SkeletonJob {
                cosine    = settings.featureCosine,
                length    = settings.featureLength,
                mesh      = fields.mesh.mesh,
                edgeLines = fields.mesh.edgeLines,
                corners   = fields.mesh.corners,
                lineCount = lineCount
            }.Run( );

            new BurstIsotropic.ConstraintsJob { mesh = fields.mesh }.Run( );

            new PrepareJob { mesh = fields.mesh, length = settings.length, mostVertices = settings.mostVertices, splits = splits }.Run( );

            report.splits   = splits[0];
            report.tooLarge = splits[1] != 0;
        }

        report.prepareMilliseconds = lap( stopwatch );

        return fields;
    }

    // level 0 of a hierarchy, from a mesh with its features: its vertices (VerticesJob) and the edges between them
    // (LinksJob)
    public static void graph( in FeatureMesh features, Hierarchy hierarchy, bool creases, ref Report report, bool freeMaterials = false ) {
        var mesh        = features.mesh;
        var vertexCount = mesh.vertexCount;

        hierarchy.positions.ResizeUninitialized( vertexCount );
        hierarchy.normals.ResizeUninitialized( vertexCount );
        hierarchy.areas.ResizeUninitialized( vertexCount );
        hierarchy.held.ResizeUninitialized( vertexCount );
        hierarchy.heldDirections.ResizeUninitialized( vertexCount );
        hierarchy.heldPositions.ResizeUninitialized( vertexCount );
        hierarchy.parents.ResizeUninitialized( vertexCount );
        hierarchy.children.ResizeUninitialized( vertexCount );
        hierarchy.orientations.Resize( vertexCount, NativeArrayOptions.ClearMemory );
        hierarchy.lattice.Resize( vertexCount, NativeArrayOptions.ClearMemory );

        using var leftOut = new NativeArray<bool>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var counts  = new NativeArray<int>( 3, Allocator.TempJob );

        new VerticesJob {
            creases        = creases,
            freeMaterials  = freeMaterials,
            meshPositions  = mesh.positions.AsArray( ),
            firstCorners   = mesh.firstCorners.AsArray( ),
            nextCorners    = mesh.nextCorners.AsArray( ),
            triangles      = mesh.triangles.AsArray( ),
            firstEdges     = mesh.firstEdges.AsArray( ),
            nextEdges      = mesh.nextEdges.AsArray( ),
            edges          = mesh.edges.AsArray( ),
            edgeTriangles  = mesh.edgeTriangles.AsArray( ),
            edgeSharp      = mesh.edgeSharp.AsArray( ),
            edgeLines      = features.edgeLines.AsArray( ),
            positions      = hierarchy.positions.AsArray( ),
            normals        = hierarchy.normals.AsArray( ),
            areas          = hierarchy.areas.AsArray( ),
            leftOut        = leftOut,
            held           = hierarchy.held.AsArray( ),
            heldDirections = hierarchy.heldDirections.AsArray( ),
            heldPositions  = hierarchy.heldPositions.AsArray( ),
            parents        = hierarchy.parents.AsArray( ),
            children       = hierarchy.children.AsArray( )
        }.Schedule( vertexCount, 256 ).Complete( );

        new LinksJob {
            firstEdges = mesh.firstEdges.AsArray( ),
            nextEdges  = mesh.nextEdges.AsArray( ),
            edges      = mesh.edges.AsArray( ),
            leftOut    = leftOut,
            subMeshes  = mesh.subMeshes.AsArray( ),
            hierarchy  = hierarchy,
            counts     = counts
        }.Run( );

        report.vertices  = vertexCount;
        report.leftOut   = counts[0];
        report.held      = counts[1];
        report.triangles = counts[2];
    }

    // the fields of a mesh of shared vertices at an edge length (sections 3.2 to 3.5): the mesh prepared, the
    // hierarchy built, the directions smoothed from the top level down, handed up again from level 0, and the lattice
    // points smoothed from the top level down along them. The levels get a line each with Settings.measure, in a list
    // that is the caller's. Where the mesh the fields are on would pass Settings.mostVertices, the report says so and
    // the hierarchy is left empty
    public static Fields solve(
        NativeArray<float3>             positions,
        NativeArray<float3>             normals,
        NativeArray<BurstMesh.Triangle> triangles,
        in Settings                     settings,
        Allocator                       allocator,
        out Report                      report,
        NativeList<LevelReport>         levels = default
    ) {
        report = default;

        var fields    = prepare( positions, normals, triangles, settings, allocator, ref report );
        var hierarchy = fields.hierarchy;
        var rho       = ( float )settings.length;
        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );

        if( report.tooLarge ) {
            return fields;
        }

        using( graphMarker.Auto( ) ) {
            graph( fields.mesh, hierarchy, settings.creases, ref report, settings.freeMaterials );
        }

        report.graphMilliseconds = lap( stopwatch );

        using( hierarchyMarker.Auto( ) ) {
            new HierarchyJob { hierarchy = hierarchy, scoreBySmallerRatio = settings.smallerRatio }.Run( );
        }

        report.levels                = hierarchy.levelCount;
        report.colours               = hierarchy.levelColours[1] - hierarchy.levelColours[0];
        report.hierarchyMilliseconds = lap( stopwatch );

        var measured = settings.measure && levels.IsCreated;

        if( measured ) {
            levels.Clear( );

            for( var level = 0; level < hierarchy.levelCount; ++level ) {
                levels.Add( new LevelReport {
                    level    = level,
                    vertices = hierarchy.levelStarts[level + 1] - hierarchy.levelStarts[level],
                    colours  = hierarchy.levelColours[level + 1] - hierarchy.levelColours[level]
                } );
            }
        }

        var view = new View( hierarchy );

        using( orientationMarker.Auto( ) ) {
            new StartJob { view = view, positions = false, rho = rho, seed = settings.seed }.Run( );

            smooth( view, hierarchy, settings, false, measured ? levels : default );

            new HandUpJob { view = view, symmetry = settings.symmetry }.Run( );
        }

        report.orientationMilliseconds = lap( stopwatch );

        using( positionMarker.Auto( ) ) {
            new StartJob { view = view, positions = true, rho = rho, seed = settings.seed }.Run( );

            smooth( view, hierarchy, settings, true, measured ? levels : default );
        }

        report.positionMilliseconds = lap( stopwatch );

        if( settings.measure ) {
            using var singularities = new NativeArray<Singularities>( 1, Allocator.TempJob );

            new SingularitiesJob {
                view          = view,
                rho           = rho,
                symmetry      = settings.symmetry,
                triangles     = fields.mesh.mesh.triangles.AsArray( ),
                subMeshes     = fields.mesh.mesh.subMeshes.AsArray( ),
                singularities = singularities
            }.Run( );

            report.singularities = singularities[0];
        }

        return fields;
    }

    // one field smoothed over every level, from the top down: the small levels at the top by one worker in one job,
    // and each level below colour by colour by all of them. Measured, every level goes the second way, so that it can
    // be looked at before it is handed down; the fields come out the same either way
    private static void smooth( View view, Hierarchy hierarchy, in Settings settings, bool positions, NativeList<LevelReport> levels ) {
        var rho   = ( float )settings.length;
        var level = hierarchy.levelCount - 1;

        if( !levels.IsCreated ) {
            var lowest = level + 1;

            while( lowest > 0 && hierarchy.levelStarts[lowest] - hierarchy.levelStarts[lowest - 1] < smallLevel ) {
                --lowest;
            }

            if( lowest <= level ) {
                new LevelsJob { view = view, from = level, to = lowest, iterations = settings.iterations, positions = positions, rho = rho, symmetry = settings.symmetry }.Run( );

                level = lowest - 1;
            }
        }

        for( ; level >= 0; --level ) {
            for( var iteration = 0; iteration < settings.iterations; ++iteration )
            for( var colour = hierarchy.levelColours[level]; colour < hierarchy.levelColours[level + 1]; ++colour ) {
                var first = hierarchy.colourStarts[colour];

                new ColourJob { view = view, start = first, positions = positions, rho = rho, symmetry = settings.symmetry }
                    .Schedule( hierarchy.colourStarts[colour + 1] - first, vertexBatch )
                    .Complete( );
            }

            if( levels.IsCreated ) {
                using var energy = new NativeArray<Energy>( 1, Allocator.TempJob );

                new EnergyJob { view = view, level = level, positions = positions, rho = rho, symmetry = settings.symmetry, energy = energy }.Run( );

                var line = levels[level];

                if( positions ) {
                    line.afterPositions = energy[0];
                }
                else {
                    line.afterOrientations = energy[0];
                }

                levels[level] = line;
            }

            if( level > 0 ) {
                var start = hierarchy.levelStarts[level];

                new HandDownJob { view = view, start = start, positions = positions }.Schedule( hierarchy.levelStarts[level + 1] - start, 256 ).Complete( );
            }
        }
    }

    private static double lap( System.Diagnostics.Stopwatch stopwatch ) {
        var milliseconds = stopwatch.Elapsed.TotalMilliseconds;

        stopwatch.Restart( );

        return milliseconds;
    }

}

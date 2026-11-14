using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;

using SmoothNormals = SurfaceExtractor.SmoothNormals;

// gives a mesh normals that are smooth across its curved surfaces and break at its sharp edges, by splitting vertices
// by angle, as a model importer's smoothing angle does. Around every vertex its triangles go into groups by their
// normals, and each group becomes a vertex of its own with the normal of its triangles. The pass runs on the finished
// mesh of any algorithm and implementation, simplified or not (Documents/Plans/Creased Normals.md).
//
// The groups are made around each vertex, largest corner first: a triangle joins the group whose normal is nearest, if
// that is within the crease angle, and otherwise starts a group. Comparing with a group's normal, and not with the
// triangle next to it, keeps a run of triangles that turn a little at a time from joining two faces into one group
// (finding 6). A group's normal moves as triangles join it, so afterwards any triangle that ended up further than the
// angle from its group's normal is given a group of its own (finding 11). No corner of the creased mesh then has a
// normal further than the angle from its own triangle's.
//
// Normals and angles are worked out in doubles with a normalize of the pass's own. Vector3.Normalize, and
// BurstSurfaceExtractor.normalize which matches it, leave vectors shorter than 1e-5 at zero, and the cross product
// of a small triangle of a fine grid is that short (finding 7)
public static class BurstCreases {

    // what a crease did
    public struct Statistics {

        public int inputVertexCount;
        public int outputVertexCount;

        // vertices whose triangles went into more than one group
        public int splitVertexCount;

        // corners given a group of their own because their group's normal moved too far from them
        public int guardedCornerCount;

        // under SmoothNormals.Contour, the vertices of one group that kept the contour's normal, and those whose
        // normal was further than the angle from one of their triangles, or zero, and gave way to the triangles'
        public int keptNormalCount;
        public int replacedNormalCount;

        // triangles too flat to have a normal (FacesJob), whose corners join their vertex's first group and have no
        // say in its normal
        public int flatTriangleCount;

    }

    // the unit vector along a vector, and zero for a vector of length zero, however short the vector is
    public static double3 unit( double3 vector ) {
        var length = math.length( vector );

        return length > 0.0 ? vector / length : double3.zero;
    }

    // the cosine an angle in degrees is held to: at 180 degrees and above every triangle around a vertex joins one
    // group, which no rounding of a dot product to just under -1 may undo
    private static double cosineOf( double angle ) {
        return angle >= 180.0 ? -2.0 : math.cos( math.radians( math.max( angle, 0.0 ) ) );
    }

    // the largest sine of an angle of a triangle under which it is too flat to have a normal: all its angles are then
    // within 0.06 degrees of 0 or 180. Occupancy splits polygons around a point on an edge between two vertices, which
    // lies on the line between them as nearly as the positions are rounded, and the triangle along that line takes its
    // normal from the rounding (Documents/Plans/Creased Normals.md, finding 14)
    public const double flatness = 1e-3;

    // the unit normal of a triangle, or zero when it is too flat to have one. Its largest sine of an angle is that of
    // the angle opposite its longest side, twice its area over the product of its two other sides, and the normal is
    // taken there, where the cross product is the least rounded
    public static double3 faceOf( double3 a, double3 b, double3 c ) {
        var ab = math.lengthsq( b - a );
        var bc = math.lengthsq( c - b );
        var ca = math.lengthsq( a - c );

        double3 cross;
        double  sides;

        if( bc >= ab && bc >= ca ) {
            cross = math.cross( b - a, c - a );
            sides = math.sqrt( ab * ca );
        }
        else if( ca >= ab ) {
            cross = math.cross( c - b, a - b );
            sides = math.sqrt( ab * bc );
        }
        else {
            cross = math.cross( a - c, b - c );
            sides = math.sqrt( bc * ca );
        }

        var length = math.length( cross );

        return length > flatness * sides ? cross / length : double3.zero;
    }

    // the unit normal of every triangle, and the angle at each of its corners, which weighs its normal at that corner
    // (Thürmer and Wüthrich). A triangle too flat to have a normal has a zero normal and corners that weigh nothing
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct FacesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        [WriteOnly] public NativeArray<double3> faces;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<double> weights;

        // the angle at a corner between its two sides, nought when either has no length
        private static double angleAt( double3 corner, double3 next, double3 previous ) {
            var along  = next - corner;
            var across = previous - corner;

            return math.atan2( math.length( math.cross( along, across ) ), math.dot( along, across ) );
        }

        public void Execute( int triangle ) {
            var t = this.triangles[triangle];

            var a = ( double3 )this.positions[t.a];
            var b = ( double3 )this.positions[t.b];
            var c = ( double3 )this.positions[t.c];

            var face = faceOf( a, b, c );

            this.faces[triangle] = face;

            var hasArea = math.any( face != double3.zero );

            this.weights[( triangle * 3 ) + 0] = hasArea ? angleAt( a, b, c ) : 0.0;
            this.weights[( triangle * 3 ) + 1] = hasArea ? angleAt( b, c, a ) : 0.0;
            this.weights[( triangle * 3 ) + 2] = hasArea ? angleAt( c, a, b ) : 0.0;
        }

    }

    private static int vertexOf( NativeArray<BurstMesh.Triangle> triangles, int corner ) {
        var t = triangles[corner / 3];

        return ( corner % 3 ) switch {
            0 => t.a,
            1 => t.b,
            _ => t.c
        };
    }

    // the corners of every vertex, in one run each: those of vertex v are runs[starts[v]] up to runs[starts[v + 1]],
    // in the order of the corners
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct RunsJob : IJob {

        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;

        public NativeArray<int> starts;
        public NativeArray<int> runs;

        public void Execute( ) {
            for( var vertex = 0; vertex < this.starts.Length; ++vertex ) {
                this.starts[vertex] = 0;
            }
            for( var corner = 0; corner < this.runs.Length; ++corner ) {
                ++this.starts[vertexOf( this.triangles, corner ) + 1];
            }
            for( var vertex = 1; vertex < this.starts.Length; ++vertex ) {
                this.starts[vertex] += this.starts[vertex - 1];
            }

            var next = new NativeArray<int>( this.starts.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory );

            NativeArray<int>.Copy( this.starts, next );

            for( var corner = 0; corner < this.runs.Length; ++corner ) {
                var vertex = vertexOf( this.triangles, corner );

                this.runs[next[vertex]++] = corner;
            }

            next.Dispose( );
        }

    }

    // the groups around every vertex. A vertex's run is sorted in place, largest corner first, and its groups' sums
    // and normals are kept in the slots of its run, which has at least as many slots as the vertex has groups. Every
    // corner gets the number of its group
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct GroupJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3>  normals;
        [ReadOnly] public NativeArray<double3> faces;
        [ReadOnly] public NativeArray<double>  weights;
        [ReadOnly] public NativeArray<int>     starts;

        public double        cosine;
        public SmoothNormals smoothNormals;

        [NativeDisableParallelForRestriction] public NativeArray<int>     runs;
        [NativeDisableParallelForRestriction] public NativeArray<double3> sums;
        [NativeDisableParallelForRestriction] public NativeArray<float3>  groupNormals;
        [NativeDisableParallelForRestriction] public NativeArray<int>     groupOfCorners;

        [WriteOnly] public NativeArray<int> groupCounts;
        [WriteOnly] public NativeArray<int> guardedCounts;

        // 1 when the vertex kept the contour's normal, 2 when that normal gave way to its triangles'
        [WriteOnly] public NativeArray<byte> outcomes;

        // larger corners first, then by their triangle's normal, x, y and z, and between corners alike in both by their
        // order. Corners alike in size and normal add the same to a group in either order, so the crease does not
        // depend on the order of the triangles, which the GPU's change from one contour to the next
        private readonly bool before( int corner, int other ) {
            var weight      = this.weights[corner];
            var otherWeight = this.weights[other];

            if( weight != otherWeight ) {
                return weight > otherWeight;
            }

            var face      = this.faces[corner / 3];
            var otherFace = this.faces[other / 3];

            if( face.x != otherFace.x ) {
                return face.x < otherFace.x;
            }
            if( face.y != otherFace.y ) {
                return face.y < otherFace.y;
            }
            if( face.z != otherFace.z ) {
                return face.z < otherFace.z;
            }
            return corner < other;
        }

        private readonly double3 faceOf( int corner ) {
            return this.faces[corner / 3];
        }

        private readonly bool hasArea( int corner ) {
            return math.any( this.faces[corner / 3] != double3.zero );
        }

        public void Execute( int vertex ) {
            var start = this.starts[vertex];
            var end   = this.starts[vertex + 1];

            this.guardedCounts[vertex] = 0;
            this.outcomes[vertex]      = 0;

            // a vertex that no triangle uses stays as it is, with the contour's normal
            if( start == end ) {
                this.groupCounts[vertex] = 1;
                return;
            }

            for( var slot = start + 1; slot < end; ++slot ) {
                var corner = this.runs[slot];
                var place  = slot - 1;

                while( place >= start && this.before( corner, this.runs[place] ) ) {
                    this.runs[place + 1] = this.runs[place];
                    --place;
                }
                this.runs[place + 1] = corner;
            }

            var groupCount = 0;

            for( var slot = start; slot < end; ++slot ) {
                var corner = this.runs[slot];

                // corners without area come last, and join the first group without changing its normal
                if( !this.hasArea( corner ) ) {
                    if( groupCount == 0 ) {
                        this.sums[start] = double3.zero;
                        groupCount       = 1;
                    }
                    this.groupOfCorners[corner] = 0;
                    continue;
                }

                var face    = this.faceOf( corner );
                var best    = -1;
                var nearest = this.cosine;

                for( var group = 0; group < groupCount; ++group ) {
                    var dot = math.dot( face, unit( this.sums[start + group] ) );

                    if( dot >= nearest ) {
                        nearest = dot;
                        best    = group;
                    }
                }

                if( best == -1 ) {
                    best = groupCount++;

                    this.sums[start + best] = double3.zero;
                }

                this.sums[start + best]    += face * this.weights[corner];
                this.groupOfCorners[corner] = best;
            }

            // the guard: a corner further than the angle from its group's normal leaves the group for one of its own,
            // which moves the normal of the group it left, until no corner is. Every pass takes a corner out of a group
            // of more than one, so it ends. A corner alone in its group stays, even when rounding puts the group's
            // normal a hair from its own: at an angle of 0 it would otherwise leave for a group of its own forever
            var guarded = 0;
            var moved   = true;

            while( moved ) {
                moved = false;

                for( var slot = start; slot < end; ++slot ) {
                    var corner = this.runs[slot];

                    if( !this.hasArea( corner ) ) {
                        continue;
                    }

                    var face  = this.faceOf( corner );
                    var group = this.groupOfCorners[corner];

                    if( math.dot( face, unit( this.sums[start + group] ) ) >= this.cosine || this.membersOf( start, end, group ) == 1 ) {
                        continue;
                    }

                    var weighted = face * this.weights[corner];

                    this.sums[start + group]      -= weighted;
                    this.sums[start + groupCount]  = weighted;
                    this.groupOfCorners[corner]    = groupCount;

                    ++groupCount;
                    ++guarded;

                    moved = true;
                }
            }

            // a group whose triangles have no area between them, or whose normals cancel, has no normal of its own and
            // takes the contour's
            for( var group = 0; group < groupCount; ++group ) {
                var normal = unit( this.sums[start + group] );

                this.groupNormals[start + group] = math.all( normal == double3.zero ) ? this.normals[vertex] : ( float3 )normal;
            }

            if( groupCount == 1 && this.smoothNormals == SmoothNormals.Contour ) {
                var keep = this.contourNormalFits( vertex, start, end );

                if( keep ) {
                    this.groupNormals[start] = this.normals[vertex];
                }

                this.outcomes[vertex] = ( byte )( keep ? 1 : 2 );
            }

            this.groupCounts[vertex]   = groupCount;
            this.guardedCounts[vertex] = guarded;
        }

        // the corners of a group that have area
        private readonly int membersOf( int start, int end, int group ) {
            var members = 0;

            for( var slot = start; slot < end; ++slot ) {
                var corner = this.runs[slot];

                if( this.groupOfCorners[corner] == group && this.hasArea( corner ) ) {
                    ++members;
                }
            }
            return members;
        }

        // whether the contour's normal of a vertex of one group may stay: it is within the angle of every triangle
        // with area at the vertex. At 180 degrees it always stays, zero or not, which gives the mesh back unchanged.
        // When no triangle at the vertex has area, nothing speaks against it
        private readonly bool contourNormalFits( int vertex, int start, int end ) {
            if( this.cosine < -1.0 ) {
                return true;
            }

            var contour = unit( ( double3 )this.normals[vertex] );

            for( var slot = start; slot < end; ++slot ) {
                var corner = this.runs[slot];

                if( !this.hasArea( corner ) ) {
                    continue;
                }

                if( math.all( contour == double3.zero ) || math.dot( contour, this.faceOf( corner ) ) < this.cosine ) {
                    return false;
                }
            }
            return true;
        }

    }

    // where every vertex's groups begin among the creased vertices, and what the groups came to
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct BasesJob : IJob {

        [ReadOnly] public NativeArray<int>     groupCounts;
        [ReadOnly] public NativeArray<int>     guardedCounts;
        [ReadOnly] public NativeArray<byte>    outcomes;
        [ReadOnly] public NativeArray<double3> faces;

        [WriteOnly] public NativeArray<int> bases;

        public NativeArray<Statistics> statistics;

        public void Execute( ) {
            var result = new Statistics { inputVertexCount = this.groupCounts.Length };
            var total  = 0;

            for( var vertex = 0; vertex < this.groupCounts.Length; ++vertex ) {
                this.bases[vertex] = total;

                total += this.groupCounts[vertex];

                if( this.groupCounts[vertex] > 1 ) {
                    ++result.splitVertexCount;
                }

                result.guardedCornerCount += this.guardedCounts[vertex];

                if( this.outcomes[vertex] == 1 ) {
                    ++result.keptNormalCount;
                }
                else if( this.outcomes[vertex] == 2 ) {
                    ++result.replacedNormalCount;
                }
            }

            for( var triangle = 0; triangle < this.faces.Length; ++triangle ) {
                if( math.all( this.faces[triangle] == double3.zero ) ) {
                    ++result.flatTriangleCount;
                }
            }

            result.outputVertexCount = total;

            this.statistics[0] = result;
        }

    }

    // the creased vertices: every vertex once for each of its groups, in the order of the vertices and then of their
    // groups, at the vertex's position
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct VerticesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> normals;
        [ReadOnly] public NativeArray<int>    starts;
        [ReadOnly] public NativeArray<int>    groupCounts;
        [ReadOnly] public NativeArray<int>    bases;
        [ReadOnly] public NativeArray<float3> groupNormals;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> creasedPositions;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> creasedNormals;

        public void Execute( int vertex ) {
            var start = this.starts[vertex];
            var first = this.bases[vertex];

            if( start == this.starts[vertex + 1] ) {
                this.creasedPositions[first] = this.positions[vertex];
                this.creasedNormals[first]   = this.normals[vertex];
                return;
            }

            for( var group = 0; group < this.groupCounts[vertex]; ++group ) {
                this.creasedPositions[first + group] = this.positions[vertex];
                this.creasedNormals[first + group]   = this.groupNormals[start + group];
            }
        }

    }

    // the triangles, each corner on the creased vertex of its group
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct TrianglesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;
        [ReadOnly] public NativeArray<int>                bases;
        [ReadOnly] public NativeArray<int>                groupOfCorners;

        [WriteOnly] public NativeArray<BurstMesh.Triangle> creasedTriangles;

        public void Execute( int triangle ) {
            var t      = this.triangles[triangle];
            var corner = triangle * 3;

            this.creasedTriangles[triangle] = new BurstMesh.Triangle(
                this.bases[t.a] + this.groupOfCorners[corner + 0],
                this.bases[t.b] + this.groupOfCorners[corner + 1],
                this.bases[t.c] + this.groupOfCorners[corner + 2],
                t.subMeshIndex
            );
        }

    }

    private static readonly ProfilerMarker readMeshMarker  = new( "Creases.Burst.ReadMesh" );
    private static readonly ProfilerMarker facesMarker     = new( "Creases.Burst.Faces" );
    private static readonly ProfilerMarker runsMarker      = new( "Creases.Burst.Runs" );
    private static readonly ProfilerMarker groupMarker     = new( "Creases.Burst.Group" );
    private static readonly ProfilerMarker writeMarker     = new( "Creases.Burst.Write" );
    private static readonly ProfilerMarker buildMeshMarker = new( "Creases.Burst.BuildMesh" );

    // how many triangles and vertices a worker takes at a time
    private const int triangleBatch = 256;
    private const int vertexBatch   = 256;

    // the mesh creased: a new mesh with the same sub-meshes and triangles, and as many vertices as the groups around
    // its vertices. The angle is in degrees
    public static Mesh crease( Mesh mesh, double angle, SmoothNormals smoothNormals, out Statistics statistics ) {
        NativeArray<float3>             positions;
        NativeArray<float3>             normals;
        NativeArray<BurstMesh.Triangle> triangles;

        using( readMeshMarker.Auto( ) ) {
            BurstMeshSimplifier.read( mesh, Allocator.TempJob, out positions, out normals, out triangles );
        }

        try {
            crease( positions, normals, triangles, angle, smoothNormals, Allocator.TempJob, out var creasedPositions, out var creasedNormals, out var creasedTriangles, out statistics );

            try {
                using( buildMeshMarker.Auto( ) ) {
                    return BurstMesh.createMesh( creasedPositions, creasedNormals, creasedTriangles );
                }
            }
            finally {
                creasedPositions.Dispose( );
                creasedNormals.Dispose( );
                creasedTriangles.Dispose( );
            }
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );
        }
    }

    // a mesh as arrays, creased into new arrays that are the caller's to dispose of. The triangles keep their order
    public static void crease(
        NativeArray<float3>                 positions,
        NativeArray<float3>                 normals,
        NativeArray<BurstMesh.Triangle>     triangles,
        double                              angle,
        SmoothNormals                       smoothNormals,
        Allocator                           allocator,
        out NativeArray<float3>             creasedPositions,
        out NativeArray<float3>             creasedNormals,
        out NativeArray<BurstMesh.Triangle> creasedTriangles,
        out Statistics                      statistics
    ) {
        var vertexCount = positions.Length;
        var cornerCount = triangles.Length * 3;

        using var faces          = new NativeArray<double3>( triangles.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var weights        = new NativeArray<double>( cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var starts         = new NativeArray<int>( vertexCount + 1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var runs           = new NativeArray<int>( cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var sums           = new NativeArray<double3>( cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var groupNormals   = new NativeArray<float3>( cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var groupOfCorners = new NativeArray<int>( cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var groupCounts    = new NativeArray<int>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var guardedCounts  = new NativeArray<int>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var outcomes       = new NativeArray<byte>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var bases          = new NativeArray<int>( vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var statisticsOut  = new NativeArray<Statistics>( 1, Allocator.TempJob );

        using( facesMarker.Auto( ) ) {
            new FacesJob {
                positions = positions,
                triangles = triangles,
                faces     = faces,
                weights   = weights
            }.Schedule( triangles.Length, triangleBatch ).Complete( );
        }

        using( runsMarker.Auto( ) ) {
            new RunsJob {
                triangles = triangles,
                starts    = starts,
                runs      = runs
            }.Run( );
        }

        using( groupMarker.Auto( ) ) {
            new GroupJob {
                normals        = normals,
                faces          = faces,
                weights        = weights,
                starts         = starts,
                cosine         = cosineOf( angle ),
                smoothNormals  = smoothNormals,
                runs           = runs,
                sums           = sums,
                groupNormals   = groupNormals,
                groupOfCorners = groupOfCorners,
                groupCounts    = groupCounts,
                guardedCounts  = guardedCounts,
                outcomes       = outcomes
            }.Schedule( vertexCount, vertexBatch ).Complete( );

            new BasesJob {
                groupCounts   = groupCounts,
                guardedCounts = guardedCounts,
                outcomes      = outcomes,
                faces         = faces,
                bases         = bases,
                statistics    = statisticsOut
            }.Run( );
        }

        statistics = statisticsOut[0];

        creasedPositions = new NativeArray<float3>( statistics.outputVertexCount, allocator, NativeArrayOptions.UninitializedMemory );
        creasedNormals   = new NativeArray<float3>( statistics.outputVertexCount, allocator, NativeArrayOptions.UninitializedMemory );
        creasedTriangles = new NativeArray<BurstMesh.Triangle>( triangles.Length, allocator, NativeArrayOptions.UninitializedMemory );

        using( writeMarker.Auto( ) ) {
            var vertices = new VerticesJob {
                positions        = positions,
                normals          = normals,
                starts           = starts,
                groupCounts      = groupCounts,
                bases            = bases,
                groupNormals     = groupNormals,
                creasedPositions = creasedPositions,
                creasedNormals   = creasedNormals
            }.Schedule( vertexCount, vertexBatch );

            var trianglesHandle = new TrianglesJob {
                triangles        = triangles,
                bases            = bases,
                groupOfCorners   = groupOfCorners,
                creasedTriangles = creasedTriangles
            }.Schedule( triangles.Length, triangleBatch );

            JobHandle.CombineDependencies( vertices, trianglesHandle ).Complete( );
        }
    }

}

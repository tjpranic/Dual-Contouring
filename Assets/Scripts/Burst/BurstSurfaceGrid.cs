using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// the surface a remesh puts its vertices back on (Documents/Plans/Remeshing.md, finding 9): the triangles of the mesh
// it was given, each with its sub-mesh and normal, and the segments of its feature lines, each with its line, in a grid
// of cubes for the search of the nearest point. Each cell keeps the triangles that may pass through it, and the
// segments that may.
//
// A search goes out from the cell of the point ring by ring, and stops once the rings that are left are further away
// than the nearest point it has found, or than the reach it was given: the next ring is as far as the walls of the
// rings so far. Only the triangles of one sub-mesh that face
// within 90 degrees of a direction count, and only the segments of one line or two
public struct SurfaceGrid : IDisposable {

    public double3 origin;
    public double  cellSize;
    public int3    cellCounts;

    // three corners, a unit normal and a sub-mesh for every triangle of the mesh, in its order. A triangle without area
    // has no normal, faces no way, and is in no cell, and one the lists left out has a sub-mesh of -1 as well
    [ReadOnly] public NativeArray<double3> corners;
    [ReadOnly] public NativeArray<double3> normals;
    [ReadOnly] public NativeArray<int>     subMeshes;

    // two ends and a line for every segment
    [ReadOnly] public NativeArray<double3> ends;
    [ReadOnly] public NativeArray<int>     lines;

    // for every cell that has any, where its triangles or its segments are among the items, in the order of their cells
    [ReadOnly] public NativeHashMap<long, int2> triangleCells;
    [ReadOnly] public NativeArray<int>          triangleItems;
    [ReadOnly] public NativeHashMap<long, int2> segmentCells;
    [ReadOnly] public NativeArray<int>          segmentItems;

    public void Dispose( ) {
        this.corners.Dispose( );
        this.normals.Dispose( );
        this.subMeshes.Dispose( );
        this.ends.Dispose( );
        this.lines.Dispose( );
        this.triangleCells.Dispose( );
        this.triangleItems.Dispose( );
        this.segmentCells.Dispose( );
        this.segmentItems.Dispose( );
    }

    public readonly int triangleCount {
        get { return this.subMeshes.Length; }
    }

    public readonly int segmentCount {
        get { return this.lines.Length; }
    }

    public readonly int3 cellOf( double3 point ) {
        return ( int3 )math.floor( ( point - this.origin ) / this.cellSize );
    }

    // 21 bits for each index of a cell inside the grid
    public static long keyOf( int3 cell ) {
        return ( ( long )cell.x << 42 ) | ( ( long )cell.y << 21 ) | ( long )cell.z;
    }

    private readonly bool inside( int3 cell ) {
        return math.all( cell >= 0 ) && math.all( cell < this.cellCounts );
    }

    // the nearest point of a triangle (Ericson, Real-Time Collision Detection, section 5.1.5), by the region of the
    // triangle's plane the point is over: a corner, an edge, or the face
    public static double3 nearestOnTriangle( double3 point, double3 a, double3 b, double3 c ) {
        var ab = b - a;
        var ac = c - a;
        var ap = point - a;
        var d1 = math.dot( ab, ap );
        var d2 = math.dot( ac, ap );

        if( d1 <= 0.0 && d2 <= 0.0 ) {
            return a;
        }

        var bp = point - b;
        var d3 = math.dot( ab, bp );
        var d4 = math.dot( ac, bp );

        if( d3 >= 0.0 && d4 <= d3 ) {
            return b;
        }

        var vc = ( d1 * d4 ) - ( d3 * d2 );

        if( vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 ) {
            return a + ( ab * ( d1 / ( d1 - d3 ) ) );
        }

        var cp = point - c;
        var d5 = math.dot( ab, cp );
        var d6 = math.dot( ac, cp );

        if( d6 >= 0.0 && d5 <= d6 ) {
            return c;
        }

        var vb = ( d5 * d2 ) - ( d1 * d6 );

        if( vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 ) {
            return a + ( ac * ( d2 / ( d2 - d6 ) ) );
        }

        var va = ( d3 * d6 ) - ( d5 * d4 );

        if( va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 ) {
            return b + ( ( c - b ) * ( ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) ) ) );
        }

        var denominator = 1.0 / ( va + vb + vc );

        return a + ( ab * ( vb * denominator ) ) + ( ac * ( vc * denominator ) );
    }

    public static double3 nearestOnSegment( double3 point, double3 a, double3 b ) {
        var along  = b - a;
        var length = math.lengthsq( along );

        return length > 0.0 ? a + ( along * math.saturate( math.dot( point - a, along ) / length ) ) : a;
    }

    // a search of the surface of any sub-mesh
    public const int anySubMesh = -2;

    // whether a triangle counts for a search of a sub-mesh, or of any, facing a direction
    public readonly bool counts( int triangle, int subMesh, double3 facing ) {
        return ( subMesh == anySubMesh ? this.subMeshes[triangle] >= 0 : this.subMeshes[triangle] == subMesh ) && math.dot( this.normals[triangle], facing ) > 0.0;
    }

    // the nearest point within the reach on a triangle of the sub-mesh, or of any, that faces within 90 degrees of the
    // direction. Gives whether there is one, and the triangle it is on
    public readonly bool nearestOnSurface( double3 point, int subMesh, double3 facing, double reach, out double3 nearest, out int triangle ) {
        return this.nearestOnSurface( point, subMesh, facing, true, reach, out nearest, out triangle );
    }

    // Choose the reference surface without filtering by the proposed face's orientation.
    public readonly bool nearestOnSurface( double3 point, int subMesh, double reach, out double3 nearest, out int triangle ) {
        return this.nearestOnSurface( point, subMesh, double3.zero, false, reach, out nearest, out triangle );
    }

    private readonly bool nearestOnSurface( double3 point, int subMesh, double3 facing, bool oriented, double reach, out double3 nearest, out int triangle ) {
        nearest  = point;
        triangle = -1;

        var best   = reach * reach;
        var centre = this.cellOf( point );
        var wall   = this.wallOf( point, centre );
        var rings  = ( int )math.ceil( reach / this.cellSize ) + 1;

        for( var ring = 0; ring <= rings; ++ring ) {
            // every cell of this ring and the rings after it is at least so far from the point: beyond the rings before
            if( ring >= 1 && best <= square( ( ( ring - 1 ) * this.cellSize ) + wall ) ) {
                break;
            }

            for( var x = -ring; x <= ring; ++x )
            for( var y = -ring; y <= ring; ++y ) {
                var onFace = math.abs( x ) == ring || math.abs( y ) == ring;

                for( var z = -ring; z <= ring; z += onFace || ring == 0 ? 1 : 2 * ring ) {
                    var cell = centre + new int3( x, y, z );

                    if( !this.inside( cell ) || !this.triangleCells.TryGetValue( keyOf( cell ), out var range ) ) {
                        continue;
                    }

                    for( var item = range.x; item < range.x + range.y; ++item ) {
                        var candidate = this.triangleItems[item];

                        if( !this.counts( candidate, subMesh, oriented ? facing : this.normals[candidate] ) ) {
                            continue;
                        }

                        var on       = nearestOnTriangle( point, this.corners[candidate * 3], this.corners[( candidate * 3 ) + 1], this.corners[( candidate * 3 ) + 2] );
                        var distance = math.distancesq( point, on );

                        if( distance < best || ( distance == best && triangle != -1 && candidate < triangle ) ) {
                            best     = distance;
                            nearest  = on;
                            triangle = candidate;
                        }
                    }
                }
            }
        }

        return triangle != -1;
    }

    // the nearest point within the reach on a segment of a line, or of either of two lines. Gives whether there is one,
    // and the segment it is on
    public readonly bool nearestOnLines( double3 point, int line, int otherLine, double reach, out double3 nearest, out int segment ) {
        nearest = point;
        segment = -1;

        var best   = reach * reach;
        var centre = this.cellOf( point );
        var wall   = this.wallOf( point, centre );
        var rings  = ( int )math.ceil( reach / this.cellSize ) + 1;

        for( var ring = 0; ring <= rings; ++ring ) {
            if( ring >= 1 && best <= square( ( ( ring - 1 ) * this.cellSize ) + wall ) ) {
                break;
            }

            for( var x = -ring; x <= ring; ++x )
            for( var y = -ring; y <= ring; ++y ) {
                var onFace = math.abs( x ) == ring || math.abs( y ) == ring;

                for( var z = -ring; z <= ring; z += onFace || ring == 0 ? 1 : 2 * ring ) {
                    var cell = centre + new int3( x, y, z );

                    if( !this.inside( cell ) || !this.segmentCells.TryGetValue( keyOf( cell ), out var range ) ) {
                        continue;
                    }

                    for( var item = range.x; item < range.x + range.y; ++item ) {
                        var candidate = this.segmentItems[item];

                        if( this.lines[candidate] != line && this.lines[candidate] != otherLine ) {
                            continue;
                        }

                        var on       = nearestOnSegment( point, this.ends[candidate * 2], this.ends[( candidate * 2 ) + 1] );
                        var distance = math.distancesq( point, on );

                        if( distance < best || ( distance == best && segment != -1 && candidate < segment ) ) {
                            best    = distance;
                            nearest = on;
                            segment = candidate;
                        }
                    }
                }
            }
        }

        return segment != -1;
    }

    // how far a point is from the nearest wall of its cell
    private readonly double wallOf( double3 point, int3 cell ) {
        var inside = point - ( this.origin + ( ( double3 )cell * this.cellSize ) );

        return math.max( 0.0, math.cmin( math.min( inside, this.cellSize - inside ) ) );
    }

    private static double square( double value ) {
        return value * value;
    }

    // an item of a cell, as the build sorts them: by cell, and in a cell by item
    private struct Entry : IComparable<Entry> {

        public long key;
        public int  item;

        public int CompareTo( Entry other ) {
            return this.key != other.key ? this.key.CompareTo( other.key ) : this.item.CompareTo( other.item );
        }

    }

    // puts the triangles and the segments into their cells. A triangle goes into the cells of its bounding box that
    // its plane passes through, a segment into those whose centres are within half a cell's diagonal of it: each into
    // every cell that holds a point of it, and a few more
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct BuildJob : IJob {

        [ReadOnly] public NativeArray<double3> positions;
        [ReadOnly] public NativeArray<int3>    triangles;
        [ReadOnly] public NativeArray<int>     triangleSubMeshes;
        [ReadOnly] public NativeArray<int2>    edges;
        [ReadOnly] public NativeArray<int>     edgeLines;

        public double3 origin;
        public double  cellSize;
        public int3    cellCounts;

        public NativeList<double3> corners;
        public NativeList<double3> normals;
        public NativeList<int>     subMeshes;
        public NativeList<double3> ends;
        public NativeList<int>     lines;

        public NativeHashMap<long, int2> triangleCells;
        public NativeList<int>           triangleItems;
        public NativeHashMap<long, int2> segmentCells;
        public NativeList<int>           segmentItems;

        public void Execute( ) {
            var entries = new NativeList<Entry>( Allocator.Temp );

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                // a triangle the lists have left out keeps its place, and is in no cell
                if( this.triangleSubMeshes[triangle] == -1 ) {
                    this.corners.Add( double3.zero );
                    this.corners.Add( double3.zero );
                    this.corners.Add( double3.zero );
                    this.normals.Add( double3.zero );
                    this.subMeshes.Add( -1 );

                    continue;
                }

                var index   = triangle;
                var corners = this.triangles[triangle];
                var a       = this.positions[corners.x];
                var b       = this.positions[corners.y];
                var c       = this.positions[corners.z];
                var normal  = math.cross( b - a, c - a );
                var length  = math.length( normal );

                normal = length > 0.0 ? normal / length : double3.zero;

                this.corners.Add( a );
                this.corners.Add( b );
                this.corners.Add( c );
                this.normals.Add( normal );
                this.subMeshes.Add( this.triangleSubMeshes[triangle] );

                if( !( length > 0.0 ) ) {
                    continue;
                }

                // how far a cell's centre may be from the plane for the plane to pass through the cell
                var slab = ( 0.5 * this.cellSize * math.csum( math.abs( normal ) ) ) + ( 1e-9 * this.cellSize );

                var low  = this.cellOf( math.min( a, math.min( b, c ) ) );
                var high = this.cellOf( math.max( a, math.max( b, c ) ) );

                for( var x = low.x; x <= high.x; ++x )
                for( var y = low.y; y <= high.y; ++y )
                for( var z = low.z; z <= high.z; ++z ) {
                    var cell   = new int3( x, y, z );
                    var middle = this.origin + ( ( ( double3 )cell + 0.5 ) * this.cellSize );

                    if( math.abs( math.dot( normal, middle - a ) ) <= slab ) {
                        entries.Add( new Entry { key = keyOf( cell ), item = index } );
                    }
                }
            }

            this.collect( entries, this.triangleCells, this.triangleItems );

            entries.Clear( );

            var reach = ( 0.5 * math.sqrt( 3.0 ) * this.cellSize ) + ( 1e-9 * this.cellSize );

            for( var edge = 0; edge < this.edges.Length; ++edge ) {
                if( this.edges[edge].x == -1 || this.edgeLines[edge] == -1 ) {
                    continue;
                }

                var index = this.lines.Length;
                var a     = this.positions[this.edges[edge].x];
                var b     = this.positions[this.edges[edge].y];

                this.ends.Add( a );
                this.ends.Add( b );
                this.lines.Add( this.edgeLines[edge] );

                var low  = this.cellOf( math.min( a, b ) );
                var high = this.cellOf( math.max( a, b ) );

                for( var x = low.x; x <= high.x; ++x )
                for( var y = low.y; y <= high.y; ++y )
                for( var z = low.z; z <= high.z; ++z ) {
                    var cell   = new int3( x, y, z );
                    var middle = this.origin + ( ( ( double3 )cell + 0.5 ) * this.cellSize );

                    if( math.distance( middle, nearestOnSegment( middle, a, b ) ) <= reach ) {
                        entries.Add( new Entry { key = keyOf( cell ), item = index } );
                    }
                }
            }

            this.collect( entries, this.segmentCells, this.segmentItems );

            entries.Dispose( );
        }

        private readonly int3 cellOf( double3 point ) {
            return ( int3 )math.floor( ( point - this.origin ) / this.cellSize );
        }

        private readonly void collect( NativeList<Entry> entries, NativeHashMap<long, int2> cells, NativeList<int> items ) {
            entries.Sort( );

            for( var start = 0; start < entries.Length; ) {
                var end = start + 1;
                while( end < entries.Length && entries[end].key == entries[start].key ) {
                    ++end;
                }

                cells.Add( entries[start].key, new int2( items.Length, end - start ) );

                for( var entry = start; entry < end; ++entry ) {
                    items.Add( entries[entry].item );
                }

                start = end;
            }
        }

    }

    // the grid of a mesh's triangles that have a sub-mesh, and of its edges that have a line, as the simplifier's lists
    // and BurstFeatures keep them, in cells of the given size. The grid is the caller's to dispose of
    public static SurfaceGrid of(
        NativeArray<double3> positions, NativeArray<int3> triangles, NativeArray<int> subMeshes, NativeArray<int2> edges, NativeArray<int> edgeLines, double cellSize, Allocator allocator
    ) {
        var low  = new double3( double.MaxValue );
        var high = new double3( double.MinValue );

        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            if( subMeshes[triangle] == -1 ) {
                continue;
            }
            for( var corner = 0; corner < 3; ++corner ) {
                low  = math.min( low, positions[triangles[triangle][corner]] );
                high = math.max( high, positions[triangles[triangle][corner]] );
            }
        }

        if( math.any( low > high ) ) {
            low  = double3.zero;
            high = double3.zero;
        }

        // a cell of margin all round, so that every point of the mesh is well inside
        var origin     = low - cellSize;
        var cellCounts = ( int3 )math.floor( ( high - origin ) / cellSize ) + 2;

        if( math.any( cellCounts >= ( 1 << 21 ) ) ) {
            throw new ArgumentException( $"cells of {cellSize} make a grid of {cellCounts} cells, more than a key holds", nameof( cellSize ) );
        }

        using var corners       = new NativeList<double3>( triangles.Length * 3, Allocator.TempJob );
        using var normals       = new NativeList<double3>( triangles.Length, Allocator.TempJob );
        using var subMeshesOut  = new NativeList<int>( triangles.Length, Allocator.TempJob );
        using var ends          = new NativeList<double3>( Allocator.TempJob );
        using var lines         = new NativeList<int>( Allocator.TempJob );
        using var triangleItems = new NativeList<int>( triangles.Length * 4, Allocator.TempJob );
        using var segmentItems  = new NativeList<int>( Allocator.TempJob );

        var grid = new SurfaceGrid {
            origin        = origin,
            cellSize      = cellSize,
            cellCounts    = cellCounts,
            triangleCells = new NativeHashMap<long, int2>( math.max( triangles.Length, 16 ), allocator ),
            segmentCells  = new NativeHashMap<long, int2>( 16, allocator )
        };

        new BuildJob {
            positions         = positions,
            triangles         = triangles,
            triangleSubMeshes = subMeshes,
            edges             = edges,
            edgeLines         = edgeLines,
            origin            = origin,
            cellSize          = cellSize,
            cellCounts        = cellCounts,
            corners           = corners,
            normals           = normals,
            subMeshes         = subMeshesOut,
            ends              = ends,
            lines             = lines,
            triangleCells     = grid.triangleCells,
            triangleItems     = triangleItems,
            segmentCells      = grid.segmentCells,
            segmentItems      = segmentItems
        }.Run( );

        grid.corners       = corners.ToArray( allocator );
        grid.normals       = normals.ToArray( allocator );
        grid.subMeshes     = subMeshesOut.ToArray( allocator );
        grid.ends          = ends.ToArray( allocator );
        grid.lines         = lines.ToArray( allocator );
        grid.triangleItems = triangleItems.ToArray( allocator );
        grid.segmentItems  = segmentItems.ToArray( allocator );

        return grid;
    }

    // the grid of a mesh with its features, before a remesh has changed it
    public static SurfaceGrid of( in BurstIsotropic.FeatureMesh mesh, double cellSize, Allocator allocator ) {
        return of(
            mesh.mesh.positions.AsArray( ), mesh.mesh.triangles.AsArray( ), mesh.mesh.subMeshes.AsArray( ), mesh.mesh.edges.AsArray( ), mesh.edgeLines.AsArray( ), cellSize, allocator
        );
    }

}

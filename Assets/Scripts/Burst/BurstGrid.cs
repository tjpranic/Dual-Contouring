using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using QEFSolverType             = QEFSolver.Type;

// where the corners, cells and edges of a grid of n^3 cells are in the arrays of the Burst implementations. Corner (i,
// j, k) is a point of the grid of (n + 1)^3, cell (x, y, z) the cell whose minimum is corner (x, y, z). Cells are in
// the order the managed implementation goes through them, x outermost. An edge is numbered within its axis by the
// corner it starts at, and the edges of the three axes follow each other
public readonly struct BurstGrid {

    public readonly int resolution;

    public BurstGrid( int resolution ) {
        this.resolution = resolution;
    }

    public int cornerCount {
        get { return ( this.resolution + 1 ) * ( this.resolution + 1 ) * ( this.resolution + 1 ); }
    }

    public int cellCount {
        get { return this.resolution * this.resolution * this.resolution; }
    }

    // rows of cells along z, which are next to each other in the order of the cells
    public int rowCount {
        get { return this.resolution * this.resolution; }
    }

    // edges along each axis: n of them along the axis times n + 1 across each of the other two
    public int edgesPerAxis {
        get { return this.resolution * ( this.resolution + 1 ) * ( this.resolution + 1 ); }
    }

    public int cornerIndex( int3 corner ) {
        return ( ( ( corner.x * ( this.resolution + 1 ) ) + corner.y ) * ( this.resolution + 1 ) ) + corner.z;
    }

    public int3 cornerOf( int index ) {
        var points = this.resolution + 1;
        return new int3( index / ( points * points ), ( index / points ) % points, index % points );
    }

    public int cellIndex( int3 cell ) {
        return ( ( ( cell.x * this.resolution ) + cell.y ) * this.resolution ) + cell.z;
    }

    public int3 cellOf( int index ) {
        return new int3( index / ( this.resolution * this.resolution ), ( index / this.resolution ) % this.resolution, index % this.resolution );
    }

    // the corners of the grid are exactly where the corners of the managed voxels are: n is a power of two
    public float3 positionOf( int3 corner ) {
        return ( new float3( corner ) / this.resolution ) - 0.5f;
    }

    public float3 centerOf( int3 cell ) {
        return ( ( new float3( cell ) + 0.5f ) / this.resolution ) - 0.5f;
    }

    // the corner of the grid that the given corner of a cell is
    public int3 cornerOfCell( int3 cell, int corner ) {
        return cell + new int3( BurstTables.lookupCornerOffset( corner, 0 ), BurstTables.lookupCornerOffset( corner, 1 ), BurstTables.lookupCornerOffset( corner, 2 ) );
    }

    // the edge of the grid that the given edge of a cell is
    public int edgeOfCell( int3 cell, int edge ) {
        var axis  = edge / 4;
        var start = this.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
        var size  = new int3( this.resolution + 1 );

        size[axis] = this.resolution;

        return ( axis * this.edgesPerAxis ) + ( ( ( ( start.x * size.y ) + start.y ) * size.z ) + start.z );
    }

    public void cornersOfEdge( int edge, out int3 start, out int3 end ) {
        var axis  = edge / this.edgesPerAxis;
        var index = edge % this.edgesPerAxis;
        var size  = new int3( this.resolution + 1 );

        size[axis] = this.resolution;

        start = new int3( index / ( size.y * size.z ), ( index / size.z ) % size.y, index % size.z );
        end   = start;

        end[axis] += 1;
    }

}

// how the jobs that do arithmetic are compiled. Strict rounds every operation to float in the order it is written,
// which keeps the Burst implementations as close to the managed ones as the tests hold them. Fast was measured at 3 to
// 5% less time, and moved the nodes that octree simplification collapses further from the managed ones
public static class BurstFloats {

    public const FloatMode      mode      = FloatMode.Strict;
    public const FloatPrecision precision = FloatPrecision.Standard;

}

// how many items a worker takes at a time from each kind of parallel job. Four times as many or a quarter of each made
// no difference that could be measured on grids of 128^3 and 256^3
public static class BurstBatches {

    public const int corners = 256;  // sampling the corners of the grid
    public const int cells   = 64;   // solving the vertices of cells or leaves
    public const int scans   = 1024; // going through cells without solving anything
    public const int rows    = 16;   // going through rows of cells
    public const int nodes   = 64;   // simplifying or clustering the nodes of a level of an octree

}

// the settings of a contour, for the jobs to read
public struct BurstSettings {

    public int                       resolution;
    public int                       minimizerIterations;
    public int                       binarySearchIterations;
    public int                       surfaceCorrectionIterations;
    public QEFSolverType             qefSolverType;
    public IntersectionApproximation intersectionApproximation;

}

// the density and material at every corner of the grid
[BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
public struct BurstSampleCornersJob : IJobParallelFor {

    [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

    public BurstGrid grid;

    [WriteOnly] public NativeArray<float>         densities;
    [WriteOnly] public NativeArray<MaterialIndex> materials;

    public void Execute( int index ) {
        var position = this.grid.positionOf( this.grid.cornerOf( index ) );

        this.densities[index] = BurstSurfaceExtractor.calculateDensityAndMaterial( position, this.densityFunctions, out var materialIndex );
        this.materials[index] = materialIndex;
    }

}

// which corners of every cell are solid, a bit for each in the order of the corner layout: zero or 255 where the contour
// does not pass through the cell, which is most of them. Everything that goes through the cells afterwards reads this
// one byte instead of the eight corners. A row of cells along z is done in one sweep: each cell shares the four corners
// at its maximum of z with the next one
[BurstCompile( CompileSynchronously = true )]
public struct BurstSolidCornersJob : IJobParallelFor {

    [ReadOnly] public NativeArray<MaterialIndex> materials;

    public BurstGrid grid;

    [NativeDisableParallelForRestriction] public NativeArray<byte> solidCorners;

    // the corners of the row at one height: those at the minimum and maximum of x at the minimum of y, then at the
    // maximum of y, a bit for each that is solid
    private int solidAt( int column00, int column10, int column01, int column11, int z ) {
        return ( this.materials[column00 + z] != MaterialIndex.Void ? 1 : 0 ) |
               ( this.materials[column10 + z] != MaterialIndex.Void ? 2 : 0 ) |
               ( this.materials[column01 + z] != MaterialIndex.Void ? 4 : 0 ) |
               ( this.materials[column11 + z] != MaterialIndex.Void ? 8 : 0 );
    }

    public void Execute( int row ) {
        var resolution = this.grid.resolution;

        var x = row / resolution;
        var y = row % resolution;

        // the four columns of corners along z that the row lies between
        var column00 = this.grid.cornerIndex( new int3( x,     y,     0 ) );
        var column10 = this.grid.cornerIndex( new int3( x + 1, y,     0 ) );
        var column01 = this.grid.cornerIndex( new int3( x,     y + 1, 0 ) );
        var column11 = this.grid.cornerIndex( new int3( x + 1, y + 1, 0 ) );

        var below = this.solidAt( column00, column10, column01, column11, 0 );

        for( var z = 0; z < resolution; ++z ) {
            var above = this.solidAt( column00, column10, column01, column11, z + 1 );

            // corners 0, 1, 5 and 6 are at the minimum of z, corners 3, 2, 4 and 7 above them at its maximum
            var solid = ( ( below & 1 ) << 0 ) | ( ( below & 2 ) << 0 ) | ( ( below & 4 ) << 3 ) | ( ( below & 8 ) << 3 ) |
                        ( ( above & 1 ) << 3 ) | ( ( above & 2 ) << 1 ) | ( ( above & 4 ) << 2 ) | ( ( above & 8 ) << 4 );

            this.solidCorners[( row * resolution ) + z] = ( byte )solid;

            below = above;
        }
    }

}

// the vertex of a cell of the grid as the managed implementations find it in a cell of a uniform grid or in a leaf of an
// octree: the hermite data of the edges of the cell in the order of the edge layout, the vertex placed from the QEF of
// those samples, and the average of the normals of the contour samples. The cell works out the hermite data of its own
// edges, so an edge that the contour crosses is worked out by each of the cells around it, as it is in the managed
// implementations too. That keeps the data of the edges, almost all of which the contour does not cross, out of memory
public static class BurstCells {

    // whether the cell has a vertex: the contour passes through it. The QEF is as the vertex left it, solved or not.
    // The solid corners of the cell are those BurstSolidCornersJob found
    public static bool solve(
        in BurstGrid                      grid,
        int3                              cell,
        int                               solidCorners,
        in BurstSettings                  settings,
        NativeArray<BurstDensityFunction> densityFunctions,
        NativeArray<float>                densities,
        NativeArray<MaterialIndex>        materials,
        out BurstQEF                      qef,
        out float3                        vertex,
        out float3                        normal
    ) {
        qef    = BurstQEF.create( settings.qefSolverType, settings.minimizerIterations );
        vertex = float3.zero;
        normal = float3.zero;

        // cell is either fully inside or outside the volume, skip
        if( solidCorners == 0 || solidCorners == 255 ) {
            return false;
        }

        var averageNormal = float3.zero;

        for( var edge = 0; edge < 12; ++edge ) {
            var start = grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
            var end   = grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) );

            var startIndex = grid.cornerIndex( start );
            var endIndex   = grid.cornerIndex( end );

            var startMaterial = materials[startIndex];
            var endMaterial   = materials[endIndex];

            if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                var intersection = BurstSurfaceExtractor.approximateIntersection(
                    grid.positionOf( start ),
                    densities[startIndex],
                    grid.positionOf( end ),
                    densities[endIndex],
                    densityFunctions,
                    BurstSurfaceExtractor.AnyMaterial,
                    settings.intersectionApproximation,
                    settings.binarySearchIterations
                );
                var edgeNormal = BurstSurfaceExtractor.calculateNormal( intersection, densityFunctions, BurstSurfaceExtractor.AnyMaterial );

                qef.add( intersection, edgeNormal );

                averageNormal += edgeNormal;
            }
            else if( BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) ) {
                // the boundary between two materials draws the vertex to where they meet on the contour. Its normal is
                // not one of the contour, and stays out of the normal of the vertex
                BurstSurfaceExtractor.calculateMaterialBoundary(
                    grid.positionOf( start ),
                    startMaterial,
                    grid.positionOf( end ),
                    endMaterial,
                    densityFunctions,
                    settings.intersectionApproximation,
                    settings.binarySearchIterations,
                    out var intersection,
                    out var edgeNormal
                );

                qef.add( intersection, edgeNormal );
            }
        }

        if( qef.intersectionCount == 0 ) {
            return false;
        }

        qef.placeVertex( BurstTables.lookupVertexCount( solidCorners ), grid.centerOf( cell ), out var placed, out _ );

        normal = BurstSurfaceExtractor.normalize( averageNormal );
        vertex = BurstSurfaceExtractor.surfaceCorrection( placed, normal, densityFunctions, settings.surfaceCorrectionIterations );

        return true;
    }

}

using Unity.Collections;
using Unity.Mathematics;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// what Occupancy-Based Dual Contouring asks of the volumes, and the searches it makes with the answer. The one question
// is whether a point is inside or outside: a point is inside, solid, where the volumes give it a material, which they
// work out from the signs of the shapes alone. No distance is ever looked at
public static class BurstOccupancy {

    public static bool isSolid( float3 position, NativeArray<BurstDensityFunction> densityFunctions ) {
        return BurstSurfaceExtractor.calculateMaterial( position, densityFunctions ) != MaterialIndex.Void;
    }

    // the point a part of the way along the edge of the grid that starts at the given corner and runs along the axis:
    // 0 at that corner, 1 at the next. Along the axis the coordinate is worked out the way BurstGrid.positionOf works out
    // those of the corners, so 0 and 1 give exactly the corners. A part of the way that a search of up to 15 steps
    // comes to is a position a float holds exactly, on grids of up to 256^3 cells
    public static float3 positionOnEdge( in BurstGrid grid, int3 start, int axis, float along ) {
        var position = grid.positionOf( start );

        position[axis] = ( ( start[axis] + along ) / grid.resolution ) - 0.5f;

        return position;
    }

    // section 5.1, the 1D point of an edge of the grid that the contour crosses: how far along the edge from its start
    // the contour is. The edge is halved the given number of times, each time keeping the half that has one end inside
    // and one outside. What is left of it is 2^-iterations of the edge long, and the point is its end that is inside.
    // The search depends on the edge alone, so every cell around the edge finds the same point
    public static float searchEdge( in BurstGrid grid, int3 start, int axis, bool startIsSolid, NativeArray<BurstDensityFunction> densityFunctions, int iterations ) {
        var solid = startIsSolid ? 0.0f : 1.0f;
        var empty = startIsSolid ? 1.0f : 0.0f;

        for( var iteration = 0; iteration < iterations; ++iteration ) {
            var middle = ( solid + empty ) * 0.5f;

            if( isSolid( positionOnEdge( grid, start, axis, middle ), densityFunctions ) ) {
                solid = middle;
            }
            else {
                empty = middle;
            }
        }

        return solid;
    }

}

using Unity.Collections;
using Unity.Mathematics;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// what Occupancy-Based Dual Contouring asks of the volumes, and the searches it makes with the answer. The one question
// is whether a point is inside or outside: a point is inside, solid, where the volumes give it a material, which they
// work out from the signs of the shapes alone. No distance is ever looked at
public static class BurstOccupancy {

    // how far each search goes, as the component sets it
    public struct Searches {

        public int edgeIterations; // halvings of an edge for its 1D point
        public int faceSamples;    // samples along the first ray of a 2D point
        public int faceIterations; // halvings after them

    }

    // appendix A.1.4: how far the rays of a 2D point reach, in cells. The first leaves the middle of the two 1D points
    // at right angles to the line through them, the other two run along that line, whose 1D points are at most the
    // diagonal of a face apart
    public const float acrossRange = 0.8f;
    public const float alongRange  = 0.70710678f;

    // lengths below this, in cells, count as nothing: two points in the same place, a search that found the surface
    // where it started, two lines that do not come together. The searches themselves step by five to ten times as much
    public const float tolerance = 1e-5f;

    // the sine of the angle below which three points count as lying on a line, and have no plane through them
    public const float flatness = 1e-4f;

    // how far inside its cell a vertex is kept, in cells
    public const float margin = 1e-3f;

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

    // a point of a face of the grid: the face has its minimum at the given corner and is at right angles to the axis, and
    // the point is given in cells from that corner along the two axes that follow the axis in turn. It may lie outside
    // the face
    public static float3 positionOnFace( in BurstGrid grid, int3 corner, int axis, float2 point ) {
        var position = grid.positionOf( corner );

        position[( axis + 1 ) % 3] += point.x / grid.resolution;
        position[( axis + 2 ) % 3] += point.y / grid.resolution;

        return position;
    }

    // appendix A.1.3, the line-binary search: how far from the start, along a direction in a face, the surface is first
    // crossed. Unlike on an edge the far end is not known to be on the other side, and the nearest crossing is wanted.
    // So the ray is first sampled at even steps out to its range, up to the first sample on the other side from the
    // start, and the step before that sample is then halved like an edge. With no such sample the last step is halved,
    // which gives a point as far out as the range. The result is the near end of what is left: a point on the same side
    // as the start
    public static float searchLine(
        in BurstGrid                      grid,
        int3                              corner,
        int                               axis,
        float2                            start,
        float2                            direction,
        bool                              startIsSolid,
        float                             range,
        int                               samples,
        int                               iterations,
        NativeArray<BurstDensityFunction> densityFunctions
    ) {
        var step = range / samples;
        var near = step * ( samples - 1 );

        for( var sample = 1; sample <= samples; ++sample ) {
            if( isSolid( positionOnFace( grid, corner, axis, start + ( direction * ( step * sample ) ) ), densityFunctions ) != startIsSolid ) {
                near = step * ( sample - 1 );

                break;
            }
        }

        var far = near + step;

        for( var iteration = 0; iteration < iterations; ++iteration ) {
            var middle = ( near + far ) * 0.5f;

            if( isSolid( positionOnFace( grid, corner, axis, start + ( direction * middle ) ), densityFunctions ) == startIsSolid ) {
                near = middle;
            }
            else {
                far = middle;
            }
        }

        return near;
    }

    // section 5.2, the 2D point of a line the contour draws across a face of the grid, between the 1D points at its two
    // ends: where the surface's two straight pieces would meet on that face, if it is flat on either side of a single
    // bend (lemma 5.1). The points are given and returned as positionOnFace takes them, and solidCorners has bit
    // i + 2j set where the corner of the face at ( i, j ) is inside.
    // - The middle of the two 1D points is sampled, and a ray leaves it at right angles to the line through them, toward
    //   the side where the corners of the face are on the other side of the surface from the middle. Where it first
    //   crosses the surface is the top.
    // - From the top two rays run along the line, one toward each 1D point, to where they cross the surface.
    // - The 2D point is where the line through the first 1D point and the end of the ray toward it meets the line
    //   through the second and the end of its ray.
    // It is the middle itself where the 1D points coincide, where the surface is at the middle already, which is a
    // straight crossing, and where the two lines do not meet beyond the top
    public static float2 searchFace(
        in BurstGrid                      grid,
        int3                              corner,
        int                               axis,
        float2                            first,
        float2                            second,
        int                               solidCorners,
        int                               samples,
        int                               iterations,
        NativeArray<BurstDensityFunction> densityFunctions
    ) {
        var middle = ( first + second ) * 0.5f;
        var chord  = first - second;
        var length = math.length( chord );

        if( length < tolerance ) {
            return middle;
        }

        var along  = chord / length;
        var across = new float2( -along.y, along.x );

        var middleIsSolid = isSolid( positionOnFace( grid, corner, axis, middle ), densityFunctions );

        // each corner of the face that is off the line says which way the other side is: toward it if it is on the
        // other side from the middle, away from it if it is not. On a face with two lines the corner beyond the other
        // line is outvoted
        var votes = 0;

        for( var faceCorner = 0; faceCorner < 4; ++faceCorner ) {
            var offset = math.dot( new float2( faceCorner & 1, faceCorner >> 1 ) - middle, across );

            if( math.abs( offset ) < tolerance ) {
                continue;
            }

            var onOtherSide = ( ( solidCorners & ( 1 << faceCorner ) ) != 0 ) != middleIsSolid;

            votes += ( offset > 0.0f ) == onOtherSide ? 1 : -1;
        }

        if( votes < 0 ) {
            across = -across;
        }

        var height = searchLine( grid, corner, axis, middle, across, middleIsSolid, acrossRange, samples, iterations, densityFunctions );

        if( height < tolerance ) {
            return middle;
        }

        // the two rays along the line take a sample fewer and a halving more, as in appendix A.1.4, so each of the
        // three searches samples as often
        var top           = middle + ( across * height );
        var alongSamples  = math.max( 1, samples - 1 );
        var towardFirst   = searchLine( grid, corner, axis, top,  along, middleIsSolid, alongRange, alongSamples, iterations + 1, densityFunctions );
        var towardSecond  = searchLine( grid, corner, axis, top, -along, middleIsSolid, alongRange, alongSamples, iterations + 1, densityFunctions );

        // at the height of the top the two lines are this much closer together than at the 1D points. They meet where
        // that has taken up all of the distance between the 1D points
        var narrowing = length - towardFirst - towardSecond;

        if( narrowing < tolerance ) {
            return middle;
        }

        var scale = length / narrowing;

        return middle + ( along * ( scale * ( towardFirst - towardSecond ) * 0.5f ) ) + ( across * ( scale * height ) );
    }

    // section 5.3, for one vertex of a cell: the 1D points of the edges of its piece of the contour, in the order they
    // follow each other around the piece, and at each the normal of the plane through that point and the two 2D points
    // next to it, on the two faces of the cell that the edge lies in. The normal points from the solid end of the edge
    // to the empty one. It is zero where the three points lie on a line and there is no plane.
    // - solid are the solid corners of the cell and lookup what it looks its vertices up with (BurstCells.vertexLookupOf).
    // - edges, points and normals have room for the twelve edges of a cell. The points are in voxelizer object space.
    // Each 2D point is searched for from the face and its two 1D points alone, the lower numbered edge of the grid
    // first, so the cell on the other side of the face finds the same one. Returns how many edges the piece has
    public static unsafe int planesOf(
        in BurstGrid                      grid,
        int3                              cell,
        int                               solid,
        int                               lookup,
        int                               vertex,
        in Searches                       searches,
        NativeArray<MaterialIndex>        materials,
        NativeArray<BurstDensityFunction> densityFunctions,
        int*                              edges,
        float3*                           points,
        float3*                           normals
    ) {
        const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

        // the faces the piece leaves each of its edges through, and the 1D and 2D points in cells from the minimum of
        // the cell. The 2D point of an edge is the one between it and the edge that follows it
        var faces      = stackalloc int[EdgeCount];
        var local      = stackalloc float3[EdgeCount];
        var facePoints = stackalloc float3[EdgeCount];

        var first = 0;

        while( BurstTables.lookupEdgeVertex( lookup, first ) != vertex ) {
            ++first;
        }

        var count = 0;
        var edge  = first;
        var side  = 0;

        do {
            // an edge runs along its axis from its first corner to its second, and the edges of an axis follow each
            // other in the edge layout
            var axis         = edge / 4;
            var startCorner  = BurstTables.lookupEdgeCorner( edge, 0 );
            var start        = grid.cornerOfCell( cell, startCorner );
            var startIsSolid = ( solid & ( 1 << startCorner ) ) != 0;

            var along = searchEdge( grid, start, axis, startIsSolid, densityFunctions, searches.edgeIterations );

            edges[count]  = edge;
            points[count] = positionOnEdge( grid, start, axis, along );
            faces[count]  = BurstTables.lookupEdgeFace( edge, side );

            local[count] = new float3( BurstTables.lookupCornerOffset( startCorner, 0 ), BurstTables.lookupCornerOffset( startCorner, 1 ), BurstTables.lookupCornerOffset( startCorner, 2 ) );

            local[count][axis] += along;

            // on to the next edge across that face, which the piece leaves through its other face
            edge = BurstTables.lookupEdgeAcrossFace( lookup, edge, side );
            side = BurstTables.lookupEdgeFace( edge, 0 ) == faces[count] ? 1 : 0;

            ++count;
        }
        while( edge != first );

        for( var index = 0; index < count; ++index ) {
            var next = ( index + 1 ) % count;

            var axis   = BurstTables.lookupFaceAxis( faces[index] );
            var across = ( axis + 1 ) % 3;
            var other  = ( axis + 2 ) % 3;

            // the face of the grid: its minimum is that of the cell, or one cell on along the axis
            var corner = cell;

            corner[axis] += BurstTables.lookupFaceSide( faces[index] );

            var faceCorners = 0;

            for( var faceCorner = 0; faceCorner < 4; ++faceCorner ) {
                var position = corner;

                position[across] += faceCorner & 1;
                position[other]  += faceCorner >> 1;

                if( materials[grid.cornerIndex( position )] != MaterialIndex.Void ) {
                    faceCorners |= 1 << faceCorner;
                }
            }

            var from = new float2( local[index][across], local[index][other] );
            var to   = new float2( local[next][across],  local[next][other] );

            var ordered = grid.edgeOfCell( cell, edges[index] ) < grid.edgeOfCell( cell, edges[next] );

            var point = searchFace( grid, corner, axis, ordered ? from : to, ordered ? to : from, faceCorners, searches.faceSamples, searches.faceIterations, densityFunctions );

            facePoints[index] = float3.zero;

            facePoints[index][axis]   = BurstTables.lookupFaceSide( faces[index] );
            facePoints[index][across] = point.x;
            facePoints[index][other]  = point.y;
        }

        for( var index = 0; index < count; ++index ) {
            var toPrevious = facePoints[( index + count - 1 ) % count] - local[index];
            var toNext     = facePoints[index] - local[index];

            var normal = math.cross( toPrevious, toNext );
            var length = math.length( normal );

            if( !( length > flatness * math.length( toPrevious ) * math.length( toNext ) ) ) {
                normals[index] = float3.zero;

                continue;
            }

            normal /= length;

            var axis         = edges[index] / 4;
            var startIsSolid = ( solid & ( 1 << BurstTables.lookupEdgeCorner( edges[index], 0 ) ) ) != 0;

            if( ( normal[axis] > 0.0f ) != startIsSolid ) {
                normal = -normal;
            }

            normals[index] = normal;
        }

        return count;
    }

    // a vertex that the QEF put outside its cell, moved back toward the anchor, a point of the cell, until it is inside
    // by the margin. The polygons of section 5.4 are only free of intersections with every vertex in its own cell. A
    // vertex that is inside already stays as it is
    public static float3 intoCell( in BurstGrid grid, int3 cell, float3 anchor, float3 vertex ) {
        var inset   = margin / grid.resolution;
        var minimum = grid.positionOf( cell ) + inset;
        var maximum = grid.positionOf( cell + 1 ) - inset;

        if( math.all( vertex >= minimum ) && math.all( vertex <= maximum ) ) {
            return vertex;
        }

        anchor = math.clamp( anchor, minimum, maximum );

        // the part of the way from the anchor to the vertex that is inside
        var part = 1.0f;

        for( var axis = 0; axis < 3; ++axis ) {
            if( vertex[axis] > maximum[axis] ) {
                part = math.min( part, ( maximum[axis] - anchor[axis] ) / ( vertex[axis] - anchor[axis] ) );
            }
            else if( vertex[axis] < minimum[axis] ) {
                part = math.min( part, ( minimum[axis] - anchor[axis] ) / ( vertex[axis] - anchor[axis] ) );
            }
        }

        return math.clamp( anchor + ( ( vertex - anchor ) * part ), minimum, maximum );
    }

}

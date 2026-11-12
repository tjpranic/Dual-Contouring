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

    // the 1D and 2D points of a grid. An edge has up to four cells around it and a face one on either side, which all
    // place their vertices from the same points, so each point is searched for once and kept here for them. Almost all
    // edges and faces have none, so the points are kept one after the other, corner by corner:
    // - a corner has the three edges that start at it and the three faces that have their minimum at it, where the
    //   grid goes on that far;
    // - its points are a number for each of those edges the contour crosses, x first, which is how far along the edge
    //   the contour is, and then two numbers for each line the contour draws across each of those faces, the face at
    //   right angles to x first, which are the 2D point as positionOnFace takes it;
    // - starts says where the points of each corner begin.
    // Where a point is among those of its corner follows from which edges are crossed and how many lines the faces
    // have, which the materials of the corners tell
    public struct Points {

        [ReadOnly] public NativeArray<int>   starts;
        [ReadOnly] public NativeArray<float> values;

        // the 1D point of a crossed edge: how far along the edge from its start the contour is
        public float alongEdge( in BurstGrid grid, NativeArray<MaterialIndex> materials, int3 start, int axis ) {
            return this.values[this.starts[grid.cornerIndex( start )] + offsetOf( grid, materials, start, axis, 0 )];
        }

        // the 2D point of a line across the face at right angles to the axis that has its minimum at the corner. A face
        // with two lines has first the one that goes through its first edge: the one that starts at the corner and runs
        // along the axis that follows the face's own
        public float2 onFace( in BurstGrid grid, NativeArray<MaterialIndex> materials, int3 corner, int axis, int line ) {
            var at = this.starts[grid.cornerIndex( corner )] + offsetOf( grid, materials, corner, 3, axis ) + ( 2 * line );

            return new float2( this.values[at], this.values[at + 1] );
        }

    }

    // whether the contour crosses the edge that starts at the corner and runs along the axis, if the grid has it
    public static bool isCrossed( in BurstGrid grid, NativeArray<MaterialIndex> materials, int3 start, int axis ) {
        if( start[axis] >= grid.resolution ) {
            return false;
        }

        var end = start;

        end[axis] += 1;

        return ( materials[grid.cornerIndex( start )] != MaterialIndex.Void ) != ( materials[grid.cornerIndex( end )] != MaterialIndex.Void );
    }

    // whether the grid has the face at right angles to the axis that has its minimum at the corner
    public static bool hasFace( in BurstGrid grid, int3 corner, int axis ) {
        return corner[( axis + 1 ) % 3] < grid.resolution && corner[( axis + 2 ) % 3] < grid.resolution;
    }

    // which corners of that face are inside: bit i + 2j for the corner at ( i, j ) along the two axes that follow the
    // face's own in turn
    public static int faceCornersOf( in BurstGrid grid, NativeArray<MaterialIndex> materials, int3 corner, int axis ) {
        var across      = ( axis + 1 ) % 3;
        var other       = ( axis + 2 ) % 3;
        var faceCorners = 0;

        for( var faceCorner = 0; faceCorner < 4; ++faceCorner ) {
            var position = corner;

            position[across] += faceCorner & 1;
            position[other]  += faceCorner >> 1;

            if( materials[grid.cornerIndex( position )] != MaterialIndex.Void ) {
                faceCorners |= 1 << faceCorner;
            }
        }

        return faceCorners;
    }

    // how many lines the contour draws across a face with those corners inside: none where they are all alike, two
    // where the two that are inside lie diagonally across it, and otherwise one
    public static int linesOf( int faceCorners ) {
        return faceCorners == 0b0000 || faceCorners == 0b1111 ? 0 : faceCorners == 0b0110 || faceCorners == 0b1001 ? 2 : 1;
    }

    // how many numbers the points of a corner take up before those of its edge along the axis `edges`, with `faces` 0,
    // or before those of its face at right angles to the axis `faces`, with `edges` 3. With both 3 that is all of them
    public static int offsetOf( in BurstGrid grid, NativeArray<MaterialIndex> materials, int3 corner, int edges, int faces ) {
        var offset = 0;

        for( var axis = 0; axis < edges; ++axis ) {
            offset += isCrossed( grid, materials, corner, axis ) ? 1 : 0;
        }
        for( var axis = 0; axis < faces; ++axis ) {
            offset += hasFace( grid, corner, axis ) ? 2 * linesOf( faceCornersOf( grid, materials, corner, axis ) ) : 0;
        }

        return offset;
    }

    // whether a corner can have points at all. Its edges and faces are those of the cell that has its minimum there,
    // which the contour passes through or not. A corner at the maximum of the grid has no such cell to tell
    public static bool mayHavePoints( in BurstGrid grid, NativeArray<byte> solidCorners, int3 corner ) {
        if( math.any( corner >= grid.resolution ) ) {
            return true;
        }

        var solid = solidCorners[grid.cellIndex( corner )];

        return solid != 0 && solid != 255;
    }

    // the 2D points of the lines across a face of the grid, written to values from the given place on: of its one line,
    // or of its two, the one through its first edge first. The face has its minimum at the corner and is at
    // right angles to the axis, and its 1D points are in values already.
    // - Each point is searched for from the face and the 1D points at the ends of its line alone, the lower numbered
    //   edge of the grid first.
    // - Which edges the two lines of a face join is up to the cells on either side of it, which agree on it
    //   (DualMarchingCubes.lookupEdgeAcrossFace): the one that has its minimum at the corner is asked, or at the maximum
    //   of the grid the one before it
    public static unsafe void searchFaceLines(
        in BurstGrid                      grid,
        int3                              corner,
        int                               axis,
        int                               faceCorners,
        in Searches                       searches,
        NativeArray<MaterialIndex>        materials,
        NativeArray<byte>                 solidCorners,
        NativeArray<BurstDensityFunction> densityFunctions,
        NativeArray<int>                  starts,
        NativeArray<float>                values,
        int                               at
    ) {
        var across = ( axis + 1 ) % 3;
        var other  = ( axis + 2 ) % 3;
        var points = new Points { starts = starts, values = values };

        // the four edges of the face, each with its number in the grid: the two along its first axis, at the minimum and
        // the maximum of its second, then the two along its second axis
        var numbers = stackalloc int[4];
        var ends    = stackalloc float2[4];
        var crossed = stackalloc bool[4];

        for( var edge = 0; edge < 4; ++edge ) {
            var edgeAxis = edge < 2 ? across : other;
            var start    = corner;

            if( edge == 1 ) {
                start[other] += 1;
            }
            if( edge == 3 ) {
                start[across] += 1;
            }

            numbers[edge] = grid.edgeIndex( start, edgeAxis );
            crossed[edge] = isCrossed( grid, materials, start, edgeAxis );
            ends[edge]    = float2.zero;

            if( crossed[edge] ) {
                var along = points.alongEdge( grid, materials, start, edgeAxis );

                ends[edge] = edge == 0 ? new float2( along, 0.0f ) : edge == 1 ? new float2( along, 1.0f ) : edge == 2 ? new float2( 0.0f, along ) : new float2( 1.0f, along );
            }
        }

        // the edges at the ends of each line
        var from = new int2( -1, -1 );
        var to   = new int2( -1, -1 );

        if( linesOf( faceCorners ) == 1 ) {
            for( var edge = 0; edge < 4; ++edge ) {
                if( crossed[edge] ) {
                    if( from[0] < 0 ) {
                        from[0] = edge;
                    }
                    else {
                        to[0] = edge;
                    }
                }
            }
        }
        else {
            var cell = corner;
            var side = 0;

            if( cell[axis] >= grid.resolution ) {
                cell[axis] -= 1;
                side        = 1;
            }

            var lookup = BurstCells.vertexLookupOf( grid, solidCorners, grid.cellIndex( cell ) );
            var joined = -1;

            for( var edge = 0; edge < SurfaceExtractor.Voxel.EdgeCount; ++edge ) {
                if( grid.edgeOfCell( cell, edge ) != numbers[0] ) {
                    continue;
                }

                for( var edgeSide = 0; edgeSide < 2; ++edgeSide ) {
                    var face = BurstTables.lookupEdgeFace( edge, edgeSide );

                    if( BurstTables.lookupFaceAxis( face ) == axis && BurstTables.lookupFaceSide( face ) == side ) {
                        joined = grid.edgeOfCell( cell, BurstTables.lookupEdgeAcrossFace( lookup, edge, edgeSide ) );
                    }
                }
            }

            // a line cuts a corner off the face, so the first edge is joined to one of the two along the other axis,
            // and the edge across from it to the other
            var withFirst = joined == numbers[2] ? 2 : 3;

            from = new int2( 0, 1 );
            to   = new int2( withFirst, 5 - withFirst );
        }

        for( var line = 0; line < 2; ++line ) {
            if( from[line] < 0 ) {
                continue;
            }

            var ordered = numbers[from[line]] < numbers[to[line]];

            var point = searchFace(
                grid,
                corner,
                axis,
                ordered ? ends[from[line]] : ends[to[line]],
                ordered ? ends[to[line]]   : ends[from[line]],
                faceCorners,
                searches.faceSamples,
                searches.faceIterations,
                densityFunctions
            );

            values[at + ( 2 * line )]     = point.x;
            values[at + ( 2 * line ) + 1] = point.y;
        }
    }

    // section 5.3, for one vertex of a cell: the 1D points of the edges of its piece of the contour, in the order they
    // follow each other around the piece, and at each the normal of the plane through that point and the two 2D points
    // next to it, on the two faces of the cell that the edge lies in. The normal points from the solid end of the edge
    // to the empty one. It is zero where the three points lie on a line and there is no plane.
    // - solid are the solid corners of the cell and lookup what it looks its vertices up with (BurstCells.vertexLookupOf).
    // - found are the 1D and 2D points of the grid, which every cell around an edge or a face reads alike.
    // - edges, points and normals have room for the twelve edges of a cell. The points are in voxelizer object space.
    // Returns how many edges the piece has
    public static unsafe int planesOf(
        in BurstGrid               grid,
        int3                       cell,
        int                        solid,
        int                        lookup,
        int                        vertex,
        in Points                  found,
        NativeArray<MaterialIndex> materials,
        int*                       edges,
        float3*                    points,
        float3*                    normals
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

            var along = found.alongEdge( grid, materials, start, axis );

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

            // of the two lines of a face, the first is the one through the face's first edge (Points.onFace)
            var line = 0;

            if( linesOf( faceCornersOf( grid, materials, corner, axis ) ) == 2 ) {
                var firstEdge = grid.edgeIndex( corner, across );

                line = grid.edgeOfCell( cell, edges[index] ) == firstEdge || grid.edgeOfCell( cell, edges[next] ) == firstEdge ? 0 : 1;
            }

            var point = found.onFace( grid, materials, corner, axis, line );

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

    // a vertex that the QEF put outside its part of its cell, moved back toward the anchor, a point of that part, until
    // it is inside by the margin. The polygons of section 5.4 are only free of intersections with every vertex in its
    // own cell. And where a cell has several vertices, which the paper leaves uncovered, each stays on its own side of
    // the planes between it and the others (DualMarchingCubes.lookupSeparation), so the polygons of one piece of the
    // contour do not pass through those of another. A vertex that is inside already stays as it is.
    // - lookup is what the cell looks its vertices up with, and index which of them this one is.
    // - the anchor is the average of the 1D points of the vertex, which lie on its side of every such plane
    public static float3 intoCell( in BurstGrid grid, int3 cell, int lookup, int index, float3 anchor, float3 vertex ) {
        // everything in cells from the minimum of the cell, where the cell is the unit cube
        var origin = grid.positionOf( cell );
        var from   = ( anchor - origin ) * grid.resolution;
        var to     = ( vertex - origin ) * grid.resolution;

        // the anchor first: it can be nearer a face or a plane than the margin
        from = math.clamp( from, margin, 1.0f - margin );

        for( var other = 0; other < 4; ++other ) {
            if( BurstTables.lookupSeparation( lookup, index, other, out var normal, out var twiceOffset ) ) {
                var past = math.dot( normal, from ) - ( ( twiceOffset * 0.5f ) - margin );

                if( past > 0.0f ) {
                    from -= ( float3 )normal * ( past / math.dot( normal, normal ) );
                }
            }
        }

        // the part of the way from the anchor to the vertex that is inside
        var part = 1.0f;

        for( var axis = 0; axis < 3; ++axis ) {
            if( to[axis] > 1.0f - margin ) {
                part = math.min( part, ( 1.0f - margin - from[axis] ) / ( to[axis] - from[axis] ) );
            }
            else if( to[axis] < margin ) {
                part = math.min( part, ( margin - from[axis] ) / ( to[axis] - from[axis] ) );
            }
        }

        for( var other = 0; other < 4; ++other ) {
            if( BurstTables.lookupSeparation( lookup, index, other, out var normal, out var twiceOffset ) ) {
                var limit = ( twiceOffset * 0.5f ) - margin;
                var past  = math.dot( normal, to ) - limit;

                if( past > 0.0f ) {
                    var inside = limit - math.dot( normal, from );

                    part = math.min( part, inside > 0.0f ? inside / ( inside + past ) : 0.0f );
                }
            }
        }

        if( part >= 1.0f ) {
            return vertex;
        }

        return origin + ( math.clamp( from + ( ( to - from ) * math.max( part, 0.0f ) ), margin, 1.0f - margin ) / grid.resolution );
    }

    // how the polygon around an edge is made into triangles. Its four vertices are numbered going around the edge, so
    // that the triangle of the first three faces the empty end of the edge
    public const int NoPolygon      = 0;
    public const int FirstDiagonal  = 1; // triangles 0 1 2 and 0 2 3
    public const int SecondDiagonal = 2; // triangles 0 1 3 and 1 2 3
    public const int Fan            = 3; // four triangles around the 1D point of the edge

    // appendix A.1.5, equations 3 and 4: a vertex of the polygon is concave when it lies below the triangle of its two
    // neighbors and the empty end of the edge, or above the triangle of its neighbors and the solid end. The diagonal
    // between those neighbors then leaves the envelope the polygon has to stay in: the four tetrahedra between the edge
    // and each side of the polygon (Intersection-free Contouring, and Wang's convex/concave analysis)
    public static bool isConcave( float3 vertex, float3 previous, float3 next, float3 solidEnd, float3 emptyEnd ) {
        return math.dot( vertex - emptyEnd, math.cross( previous - emptyEnd, next - emptyEnd ) ) < 0.0f ||
               math.dot( vertex - solidEnd, math.cross( previous - solidEnd, next - solidEnd ) ) > 0.0f;
    }

    // section 5.4 and appendix A.1.5: which way to split the polygon around an edge so that its triangles stay inside
    // the envelope. A diagonal may be used when neither of the two vertices it passes between is concave. With neither
    // diagonal the polygon becomes four triangles around the 1D point of the edge. The paper leaves open which to take
    // when both may be used: here it is the one whose triangles pass nearer the 1D point, which is on the surface, and
    // the second, the one the other algorithms always take, when they are as near to within the given distance
    public static int splitOf( float3 vertex0, float3 vertex1, float3 vertex2, float3 vertex3, float3 solidEnd, float3 emptyEnd, float3 edgePoint, float nothing ) {
        var first  = !isConcave( vertex1, vertex0, vertex2, solidEnd, emptyEnd ) && !isConcave( vertex3, vertex2, vertex0, solidEnd, emptyEnd );
        var second = !isConcave( vertex0, vertex3, vertex1, solidEnd, emptyEnd ) && !isConcave( vertex2, vertex1, vertex3, solidEnd, emptyEnd );

        if( !first && !second ) {
            return Fan;
        }
        if( first != second ) {
            return first ? FirstDiagonal : SecondDiagonal;
        }

        var alongFirst  = math.min( distanceToTriangle( edgePoint, vertex0, vertex1, vertex2 ), distanceToTriangle( edgePoint, vertex0, vertex2, vertex3 ) );
        var alongSecond = math.min( distanceToTriangle( edgePoint, vertex0, vertex1, vertex3 ), distanceToTriangle( edgePoint, vertex1, vertex2, vertex3 ) );

        return alongFirst < alongSecond - nothing ? FirstDiagonal : SecondDiagonal;
    }

    // how far a point is from a triangle: from the nearest point of its inside, its sides or its corners (Ericson,
    // Real-Time Collision Detection, section 5.1.5)
    public static float distanceToTriangle( float3 point, float3 a, float3 b, float3 c ) {
        var ab = b - a;
        var ac = c - a;
        var ap = point - a;

        var d1 = math.dot( ab, ap );
        var d2 = math.dot( ac, ap );

        if( d1 <= 0.0f && d2 <= 0.0f ) {
            return math.length( ap );
        }

        var bp = point - b;
        var d3 = math.dot( ab, bp );
        var d4 = math.dot( ac, bp );

        if( d3 >= 0.0f && d4 <= d3 ) {
            return math.length( bp );
        }

        var vc = ( d1 * d4 ) - ( d3 * d2 );

        if( vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f ) {
            return math.length( point - ( a + ( ab * ( d1 / ( d1 - d3 ) ) ) ) );
        }

        var cp = point - c;
        var d5 = math.dot( ab, cp );
        var d6 = math.dot( ac, cp );

        if( d6 >= 0.0f && d5 <= d6 ) {
            return math.length( cp );
        }

        var vb = ( d5 * d2 ) - ( d1 * d6 );

        if( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f ) {
            return math.length( point - ( a + ( ac * ( d2 / ( d2 - d6 ) ) ) ) );
        }

        var va = ( d3 * d6 ) - ( d5 * d4 );

        if( va <= 0.0f && ( d4 - d3 ) >= 0.0f && ( d5 - d6 ) >= 0.0f ) {
            return math.length( point - ( b + ( ( c - b ) * ( ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) ) ) ) ) );
        }

        var denominator = 1.0f / ( va + vb + vc );

        return math.length( point - ( a + ( ab * ( vb * denominator ) ) + ( ac * ( vc * denominator ) ) ) );
    }

}

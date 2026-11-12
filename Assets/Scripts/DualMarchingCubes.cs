using System.Collections.Generic;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Manifold Dual Contouring, section III: a cell holds one vertex for every piece of the contour that passes through it,
// and each edge of the cell that the contour intersects is associated with exactly one of those vertices. The pieces
// are the cycles marching cubes would draw in the cell, which makes the contour the dual of the one marching cubes
// extracts, and a manifold like it wherever no two cells have a joined face in common (lookupJoinedFace). Corners and
// edges are numbered as in the corner layout of the voxels
public static class DualMarchingCubes {

    // the corners at the ends of each of the twelve edges of a cell
    private static readonly int[,] edgeCorners = {
        { 0, 1 }, { 3, 2 }, { 5, 6 }, { 4, 7 }, // x axis
        { 0, 5 }, { 1, 6 }, { 3, 4 }, { 2, 7 }, // y axis
        { 0, 3 }, { 1, 2 }, { 5, 4 }, { 6, 7 }  // z axis
    };

    // the edges around each of the six faces of a cell
    private static readonly int[,] faceEdges = {
        { 0, 1,  8,  9 }, // bottom
        { 2, 3, 10, 11 }, // top
        { 4, 6,  8, 10 }, // minimum x
        { 5, 7,  9, 11 }, // maximum x
        { 0, 2,  4,  5 }, // minimum z
        { 1, 3,  6,  7 }  // maximum z
    };

    // the edges associated with each vertex of a cell, for each of the 256 ways its corners can be solid or empty
    private static readonly int[][][] vertexEdges = buildVertexEdges( );

    // the faces of a cell as the rest of the project numbers them, each as the four edges around it. faceEdges above
    // lists the same faces in an order of its own, which the numbering of the vertices of a cell follows from
    private static readonly int[,] faceLayout = {
        { 0, 1,  8,  9 }, // minimum y
        { 0, 2,  4,  5 }, // minimum z
        { 1, 3,  6,  7 }, // maximum z
        { 4, 6,  8, 10 }, // minimum x
        { 5, 7,  9, 11 }, // maximum x
        { 2, 3, 10, 11 }  // maximum y
    };

    // the axis each of those faces is at right angles to, 0 for x, 1 for y and 2 for z, and the side of the cell it is
    // on: 0 at the minimum of that axis and 1 at the maximum
    private static readonly int[] faceAxes  = { 1, 2, 2, 0, 0, 1 };
    private static readonly int[] faceSides = { 0, 0, 1, 0, 1, 1 };

    // the two faces each edge lies in, the lower numbered one first
    private static readonly int[,] edgeFaces = buildEdgeFaces( );

    // for each of the 256 ways, the edge that follows each edge around its piece of the contour across either of its
    // two faces, and the face whose four edges belong to a single vertex
    private static readonly int[,,] edgesAcrossFaces = buildEdgesAcrossFaces( );
    private static readonly int[]   joinedFaces      = buildJoinedFaces( );

    // where each corner of a cell is, in units of the cell from its minimum
    private static readonly int[,] cornerOffsets = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 },
        { 0, 1, 1 }, { 0, 1, 0 }, { 1, 1, 0 }, { 1, 1, 1 }
    };

    // for each of the 256 ways and each pair of vertices of the cell, the lower numbered one first, a plane between
    // their pieces of the contour: its normal and twice its offset, all zero where there is no such pair
    private static readonly int[,,] separations = buildSeparations( );

    // the edges of a cell that each of its vertices is associated with, given the materials at its corners. A cell
    // that the contour does not pass through has no vertices
    public static int[][] lookupVertexEdges( MaterialIndex[] corners ) {
        return lookupVertexEdges( solidCornersOf( corners ) );
    }

    // the corners of a cell that are solid, as a set of bits: bit 0 for corner 0 up to bit 7 for corner 7
    public static int solidCornersOf( MaterialIndex[] corners ) {
        var solidCorners = 0;

        for( var corner = 0; corner < corners.Length; ++corner ) {
            if( corners[corner] != MaterialIndex.Void ) {
                solidCorners |= 1 << corner;
            }
        }

        return solidCorners;
    }

    // the same, given the solid corners as a set of bits: bit 0 for corner 0 up to bit 7 for corner 7
    public static int[][] lookupVertexEdges( int solidCorners ) {
        return vertexEdges[solidCorners];
    }

    public static int lookupFaceAxis( int face ) {
        return faceAxes[face];
    }

    public static int lookupFaceSide( int face ) {
        return faceSides[face];
    }

    // one of the two faces an edge lies in: side 0 for the lower numbered one, side 1 for the other
    public static int lookupEdgeFace( int edge, int side ) {
        return edgeFaces[edge, side];
    }

    // a piece of the contour crosses the faces of its cell in lines, each from one of its edges to the next, so its
    // edges form a ring. This is the edge that follows the given one in that ring across one of the two faces the edge
    // lies in, or -1 for an edge the contour does not intersect. Occupancy-Based Dual Contouring, appendix A.1.1
    public static int lookupEdgeAcrossFace( int solidCorners, int edge, int side ) {
        return edgesAcrossFaces[solidCorners, edge, side];
    }

    // the face of the cell on which two solid corners lie diagonally across from each other while all four edges of
    // the face belong to the same vertex, or -1: the pieces that cut off each of the two solid corners on that face are
    // one piece elsewhere in the cell. Where two cells have such a face in common, the four polygons around the edges
    // of the face all run through the same two vertices, one in each cell, and the edge between those vertices is part
    // of four polygons instead of two (Occupancy-Based Dual Contouring, appendix A.1.1). No cell has two such faces
    public static int lookupJoinedFace( int solidCorners ) {
        return joinedFaces[solidCorners];
    }

    // the same cell with solid and empty swapped. The contour intersects the same edges, and wherever a face is
    // ambiguous the table looked up with the complement cuts off the empty corners of that face instead of the solid
    // ones. Two cells that have a joined face in common both take their vertices from their complements, which have
    // no joined face, and the edge between them is part of two polygons again
    public static int complement( int solidCorners ) {
        return ~solidCorners & 255;
    }

    // a plane that keeps two vertices of a cell apart. The edges of the piece of the contour of the first lie on one side
    // of it or in it, and those of the second on the other side or in it: for every point of an edge of the first, the
    // normal times the point is at most the offset, in units of the cell from its minimum. Each vertex of a cell with
    // several stays on its own side of such planes, so that the polygons of one piece do not pass through those of
    // another (Occupancy-Based Dual Contouring, appendix A.1.5, leaves cells with several vertices uncovered). The
    // offset is given twice, which makes it a whole number. False where the cell has no such two vertices
    public static bool lookupSeparation( int solidCorners, int vertex, int other, out UnityEngine.Vector3Int normal, out int twiceOffset ) {
        normal      = UnityEngine.Vector3Int.zero;
        twiceOffset = 0;

        var count = vertexEdges[solidCorners].Length;

        if( vertex == other || vertex < 0 || other < 0 || vertex >= count || other >= count ) {
            return false;
        }

        var lower  = System.Math.Min( vertex, other );
        var higher = System.Math.Max( vertex, other );
        var pair   = ( lower * ( 7 - lower ) / 2 ) + ( higher - lower - 1 );
        var sign   = vertex < other ? 1 : -1;

        normal      = new UnityEngine.Vector3Int( separations[solidCorners, pair, 0], separations[solidCorners, pair, 1], separations[solidCorners, pair, 2] ) * sign;
        twiceOffset = separations[solidCorners, pair, 3] * sign;

        return true;
    }

    // whether a cell and the cell across its joined face both have the face between them joined, given the solid corners
    // of each. Such a cell takes its vertices from its complement
    public static bool sharesJoinedFace( int solidCorners, int solidCornersAcross ) {
        var face  = joinedFaces[solidCorners];
        var other = joinedFaces[solidCornersAcross];

        return face != -1 && other != -1 && faceAxes[face] == faceAxes[other] && faceSides[face] != faceSides[other];
    }

    private static int[][][] buildVertexEdges( ) {
        var table = new int[256][][];

        for( var solidCorners = 0; solidCorners < table.Length; ++solidCorners ) {
            table[solidCorners] = findVertexEdges( solidCorners );
        }

        return table;
    }

    // the contour crosses a face of the cell in lines that run from one intersected edge of the face to another, and
    // edges that such a line connects belong to the same piece of the contour
    private static int[][] findVertexEdges( int solidCorners ) {
        bool isSolid( int corner ) {
            return ( solidCorners & ( 1 << corner ) ) != 0;
        }

        bool isIntersected( int edge ) {
            return isSolid( edgeCorners[edge, 0] ) != isSolid( edgeCorners[edge, 1] );
        }

        // the edge that stands for the piece each edge was found to belong to
        var pieces = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

        int pieceOf( int edge ) {
            while( pieces[edge] != edge ) {
                edge = pieces[edge];
            }
            return edge;
        }

        for( var face = 0; face < faceEdges.GetLength( 0 ); ++face ) {
            var intersected = new List<int>( );

            for( var index = 0; index < faceEdges.GetLength( 1 ); ++index ) {
                if( isIntersected( faceEdges[face, index] ) ) {
                    intersected.Add( faceEdges[face, index] );
                }
            }

            // a single line crosses the face
            if( intersected.Count == 2 ) {
                pieces[pieceOf( intersected[0] )] = pieceOf( intersected[1] );
            }

            // the solid corners of the face lie diagonally across from each other, and two lines cross it. They could
            // cut off either the solid or the empty corners: here they always cut off the solid ones, so that the two
            // cells that share the face agree and solid corners that only meet diagonally stay separate
            if( intersected.Count == 4 ) {
                foreach( var edge in intersected ) {
                    foreach( var other in intersected ) {
                        var corner = findSharedCorner( edge, other );

                        if( edge != other && corner != -1 && isSolid( corner ) ) {
                            pieces[pieceOf( edge )] = pieceOf( other );
                        }
                    }
                }
            }
        }

        var vertices = new List<int[]>( );

        for( var piece = 0; piece < pieces.Length; ++piece ) {
            if( !isIntersected( piece ) || pieceOf( piece ) != piece ) {
                continue;
            }

            var edges = new List<int>( );

            for( var edge = 0; edge < pieces.Length; ++edge ) {
                if( isIntersected( edge ) && pieceOf( edge ) == piece ) {
                    edges.Add( edge );
                }
            }

            vertices.Add( edges.ToArray( ) );
        }

        return vertices.ToArray( );
    }

    private static int[,] buildEdgeFaces( ) {
        var table = new int[edgeCorners.GetLength( 0 ), 2];

        for( var edge = 0; edge < table.GetLength( 0 ); ++edge ) {
            var side = 0;

            for( var face = 0; face < faceLayout.GetLength( 0 ); ++face ) {
                for( var index = 0; index < faceLayout.GetLength( 1 ); ++index ) {
                    if( faceLayout[face, index] == edge ) {
                        table[edge, side++] = face;
                    }
                }
            }
        }

        return table;
    }

    private static int[,,] buildEdgesAcrossFaces( ) {
        var table = new int[256, edgeCorners.GetLength( 0 ), 2];

        for( var solidCorners = 0; solidCorners < table.GetLength( 0 ); ++solidCorners ) {
            for( var edge = 0; edge < table.GetLength( 1 ); ++edge ) {
                for( var side = 0; side < table.GetLength( 2 ); ++side ) {
                    table[solidCorners, edge, side] = findEdgeAcrossFace( solidCorners, edge, edgeFaces[edge, side] );
                }
            }
        }

        return table;
    }

    // the other end of the line that crosses the face from the given edge, by the rule findVertexEdges connects the
    // edges of a face with
    private static int findEdgeAcrossFace( int solidCorners, int edge, int face ) {
        bool isSolid( int corner ) {
            return ( solidCorners & ( 1 << corner ) ) != 0;
        }

        bool isIntersected( int edge ) {
            return isSolid( edgeCorners[edge, 0] ) != isSolid( edgeCorners[edge, 1] );
        }

        if( !isIntersected( edge ) ) {
            return -1;
        }

        var others = new List<int>( );

        for( var index = 0; index < faceLayout.GetLength( 1 ); ++index ) {
            var other = faceLayout[face, index];

            if( other != edge && isIntersected( other ) ) {
                others.Add( other );
            }
        }

        // a single line crosses the face
        if( others.Count == 1 ) {
            return others[0];
        }

        // two lines cross the face, and the one from this edge cuts off the solid corner at its end
        foreach( var other in others ) {
            var corner = findSharedCorner( edge, other );

            if( corner != -1 && isSolid( corner ) ) {
                return other;
            }
        }

        throw new System.Exception( "An intersected edge of a face without another one to connect it to" );
    }

    private static int[] buildJoinedFaces( ) {
        var table = new int[256];

        for( var solidCorners = 0; solidCorners < table.Length; ++solidCorners ) {
            table[solidCorners] = -1;

            for( var face = 0; face < faceLayout.GetLength( 0 ); ++face ) {
                // the vertex that every edge of the face belongs to, if there is one
                var vertex = System.Array.FindIndex( vertexEdges[solidCorners], ( edges ) => System.Array.IndexOf( edges, faceLayout[face, 0] ) != -1 );
                var joined = vertex != -1;

                for( var index = 1; index < faceLayout.GetLength( 1 ) && joined; ++index ) {
                    joined = System.Array.IndexOf( vertexEdges[solidCorners][vertex], faceLayout[face, index] ) != -1;
                }

                if( joined ) {
                    table[solidCorners] = face;
                }
            }
        }

        return table;
    }

    // the pieces of a cell are kept apart by the plane that leaves the widest gap between the corners at the ends of
    // their edges, midway in that gap. Most pairs of pieces touch, at a corner or along a diagonal, and every plane
    // through where they touch that has one on either side leaves no gap: of those it is the plane that faces most
    // squarely from the middle of the corners of the one piece to the middle of those of the other. Every pair has a
    // plane among those whose normals are -1, 0 or 1 along each axis, which are the directions of the planes through
    // any three corners of a cell
    private static int[,,] buildSeparations( ) {
        var table = new int[256, 6, 4];

        for( var solidCorners = 0; solidCorners < table.GetLength( 0 ); ++solidCorners ) {
            var vertices = vertexEdges[solidCorners];

            for( var vertex = 0; vertex < vertices.Length; ++vertex ) {
                for( var other = vertex + 1; other < vertices.Length; ++other ) {
                    var pair = ( vertex * ( 7 - vertex ) / 2 ) + ( other - vertex - 1 );

                    var corners      = findCorners( vertices[vertex] );
                    var otherCorners = findCorners( vertices[other] );

                    var found = false;

                    // the best plane so far: its gap and how far apart the middles of the two pieces are along its
                    // normal, each as its square over the square of the length of the normal
                    var widest   = 0L;
                    var farthest = 0L;
                    var length   = 1L;

                    for( var x = -1; x <= 1; ++x ) {
                        for( var y = -1; y <= 1; ++y ) {
                            for( var z = -1; z <= 1; ++z ) {
                                if( x == 0 && y == 0 && z == 0 ) {
                                    continue;
                                }

                                int along( int corner ) {
                                    return ( x * cornerOffsets[corner, 0] ) + ( y * cornerOffsets[corner, 1] ) + ( z * cornerOffsets[corner, 2] );
                                }

                                var highest = int.MinValue;
                                var lowest  = int.MaxValue;
                                var sum     = 0;
                                var otherSum = 0;

                                foreach( var corner in corners ) {
                                    highest = System.Math.Max( highest, along( corner ) );
                                    sum    += along( corner );
                                }
                                foreach( var corner in otherCorners ) {
                                    lowest    = System.Math.Min( lowest, along( corner ) );
                                    otherSum += along( corner );
                                }

                                if( highest > lowest ) {
                                    continue;
                                }

                                // from the middle of the one to the middle of the other, times the number of corners
                                // of both, which is the same for every plane
                                long apart   = ( otherSum * corners.Count ) - ( sum * otherCorners.Count );
                                long gap     = lowest - highest;
                                long squared = ( x * x ) + ( y * y ) + ( z * z );

                                var wider   = gap * gap * length > widest * squared;
                                var asWide  = gap * gap * length == widest * squared;
                                var farther = apart * apart * length > farthest * squared;

                                if( !found || wider || ( asWide && farther ) ) {
                                    found    = true;
                                    widest   = gap * gap;
                                    farthest = apart * apart;
                                    length   = squared;

                                    table[solidCorners, pair, 0] = x;
                                    table[solidCorners, pair, 1] = y;
                                    table[solidCorners, pair, 2] = z;
                                    table[solidCorners, pair, 3] = highest + lowest;
                                }
                            }
                        }
                    }

                    if( !found ) {
                        throw new System.Exception( "Two pieces of the contour in a cell without a plane between them" );
                    }
                }
            }
        }

        return table;
    }

    // the corners at the ends of the given edges, each once
    private static List<int> findCorners( int[] edges ) {
        var corners = new List<int>( );

        foreach( var edge in edges ) {
            for( var end = 0; end < 2; ++end ) {
                if( !corners.Contains( edgeCorners[edge, end] ) ) {
                    corners.Add( edgeCorners[edge, end] );
                }
            }
        }

        return corners;
    }

    // the corner that two edges have in common, or -1 where they do not meet
    private static int findSharedCorner( int edge, int other ) {
        for( var end = 0; end < 2; ++end ) {
            for( var otherEnd = 0; otherEnd < 2; ++otherEnd ) {
                if( edgeCorners[edge, end] == edgeCorners[other, otherEnd] ) {
                    return edgeCorners[edge, end];
                }
            }
        }
        return -1;
    }

}

using System.Collections.Generic;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Manifold Dual Contouring, section III: a cell holds one vertex for every piece of the contour that passes through it,
// and each edge of the cell that the contour intersects is associated with exactly one of those vertices. The pieces
// are the cycles marching cubes would draw in the cell, which makes the contour the dual of the one marching cubes
// extracts, and a manifold like it. Corners and edges are numbered as in the corner layout of the voxels
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

    // the edges of a cell that each of its vertices is associated with, given the materials at its corners. A cell
    // that the contour does not pass through has no vertices
    public static int[][] lookupVertexEdges( MaterialIndex[] corners ) {
        var solidCorners = 0;

        for( var corner = 0; corner < corners.Length; ++corner ) {
            if( corners[corner] != MaterialIndex.Void ) {
                solidCorners |= 1 << corner;
            }
        }

        return vertexEdges[solidCorners];
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

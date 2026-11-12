using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Manifold Dual Contouring, section III: "each vertex is associated with a set of edges and each edge is associated with
// exactly one vertex", with a vertex for every piece of the contour in a cell
public class DualMarchingCubesTest {

    private const MaterialIndex _ = MaterialIndex.Void;
    private const MaterialIndex A = MaterialIndex.Material0;
    private const MaterialIndex B = MaterialIndex.Material1;

    /*
    corner layout:          edges:
         4--------------7        +------3-------+
        /|             /|       /|             /|
       / |            / |     10 |           11 |
      /  |           /  |     /  6           /  7
     5--------------6   |    +------2-------+   |
     |   |          |   |    |   |          |   |
     |   |          |   |    |   |          |   |
     |   3----------|---2    4   +------1---5---+
     |  /           |  /     |  /           |  /
     | /            | /      | 8            | 9
     |/             |/       |/             |/
     0--------------1        +------0-------+
    */
    private static readonly int[,] edgeCorners = {
        { 0, 1 }, { 3, 2 }, { 5, 6 }, { 4, 7 },
        { 0, 5 }, { 1, 6 }, { 3, 4 }, { 2, 7 },
        { 0, 3 }, { 1, 2 }, { 5, 4 }, { 6, 7 }
    };

    // where each corner is, in units of the cell from its minimum
    private static readonly int[,] cornerOffsets = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 },
        { 0, 1, 1 }, { 0, 1, 0 }, { 1, 1, 0 }, { 1, 1, 1 }
    };

    private const int FaceCount = SurfaceExtractor.Voxel.FaceCount;
    private const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

    private static bool isSolid( int solidCorners, int corner ) {
        return ( solidCorners & ( 1 << corner ) ) != 0;
    }

    private static bool isIntersected( int solidCorners, int edge ) {
        return isSolid( solidCorners, edgeCorners[edge, 0] ) != isSolid( solidCorners, edgeCorners[edge, 1] );
    }

    // the vertex an edge belongs to, numbered as the table lists the vertices, or -1
    private static int vertexOf( int solidCorners, int edge ) {
        return Array.FindIndex( DualMarchingCubes.lookupVertexEdges( solidCorners ), ( edges ) => edges.Contains( edge ) );
    }

    private static int[] edgesOf( int face ) {
        return Enumerable.Range( 0, EdgeCount ).Where(
            ( edge ) => DualMarchingCubes.lookupEdgeFace( edge, 0 ) == face || DualMarchingCubes.lookupEdgeFace( edge, 1 ) == face
        ).ToArray( );
    }

    private static int faceAt( int axis, int side ) {
        return Enumerable.Range( 0, FaceCount ).Single(
            ( face ) => DualMarchingCubes.lookupFaceAxis( face ) == axis && DualMarchingCubes.lookupFaceSide( face ) == side
        );
    }

    // a face is ambiguous when its solid corners lie diagonally across from each other: all four of its edges are
    // intersected
    private static bool isAmbiguous( int solidCorners, int face ) {
        return edgesOf( face ).All( ( edge ) => isIntersected( solidCorners, edge ) );
    }

    // worked out here from the vertices of the cell, not looked up
    private static int[] joinedFacesOf( int solidCorners ) {
        return Enumerable.Range( 0, FaceCount ).Where(
            ( face ) => isAmbiguous( solidCorners, face ) && edgesOf( face ).Select( ( edge ) => vertexOf( solidCorners, edge ) ).Distinct( ).Count( ) == 1
        ).ToArray( );
    }

    // the edges of each vertex in a cell with the given corners, as "0 4 8 | 1 7 9": vertices in order of their first edge
    private static string vertexEdges( params MaterialIndex[] corners ) {
        return string.Join(
            " | ",
            DualMarchingCubes.lookupVertexEdges( corners ).Select(
                ( edges ) => edges.OrderBy( ( edge ) => edge ).ToArray( )
            ).OrderBy(
                ( edges ) => edges[0]
            ).Select(
                ( edges ) => string.Join( " ", edges )
            )
        );
    }

    [Test]
    public void cellWithoutContourHasNoVertices( ) {
        Assert.AreEqual( "", vertexEdges( _, _, _, _, _, _, _, _ ), "empty" );
        Assert.AreEqual( "", vertexEdges( A, A, A, A, A, A, A, A ), "solid" );
        Assert.AreEqual( "", vertexEdges( A, B, A, B, A, B, A, B ), "solid, of two materials" );
    }

    [Test]
    public void cellCutByOneSheetHasOneVertex( ) {
        Assert.AreEqual( "0 4 8",   vertexEdges( A, _, _, _, _, _, _, _ ), "one solid corner" );
        Assert.AreEqual( "4 5 8 9", vertexEdges( A, A, _, _, _, _, _, _ ), "two solid corners along an edge" );
        Assert.AreEqual( "4 5 6 7", vertexEdges( A, A, A, A, _, _, _, _ ), "solid bottom face" );
        Assert.AreEqual( "3 7 11",  vertexEdges( A, A, A, A, A, A, A, _ ), "one empty corner" );
    }

    // solid corners that only meet diagonally are not connected, the contour cuts off each of them by itself
    [Test]
    public void solidCornersAcrossADiagonalHaveAVertexEach( ) {
        Assert.AreEqual( "0 4 8 | 1 7 9",  vertexEdges( A, _, A, _, _, _, _, _ ), "diagonally across a face" );
        Assert.AreEqual( "0 4 8 | 3 7 11", vertexEdges( A, _, _, _, _, _, _, A ), "diagonally across the cell" );

        Assert.AreEqual( "0 4 8 | 1 7 9 | 2 5 11 | 3 6 10", vertexEdges( A, _, A, _, A, _, A, _ ), "four corners that are all diagonal to each other" );
        Assert.AreEqual( "0 5 9 | 1 6 8 | 2 4 10 | 3 7 11", vertexEdges( _, A, _, A, _, A, _, A ), "the other four" );
    }

    // empty corners diagonally across a face are connected, through the gap between the solid corners of that face.
    // Which is the same face seen from the cell on the other side of it, so neighboring cells agree on where the
    // contour crosses the face
    [Test]
    public void emptyCornersAcrossAFaceDiagonalShareAVertex( ) {
        Assert.AreEqual( "0 1 4 7 8 9", vertexEdges( _, A, _, A, A, A, A, A ) );
    }

    // there is no face between corners diagonally across the cell, and nothing that tells whether the contour runs
    // around both or around each. It runs around each, as it does for solid corners
    [Test]
    public void emptyCornersAcrossTheCellHaveAVertexEach( ) {
        Assert.AreEqual( "0 4 8 | 3 7 11", vertexEdges( _, A, A, A, A, A, A, _ ) );
    }

    [Test]
    public void piecesOfTheContourAreToldApartBySolidAndEmptyOnly( ) {
        Assert.AreEqual( "4 5 8 9",       vertexEdges( A, B, _, _, _, _, _, _ ), "two materials along an edge" );
        Assert.AreEqual( "0 4 8 | 1 7 9", vertexEdges( A, _, B, _, _, _, _, _ ), "two materials diagonally across a face" );
    }

    [Test]
    public void everyIntersectedEdgeBelongsToExactlyOneVertex( ) {
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var corners = Enumerable.Range( 0, 8 ).Select( ( corner ) => ( solidCorners & ( 1 << corner ) ) != 0 ? A : _ ).ToArray( );

            var vertices = DualMarchingCubes.lookupVertexEdges( corners );

            for( var edge = 0; edge < edgeCorners.GetLength( 0 ); ++edge ) {
                var intersected = corners[edgeCorners[edge, 0]] != corners[edgeCorners[edge, 1]];

                Assert.AreEqual( intersected ? 1 : 0, vertices.Count( ( edges ) => edges.Contains( edge ) ), $"vertices of edge {edge} with solid corners {solidCorners}" );
            }

            // a piece of the contour is a polygon around at least three edges, and there are twelve edges to go around
            Assert.LessOrEqual( vertices.Length, 4, $"vertices with solid corners {solidCorners}" );

            foreach( var edges in vertices ) {
                Assert.GreaterOrEqual( edges.Length, 3, $"edges of a vertex with solid corners {solidCorners}" );
            }
        }
    }

    // the faces are those of the face layout the octrees use: four edges each, all in one of the six planes that
    // bound the cell
    [Test]
    public void everyEdgeLiesInTheTwoFacesThatHoldBothOfItsCorners( ) {
        var planes = new HashSet<( int axis, int side )>( );

        for( var face = 0; face < FaceCount; ++face ) {
            var axis = DualMarchingCubes.lookupFaceAxis( face );
            var side = DualMarchingCubes.lookupFaceSide( face );

            planes.Add( ( axis, side ) );

            Assert.AreEqual( 4, edgesOf( face ).Length, $"edges of face {face}" );

            foreach( var edge in edgesOf( face ) ) {
                Assert.AreEqual( side, cornerOffsets[edgeCorners[edge, 0], axis], $"start of edge {edge} in face {face}" );
                Assert.AreEqual( side, cornerOffsets[edgeCorners[edge, 1], axis], $"end of edge {edge} in face {face}" );
            }
        }

        Assert.AreEqual( 6, planes.Count, "faces" );

        for( var edge = 0; edge < EdgeCount; ++edge ) {
            Assert.Less( DualMarchingCubes.lookupEdgeFace( edge, 0 ), DualMarchingCubes.lookupEdgeFace( edge, 1 ), $"faces of edge {edge}" );
        }
    }

    // Occupancy-Based Dual Contouring, appendix A.1.1: a piece of the contour crosses a face of the cell from one of
    // its edges to another
    [Test]
    public void theEdgeAcrossAFaceIsAnotherEdgeOfTheSameVertexInThatFace( ) {
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            for( var edge = 0; edge < EdgeCount; ++edge ) {
                for( var side = 0; side < 2; ++side ) {
                    var across = DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, side );
                    var where  = $"across side {side} of edge {edge} with solid corners {solidCorners}";

                    if( !isIntersected( solidCorners, edge ) ) {
                        Assert.AreEqual( -1, across, where );

                        continue;
                    }

                    var face = DualMarchingCubes.lookupEdgeFace( edge, side );

                    Assert.AreNotEqual( edge, across, where );
                    Assert.Contains( across, edgesOf( face ), where );
                    Assert.AreEqual( vertexOf( solidCorners, edge ), vertexOf( solidCorners, across ), $"vertex {where}" );

                    // and back again across the same face
                    var back = DualMarchingCubes.lookupEdgeFace( across, 0 ) == face ? 0 : 1;

                    Assert.AreEqual( edge, DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, across, back ), $"back {where}" );
                }
            }
        }
    }

    // going from edge to edge, in through one face of an edge and out through the other, comes past every edge of the
    // vertex once and ends where it started
    [Test]
    public void theEdgesOfAVertexFormOneRing( ) {
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            foreach( var edges in DualMarchingCubes.lookupVertexEdges( solidCorners ) ) {
                var ring = new List<int>( );
                var edge = edges[0];
                var side = 0;

                do {
                    ring.Add( edge );

                    Assert.LessOrEqual( ring.Count, edges.Length, $"edges in the ring of edge {edges[0]} with solid corners {solidCorners}" );

                    var face = DualMarchingCubes.lookupEdgeFace( edge, side );

                    edge = DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, side );
                    side = DualMarchingCubes.lookupEdgeFace( edge, 0 ) == face ? 1 : 0;
                }
                while( edge != edges[0] );

                CollectionAssert.AreEquivalent( edges, ring, $"ring of edge {edges[0]} with solid corners {solidCorners}" );
            }
        }
    }

    // on an ambiguous face the lines cut off the solid corners, and with the complement of the cell the empty ones
    [Test]
    public void theComplementCutsOffTheEmptyCornersOfAnAmbiguousFace( ) {
        int sharedCorner( int edge, int other ) {
            foreach( var end in new[] { 0, 1 } ) {
                foreach( var otherEnd in new[] { 0, 1 } ) {
                    if( edgeCorners[edge, end] == edgeCorners[other, otherEnd] ) {
                        return edgeCorners[edge, end];
                    }
                }
            }
            return -1;
        }

        var ambiguous = 0;

        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var complement = DualMarchingCubes.complement( solidCorners );

            Assert.AreEqual( 255 - solidCorners, complement, $"complement of {solidCorners}" );

            for( var edge = 0; edge < EdgeCount; ++edge ) {
                Assert.AreEqual( isIntersected( solidCorners, edge ), isIntersected( complement, edge ), $"edge {edge} with solid corners {solidCorners}" );

                for( var side = 0; side < 2; ++side ) {
                    if( !isAmbiguous( solidCorners, DualMarchingCubes.lookupEdgeFace( edge, side ) ) ) {
                        continue;
                    }

                    ++ambiguous;

                    var corner     = sharedCorner( edge, DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, side ) );
                    var otherwise  = sharedCorner( edge, DualMarchingCubes.lookupEdgeAcrossFace( complement,   edge, side ) );

                    Assert.AreNotEqual( -1, corner,    $"corner cut off at edge {edge} with solid corners {solidCorners}" );
                    Assert.AreNotEqual( -1, otherwise, $"corner cut off at edge {edge} with the complement of solid corners {solidCorners}" );

                    Assert.IsTrue(  isSolid( solidCorners, corner ),    $"the corner cut off at edge {edge} with solid corners {solidCorners} is solid" );
                    Assert.IsFalse( isSolid( solidCorners, otherwise ), $"the corner cut off at edge {edge} with the complement of solid corners {solidCorners} is empty" );
                }
            }
        }

        Assert.Greater( ambiguous, 0, "ambiguous faces" );
    }

    [Test]
    public void aJoinedFaceIsAnAmbiguousFaceWhoseEdgesAllBelongToOneVertex( ) {
        var joined = new List<int>( );

        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var faces = joinedFacesOf( solidCorners );

            Assert.LessOrEqual( faces.Length, 1, $"joined faces with solid corners {solidCorners}" );
            Assert.AreEqual( faces.Length == 1 ? faces[0] : -1, DualMarchingCubes.lookupJoinedFace( solidCorners ), $"joined face with solid corners {solidCorners}" );

            if( faces.Length == 1 ) {
                joined.Add( solidCorners );
            }
        }

        int solidCount( int solidCorners ) {
            return Enumerable.Range( 0, 8 ).Count( ( corner ) => isSolid( solidCorners, corner ) );
        }

        // two empty corners diagonally across a face, alone or with the corner behind one of them
        Assert.AreEqual( 36, joined.Count, "cells with a joined face" );
        Assert.AreEqual( 12, joined.Count( ( solidCorners ) => solidCount( solidCorners ) == 6 ), "of those, with six solid corners" );
        Assert.AreEqual( 24, joined.Count( ( solidCorners ) => solidCount( solidCorners ) == 5 ), "of those, with five solid corners" );
    }

    // what makes it safe for a cell to take its vertices from its complement: that has no joined face of its own, and
    // the joined face is the only face on which it connects the edges another way than the cell itself does. Across any
    // other face the cell and its neighbor still agree on which edges belong together
    [Test]
    public void theComplementOfACellWithAJoinedFaceDiffersOnThatFaceOnly( ) {
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var joined = DualMarchingCubes.lookupJoinedFace( solidCorners );

            if( joined == -1 ) {
                continue;
            }

            var complement = DualMarchingCubes.complement( solidCorners );

            Assert.AreEqual( -1, DualMarchingCubes.lookupJoinedFace( complement ), $"joined face of the complement of solid corners {solidCorners}" );

            for( var face = 0; face < FaceCount; ++face ) {
                Assert.AreEqual( face == joined, isAmbiguous( solidCorners, face ), $"face {face} with solid corners {solidCorners} is ambiguous" );
            }

            // on the joined face the complement has a vertex for each of the two lines
            Assert.AreEqual( 2, edgesOf( joined ).Select( ( edge ) => vertexOf( complement, edge ) ).Distinct( ).Count( ), $"vertices on the joined face of the complement of solid corners {solidCorners}" );
        }
    }

    // two cells share a joined face when the joined face of the one and that of the other are the two sides of the
    // same face: at right angles to the same axis, one at the maximum of its cell and one at the minimum
    [Test]
    public void twoCellsShareAJoinedFaceWhenTheirJoinedFacesFaceEachOther( ) {
        var joined  = Enumerable.Range( 0, 256 ).Select( joinedFacesOf ).ToArray( );
        var sharing = 0;

        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            for( var across = 0; across < 256; ++across ) {
                var face  = joined[solidCorners];
                var other = joined[across];

                var expected = face.Length == 1 && other.Length == 1 &&
                               DualMarchingCubes.lookupFaceAxis( face[0] ) == DualMarchingCubes.lookupFaceAxis( other[0] ) &&
                               DualMarchingCubes.lookupFaceSide( face[0] ) != DualMarchingCubes.lookupFaceSide( other[0] );

                Assert.AreEqual( expected, DualMarchingCubes.sharesJoinedFace( solidCorners, across ), $"cells with solid corners {solidCorners} and {across}" );

                if( expected ) {
                    ++sharing;
                }
            }
        }

        // six of the 36 cells have their joined face on each side
        Assert.AreEqual( 6 * 6 * 6, sharing, "pairs of cells that share a joined face" );
    }

    // the vertices to take for each of two cells that share a face along the given axis, given whether each takes its
    // complement, and how many polygons then run along the edge between each pair of their vertices: one polygon for
    // each intersected edge of the face, from the vertex of that edge in the one cell to its vertex in the other
    private static IEnumerable<int> polygonsAlongEdgesBetween( int axis, int first, int second ) {
        var across = faceAt( axis, 1 );
        var other  = faceAt( axis, 0 );

        var counts = new Dictionary<( int, int ), int>( );

        foreach( var edge in edgesOf( across ) ) {
            if( !isIntersected( first, edge ) ) {
                continue;
            }

            // the same edge of the grid, as the second cell numbers it: its corners are the same but for the axis
            var same = edgesOf( other ).Single(
                ( candidate ) => Enumerable.Range( 0, 2 ).All(
                    ( end ) => Enumerable.Range( 0, 3 ).All(
                        ( coordinate ) => coordinate == axis || cornerOffsets[edgeCorners[candidate, end], coordinate] == cornerOffsets[edgeCorners[edge, end], coordinate]
                    )
                )
            );

            var pair = ( vertexOf( first, edge ), vertexOf( second, same ) );

            counts[pair] = counts.GetValueOrDefault( pair ) + 1;
        }

        return counts.Values;
    }

    // every pair of cells that share a face: the twelve corners of the two, solid or empty in every way
    private static IEnumerable<( int axis, int first, int second )> pairsOfCells( ) {
        for( var axis = 0; axis < 3; ++axis ) {
            for( var corners = 0; corners < 4096; ++corners ) {
                // the corners in three layers of four along the axis: the first cell has the lower two layers, the
                // second the upper two
                int cellAt( int layer ) {
                    var solidCorners = 0;

                    for( var corner = 0; corner < 8; ++corner ) {
                        var index = ( ( layer + cornerOffsets[corner, axis] ) * 4 ) + ( cornerOffsets[corner, ( axis + 1 ) % 3] * 2 ) + cornerOffsets[corner, ( axis + 2 ) % 3];

                        if( ( corners & ( 1 << index ) ) != 0 ) {
                            solidCorners |= 1 << corner;
                        }
                    }

                    return solidCorners;
                }

                yield return ( axis, cellAt( 0 ), cellAt( 1 ) );
            }
        }
    }

    // Occupancy-Based Dual Contouring, appendix A.1.1: the table by itself is not manifold everywhere
    [Test]
    public void twoCellsWithTheirJoinedFaceInCommonShareAnEdgeOfFourPolygons( ) {
        var edgesOfFour = 0;

        foreach( var ( axis, first, second ) in pairsOfCells( ) ) {
            var joinedInBoth = DualMarchingCubes.lookupJoinedFace( first ) == faceAt( axis, 1 ) && DualMarchingCubes.lookupJoinedFace( second ) == faceAt( axis, 0 );

            foreach( var polygons in polygonsAlongEdgesBetween( axis, first, second ) ) {
                Assert.AreEqual( joinedInBoth ? 4 : 2, polygons, $"polygons along an edge between cells with solid corners {first} and {second} along axis {axis}" );

                if( polygons == 4 ) {
                    ++edgesOfFour;
                }
            }
        }

        Assert.Greater( edgesOfFour, 0, "edges that four polygons run along" );
    }

    // with the rule every edge between the vertices of two cells is part of two polygons, whatever either cell takes
    // because of the neighbor across its joined face when that is another face than the one the two share
    [Test]
    public void withTheComplementRuleEveryEdgeBetweenTwoCellsIsPartOfTwoPolygons( ) {
        // whether a cell takes its complement, as far as the pair tells: it does when its joined face is the shared one
        // and the other cell has it joined too, and it may or may not when its joined face is another one
        bool[] choices( int solidCorners, int face, int other, int otherFace ) {
            var joined = DualMarchingCubes.lookupJoinedFace( solidCorners );

            if( joined == -1 ) {
                return new[] { false };
            }
            if( joined == face ) {
                return new[] { DualMarchingCubes.lookupJoinedFace( other ) == otherFace };
            }
            return new[] { false, true };
        }

        foreach( var ( axis, first, second ) in pairsOfCells( ) ) {
            foreach( var firstTakesComplement in choices( first, faceAt( axis, 1 ), second, faceAt( axis, 0 ) ) ) {
                foreach( var secondTakesComplement in choices( second, faceAt( axis, 0 ), first, faceAt( axis, 1 ) ) ) {
                    var polygons = polygonsAlongEdgesBetween(
                        axis,
                        firstTakesComplement  ? DualMarchingCubes.complement( first )  : first,
                        secondTakesComplement ? DualMarchingCubes.complement( second ) : second
                    );

                    foreach( var count in polygons ) {
                        Assert.AreEqual( 2, count, $"polygons along an edge between cells with solid corners {first} and {second} along axis {axis}, complements {firstTakesComplement} and {secondTakesComplement}" );
                    }
                }
            }
        }
    }

    // the contour of a whole grid whose corners are solid at random, which is full of ambiguous faces: a polygon
    // around every intersected edge, through the vertex of that edge in each of the four cells around it. The corners
    // on the outside of the grid are empty, so the contour is closed. Returns how many edges between two vertices are
    // not part of exactly two polygons, how many vertices have polygons around them that form more than one fan, and
    // how many cells took their complement
    private static ( int edges, int pinched, int complements ) contourRandomGrid( int seed, float solid, bool useComplements ) {
        const int cells  = 6;
        const int points = cells + 1;

        var random = new Random( seed );
        var filled = new bool[points, points, points];

        for( var x = 1; x < cells; ++x ) {
            for( var y = 1; y < cells; ++y ) {
                for( var z = 1; z < cells; ++z ) {
                    filled[x, y, z] = random.NextDouble( ) < solid;
                }
            }
        }

        bool inGrid( int[] cell ) {
            return cell.All( ( coordinate ) => coordinate >= 0 && coordinate < cells );
        }

        int solidCornersOf( int[] cell ) {
            var solidCorners = 0;

            for( var corner = 0; corner < 8; ++corner ) {
                if( filled[cell[0] + cornerOffsets[corner, 0], cell[1] + cornerOffsets[corner, 1], cell[2] + cornerOffsets[corner, 2]] ) {
                    solidCorners |= 1 << corner;
                }
            }

            return solidCorners;
        }

        // what each cell looks its vertices up with: its solid corners, or their complement when the cell across its
        // joined face has that face joined too
        var lookups     = new int[cells, cells, cells];
        var complements = 0;

        for( var x = 0; x < cells; ++x ) {
            for( var y = 0; y < cells; ++y ) {
                for( var z = 0; z < cells; ++z ) {
                    var solidCorners = solidCornersOf( new[] { x, y, z } );
                    var joined       = DualMarchingCubes.lookupJoinedFace( solidCorners );

                    lookups[x, y, z] = solidCorners;

                    if( !useComplements || joined == -1 ) {
                        continue;
                    }

                    var axis = DualMarchingCubes.lookupFaceAxis( joined );
                    var side = DualMarchingCubes.lookupFaceSide( joined );

                    var neighbor = new[] { x, y, z };

                    neighbor[axis] += side == 1 ? 1 : -1;

                    if( inGrid( neighbor ) && DualMarchingCubes.sharesJoinedFace( solidCorners, solidCornersOf( neighbor ) ) ) {
                        lookups[x, y, z] = DualMarchingCubes.complement( solidCorners );

                        ++complements;
                    }
                }
            }
        }

        // a vertex as a number: its cell and which of the cell's vertices it is
        int vertexAt( int[] cell, int[] start, int axis ) {
            // the edge of the cell that starts at this corner of the grid and runs along the axis
            var edge = Enumerable.Range( axis * 4, 4 ).Single(
                ( candidate ) => Enumerable.Range( 0, 3 ).All( ( coordinate ) => cell[coordinate] + cornerOffsets[edgeCorners[candidate, 0], coordinate] == start[coordinate] )
            );

            var vertex = vertexOf( lookups[cell[0], cell[1], cell[2]], edge );

            Assert.AreNotEqual( -1, vertex, "an intersected edge without a vertex" );

            return ( ( ( ( ( cell[0] * cells ) + cell[1] ) * cells ) + cell[2] ) * 4 ) + vertex;
        }

        var polygons = new List<int[]>( );

        for( var x = 0; x < points; ++x ) {
            for( var y = 0; y < points; ++y ) {
                for( var z = 0; z < points; ++z ) {
                    for( var axis = 0; axis < 3; ++axis ) {
                        var start = new[] { x, y, z };
                        var end   = new[] { x, y, z };

                        end[axis] += 1;

                        if( end[axis] >= points || filled[x, y, z] == filled[end[0], end[1], end[2]] ) {
                            continue;
                        }

                        // the four cells around the edge, going around it
                        var polygon = new List<int>( );

                        foreach( var ( across, other ) in new[] { ( -1, -1 ), ( 0, -1 ), ( 0, 0 ), ( -1, 0 ) } ) {
                            var cell = new[] { x, y, z };

                            cell[( axis + 1 ) % 3] += across;
                            cell[( axis + 2 ) % 3] += other;

                            polygon.Add( vertexAt( cell, start, axis ) );
                        }

                        polygons.Add( polygon.ToArray( ) );
                    }
                }
            }
        }

        // the polygons along each edge between two vertices, and around each vertex
        var along  = new Dictionary<( int, int ), List<int>>( );
        var around = new Dictionary<int, List<int>>( );

        for( var index = 0; index < polygons.Count; ++index ) {
            var polygon = polygons[index];

            Assert.AreEqual( 4, polygon.Distinct( ).Count( ), "vertices of a polygon" );

            for( var corner = 0; corner < 4; ++corner ) {
                var vertex = polygon[corner];
                var next   = polygon[( corner + 1 ) % 4];
                var edge   = ( Math.Min( vertex, next ), Math.Max( vertex, next ) );

                if( !along.ContainsKey( edge ) ) {
                    along[edge] = new( );
                }
                if( !around.ContainsKey( vertex ) ) {
                    around[vertex] = new( );
                }

                along[edge].Add( index );
                around[vertex].Add( index );
            }
        }

        // the polygons around a vertex form one fan when each can be reached from any other through the edges of the
        // vertex that two of them share
        var pinched = 0;

        foreach( var ( vertex, fan ) in around ) {
            var reached = new HashSet<int> { fan[0] };
            var pending = new Stack<int>( reached );

            while( pending.Count > 0 ) {
                var polygon = pending.Pop( );

                foreach( var ( edge, sharing ) in along ) {
                    if( ( edge.Item1 == vertex || edge.Item2 == vertex ) && sharing.Contains( polygon ) ) {
                        foreach( var other in sharing ) {
                            if( reached.Add( other ) ) {
                                pending.Push( other );
                            }
                        }
                    }
                }
            }

            if( reached.Count != fan.Count ) {
                ++pinched;
            }
        }

        return ( along.Values.Count( ( sharing ) => sharing.Count != 2 ), pinched, complements );
    }

    [Test]
    public void aGridOfRandomCornersIsManifoldWithTheComplementRule( [Range( 0, 19 )] int seed ) {
        var ( edges, pinched, _ ) = contourRandomGrid( seed, seed % 2 == 0 ? 0.7f : 0.5f, true );

        Assert.AreEqual( 0, edges,   "edges that are not part of exactly two polygons" );
        Assert.AreEqual( 0, pinched, "vertices whose polygons form more than one fan" );
    }

    // the grids above do hold the case the rule is for
    [Test]
    public void gridsOfRandomCornersAreNotManifoldWithoutTheComplementRule( ) {
        var edges       = 0;
        var complements = 0;

        for( var seed = 0; seed < 20; ++seed ) {
            edges       += contourRandomGrid( seed, seed % 2 == 0 ? 0.7f : 0.5f, false ).edges;
            complements += contourRandomGrid( seed, seed % 2 == 0 ? 0.7f : 0.5f, true ).complements;
        }

        Assert.Greater( edges,       0, "edges that are not part of exactly two polygons, without the rule" );
        Assert.Greater( complements, 0, "cells that took their complement, with the rule" );
    }

}

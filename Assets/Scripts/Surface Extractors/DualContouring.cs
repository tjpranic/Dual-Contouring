using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DualContouring : SurfaceExtractor {

    // positive directions for determining cell adjacencies
    private int[,] directions = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

    /*
    table of edge indices for given positive x, y, z cell adjacencies

    0, 0 -> unused
    0, 1 -> unused
    1, 0 -> ( 0, 1, 0 ), ( 1, 0, 0 ), ( 1, 1, 0 )
    1, 1 -> unused
    2, 0 -> ( 0, 0, 1 ), ( 1, 0, 0 ), ( 1, 0, 1 )
    2, 1 -> ( 0, 0, 1 ), ( 0, 1, 0 ), ( 0, 1, 1 )
    */
    private int[,] edgeIndices = new int[3, 2] { { -1, -1 }, { 11, -1 }, {  7,  3 } };

    private Cell[,,] grid;

    public ( Vector3[] positions, Vector3[] normals, int[] indices ) voxelize( DensityFunction densityFunction, int resolution ) {
        var gridSize = ( int )Mathf.Pow( 2, resolution - 1 );

        this.grid = new Cell[gridSize, gridSize, gridSize];

        for( var x = 0; x < gridSize; ++x ) {
            for( var y = 0; y < gridSize; ++y ) {
                for( var z = 0; z < gridSize; ++z ) {
                    var cell = new Cell( new Vector3( x, y, z ) / gridSize, Vector3.one / gridSize );

                    // center relative to 0, 0, 0
                    cell.center -= cell.extents * ( gridSize - 1 );

                    this.grid[x, y, z] = cell;
                }
            }
        }

        var index = 0;
        foreach( var cell in this.grid ) {
            foreach( var corner in cell.corners ) {
                corner.density = densityFunction.sample( corner.position, Vector3.zero, Vector3.one * 0.5f );
            }

            if(
                cell.corners.All( ( corner ) => corner.sign == Cell.Sign.Inside  ) ||
                cell.corners.All( ( corner ) => corner.sign == Cell.Sign.Outside )
            ) {
                // cell is either fully inside or outside the volume, skip
                continue;
            }

            var planes = new List<Plane>( );

            foreach( var edge in cell.edges ) {
                if(
                    ( edge.corners.Item1.sign == Cell.Sign.Inside  && edge.corners.Item2.sign == Cell.Sign.Inside  ) ||
                    ( edge.corners.Item1.sign == Cell.Sign.Outside && edge.corners.Item2.sign == Cell.Sign.Outside )
                ) {
                    // no sign change detected on edge, skip
                    continue;
                }

                edge.intersection = this.approximateIntersection ( edge.corners,      densityFunction );
                edge.normal       = this.calculateEdgeNormal     ( edge.intersection, densityFunction );

                planes.Add( new Plane( edge.normal, edge.intersection ) );
            }

            // calculate minimizing vertex
            // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
            // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
            var minimizingVertex = cell.center;

            var iterations = 8;
            for( var i = 0; i < iterations; i++ ) {
                minimizingVertex -= planes.Aggregate(
                    Vector3.zero,
                    ( accumulator, plane ) => {
                        accumulator += plane.GetDistanceToPoint( minimizingVertex ) * plane.normal;
                        return accumulator;
                    }
                ) / planes.Count;
            }

            cell.vertex = minimizingVertex;

            // calculate surface normal
            cell.normal = Vector3.Normalize(
                planes.Aggregate(
                    Vector3.zero,
                    ( accumulator, plane ) => {
                        accumulator += plane.normal;
                        return accumulator;
                    }
                ) / planes.Count
            );

            cell.index = index++;
        }

        var positions = new List<Vector3>( );
        var normals   = new List<Vector3>( );
        var indices   = new List<int>( );

        for( var x = 0; x < gridSize; ++x ) {
            for( var y = 0; y < gridSize; ++y ) {
                for( var z = 0; z < gridSize; ++z ) {
                    var cell = this.grid[x, y, z];

                    if( cell.index == -1 ) {
                        continue;
                    }

                    positions.Add ( cell.vertex );
                    normals.Add   ( cell.normal );

                    /*
                    for( var edgeIndex = 0; edgeIndex < 12; ++edgeIndex ) {
                        var edge = cell.edges[edgeIndex];
                        if(
                            ( edge.corners.Item1.sign == Cell.Sign.Inside  && edge.corners.Item2.sign == Cell.Sign.Inside  ) ||
                            ( edge.corners.Item1.sign == Cell.Sign.Outside && edge.corners.Item2.sign == Cell.Sign.Outside )
                        ) {
                            continue;
                        }

                        var cellIndex0 = new Vector3Int( x, y, z );
                        var cellIndex1 = cellIndex0 + this.cellAdjacencies[edgeIndex, 0];
                        var cellIndex2 = cellIndex0 + this.cellAdjacencies[edgeIndex, 1];
                        var cellIndex3 = cellIndex0 + this.cellAdjacencies[edgeIndex, 2];

                        var cells = new Cell[4] {
                            this.grid[cellIndex0.x, cellIndex0.y, cellIndex0.z],
                            this.grid[cellIndex1.x, cellIndex1.y, cellIndex1.z],
                            this.grid[cellIndex2.x, cellIndex2.y, cellIndex2.z],
                            this.grid[cellIndex3.x, cellIndex3.y, cellIndex3.z]
                        };

                        indices.Add( cells[0].index );
                        indices.Add( cells[1].index );
                        indices.Add( cells[2].index );

                        indices.Add( cells[2].index );
                        indices.Add( cells[3].index );
                        indices.Add( cells[0].index );
                    }
                    */

                    var cells = new Cell[4];

                    cells[0] = cell;

                    // enumerate all valid 2x2x1 cube permutations along positive x, y, z directions
                    for( var directionIndex1 = 0; directionIndex1 < 3; ++directionIndex1 ) {
                        for( var directionIndex2 = 0; directionIndex2 < directionIndex1; ++directionIndex2 ) {
                            var cellIndex1 = new Vector3Int(
                                x + this.directions[directionIndex1, 0],
                                y + this.directions[directionIndex1, 1],
                                z + this.directions[directionIndex1, 2]
                            );
                            if(
                                cellIndex1.x >= this.grid.GetLength( 0 ) ||
                                cellIndex1.y >= this.grid.GetLength( 1 ) ||
                                cellIndex1.z >= this.grid.GetLength( 2 )
                            ) {
                                continue;
                            }

                            var cellIndex2 = new Vector3Int(
                                x + this.directions[directionIndex2, 0],
                                y + this.directions[directionIndex2, 1],
                                z + this.directions[directionIndex2, 2]
                            );
                            if(
                                cellIndex2.x >= this.grid.GetLength( 0 ) ||
                                cellIndex2.y >= this.grid.GetLength( 1 ) ||
                                cellIndex2.z >= this.grid.GetLength( 2 )
                            ) {
                                continue;
                            }

                            var cellIndex3 = new Vector3Int(
                                x + this.directions[directionIndex1, 0] + this.directions[directionIndex2, 0],
                                y + this.directions[directionIndex1, 1] + this.directions[directionIndex2, 1],
                                z + this.directions[directionIndex1, 2] + this.directions[directionIndex2, 2]
                            );
                            if(
                                cellIndex3.x >= this.grid.GetLength( 0 ) ||
                                cellIndex3.y >= this.grid.GetLength( 1 ) ||
                                cellIndex3.z >= this.grid.GetLength( 2 )
                            ) {
                                continue;
                            }

                            cells[1] = this.grid[cellIndex1.x, cellIndex1.y, cellIndex1.z];
                            cells[2] = this.grid[cellIndex2.x, cellIndex2.y, cellIndex2.z];
                            cells[3] = this.grid[cellIndex3.x, cellIndex3.y, cellIndex3.z];

                            if( cells[1].index == -1 || cells[2].index == -1 || cells[3].index == -1 ) {
                                continue;
                            }

                            // lookup common edge shared by 2x2x1 cube grid and determine triangle winding order
                            var edgeIndex = this.edgeIndices[directionIndex1, directionIndex2];
                            var edge      = cells[0].edges[edgeIndex];
                            var flip      = edge.corners.Item1.sign == Cell.Sign.Inside;

                            if( !flip ) {
                                indices.Add( cells[0].index );
                                indices.Add( cells[1].index );
                                indices.Add( cells[2].index );

                                indices.Add( cells[3].index );
                                indices.Add( cells[2].index );
                                indices.Add( cells[1].index );
                            }
                            else {
                                indices.Add( cells[1].index );
                                indices.Add( cells[2].index );
                                indices.Add( cells[3].index );

                                indices.Add( cells[2].index );
                                indices.Add( cells[1].index );
                                indices.Add( cells[0].index );
                            }
                        }
                    }
                }
            }
        }

        // TODO: fix redundant indices caused by duplicate edges

        return ( positions.ToArray( ), normals.ToArray( ), indices.ToArray( ) );
    }

    /*
    // table of cell index offsets for every edge in a cell
    // offsets give the indices of 3 other cells surrounding an edge
    private Vector3Int[,] cellAdjacencies = new Vector3Int[12, 3] {
        // x axis aligned edges
        { new Vector3Int(  0, -1, 0 ), new Vector3Int(  0, -1, -1 ), new Vector3Int( 0,  0, -1 ) },
        { new Vector3Int(  0, -1, 0 ), new Vector3Int(  0, -1,  1 ), new Vector3Int( 0,  0,  1 ) },
        { new Vector3Int(  0,  1, 0 ), new Vector3Int(  0,  1, -1 ), new Vector3Int( 0,  0, -1 ) },
        { new Vector3Int(  0,  1, 0 ), new Vector3Int(  0,  1,  1 ), new Vector3Int( 0,  0,  1 ) },
        // y axis aligned edges
        { new Vector3Int( -1,  0, 0 ), new Vector3Int( -1,  0, -1 ), new Vector3Int( 0,  0, -1 ) },
        { new Vector3Int(  1,  0, 0 ), new Vector3Int(  1,  0, -1 ), new Vector3Int( 0,  0, -1 ) },
        { new Vector3Int( -1,  0, 0 ), new Vector3Int( -1,  0,  1 ), new Vector3Int( 0,  0,  1 ) },
        { new Vector3Int(  1,  0, 0 ), new Vector3Int(  1,  0,  1 ), new Vector3Int( 0,  0,  1 ) },
        // z axis aligned edges
        { new Vector3Int( -1,  0, 0 ), new Vector3Int( -1, -1,  0 ), new Vector3Int( 0, -1,  0 ) },
        { new Vector3Int(  1,  0, 0 ), new Vector3Int(  1, -1,  0 ), new Vector3Int( 0, -1,  0 ) },
        { new Vector3Int( -1,  0, 0 ), new Vector3Int( -1,  1,  0 ), new Vector3Int( 0,  1,  0 ) },
        { new Vector3Int(  1,  0, 0 ), new Vector3Int(  1,  1,  0 ), new Vector3Int( 0,  1,  0 ) }
    };
    */

    public List<Cell> getGridCells( ) {
        return this.grid.Flatten( ).ToList( );
    }

    private Vector3 approximateIntersection( ( Cell.Corner, Cell.Corner ) corners, DensityFunction densityFunction ) {
        /*
        // iteratively determine intersection
        var steps     = 8;
        var increment = 1.0f / steps;
        var minimum   = float.MaxValue;

        for( var distance = 0.0f; distance <= 1.0f; distance += increment ) {
            var point   = corners.Item1.position + ( ( corners.Item2.position - corners.Item1.position ) * distance );
            var density = densityFunction.sample( point, Vector3.zero, Vector3.one );
            if( density < minimum ) {
                minimum = density;
            }
        }

        return corners.Item1.position + ( ( corners.Item2.position - corners.Item1.position ) * minimum );
        */

        // return Vector3.Lerp( corners.Item1.position, corners.Item2.position, corners.Item2.density - corners.Item1.density );

        // linear interpolation
        return corners.Item1.position + ( ( -corners.Item1.density ) * ( corners.Item2.position - corners.Item1.position ) / ( corners.Item2.density - corners.Item1.density ) );
    }

    private Vector3 calculateEdgeNormal( Vector3 point, DensityFunction densityFunction ) {
        var step = 0.1f;

        return Vector3.Normalize(
            new Vector3(
                densityFunction.sample( point + new Vector3( step, 0.0f, 0.0f ), Vector3.zero, Vector3.one * 0.5f ) - densityFunction.sample( point - new Vector3( step, 0.0f, 0.0f ), Vector3.zero, Vector3.one * 0.5f ),
                densityFunction.sample( point + new Vector3( 0.0f, step, 0.0f ), Vector3.zero, Vector3.one * 0.5f ) - densityFunction.sample( point - new Vector3( 0.0f, step, 0.0f ), Vector3.zero, Vector3.one * 0.5f ),
                densityFunction.sample( point + new Vector3( 0.0f, 0.0f, step ), Vector3.zero, Vector3.one * 0.5f ) - densityFunction.sample( point - new Vector3( 0.0f, 0.0f, step ), Vector3.zero, Vector3.one * 0.5f )
            )
        );
    }

    /*
    private readonly ( ( int x, int y, int z ), ( int x, int y, int z ) )[] cellFacePairAdjacencies = new ( ( int x, int y, int z ), ( int x, int y, int z ) )[] {
        ( ( 0, 0, 0 ), ( 1, 0, 0 ) ),
        //...
    };

    private void contourCell( ( int x, int y, int z ) index, List<int> indices ) {

        // for every adjacent cell pair
        for( var i = 0; i < 12; ++i ) {
            var adjacencies = this.cellFacePairAdjacencies[i];

            var cells = (
                this.grid[index.x + adjacencies.Item1.x, index.y + adjacencies.Item1.y, index.z + adjacencies.Item1.z],
                this.grid[index.x + adjacencies.Item2.x, index.y + adjacencies.Item2.y, index.z + adjacencies.Item2.z]
            );

            this.contourFace( cells, indices );
        }

        // 
        for( var i = 0; i < 6; ++i ) {
            var adjacencies = this.cellEdgePairAdjacencies[i];

            var cells = (
                this.grid[index.x + adjacencies.Item1.x, index.y + adjacencies.Item1.y, index.z + adjacencies.Item1.z],
                this.grid[index.x + adjacencies.Item2.x, index.y + adjacencies.Item2.y, index.z + adjacencies.Item2.z],
                this.grid[index.x + adjacencies.Item3.x, index.y + adjacencies.Item3.y, index.z + adjacencies.Item3.z],
                this.grid[index.x + adjacencies.Item4.x, index.y + adjacencies.Item4.y, index.z + adjacencies.Item4.z]
            );

            this.contourEdge( cells, indices );
        }

    }

    private void contourFace( ( Cell, Cell ) cells, List<int> indices ) {

    }

    private void contourEdge( ( Cell, Cell, Cell, Cell ) cells, List<int> indices ) {

    }
    */

}

public static class LINQExtensions {

    public static IEnumerable<T> Flatten<T>( this T[,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                yield return array[x, y];
            }
        }
    }

    public static IEnumerable<T> Flatten<T>( this T[,,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                for( var z = 0; z < array.GetLength( 2 ); z++ ) {
                    yield return array[x, y, z];
                }
            }
        }
    }

}
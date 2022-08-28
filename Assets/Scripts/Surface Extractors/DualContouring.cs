using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// dual contouring implementation using uniform grid

public class DualContouring : SurfaceExtractor {

    private const int MinimizerIterations = 12;

    // used to enumerate all positive x, y, z 2x2x1 cube grids containing a common edge
    private readonly Cell.Adjacency[] adjacencies = {
        new( new Vector3Int[] { new( 0, 0, 1 ), new( 0, 1, 0 ), new( 0, 1, 1 ) },  3 ),
        new( new Vector3Int[] { new( 0, 0, 1 ), new( 1, 0, 0 ), new( 1, 0, 1 ) },  7 ),
        new( new Vector3Int[] { new( 0, 1, 0 ), new( 1, 0, 0 ), new( 1, 1, 0 ) }, 11 )
    };

    private Cell[,,] grid;

    private readonly Vector3 origin = Vector3.zero;
    private readonly Vector3 scale  = Vector3.one / 2;

    public ( Vector3[] positions, Vector3[] normals, int[] indices ) voxelize( DensityFunction densityFunction, int resolution ) {
        Debug.Assert( MathUtilities.isPowerOfTwo( resolution ), "resolution must be a power of 2" );

        var gridSize = ( int )Mathf.Pow( 2, resolution - 1 );

        this.grid = new Cell[gridSize, gridSize, gridSize];

        // build uniformly subdivided grid

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

        // find minimizing vertices and surface normals in each cell with at least one edge exhibiting a sign change

        var index = 0;
        foreach( var cell in this.grid ) {
            foreach( var corner in cell.corners ) {
                corner.density = densityFunction.sample( corner.position, this.origin, this.scale );
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
                    ( edge.corners[0].sign == Cell.Sign.Inside  && edge.corners[1].sign == Cell.Sign.Inside  ) ||
                    ( edge.corners[0].sign == Cell.Sign.Outside && edge.corners[1].sign == Cell.Sign.Outside )
                ) {
                    // no sign change detected on edge, skip
                    continue;
                }

                edge.intersection = this.approximateIntersection ( edge );
                edge.normal       = this.calculateEdgeNormal     ( edge, densityFunction );

                planes.Add( new( edge.normal, edge.intersection ) );
            }

            // calculate minimizing vertex
            // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
            // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
            var minimizingVertex = cell.center;

            for( var i = 0; i < MinimizerIterations; i++ ) {
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

        // contour and generate indices

        for( var x = 0; x < gridSize; ++x ) {
            for( var y = 0; y < gridSize; ++y ) {
                for( var z = 0; z < gridSize; ++z ) {
                    var cell = this.grid[x, y, z];

                    if( cell.index == -1 ) {
                        continue;
                    }

                    positions.Add ( cell.vertex );
                    normals.Add   ( cell.normal );

                    var cells = new Cell[4];

                    cells[0] = cell;

                    var current = new Vector3Int( x, y, z );

                    // for all valid 2x2x1 cube permutations along positive x, y, z directions
                    foreach( var adjacency in this.adjacencies ) {
                        var cellIndex1 = current + adjacency.offsets[0];
                        if(
                            cellIndex1.x >= this.grid.GetLength( 0 ) ||
                            cellIndex1.y >= this.grid.GetLength( 1 ) ||
                            cellIndex1.z >= this.grid.GetLength( 2 )
                        ) {
                            continue;
                        }

                        var cellIndex2 = current + adjacency.offsets[1];
                        if(
                            cellIndex2.x >= this.grid.GetLength( 0 ) ||
                            cellIndex2.y >= this.grid.GetLength( 1 ) ||
                            cellIndex2.z >= this.grid.GetLength( 2 )
                        ) {
                            continue;
                        }

                        var cellIndex3 = current + adjacency.offsets[2];
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

                        // use common edge contained by 2x2x1 cube grid to determine triangle winding order
                        var flip = cells[0].edges[adjacency.edgeIndex].corners[0].sign == Cell.Sign.Inside;

                        if( !flip ) {
                            indices.Add( cells[0].index );
                            indices.Add( cells[1].index );
                            indices.Add( cells[2].index );

                            indices.Add( cells[2].index );
                            indices.Add( cells[1].index );
                            indices.Add( cells[3].index );
                        }
                        else {
                            indices.Add( cells[3].index );
                            indices.Add( cells[1].index );
                            indices.Add( cells[2].index );

                            indices.Add( cells[2].index );
                            indices.Add( cells[1].index );
                            indices.Add( cells[0].index );
                        }
                    }
                }
            }
        }

        return ( positions.ToArray( ), normals.ToArray( ), indices.ToArray( ) );
    }

    public List<Cell> getGridCells( ) {
        return this.grid.Flatten( ).ToList( );
    }

    private Vector3 approximateIntersection( Cell.Edge edge ) {
        // linear interpolation
        return edge.corners[0].position + ( ( -edge.corners[0].density ) * ( edge.corners[1].position - edge.corners[0].position ) / ( edge.corners[1].density - edge.corners[0].density ) );
    }

    private Vector3 calculateEdgeNormal( Cell.Edge edge, DensityFunction densityFunction ) {
        var step = 0.1f;

        // sample surrounding x, y, z locations and take the difference
        return Vector3.Normalize(
            new Vector3(
                densityFunction.sample( edge.intersection + new Vector3( step, 0.0f, 0.0f ), this.origin, this.scale ) - densityFunction.sample( edge.intersection - new Vector3( step, 0.0f, 0.0f ), this.origin, this.scale ),
                densityFunction.sample( edge.intersection + new Vector3( 0.0f, step, 0.0f ), this.origin, this.scale ) - densityFunction.sample( edge.intersection - new Vector3( 0.0f, step, 0.0f ), this.origin, this.scale ),
                densityFunction.sample( edge.intersection + new Vector3( 0.0f, 0.0f, step ), this.origin, this.scale ) - densityFunction.sample( edge.intersection - new Vector3( 0.0f, 0.0f, step ), this.origin, this.scale )
            )
        );
    }

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

public static class MathUtilities {

    public static bool isPowerOfTwo( int number ) {
        return ( number != 0 ) && ( ( number & ( number - 1 ) ) == 0 );
    }

}
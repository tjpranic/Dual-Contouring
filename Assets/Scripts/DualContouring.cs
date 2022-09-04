using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// dual contouring implementation using uniform grid

public class DualContouring : Voxelizer {

    private class Corner : SurfaceExtractor.Corner {

        public readonly Vector3 position;

        private float _density;
        public float density {
            get { return this._density; }
            set {
                this._density = value;
                this.sign     = value < 0.0f ? SurfaceExtractor.Corner.Sign.Inside : SurfaceExtractor.Corner.Sign.Outside;
            }
        }

        public SurfaceExtractor.Corner.Sign sign { get; private set; } = SurfaceExtractor.Corner.Sign.None;

        public Corner( Vector3 position ) {
            this.position = position;
        }

        public Vector3 getPosition( ) {
            return this.position;
        }

        public SurfaceExtractor.Corner.Sign getSign( ) {
            return this.sign;
        }

        public bool Equals( SurfaceExtractor.Corner other ) {
            return this.position.Equals( other.getPosition( ) );
        }

        public override int GetHashCode( ) {
            return this.position.GetHashCode( );
        }

    }

    private class Edge : SurfaceExtractor.Edge {

        public readonly Corner[] corners;

        public Vector3 intersection;
        public Vector3 normal;

        public Edge( Corner[] corners ) {
            this.corners = corners;
        }

        public SurfaceExtractor.Corner[] getCorners( ) {
            return this.corners;
        }

        public bool Equals( SurfaceExtractor.Edge other ) {
            return this.corners[0].Equals( other.getCorners( )[0] ) && this.corners[1].Equals( other.getCorners( )[1] );
        }

        public override int GetHashCode( ) {
            return this.corners[0].GetHashCode( ) ^ this.corners[1].GetHashCode( );
        }

    }

    private class Voxel : SurfaceExtractor.Voxel {

        public readonly Vector3  center;
        public readonly Vector3  size;
        public readonly Vector3  extents;
        public readonly Vector3  minimum;
        public readonly Vector3  maximum;
        public readonly Corner[] corners;
        public readonly Edge[]   edges;

        public Vector3 vertex;
        public Vector3 normal;
        public int     index = -1;

        public Voxel( Vector3 center, Vector3 size ) {
            this.center  = center;
            this.size    = size;
            this.extents = size / 2;
            this.minimum = this.center - this.extents;
            this.maximum = this.center + this.extents;

            /*
            corner layout:
                 4--------------7
                /|             /|
               / |            / |
              /  |           /  |
             5--------------6   |
             |   |          |   |
             |   |          |   |
             |   3----------|---2
             |  /           |  /
             | /            | /
             |/             |/
             0--------------1
            */
            this.corners = new Corner[] {
                // bottom
                new( this.minimum ),
                new( new Vector3( this.maximum.x, this.minimum.y, this.minimum.z ) ),
                new( new Vector3( this.maximum.x, this.minimum.y, this.maximum.z ) ),
                new( new Vector3( this.minimum.x, this.minimum.y, this.maximum.z ) ),
                // top
                new( new Vector3( this.minimum.x, this.maximum.y, this.maximum.z ) ),
                new( new Vector3( this.minimum.x, this.maximum.y, this.minimum.z ) ),
                new( new Vector3( this.maximum.x, this.maximum.y, this.minimum.z ) ),
                new( this.maximum ),
            };

            /*
            edge layout:
                 +------3-------+
                /|             /|
               10|            11|
              /  |           /  |
             +---6---2------+   7
             |   |          |   |
             |   |          |   |
             4   +------1---5---+
             |  /           |  /
             | 8            | 9
             |/             |/
             +-------0------+
            */
            this.edges = new Edge[] {
                // x axis
                new( new Corner[] { this.corners[0], this.corners[1] } ),
                new( new Corner[] { this.corners[3], this.corners[2] } ),
                new( new Corner[] { this.corners[5], this.corners[6] } ),
                new( new Corner[] { this.corners[4], this.corners[7] } ),
                // y axis
                new( new Corner[] { this.corners[5], this.corners[0] } ),
                new( new Corner[] { this.corners[6], this.corners[1] } ),
                new( new Corner[] { this.corners[4], this.corners[3] } ),
                new( new Corner[] { this.corners[7], this.corners[2] } ),
                // z axis
                new( new Corner[] { this.corners[0], this.corners[3] } ),
                new( new Corner[] { this.corners[1], this.corners[2] } ),
                new( new Corner[] { this.corners[5], this.corners[4] } ),
                new( new Corner[] { this.corners[6], this.corners[7] } )
            };
        }

        public Vector3 getCenter( ) {
            return this.center;
        }

        public Vector3 getSize( ) {
            return this.size;
        }

        public Vector3 getVertex( ) {
            return this.vertex;
        }

        public Vector3 getNormal( ) {
            return this.normal;
        }

        public bool intersectsContour( ) {
            return Vector3.Magnitude( this.vertex ) > 0.0f && Vector3.Magnitude( this.normal ) > 0.0f;
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.getCenter( ) && this.size == other.getSize( );
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    private const int BinarySearchIterations = 16;
    private const int MinimizerIterations    = 16;

    private Voxel[,,] voxels;

    public enum IntersectionApproximationMode {
        BinarySearch,
        LinearInterpolation
    }

    [Space( )]

    public IntersectionApproximationMode intersectionApproximationMode = IntersectionApproximationMode.BinarySearch;

    public override IEnumerable<SurfaceExtractor.Corner> getCorners( ) {
        return this.voxels.Flatten( ).Aggregate(
            new List<SurfaceExtractor.Corner>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.corners );
                return accumulator;
            }
        ).Distinct( );
    }

    public override IEnumerable<SurfaceExtractor.Edge> getEdges( ) {
        return this.voxels.Flatten( ).Aggregate(
            new List<SurfaceExtractor.Edge>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.edges );
                return accumulator;
            }
        ).Distinct( );
    }

    public override IEnumerable<SurfaceExtractor.Voxel> getVoxels( ) {
        return this.voxels.Flatten( );
    }

    public override Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions ) {

        // create uniformly subdivided voxel grid

        this.voxels = new Voxel[resolution, resolution, resolution];
        for( var x = 0; x < this.voxels.GetLength( 0 ); ++x ) {
            for( var y = 0; y < this.voxels.GetLength( 1 ); ++y ) {
                for( var z = 0; z < this.voxels.GetLength( 2 ); ++z ) {
                    var size    = Vector3.one / resolution;                 // center voxel relative to 0, 0, 0
                    var center  = ( new Vector3( x, y, z ) / resolution ) - ( size / 2 * ( resolution - 1 ) );

                    this.voxels[x, y, z] = new Voxel( center, size );
                }
            }
        }

        // TODO: add multi-material functionality

        densityFunctions = densityFunctions.OrderBy( ( densityFunction ) => densityFunction.getCombinationMode( ) );

        // sample corner densities

        foreach( var voxel in this.voxels ) {
            foreach( var corner in voxel.corners ) {
                corner.density = densityFunctions.Aggregate(
                    float.MaxValue,
                    ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, corner.position )
                );
            }
        }

        // find contour intersections and calculate minimizing vertices

        var index = 0;
        foreach( var voxel in this.voxels ) {
            if(
                voxel.corners.All( ( corner ) => corner.sign == SurfaceExtractor.Corner.Sign.Inside  ) ||
                voxel.corners.All( ( corner ) => corner.sign == SurfaceExtractor.Corner.Sign.Outside )
            ) {
                // cell is either fully inside or outside the volume, skip
                continue;
            }

            var intersectionPlanes = new List<Plane>( );

            foreach( var edge in voxel.edges ) {
                if(
                    ( edge.corners[0].sign == SurfaceExtractor.Corner.Sign.Inside  && edge.corners[1].sign == SurfaceExtractor.Corner.Sign.Inside  ) ||
                    ( edge.corners[0].sign == SurfaceExtractor.Corner.Sign.Outside && edge.corners[1].sign == SurfaceExtractor.Corner.Sign.Outside )
                ) {
                    // no sign change detected on edge, skip
                    continue;
                }

                edge.intersection = this.approximateIntersection ( edge, densityFunctions );
                edge.normal       = this.calculateEdgeNormal     ( edge, densityFunctions );

                intersectionPlanes.Add( new( edge.normal, edge.intersection ) );
            }

            // calculate minimizing vertex
            // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
            // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
            var minimizingVertex = voxel.center;

            for( var i = 0; i < MinimizerIterations; i++ ) {
                minimizingVertex -= intersectionPlanes.Aggregate(
                    Vector3.zero,
                    ( accumulator, plane ) => {
                        accumulator += plane.GetDistanceToPoint( minimizingVertex ) * plane.normal;
                        return accumulator;
                    }
                ) / intersectionPlanes.Count;
            }

            voxel.vertex = minimizingVertex;

            // calculate surface normal
            voxel.normal = Vector3.Normalize(
                intersectionPlanes.Aggregate(
                    Vector3.zero,
                    ( accumulator, plane ) => {
                        accumulator += plane.normal;
                        return accumulator;
                    }
                ) / intersectionPlanes.Count
            );

            voxel.index = voxel.index == -1 ? index++ : voxel.index;
        }


        var vertices = new List<Vector3>( );
        var normals  = new List<Vector3>( );
        var indices  = new List<int>( );

        // generate vertices and indices

        for( var x = 0; x < this.voxels.GetLength( 0 ); ++x ) {
            for( var y = 0; y < this.voxels.GetLength( 1 ); ++y ) {
                for( var z = 0; z < this.voxels.GetLength( 2 ); ++z ) {
                    var voxel = this.voxels[x, y, z];

                    if( voxel.index == -1 ) {
                        continue;
                    }

                    vertices.Add ( voxel.vertex );
                    normals.Add  ( voxel.normal );

                    // on every positive axis, generate indices using 4 voxel surrounding a common edge

                    // x axis
                    if( y + 1 < this.voxels.GetLength( 1 ) && z + 1 < this.voxels.GetLength( 2 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.voxels[x, y,     z + 1],
                            this.voxels[x, y + 1, z    ],
                            this.voxels[x, y + 1, z + 1]
                        };
                        if( voxels[1].index > -1 && voxels[2].index > -1 && voxels[3].index > -1 ) {
                            indices.AddRange( this.generateIndices( voxels, voxels[0].edges[3] ) );
                        }
                    }

                    // y axis
                    if( x + 1 < this.voxels.GetLength( 0 ) && z + 1 < this.voxels.GetLength( 2 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.voxels[x,     y, z + 1],
                            this.voxels[x + 1, y, z    ],
                            this.voxels[x + 1, y, z + 1]
                        };
                        if( voxels[1].index > -1 && voxels[2].index > -1 && voxels[3].index > -1 ) {
                            indices.AddRange( this.generateIndices( voxels, voxels[0].edges[7] ) );
                        }
                    }

                    // z axis
                    if( x + 1 < this.voxels.GetLength( 0 ) && y + 1 < this.voxels.GetLength( 1 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.voxels[x,     y + 1, z],
                            this.voxels[x + 1, y,     z],
                            this.voxels[x + 1, y + 1, z]
                        };
                        if( voxels[1].index > -1 && voxels[2].index > -1 && voxels[3].index > -1 ) {
                            indices.AddRange( this.generateIndices( voxels, voxels[0].edges[11] ) );
                        }
                    }
                }
            }
        }

        return new Mesh {
            vertices  = vertices.ToArray( ),
            normals   = normals.ToArray( ),
            triangles = indices.ToArray( )
        };
    }

    private float sampleDensityFunction( float density, DensityFunction densityFunction, Vector3 position ) {
        density = densityFunction.getCombinationMode( ) switch {
            DensityFunction.CombinationMode.Union        => Mathf.Min( density,  densityFunction.sample( position ) ),
            DensityFunction.CombinationMode.Intersection => Mathf.Max( density,  densityFunction.sample( position ) ),
            DensityFunction.CombinationMode.Subtraction  => Mathf.Max( density, -densityFunction.sample( position ) ),
            _                                            => throw new Exception( "Unknown combination mode specified" ),
        };
        return density;
    }

    private Vector3 approximateIntersection( Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
        if( this.intersectionApproximationMode == IntersectionApproximationMode.BinarySearch ) {

            var ( start, end ) = edge.corners[0].density < edge.corners[1].density
                ? ( edge.corners[0].position, edge.corners[1].position )
                : ( edge.corners[1].position, edge.corners[0].position );

            var intersection = Vector3.zero;
            for( var iteration = 0; iteration < BinarySearchIterations; ++iteration ) {
                intersection = start + ( 0.5f * ( end - start ) );

                var density = densityFunctions.Aggregate(
                    float.MaxValue,
                    ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, intersection )
                );

                if( density < 0.0f ) {
                    start = intersection;
                }
                else if( density > 0.0f ) {
                    end = intersection;
                }
                else if( density == 0.0f ) {
                    break;
                }
            }

            return intersection;
        }
        else if( this.intersectionApproximationMode == IntersectionApproximationMode.LinearInterpolation ) {
            return edge.corners[0].position + ( ( -edge.corners[0].density ) * ( edge.corners[1].position - edge.corners[0].position ) / ( edge.corners[1].density - edge.corners[0].density ) );
        }

        throw new Exception( "Unknown intersection approximation mode specified" );
    }

    private Vector3 calculateEdgeNormal( Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
        var step = 0.1f;

        // sample surrounding x, y, z locations and take the difference

        var positive = Vector3.positiveInfinity;

        positive.x = densityFunctions.Aggregate(
            positive.x,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection + new Vector3( step, 0.0f, 0.0f ) )
        );
        positive.y = densityFunctions.Aggregate(
            positive.y,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection + new Vector3( 0.0f, step, 0.0f ) )
        );
        positive.z = densityFunctions.Aggregate(
            positive.z,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection + new Vector3( 0.0f, 0.0f, step ) )
        );

        var negative = Vector3.positiveInfinity;

        negative.x = densityFunctions.Aggregate(
            negative.x,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection - new Vector3( step, 0.0f, 0.0f ) )
        );
        negative.y = densityFunctions.Aggregate(
            negative.y,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection - new Vector3( 0.0f, step, 0.0f ) )
        );
        negative.z = densityFunctions.Aggregate(
            negative.z,
            ( density, densityFunction ) => this.sampleDensityFunction( density, densityFunction, edge.intersection - new Vector3( 0.0f, 0.0f, step ) )
        );

        return Vector3.Normalize( positive - negative );
    }

    private int[] generateIndices( Voxel[] voxels, Edge edge ) {
        // ensure quad is indexed facing outward
        if( edge.corners[0].sign != SurfaceExtractor.Corner.Sign.Inside ) {
            return new int[] {
                voxels[0].index,
                voxels[1].index,
                voxels[2].index,
                voxels[2].index,
                voxels[1].index,
                voxels[3].index
            };
        }
        else {
            return new int[] {
                voxels[3].index,
                voxels[1].index,
                voxels[2].index,
                voxels[2].index,
                voxels[1].index,
                voxels[0].index
            };
        }
    }

}
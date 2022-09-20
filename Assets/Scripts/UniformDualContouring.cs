using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// dual contouring implementation using uniform grid

public class UniformDualContouring : Voxelizer {

    private class Corner : SurfaceExtractor.Corner {

        public Vector3                        position      { get; }
        public float                          density       { get; set; } = float.MaxValue;
        public SurfaceExtractor.MaterialIndex materialIndex { get; set; } = SurfaceExtractor.MaterialIndex.Void;

        public Corner( Vector3 position ) {
            this.position = position;
        }

        public bool Equals( SurfaceExtractor.Corner other ) {
            return this.position.Equals( other.position );
        }

        public override int GetHashCode( ) {
            return this.position.GetHashCode( );
        }

    }

    private class Edge : SurfaceExtractor.Edge {

        public SurfaceExtractor.Corner[] corners      { get; }
        public Vector3                   intersection { get; set; }
        public Vector3                   normal       { get; set; }

        public Edge( SurfaceExtractor.Corner[] corners ) {
            this.corners = corners;
        }

        public bool intersectsContour( ) {
            return (
                this.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void && this.corners[1].materialIndex >= SurfaceExtractor.MaterialIndex.Material1
            ) || (
                this.corners[1].materialIndex == SurfaceExtractor.MaterialIndex.Void && this.corners[0].materialIndex >= SurfaceExtractor.MaterialIndex.Material1
            );
        }

        public bool Equals( SurfaceExtractor.Edge other ) {
            return this.corners[0].Equals( other.corners[0] ) && this.corners[1].Equals( other.corners[1] );
        }

        public override int GetHashCode( ) {
            return this.corners[0].GetHashCode( ) ^ this.corners[1].GetHashCode( );
        }

    }

    private class Voxel : SurfaceExtractor.Voxel {

        public SurfaceExtractor.Voxel.Type type    { get; set; }
        public Vector3                     center  { get; }
        public Vector3                     size    { get; }
        public Vector3                     extents { get; }
        public Vector3                     minimum { get; }
        public Vector3                     maximum { get; }
        public SurfaceExtractor.Corner[]   corners { get; }
        public SurfaceExtractor.Edge[]     edges   { get; }

        public QEFSolver<QEF> qef    { get; set; }
        public Vector3        vertex { get; set; } = Vector3.zero;
        public Vector3        normal { get; set; } = Vector3.zero;
        public int            index  { get; set; } = -1;

        public Voxel( Vector3 center, Vector3 size ) {
            this.type    = SurfaceExtractor.Voxel.Type.None;
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
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[1] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[3], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[6] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[4], this.corners[7] } ),
                // y axis
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[0] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[6], this.corners[1] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[4], this.corners[3] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[7], this.corners[2] } ),
                // z axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[3] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[1], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[4] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[6], this.corners[7] } )
            };
        }

        public bool hasFeaturePoint( ) {
            return this.index > -1;
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.center && this.size == other.size;
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    private Voxel[,,] grid;

    public enum IntersectionApproximationMode {
        BinarySearch,
        LinearInterpolation
    }

    [Space( )]

    public int minimizerIterations         = 6;
    public int binarySearchIterations      = 6;
    public int surfaceCorrectionIterations = 6;

    public IntersectionApproximationMode intersectionApproximationMode = IntersectionApproximationMode.BinarySearch;

    public override IEnumerable<SurfaceExtractor.Corner> corners {
        get {
            return this.grid.Flatten( ).Aggregate(
                new List<SurfaceExtractor.Corner>( ),
                ( accumulator, voxel ) => {
                    accumulator.AddRange( voxel.corners );
                    return accumulator;
                }
            ).Distinct( );
        }
    }

    public override IEnumerable<SurfaceExtractor.Edge> edges {
        get {
            return this.grid.Flatten( ).Aggregate(
                new List<SurfaceExtractor.Edge>( ),
                ( accumulator, voxel ) => {
                    accumulator.AddRange( voxel.edges );
                    return accumulator;
                }
            ).Distinct( );
        }
    }

    public override IEnumerable<SurfaceExtractor.Voxel> voxels {
        get { return this.grid.Flatten( ); }
    }

    public override Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions ) {

        // create uniformly subdivided voxel grid

        this.grid = new Voxel[resolution, resolution, resolution];
        for( var x = 0; x < this.grid.GetLength( 0 ); ++x ) {
            for( var y = 0; y < this.grid.GetLength( 1 ); ++y ) {
                for( var z = 0; z < this.grid.GetLength( 2 ); ++z ) {
                    var size   = Vector3.one / resolution;                 // center voxel relative to 0, 0, 0
                    var center = ( new Vector3( x, y, z ) / resolution ) - ( size / 2 * ( resolution - 1 ) );

                    this.grid[x, y, z] = new Voxel( center, size );
                }
            }
        }

        // sample corner densities

        foreach( var voxel in this.grid ) {
            foreach( var corner in voxel.corners ) {
                foreach( var densityFunction in densityFunctions ) {
                    var density = densityFunction.sample( corner.position );

                    // set material bit if the corner is inside of the shape
                    if( densityFunction.combinationMode == DensityFunction.CombinationMode.Union && density < 0.0f ) {
                        corner.materialIndex |= densityFunction.materialIndex;
                    }
                    // unset material bit if the corner is outside of the shape
                    if( densityFunction.combinationMode == DensityFunction.CombinationMode.Intersection && density > 0.0f ) {
                        corner.materialIndex &= ~densityFunction.materialIndex;
                    }
                    // unset material bit if the corner is inside of the shape
                    if( densityFunction.combinationMode == DensityFunction.CombinationMode.Subtraction && density < 0.0f ) {
                        corner.materialIndex &= ~densityFunction.materialIndex;
                    }

                    corner.density = densityFunction.combinationMode switch {
                        DensityFunction.CombinationMode.Union        => Mathf.Min( corner.density,  density ),
                        DensityFunction.CombinationMode.Intersection => Mathf.Max( corner.density,  density ),
                        DensityFunction.CombinationMode.Subtraction  => Mathf.Max( corner.density, -density ),
                        _                                            => throw new Exception( "Unknown combination mode specified" ),
                    };
                }
            }
        }

        // find contour intersections and calculate minimizing vertices

        var index = 0;
        foreach( var voxel in this.grid ) {
            if(
                voxel.corners.All( ( corner ) => corner.materialIndex == SurfaceExtractor.MaterialIndex.Void      ) ||
                voxel.corners.All( ( corner ) => corner.materialIndex >= SurfaceExtractor.MaterialIndex.Material1 )
            ) {
                // cell is either fully inside or outside the volume, skip
                continue;
            }

            voxel.qef = new QEF( this.minimizerIterations, this.surfaceCorrectionIterations );

            foreach( var edge in voxel.edges ) {
                if( !edge.intersectsContour( ) ) {
                    continue;
                }

                edge.intersection = this.approximateIntersection ( edge,              densityFunctions );
                edge.normal       = this.calculateNormal         ( edge.intersection, densityFunctions );

                voxel.qef.add( edge.intersection, edge.normal );
            }

            ( voxel.vertex, voxel.normal, _ ) = voxel.qef.solve( voxel, densityFunctions );

            voxel.index = index++;
        }

        // generate vertices and indices

        var vertices = new List<Vector3>( );
        var normals  = new List<Vector3>( );
        var indices  = new Dictionary<int, List<int>>( );

        for( var x = 0; x < this.grid.GetLength( 0 ); ++x ) {
            for( var y = 0; y < this.grid.GetLength( 1 ); ++y ) {
                for( var z = 0; z < this.grid.GetLength( 2 ); ++z ) {
                    var voxel = this.grid[x, y, z];

                    if( !voxel.hasFeaturePoint( ) ) {
                        continue;
                    }

                    vertices.Add ( voxel.vertex );
                    normals.Add  ( voxel.normal );

                    // on every positive axis, generate indices using 4 voxel surrounding a common edge

                    // x axis
                    if( y + 1 < this.grid.GetLength( 1 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.grid[x, y,     z + 1],
                            this.grid[x, y + 1, z    ],
                            this.grid[x, y + 1, z + 1]
                        };
                        var edge = voxel.edges[3]; // common edge surrounded by all 4 voxels, refer to edge layout diagram

                        if( voxels.All( ( voxel ) => voxel.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {
                             this.generateIndices( indices, voxels, edge );
                        }
                    }

                    // y axis
                    if( x + 1 < this.grid.GetLength( 0 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.grid[x,     y, z + 1],
                            this.grid[x + 1, y, z    ],
                            this.grid[x + 1, y, z + 1]
                        };
                        var edge = voxel.edges[7];

                        if( voxels.All( ( voxel ) => voxel.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {
                            this.generateIndices( indices, voxels, edge );
                        }
                    }

                    // z axis
                    if( x + 1 < this.grid.GetLength( 0 ) && y + 1 < this.grid.GetLength( 1 ) ) {
                        var voxels = new Voxel[] {
                            voxel,
                            this.grid[x,     y + 1, z],
                            this.grid[x + 1, y,     z],
                            this.grid[x + 1, y + 1, z]
                        };
                        var edge = voxel.edges[11];

                        if( voxels.All( ( voxel ) => voxel.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {
                            this.generateIndices( indices, voxels, edge );
                        }
                    }
                }
            }
        }

        var mesh = new Mesh {
            vertices = vertices.ToArray( ),
            normals  = normals.ToArray( )
        };

        mesh.subMeshCount = indices.Keys.Count;

        var subMeshCount = 0;
        foreach( var triangles in indices ) {
            mesh.SetTriangles( triangles.Value, subMeshCount );
            ++subMeshCount;
        }

        return mesh;
    }

    private Vector3 approximateIntersection( SurfaceExtractor.Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
        if( edge.corners[0].density == 0.0f || edge.corners[1].density == 0.0f ) {
            // one of the corners is at the exact intersection
            return edge.corners[0].density == 0.0f ? edge.corners[0].position : edge.corners[1].position;
        }
        if( this.intersectionApproximationMode == IntersectionApproximationMode.BinarySearch ) {
            var ( start, end ) = edge.corners[0].density < edge.corners[1].density
                ? ( edge.corners[0].position, edge.corners[1].position )
                : ( edge.corners[1].position, edge.corners[0].position );

            var intersection = Vector3.zero;
            for( var binarySearchIterations = 0; binarySearchIterations < this.binarySearchIterations; ++binarySearchIterations ) {
                intersection = start + ( 0.5f * ( end - start ) );

                var density = SurfaceExtractor.calculateDensity( intersection, densityFunctions );

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

    private Vector3 calculateNormal( Vector3 point, IEnumerable<DensityFunction> densityFunctions ) {
        var step = 0.1f;

        // sample surrounding x, y, z locations and take the difference

        var positive = Vector3.positiveInfinity;

        positive.x = SurfaceExtractor.calculateDensity( point + new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        positive.y = SurfaceExtractor.calculateDensity( point + new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        positive.z = SurfaceExtractor.calculateDensity( point + new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        var negative = Vector3.positiveInfinity;

        negative.x = SurfaceExtractor.calculateDensity( point - new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        negative.y = SurfaceExtractor.calculateDensity( point - new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        negative.z = SurfaceExtractor.calculateDensity( point - new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        return Vector3.Normalize( positive - negative );
    }

    private void generateIndices( Dictionary<int, List<int>> indices, Voxel[] voxels, SurfaceExtractor.Edge edge ) {
        // indices should only be generated from void - solid intersections
        UnityEngine.Debug.Assert(
            ( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void && edge.corners[1].materialIndex >= SurfaceExtractor.MaterialIndex.Material1 ) ||
            ( edge.corners[1].materialIndex == SurfaceExtractor.MaterialIndex.Void && edge.corners[0].materialIndex >= SurfaceExtractor.MaterialIndex.Material1 )
        );

        int[] triangles;

        // ensure quad is indexed facing outward
        if( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void ) {
            triangles = new int[] {
                voxels[0].index,
                voxels[1].index,
                voxels[2].index,
                voxels[2].index,
                voxels[1].index,
                voxels[3].index
            };
        }
        else {
            triangles = new int[] {
                voxels[3].index,
                voxels[1].index,
                voxels[2].index,
                voxels[2].index,
                voxels[1].index,
                voxels[0].index
            };
        }

        var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( edge );
        if( !indices.ContainsKey( subMeshIndex ) ) {
            indices.Add( subMeshIndex, new( ) );
        }
        indices[subMeshIndex].AddRange( triangles );
    }

}
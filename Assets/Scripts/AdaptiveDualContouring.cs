using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AdaptiveDualContouring : Voxelizer {

    public class Corner : SurfaceExtractor.Corner {

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

    public class Edge : SurfaceExtractor.Edge {

        public SurfaceExtractor.Corner[] corners      { get; }
        public Vector3                   intersection { get; set; }
        public Vector3                   normal       { get; set; }

        public Edge( Corner[] corners ) {
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

    public class Voxel : SurfaceExtractor.Voxel {

        public enum Type {
            Internal,
            Pseudo,
            Leaf
        }

        public Vector3  center  { get; }
        public Vector3  size    { get; }
        public Vector3  extents { get; }
        public Vector3  minimum { get; }
        public Vector3  maximum { get; }
        public Corner[] corners { get; }
        public Edge[]   edges   { get; }

        public Type    type   { get; set; }
        public int     depth  { get; set; }
        public Vector3 vertex { get; set; } = Vector3.zero;
        public Vector3 normal { get; set; } = Vector3.zero;
        public int     index  { get; set; } = -1;

        public Voxel( Type type, int depth, Vector3 center, Vector3 size ) {
            this.type    = type;
            this.depth   = depth;
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

        public bool hasFeaturePoint( ) {
            return Vector3.Magnitude( this.vertex ) > 0.0f && Vector3.Magnitude( this.normal ) > 0.0f;
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.center && this.size == other.size;
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    public override IEnumerable<SurfaceExtractor.Corner> corners {
        get {
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( node ) => node.type != Voxel.Type.Internal
            ).Aggregate(
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
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( node ) => node.type != Voxel.Type.Internal
            ).Aggregate(
                new List<SurfaceExtractor.Edge>( ),
                ( accumulator, voxel ) => {
                    accumulator.AddRange( voxel.edges );
                    return accumulator;
                }
            ).Distinct( );
        }
    }

    public override IEnumerable<SurfaceExtractor.Voxel> voxels {
        get {
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( node ) => node.type != Voxel.Type.Internal
            );
        }
    }

    private Octree<Voxel> octree;

    [Space( )]

    public bool simplification = false;

    [Min( 0.0f )]
    public float errorThreshold = 0.01f;

    public int minimizerIterations         = 6;
    public int binarySearchIterations      = 6;
    public int surfaceCorrectionIterations = 6;

    public enum IntersectionApproximationMode {
        BinarySearch,
        LinearInterpolation
    }

    public IntersectionApproximationMode intersectionApproximationMode = IntersectionApproximationMode.BinarySearch;

    public override Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions ) {

        // build octree with depth equal to resolution

        this.octree = Octree<Voxel>.build(
            new( Voxel.Type.Internal, 0, Vector3.zero, Vector3.one ),
            ( parent ) => {
                if( parent.depth == ( resolution - 1 ) ) {
                    return null;
                }

                var depth = parent.depth + 1;
                var type  = depth < ( resolution - 1 ) ? Voxel.Type.Internal : Voxel.Type.Leaf;
                var scale = Mathf.Pow( 2, depth + 1 );
                var size  = Vector3.one / Mathf.Pow( 2, depth );

                return new Voxel[8] {
                    /*
                    lower child voxels layout:
                                    3               2
                             +--------------+--------------+
                            /|             /|             /|
                           / |            / |            / |
                          /  |           /  |           /  |
                         +--------------+--------------+   |
                        /|   |         /|   |         /|   |
                       / |   |        / |   |        / |   |
                      /  |   +-------/--|---+-------/--|---+
                     +--------------+--------------+   |  /
                     |   | /        |   | /        |   | /
                     |   |/         |   |/         |   |/
                     |   +----------|---+----------|---+
                     |  /           |  /           |  /
                     | /            | /            | /
                     |/             |/             |/
                     +--------------+--------------+
                            0              1
                    */
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f,  1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f,  1.0f ) / scale ), size ),
                    /*
                    upper child voxels layout:
                                    4               7
                             +--------------+--------------+
                            /|             /|             /|
                           / |            / |            / |
                          /  |           /  |           /  |
                         +--------------+--------------+   |
                        /|   |         /|   |         /|   |
                       / |   |        / |   |        / |   |
                      /  |   +-------/--|---+-------/--|---+
                     +--------------+--------------+   |  /
                     |   | /        |   | /        |   | /
                     |   |/         |   |/         |   |/
                     |   +----------|---+----------|---+
                     |  /           |  /           |  /
                     | /            | /            | /
                     |/             |/             |/
                     +--------------+--------------+
                            5              6
                    */
                    new( type, depth, parent.center + ( new Vector3( -1.0f,  1.0f,  1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3( -1.0f,  1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f,  1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f,  1.0f,  1.0f ) / scale ), size )
                };
            }
        );

        // sample corner densities

        Octree<Voxel>.walk(
            this.octree,
            ( node ) => {
                var voxel = node.data;

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
        );

        // find contour intersections and calculate minimizing vertices

        Octree<Voxel>.walk(
            this.octree,
            ( node ) => {
                var voxel = node.data;

                if(
                    voxel.corners.All( ( corner ) => corner.materialIndex == SurfaceExtractor.MaterialIndex.Void      ) ||
                    voxel.corners.All( ( corner ) => corner.materialIndex >= SurfaceExtractor.MaterialIndex.Material1 )
                ) {
                    // cell is either fully inside or outside the volume, skip
                    return;
                }

                var intersectionPlanes = new List<Plane>( );

                foreach( var edge in voxel.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = this.approximateIntersection ( edge,              densityFunctions );
                    edge.normal       = this.calculateNormal         ( edge.intersection, densityFunctions );

                    intersectionPlanes.Add( new( edge.normal, edge.intersection ) );
                }

                // calculate minimizing vertex
                // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
                // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
                var minimizingVertex = voxel.center;

                for( var minimizingIteration = 0; minimizingIteration < this.minimizerIterations; ++minimizingIteration ) {
                    minimizingVertex -= intersectionPlanes.Aggregate(
                        Vector3.zero,
                        ( accumulator, plane ) => {
                            accumulator += plane.GetDistanceToPoint( minimizingVertex ) * plane.normal;
                            return accumulator;
                        }
                    ) / intersectionPlanes.Count;
                }

                // calculate surface normal
                var surfaceNormal = Vector3.Normalize(
                    intersectionPlanes.Aggregate(
                        Vector3.zero,
                        ( accumulator, plane ) => {
                            accumulator += plane.normal;
                            return accumulator;
                        }
                    ) / intersectionPlanes.Count
                );

                // correct surface by forcing the minimizing vertex towards the zero crossing
                // see https://www.reddit.com/r/VoxelGameDev/comments/mhiec0/how_are_people_getting_good_results_with_dual/gti0b8d/
                for( var surfaceCorrectionIteration = 0; surfaceCorrectionIteration < this.surfaceCorrectionIterations; ++surfaceCorrectionIteration ) {
                    var density = SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions );
                    if( density == 0.0f ) {
                        // vertex is at the surface
                        break;
                    }
                    minimizingVertex -= surfaceNormal * density;
                }

                voxel.vertex = minimizingVertex;
                voxel.normal = surfaceNormal;
            }
        );

        // simplify octree

        if( this.simplification ) {
            UnityEngine.Debug.Assert( this.errorThreshold != 0.0f );

            this.octree = this.simplify( this.octree, this.errorThreshold, densityFunctions );

        }

        // generate vertices and indices

        var vertices = new List<Vector3>( );
        var normals  = new List<Vector3>( );
        var indices  = new Dictionary<int, List<int>>( );

        this.contourCell( this.octree, vertices, normals, indices );

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

    private Vector3 approximateIntersection( Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
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

    private enum Axis {
        X,
        Y,
        Z
    }

    private void contourCell( Octree<Voxel> node, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        if( node.data.type == Voxel.Type.Internal ) {

            // contour cells in children
            this.contourCell( node.children[0], vertices, normals, indices );
            this.contourCell( node.children[1], vertices, normals, indices );
            this.contourCell( node.children[2], vertices, normals, indices );
            this.contourCell( node.children[3], vertices, normals, indices );
            this.contourCell( node.children[4], vertices, normals, indices );
            this.contourCell( node.children[5], vertices, normals, indices );
            this.contourCell( node.children[6], vertices, normals, indices );
            this.contourCell( node.children[7], vertices, normals, indices );

            // contour common face pairs in children

            /*
                    +--------------+--------------+
                   /|             /|             /|
                  / |            / |            / |
                 /  |           /  |           /  |
                +--------------+--------------+   |
                |   |          |   |          |   |
                |   |          |   |          |   |
                |   +----------|---+----------|---+
                |  /           |  /           |  /
                | /            | /            | /
                |/             |/             |/
                +--------------+--------------+
                       0              1
            */

            // x axis faces
            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[0],
                    node.children[1]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[3],
                    node.children[2]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[5],
                    node.children[6]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[4],
                    node.children[7]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            /*
                    +--------------+
                   /|             /|
                  / |            / |
                 /  |           /  | 1
                +--------------+   |
                |   |          |   |
                |   |          |   |
                |   +----------|---+
                |  /|          |  /|
                | / |          | / |
                |/  |          |/  |
                +--------------+   |
                |   |          |   | 0
                |   |          |   |
                |   +----------|---+
                |  /           |  /
                | /            | /
                |/             |/
                +--------------+
                 
            */

            // y axis faces
            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[0],
                    node.children[5]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[1],
                    node.children[6]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[2],
                    node.children[7]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[3],
                    node.children[4]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            /*
                        +--------------+
                       /|             /|
                      / |            / |
                     /  |           /  |
                    +--------------+   |
                   /|   |         /|   |
                  / |   |        / |   |
                 /  |   +-------/--|---+
                +--------------+   |  /
                |   | /        |   | / 1
                |   |/         |   |/
                |   +----------|---+
                |  /           |  /
                | /            | / 0
                |/             |/
                +--------------+
                 
            */

            // z axis faces
            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[0],
                    node.children[3]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[1],
                    node.children[2]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[5],
                    node.children[4]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );

            this.contourFace(
                new Octree<Voxel>[] {
                    node.children[6],
                    node.children[7]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );

            // contour common edges of children

            /*
            common edges of child voxels:
                         Y+   Z+
                         +   +
                         |  /
                         | /
                         |/
              X- +-------+-------+ X+
                        /|
                       / |
                      /  |
                     +   +
                    Z-   Y-
            */

            // x axis edges
            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[0],
                    node.children[3],
                    node.children[4],
                    node.children[5]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[1],
                    node.children[2],
                    node.children[7],
                    node.children[6]
                },
                Axis.X,
                vertices,
                normals,
                indices
            );

            // y axis edges
            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[0],
                    node.children[3],
                    node.children[2],
                    node.children[1]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[5],
                    node.children[4],
                    node.children[7],
                    node.children[6]
                },
                Axis.Y,
                vertices,
                normals,
                indices
            );

            // z axis edges
            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[0],
                    node.children[5],
                    node.children[6],
                    node.children[1]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );

            this.contourEdge(
                new Octree<Voxel>[4] {
                    node.children[3],
                    node.children[4],
                    node.children[7],
                    node.children[2]
                },
                Axis.Z,
                vertices,
                normals,
                indices
            );
        }

    }

    private void contourFace( Octree<Voxel>[] nodes, Axis axis, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if(
            nodes[0].data.type == Voxel.Type.Internal ||
            nodes[1].data.type == Voxel.Type.Internal
        ) {

            // contour common face pairs in children of given voxel pairs

            switch( axis ) {
                case Axis.X:
                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[4]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Y:
                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[2]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Z:
                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourFace(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[6]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                default:
                    throw new Exception( "Unknown axis specified" );
            }

            // contour common edges in children of given voxel pairs

            switch( axis ) {
                case Axis.X:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Y:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[2],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Z:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[6],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[6],
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                default:
                    throw new Exception( "Unknown axis specified" );
            }
        }
    }

    private void contourEdge( Octree<Voxel>[] nodes, Axis axis, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        UnityEngine.Debug.Assert( nodes.Length == 4 );

        if(
            nodes[0].data.type != Voxel.Type.Internal &&
            nodes[1].data.type != Voxel.Type.Internal &&
            nodes[2].data.type != Voxel.Type.Internal &&
            nodes[3].data.type != Voxel.Type.Internal
        ) {
            this.generateIndices( nodes, axis, vertices, normals, indices );
        }
        else {

            // contour common edges in children of given voxels

            switch( axis ) {
                case Axis.X:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[6],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[1],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[2]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[0],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[3]
                        },
                        Axis.X,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Y:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[0],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[3]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[6],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[5],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[4]
                        },
                        Axis.Y,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                case Axis.Z:
                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[2],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[3],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[4]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );

                    this.contourEdge(
                        new Octree<Voxel>[] {
                            nodes[0].data.type != Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[2].data.type != Voxel.Type.Internal ? nodes[2] : nodes[2].children[0],
                            nodes[3].data.type != Voxel.Type.Internal ? nodes[3] : nodes[3].children[5]
                        },
                        Axis.Z,
                        vertices,
                        normals,
                        indices
                    );
                    break;

                default:
                    throw new Exception( "Unknown axis specified" );
            }
        }
    }

    private void generateIndices( Octree<Voxel>[] nodes, Axis axis, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        // node 0 will always be positioned at the most negative on every axis, allowing easy common edge finding
        var edge = axis switch {
            Axis.X => nodes[0].data.edges[3],
            Axis.Y => nodes[0].data.edges[7],
            Axis.Z => nodes[0].data.edges[11],
            _      => throw new Exception( "Unknown axis specified" ),
        };

        if( nodes.All( ( node ) => node.data.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {

            // generate vertex and normal
            foreach( var node in nodes ) {
                if( node.data.index == -1 ) {
                    node.data.index = vertices.Count;

                    vertices.Add ( node.data.vertex );
                    normals.Add  ( node.data.normal );
                }
            }

            // generate indices

            int[] triangles;

            // ensure quad is indexed facing outward
            if( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void ) {
                triangles = new int[] {
                    nodes[0].data.index,
                    nodes[1].data.index,
                    nodes[2].data.index,
                    nodes[0].data.index,
                    nodes[2].data.index,
                    nodes[3].data.index
                };
            }
            else {
                triangles = new int[] {
                    nodes[3].data.index,
                    nodes[2].data.index,
                    nodes[0].data.index,
                    nodes[2].data.index,
                    nodes[1].data.index,
                    nodes[0].data.index
                };
            }

            var subMeshIndex = this.findHighestMaterialBit( edge );
            if( !indices.ContainsKey( subMeshIndex ) ) {
                indices.Add( subMeshIndex, new( ) );
            }
            indices[subMeshIndex].AddRange( triangles );
        }
    }

    private int findHighestMaterialBit( Edge edge ) {
        var materialIndex = edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void
            ? edge.corners[1].materialIndex
            : edge.corners[0].materialIndex;

        // return index of highest set bit in material index
        for( var bitIndex = ( sizeof( int ) * 8 ) - 1; bitIndex >= 0; --bitIndex ) {
            if( ( ( int )materialIndex >> bitIndex ) > 0 ) {
                return bitIndex;
            }
        }

        throw new Exception( "Unable to calculate sub mesh index" );
    }

    // not topologically safe and i'm not gonna bother to fix it
    private Octree<Voxel> simplify( Octree<Voxel> node, float errorThreshold, IEnumerable<DensityFunction> densityFunctions ) {
        var voxel = node.data;

        if( voxel.type != Voxel.Type.Internal ) {
            return node;
        }

        var intersectionPlanes = new List<Plane>( );

        var collapsible = true;
        for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
            node.children[childIndex] = this.simplify( node.children[childIndex], errorThreshold, densityFunctions );

            var child = node.children[childIndex].data;
            if( child.type == Voxel.Type.Internal ) {
                collapsible = false;
            }
            else {
                foreach( var edge in child.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = this.approximateIntersection ( edge,              densityFunctions );
                    edge.normal       = this.calculateNormal         ( edge.intersection, densityFunctions );

                    intersectionPlanes.Add( new( edge.normal, edge.intersection ) );
                }
            }
        }

        if( !collapsible ) {
            return node;
        }

        // calculate minimizing vertex
        // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
        // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
        var minimizingVertex = voxel.center;

        for( var minimizingIteration = 0; minimizingIteration < this.minimizerIterations; ++minimizingIteration ) {
            minimizingVertex -= intersectionPlanes.Aggregate(
                Vector3.zero,
                ( accumulator, plane ) => {
                    accumulator += plane.GetDistanceToPoint( minimizingVertex ) * plane.normal;
                    return accumulator;
                }
            ) / intersectionPlanes.Count;
        }

        // calculate surface normal
        var surfaceNormal = Vector3.Normalize(
            intersectionPlanes.Aggregate(
                Vector3.zero,
                ( accumulator, plane ) => {
                    accumulator += plane.normal;
                    return accumulator;
                }
            ) / intersectionPlanes.Count
        );

        // error value is simply how far the minimizing vertex is from the surface before correction
        var error = Mathf.Abs( SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions ) );

        // correct surface by forcing the minimizing vertex towards the zero crossing
        // see https://www.reddit.com/r/VoxelGameDev/comments/mhiec0/how_are_people_getting_good_results_with_dual/gti0b8d/
        for( var surfaceCorrectionIteration = 0; surfaceCorrectionIteration < this.surfaceCorrectionIterations; ++surfaceCorrectionIteration ) {
            var density = SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions );
            if( density == 0.0f ) {
                // vertex is at the surface
                break;
            }
            minimizingVertex -= surfaceNormal * density;
        }

        if( error > errorThreshold ) {
            return node;
        }

        voxel.type   = Voxel.Type.Pseudo;
        voxel.vertex = minimizingVertex;
        voxel.normal = surfaceNormal;

        node.children = null;

        return node;
    }

}
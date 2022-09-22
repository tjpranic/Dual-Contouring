using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

using Axis     = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Axis;
using Position = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Position;

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

    public class Voxel : SurfaceExtractor.Voxel {

        public SurfaceExtractor.Voxel.Type type    { get; set; }
        public Vector3                     center  { get; }
        public Vector3                     size    { get; }
        public Vector3                     extents { get; }
        public Vector3                     minimum { get; }
        public Vector3                     maximum { get; }
        public SurfaceExtractor.Corner[]   corners { get; }
        public SurfaceExtractor.Edge[]     edges   { get; }

        public int       depth  { get; set; }
        public QEFSolver qef    { get; set; }
        public Vector3   vertex { get; set; } = Vector3.zero;
        public Vector3   normal { get; set; } = Vector3.zero;
        public float     error  { get; set; } = 0.0f;
        public int       index  { get; set; } = -1;

        public Voxel( SurfaceExtractor.Voxel.Type type, int depth, Vector3 center, Vector3 size ) {
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
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[1] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[3], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[6] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[4], this.corners[7] } ),
                // y axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[5] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[1], this.corners[6] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[3], this.corners[4] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[2], this.corners[7] } ),
                // z axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[3] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[1], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[4] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[6], this.corners[7] } )
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
                ( node ) => node.type != SurfaceExtractor.Voxel.Type.Internal
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
                ( node ) => node.type != SurfaceExtractor.Voxel.Type.Internal
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
                ( node ) => node.type != SurfaceExtractor.Voxel.Type.Internal
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

    public enum SolverType {
        Simple,
        SVD
    }

    public SolverType solverType = SolverType.Simple;

    public override Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions ) {

        // build octree with depth equal to resolution

        this.octree = Octree<Voxel>.build(
            new( SurfaceExtractor.Voxel.Type.Internal, 0, Vector3.zero, Vector3.one ),
            ( parent ) => {
                if( parent.depth == ( resolution - 1 ) ) {
                    return null;
                }

                var depth = parent.depth + 1;
                var type  = depth < ( resolution - 1 ) ? SurfaceExtractor.Voxel.Type.Internal : SurfaceExtractor.Voxel.Type.Leaf;
                var scale = Mathf.Pow( 2, depth + 1 );
                var size  = Vector3.one / Mathf.Pow( 2, depth );

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
                return new Voxel[8] {
                    // lower child voxels
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f,  1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f,  1.0f ) / scale ), size ),
                    // upper child voxels
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
                        ( corner.density, corner.materialIndex ) = SurfaceExtractor.calculateDensity( corner, densityFunction );
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

                voxel.qef = solverType switch {
                    SolverType.Simple => new SimpleQEF ( this.minimizerIterations, this.surfaceCorrectionIterations ),
                    SolverType.SVD    => new SVDQEF    ( this.minimizerIterations, this.surfaceCorrectionIterations ),
                    _                 => throw new Exception( "Unknown solver type specified" )
                };

                foreach( var edge in voxel.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = this.approximateIntersection ( edge,              densityFunctions );
                    edge.normal       = this.calculateNormal         ( edge.intersection, densityFunctions );

                    voxel.qef.add( edge.intersection, edge.normal );
                }

                ( voxel.vertex, voxel.normal, voxel.error ) = voxel.qef.solve( voxel, densityFunctions );
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

        this.contourCell( this.octree, Position.Root, vertices, normals, indices );

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

    private void contourCell( Octree<Voxel> node, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        if( node.data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            // contour cells in children

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                this.contourCell( node.children[childIndex], position /*( Position )childIndex*/, vertices, normals, indices );
            }

            // contour common face pairs in children

            // x axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.X, position ) ) {
                this.contourFace( facePair, Axis.X, position, vertices, normals, indices );
            }

            // y axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Y, position ) ) {
                this.contourFace( facePair, Axis.Y, position, vertices, normals, indices );
            }

            // z axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Z, position ) ) {
                this.contourFace( facePair, Axis.Z, position, vertices, normals, indices );
            }

            // contour common edges of children

            // x axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.X, position ) ) {
                this.contourEdge( edgeNodes, Axis.X, position, vertices, normals, indices );
            }

            // y axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Y, position ) ) {
                this.contourEdge( edgeNodes, Axis.Y, position, vertices, normals, indices );
            }

            // z axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Z, position ) ) {
                this.contourEdge( edgeNodes, Axis.Z, position, vertices, normals, indices );
            }
        }
    }

    private void contourFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == SurfaceExtractor.Voxel.Type.Internal || nodes[1].data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            // contour common face pairs in children of given face pairs

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinFacePairs( nodes, axis, position ) ) {
                this.contourFace( facePair, axis, position, vertices, normals, indices );
            }

            // contour common edges in children of given face pairs

            foreach( var ( edgeNodes, newAxis ) in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinFacePairs( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, newAxis, position, vertices, normals, indices );
            }
        }
    }

    private void contourEdge( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {
        UnityEngine.Debug.Assert( nodes.Length == 4 );

        if(
            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal &&
            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal &&
            nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal &&
            nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal
        ) {
            this.generateIndices( nodes, axis, position, vertices, normals, indices );
        }
        else {

            // contour common edges in children of given voxels

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinEdgeNodes( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, axis, position, vertices, normals, indices );
            }
        }
    }

    private void generateIndices( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, List<int>> indices ) {

        var edge = OctreeContouringTables<Voxel>.lookupEdgeWithinEdgeNodes( nodes, axis, position );

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
                    nodes[3].data.index,
                    nodes[2].data.index,
                    nodes[1].data.index
                };
            }
            else {
                triangles = new int[] {
                    nodes[1].data.index,
                    nodes[2].data.index,
                    nodes[3].data.index,
                    nodes[2].data.index,
                    nodes[1].data.index,
                    nodes[0].data.index
                };
            }

            var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( edge );
            if( !indices.ContainsKey( subMeshIndex ) ) {
                indices.Add( subMeshIndex, new( ) );
            }
            indices[subMeshIndex].AddRange( triangles );
        }
    }

    // not topologically safe and i'm not gonna bother to fix it
    private Octree<Voxel> simplify( Octree<Voxel> node, float errorThreshold, IEnumerable<DensityFunction> densityFunctions ) {
        var voxel = node.data;

        if( voxel.type != SurfaceExtractor.Voxel.Type.Internal ) {
            return node;
        }

        voxel.qef = solverType switch {
            SolverType.Simple => new SimpleQEF ( this.minimizerIterations, this.surfaceCorrectionIterations ),
            SolverType.SVD    => new SVDQEF    ( this.minimizerIterations, this.surfaceCorrectionIterations ),
            _                 => throw new Exception( "Unknown solver type specified" )
        };

        var collapsible = true;
        for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
            node.children[childIndex] = this.simplify( node.children[childIndex], errorThreshold, densityFunctions );

            var child = node.children[childIndex].data;
            if( child.type == SurfaceExtractor.Voxel.Type.Internal ) {
                collapsible = false;
            }
            else if( child.qef != null ) {
                voxel.qef.combine( child.qef );
            }
        }

        if( !collapsible ) {
            return node;
        }

        if( voxel.qef.empty ) {
            return node;
        }

        var ( minimizingVertex, surfaceNormal, error ) = voxel.qef.solve( voxel, densityFunctions );

        if( error > errorThreshold ) {
            return node;
        }

        voxel.type   = SurfaceExtractor.Voxel.Type.Pseudo;
        voxel.vertex = minimizingVertex;
        voxel.normal = surfaceNormal;

        node.children = null;

        return node;
    }

}
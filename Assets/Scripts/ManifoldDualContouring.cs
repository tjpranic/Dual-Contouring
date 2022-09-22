using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// TODO: separate vertex tree from the octree
// TODO: implement manifold criterion
// TODO: implement support for multiple vertices per voxel

using Axis     = OctreeContouringTables<ManifoldDualContouring.Voxel>.Axis;
using Position = OctreeContouringTables<ManifoldDualContouring.Voxel>.Position;

public class ManifoldDualContouring : Voxelizer {

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

        public int       depth       { get; set; }
        public QEFSolver qef         { get; set; }
        public Vector3   vertex      { get; set; } = Vector3.zero;
        public Vector3   normal      { get; set; } = Vector3.zero;
        public float     error       { get; set; } = 0.0f;
        public Voxel     parent      { get; set; } = null;
        public bool      collapsible { get; set; } = true;
        public int       index       { get; set; } = -1;

        public Voxel( SurfaceExtractor.Voxel.Type type, int depth, Vector3 center, Vector3 size ) {
            this.type    = type;
            this.depth   = depth;
            this.center  = center;
            this.size    = size;
            this.extents = size / 2;
            this.minimum = this.center - this.extents;
            this.maximum = this.center + this.extents;

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

    public class Triangle : IEquatable<Triangle> {

        public int[] indices = new int[3];

        public Triangle( int[] indices ) {
            this.indices = indices;
        }

        public Triangle( int a, int b, int c ) {
            this.indices[0] = a;
            this.indices[1] = b;
            this.indices[2] = c;
        }

        public bool Equals( Triangle other ) {
            return this.indices[0] == other.indices[0] && this.indices[1] == other.indices[1] && this.indices[2] == other.indices[2];
        }

        public override int GetHashCode( ) {
            return this.cantor( this.indices[0], this.cantor( this.indices[1], this.indices[2] ) );
        }

        private int cantor( int a, int b ) {
           return ( ( a + b + 1 ) * ( a + b ) / 2 ) + b;
        }

    }

    public override IEnumerable<SurfaceExtractor.Corner> corners {
        get {
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == SurfaceExtractor.Voxel.Type.Leaf
            ).Aggregate(
                new List<SurfaceExtractor.Corner>( ),
                ( accumulator, voxel ) => {
                    Voxel highest = voxel;

                    var parent = voxel.parent;
                    while( parent != null ) {
                        if( parent.collapsible ) {
                            highest = parent;
                        }
                        parent = parent.parent;
                    }

                    UnityEngine.Debug.Assert( highest != null );

                    accumulator.AddRange( highest.corners );
                    return accumulator;
                }
            ).Distinct( );
        }
    }

    public override IEnumerable<SurfaceExtractor.Edge> edges {
        get {
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == SurfaceExtractor.Voxel.Type.Leaf
            ).Aggregate(
                new List<SurfaceExtractor.Edge>( ),
                ( accumulator, voxel ) => {
                    Voxel highest = voxel;

                    var parent = voxel.parent;
                    while( parent != null ) {
                        if( parent.collapsible ) {
                            highest = parent;
                        }
                        parent = parent.parent;
                    }

                    UnityEngine.Debug.Assert( highest != null );

                    accumulator.AddRange( highest.edges );
                    return accumulator;
                }
            ).Distinct( );
        }
    }

    public override IEnumerable<SurfaceExtractor.Voxel> voxels {
        get {
            return Octree<Voxel>.flatten( this.octree ).Where(
                ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == SurfaceExtractor.Voxel.Type.Leaf
            ).Select(
                ( voxel ) => {
                    Voxel highest = voxel;

                    var parent = voxel.parent;
                    while( parent != null ) {
                        if( parent.collapsible ) {
                            highest = parent;
                        }
                        parent = parent.parent;
                    }

                    UnityEngine.Debug.Assert( highest != null );

                    return highest;
                }
            ).Distinct( );
        }
    }

    private Octree<Voxel> octree;

    [Space( )]

    [Min( 0.0f )]
    public float errorThreshold = 6e-12f;

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

                return new Voxel[8] {
                    // bottom 4 child voxels
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f, -1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3(  1.0f, -1.0f,  1.0f ) / scale ), size ),
                    new( type, depth, parent.center + ( new Vector3( -1.0f, -1.0f,  1.0f ) / scale ), size ),
                    // top 4 child voxels
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

                if( voxel.type == SurfaceExtractor.Voxel.Type.Internal ) {
                    // only process leaf nodes
                    return;
                }

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

        // generate vertex tree

        this.clusterCell( this.octree, Position.Root, densityFunctions );

        if( UnityEngine.Debug.isDebugBuild ) {
            // verify the validity of the vertex clustering
            Octree<Voxel>.walk(
                this.octree,
                ( node ) => {
                    if( node.data.hasFeaturePoint( ) && node.data.type == SurfaceExtractor.Voxel.Type.Internal ) {
                        // every node other than the root should have a parent
                        UnityEngine.Debug.Assert( node.data.depth == 0 || node.data.parent != null, "Non-root octree cluster node missing a parent" );

                        var valid = true;
                        foreach( var child in node.children ) {
                            if( child.data.parent != node.data ) {
                                valid = false;
                                break;
                            }
                        }
                        // ensure the parent of the child nodes point to the enclosing voxel
                        UnityEngine.Debug.Assert( valid, "Invalid octree cluster detected" );
                    }
                }
            );
        }

        // mark voxels as collapsible

        Octree<Voxel>.climb(
            this.octree,
            ( node ) => {
                if( node.data.type == SurfaceExtractor.Voxel.Type.Internal ) {
                    var collapsible = node.data.error < this.errorThreshold;
                    if( collapsible ) {
                        foreach( var child in node.children ) {
                            if( child.data.type == SurfaceExtractor.Voxel.Type.Internal && !child.data.collapsible ) {
                                collapsible = false;
                                break;
                            }
                        }
                    }
                    node.data.collapsible = collapsible;
                }
            }
        );

        // generate vertices and indices

        var vertices = new List<Vector3>( );
        var normals  = new List<Vector3>( );
        var indices  = new Dictionary<int, HashSet<Triangle>>( );

        this.contourCell( this.octree, Position.Root, vertices, normals, indices );

        var mesh = new Mesh {
            vertices = vertices.ToArray( ),
            normals  = normals.ToArray( )
        };

        mesh.subMeshCount = indices.Keys.Count;

        var subMeshCount = 0;
        foreach( var triangles in indices ) {
            mesh.SetTriangles(
                triangles.Value.Aggregate(
                    new List<int>( triangles.Value.Count * 3 ),
                    ( accumulator, triangle ) => {
                        accumulator.AddRange( triangle.indices );
                        return accumulator;
                    }
                ),
                subMeshCount
            );
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
            for( var binarySearchIteration = 0; binarySearchIteration < this.binarySearchIterations; ++binarySearchIteration ) {
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

    private void clusterCell( Octree<Voxel> node, Position position, IEnumerable<DensityFunction> densityFunctions ) {
        var cluster = new List<Voxel>( );

        if( node.data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            // contour cells in children

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                this.clusterCell( node.children[childIndex], position /*( Position )childIndex*/, densityFunctions );
            }

            // contour common face pairs in children

            // x axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.X, position ) ) {
                this.clusterFace( facePair, Axis.X, position, cluster );
            }

            // y axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Y, position ) ) {
                this.clusterFace( facePair, Axis.Y, position, cluster );
            }

            // z axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Z, position ) ) {
                this.clusterFace( facePair, Axis.Z, position, cluster );
            }

            // contour common edges of children

            // x axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.X, position ) ) {
                this.clusterEdge( edgeNodes, Axis.X, position, cluster );
            }

            // y axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Y, position ) ) {
                this.clusterEdge( edgeNodes, Axis.Y, position, cluster );
            }

            // z axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Z, position ) ) {
                this.clusterEdge( edgeNodes, Axis.Z, position, cluster );
            }

            // grab voxels of direct children
            foreach( var child in node.children ) {
                cluster.Add( child.data );
            }
        }

        if( cluster.Count == 0 ) {
            return;
        }

        // solve QEF for vertex cluster

        node.data.qef = solverType switch {
            SolverType.Simple => new SimpleQEF ( this.minimizerIterations, this.surfaceCorrectionIterations ),
            SolverType.SVD    => new SVDQEF    ( this.minimizerIterations, this.surfaceCorrectionIterations ),
            _                 => throw new Exception( "Unknown solver type specified" )
        };

        foreach( var child in cluster ) {
            if( child.qef != null ) {
                node.data.qef.combine( child.qef );
            }
        }

        if( node.data.qef.empty ) {
            return;
        }

        ( node.data.vertex, node.data.normal, node.data.error ) = node.data.qef.solve( node.data, densityFunctions );

        foreach( var child in cluster ) {
            child.parent = node.data;
        }
    }

    private void clusterFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Voxel> cluster ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == SurfaceExtractor.Voxel.Type.Internal || nodes[1].data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            // contour common face pairs in children of given face pairs

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinFacePairs( nodes, axis, position ) ) {
                this.clusterFace( facePair, axis, position, cluster );
            }

            // contour common edges in children of given face pairs

            foreach( var ( edgeNodes, newAxis ) in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinFacePairs( nodes, axis, position ) ) {
                this.clusterEdge( edgeNodes, newAxis, position, cluster );
            }
        }
    }

    private void clusterEdge( Octree<Voxel>[] nodes, Axis axis, Position position, List<Voxel> cluster ) {
        UnityEngine.Debug.Assert( nodes.Length == 4 );

        if(
            nodes[0].data.type == SurfaceExtractor.Voxel.Type.Internal ||
            nodes[1].data.type == SurfaceExtractor.Voxel.Type.Internal ||
            nodes[2].data.type == SurfaceExtractor.Voxel.Type.Internal ||
            nodes[3].data.type == SurfaceExtractor.Voxel.Type.Internal
        ) {

            // contour common edges in children of given voxels

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinEdgeNodes( nodes, axis, position ) ) {
                this.clusterEdge( edgeNodes, axis, position, cluster );
            }
        }

        // surface indices would be assigned down here, if I allowed more than 1 vertex per cell :^)
    }

    private void contourCell( Octree<Voxel> node, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, HashSet<Triangle>> indices ) {
        if( node.data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            if( node.data.collapsible ) {
                return;
            }

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

    private void contourFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, HashSet<Triangle>> indices ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == SurfaceExtractor.Voxel.Type.Internal || nodes[1].data.type == SurfaceExtractor.Voxel.Type.Internal ) {

            if( nodes[0].data.collapsible && nodes[1].data.collapsible ) {
                return;
            }

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

    private void contourEdge( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, HashSet<Triangle>> indices ) {
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

    private void generateIndices( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, Dictionary<int, HashSet<Triangle>> indices ) {

        var edge = OctreeContouringTables<Voxel>.lookupEdgeWithinEdgeNodes( nodes, axis, position );

        if( nodes.All( ( node ) => node.data.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {

            var voxels = new Voxel[4] {
                nodes[0].data,
                nodes[1].data,
                nodes[2].data,
                nodes[3].data
            };

            // find the collapsible node furthest up the tree for each voxel in the cluster
            for( var nodeIndex = 0; nodeIndex < nodes.Length; ++nodeIndex ) {
                var parent = nodes[nodeIndex].data.parent;
                while( parent != null ) {
                    if( parent.collapsible ) {
                        voxels[nodeIndex] = parent;
                    }
                    parent = parent.parent;
                }
            }

            // generate vertex and normal
            foreach( var voxel in voxels ) {
                if( voxel.index == -1 ) {
                    voxel.index = vertices.Count;

                    vertices.Add ( voxel.vertex );
                    normals.Add  ( voxel.normal );
                }
            }

            // generate indices

            var triangles = new Triangle[2];

            // ensure quad is indexed facing outward and reject polygons that were collapsed to an edge or point
            if( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void ) {
                if( voxels[0].index != voxels[1].index && voxels[1].index != voxels[2].index ) {
                    triangles[0] = new Triangle(
                        voxels[0].index,
                        voxels[1].index,
                        voxels[2].index
                    );
                }
                if( voxels[3].index != voxels[2].index && voxels[2].index != voxels[1].index ) {
                    triangles[1] = new Triangle(
                        voxels[3].index,
                        voxels[2].index,
                        voxels[1].index
                    );
                }
            }
            else {
                if( voxels[1].index != voxels[2].index && voxels[2].index != voxels[3].index ) {
                    triangles[0] = new Triangle(
                        voxels[1].index,
                        voxels[2].index,
                        voxels[3].index
                    );
                }
                if( voxels[2].index != voxels[1].index && voxels[1].index != voxels[0].index ) {
                    triangles[1] = new Triangle(
                        voxels[2].index,
                        voxels[1].index,
                        voxels[0].index
                    );
                }
            }

            var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( edge );
            if( !indices.ContainsKey( subMeshIndex ) ) {
                indices.Add( subMeshIndex, new( ) );
            }

            if( triangles[0] != null ) {
                var _ = indices[subMeshIndex].Add( triangles[0] );
            }
            if( triangles[1] != null ) {
                var _ = indices[subMeshIndex].Add( triangles[1] );
            }
        }
    }

}
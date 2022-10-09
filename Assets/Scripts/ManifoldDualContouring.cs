using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

// TODO: implement manifold criterion
// TODO: implement support for multiple vertices per voxel

using Axis     = OctreeContouringTables<ManifoldDualContouring.Voxel>.Axis;
using Position = OctreeContouringTables<ManifoldDualContouring.Voxel>.Position;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using ImplementationType        = SurfaceExtractor.Implementation.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

public class ManifoldDualContouring : Voxelizer {

    public class Corner : SurfaceExtractor.Corner {

        public Vector3                        position      { get; }
        public float                          density       { get; set; } = float.MaxValue;
        public MaterialIndex materialIndex { get; set; } = MaterialIndex.Void;

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
                this.corners[0].materialIndex == MaterialIndex.Void && this.corners[1].materialIndex >= MaterialIndex.Material0
            ) || (
                this.corners[1].materialIndex == MaterialIndex.Void && this.corners[0].materialIndex >= MaterialIndex.Material0
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

        public VoxelType                 type    { get; set; }
        public Vector3                   center  { get; }
        public Vector3                   size    { get; }
        public Vector3                   extents { get; }
        public Vector3                   minimum { get; }
        public Vector3                   maximum { get; }
        public SurfaceExtractor.Corner[] corners { get; }
        public SurfaceExtractor.Edge[]   edges   { get; }

        public int       depth       { get; set; }
        public QEFSolver qef         { get; set; }
        public Vector3   vertex      { get; set; } = Vector3.zero;
        public Vector3   normal      { get; set; } = Vector3.zero;
        public float     error       { get; set; } = 0.0f;
        public Voxel     parent      { get; set; } = null;
        public bool      collapsible { get; set; } = true;
        public int       index       { get; set; } = -1;

        public Voxel( VoxelType type, int depth, Vector3 center, Vector3 size ) {
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

        public int[] indices      = new int[3];
        public int   subMeshIndex = -1;

        public Triangle( int[] indices, int subMeshIndex ) {
            this.indices      = indices;
            this.subMeshIndex = subMeshIndex;
        }

        public Triangle( int a, int b, int c, int subMeshIndex ) {
            this.indices[0] = a;
            this.indices[1] = b;
            this.indices[2] = c;

            this.subMeshIndex = subMeshIndex;
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

    [Space( )]

    [Min( 0.0f )]
    public float errorThreshold = 6e-12f;

    private Octree<Voxel> octree;

    protected override Implementation implementation {
        get { throw new NotImplementedException( ); }
        set { throw new NotImplementedException( ); }
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {

        // build octree with depth equal to resolution

        this.octree = Octree<Voxel>.build(
            new( VoxelType.Internal, 0, Vector3.zero, Vector3.one ),
            ( parent ) => {
                if( parent.depth == ( this.resolution - 1 ) ) {
                    return null;
                }

                var depth = parent.depth + 1;
                var type  = depth < ( this.resolution - 1 ) ? VoxelType.Internal : VoxelType.Leaf;
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
                        corner.density       = SurfaceExtractor.calculateDensity  ( corner.position, densityFunctions );
                        corner.materialIndex = SurfaceExtractor.calculateMaterial ( corner.position, densityFunctions );
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
                    voxel.corners.All( ( corner ) => corner.materialIndex == MaterialIndex.Void      ) ||
                    voxel.corners.All( ( corner ) => corner.materialIndex >= MaterialIndex.Material0 )
                ) {
                    // cell is either fully inside or outside the volume, skip
                    return;
                }

                voxel.qef = this.qefSolverType switch {
                    QEFSolverType.Simple => new SimpleQEF ( this.minimizerIterations ),
                    QEFSolverType.SVD    => new SVDQEF    ( this.minimizerIterations ),
                    _                    => throw new Exception( "Unknown solver type specified" )
                };

                var averageNormal = Vector3.zero;

                foreach( var edge in voxel.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = SurfaceExtractor.approximateIntersection ( edge, densityFunctions, this.intersectionApproximation, this.binarySearchIterations );
                    edge.normal       = SurfaceExtractor.calculateNormal         ( edge, densityFunctions );

                    voxel.qef.add( edge.intersection, edge.normal );

                    averageNormal += edge.normal;
                }

                if( voxel.qef.intersectionCount > 0 ) {
                    ( voxel.vertex, voxel.error ) = voxel.qef.solve( voxel, densityFunctions );

                    voxel.normal = Vector3.Normalize( averageNormal / voxel.qef.intersectionCount );

                    voxel.vertex = QEFSolver.surfaceCorrection( voxel.vertex, voxel.normal, densityFunctions, this.surfaceCorrectionIterations );
                }
            }
        );

        // generate vertex tree

        this.clusterCell( this.octree, Position.Root, densityFunctions );

        if( UnityEngine.Debug.isDebugBuild ) {
            // verify the validity of the vertex clustering
            Octree<Voxel>.walk(
                this.octree,
                ( node ) => {
                    if( node.data.hasFeaturePoint( ) && node.data.type == VoxelType.Internal ) {
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
                if( node.data.type == VoxelType.Internal ) {
                    var collapsible = node.data.error < this.errorThreshold;
                    if( collapsible ) {
                        foreach( var child in node.children ) {
                            if( child.data.type == VoxelType.Internal && !child.data.collapsible ) {
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

        var vertices  = new List<Vector3>( );
        var normals   = new List<Vector3>( );
        var triangles = new List<Triangle>( );

        this.contourCell( this.octree, Position.Root, vertices, normals, triangles );

        // build mesh
        var mesh = new Mesh {
            vertices = vertices.ToArray( ),
            normals  = normals.ToArray( )
        };

        var subMeshIndices = new Dictionary<int, List<int>>( );

        foreach( var triangle in triangles.Distinct( ) ) {
            var subMeshIndex = triangle.subMeshIndex;
            if( !subMeshIndices.ContainsKey( subMeshIndex ) ) {
                subMeshIndices.Add( subMeshIndex, new( ) );
            }
            subMeshIndices[subMeshIndex].AddRange( triangle.indices );
        }

        mesh.subMeshCount = subMeshIndices.Keys.Count;

        var subMeshCount = 0;
        foreach( var indices in subMeshIndices ) {
            mesh.SetTriangles( indices.Value, subMeshCount );
            ++subMeshCount;
        }

        // build debug information
        var voxels = Octree<Voxel>.flatten( this.octree ).Where(
            ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == VoxelType.Leaf
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

        var corners = Octree<Voxel>.flatten( this.octree ).Where(
            ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == VoxelType.Leaf
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

        var edges = Octree<Voxel>.flatten( this.octree ).Where(
            ( voxel ) => voxel.hasFeaturePoint( ) && voxel.type == VoxelType.Leaf
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

        return new Either<CPUVoxelization, GPUVoxelization>.Type0( new( ) {
            mesh    = mesh,
            corners = corners,
            edges   = edges,
            voxels  = voxels
        } );
    }

    private void clusterCell( Octree<Voxel> node, Position position, IEnumerable<DensityFunction> densityFunctions ) {
        var voxel = node.data;

        var cluster = new List<Voxel>( );

        if( voxel.type == VoxelType.Internal ) {

            // cluster cells in children

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                this.clusterCell( node.children[childIndex], position /*( Position )childIndex*/, densityFunctions );
            }

            // cluster common face pairs in children

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

            // cluster common edges of children

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

        voxel.qef = this.qefSolverType switch {
            QEFSolverType.Simple => new SimpleQEF ( this.minimizerIterations ),
            QEFSolverType.SVD    => new SVDQEF    ( this.minimizerIterations ),
            _                    => throw new Exception( "Unknown solver type specified" )
        };

        var averageNormal = Vector3.zero;

        foreach( var child in cluster ) {
            if( child.qef != null ) {
                voxel.qef.combine( child.qef );

                averageNormal += child.normal;
            }
        }

        if( voxel.qef.intersectionCount == 0 ) {
            return;
        }

        ( voxel.vertex, voxel.error ) = voxel.qef.solve( node.data, densityFunctions );

        voxel.normal = Vector3.Normalize( averageNormal );

        foreach( var child in cluster ) {
            child.parent = voxel;
        }
    }

    private void clusterFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Voxel> cluster ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == VoxelType.Internal || nodes[1].data.type == VoxelType.Internal ) {

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
            nodes[0].data.type == VoxelType.Internal ||
            nodes[1].data.type == VoxelType.Internal ||
            nodes[2].data.type == VoxelType.Internal ||
            nodes[3].data.type == VoxelType.Internal
        ) {

            // contour common edges in children of given voxels

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinEdgeNodes( nodes, axis, position ) ) {
                this.clusterEdge( edgeNodes, axis, position, cluster );
            }
        }

        // surface indices would be assigned down here, if I allowed more than 1 vertex per cell :^)
    }

    private void contourCell( Octree<Voxel> node, Position position, List<Vector3> vertices, List<Vector3> normals, List<Triangle> triangles ) {
        if( node.data.type == VoxelType.Internal ) {

            if( node.data.collapsible ) {
                return;
            }

            // contour cells in children

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                this.contourCell( node.children[childIndex], position /*( Position )childIndex*/, vertices, normals, triangles );
            }

            // contour common face pairs in children

            // x axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.X, position ) ) {
                this.contourFace( facePair, Axis.X, position, vertices, normals, triangles );
            }

            // y axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Y, position ) ) {
                this.contourFace( facePair, Axis.Y, position, vertices, normals, triangles );
            }

            // z axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Z, position ) ) {
                this.contourFace( facePair, Axis.Z, position, vertices, normals, triangles );
            }

            // contour common edges of children

            // x axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.X, position ) ) {
                this.contourEdge( edgeNodes, Axis.X, position, vertices, normals, triangles );
            }

            // y axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Y, position ) ) {
                this.contourEdge( edgeNodes, Axis.Y, position, vertices, normals, triangles );
            }

            // z axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Z, position ) ) {
                this.contourEdge( edgeNodes, Axis.Z, position, vertices, normals, triangles );
            }
        }
    }

    private void contourFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Triangle> triangles ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == VoxelType.Internal || nodes[1].data.type == VoxelType.Internal ) {

            if( nodes[0].data.collapsible && nodes[1].data.collapsible ) {
                return;
            }

            // contour common face pairs in children of given face pairs

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinFacePairs( nodes, axis, position ) ) {
                this.contourFace( facePair, axis, position, vertices, normals, triangles );
            }

            // contour common edges in children of given face pairs

            foreach( var ( edgeNodes, newAxis ) in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinFacePairs( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, newAxis, position, vertices, normals, triangles );
            }
        }
    }

    private void contourEdge( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Triangle> triangles ) {
        UnityEngine.Debug.Assert( nodes.Length == 4 );

        if(
            nodes[0].data.type != VoxelType.Internal &&
            nodes[1].data.type != VoxelType.Internal &&
            nodes[2].data.type != VoxelType.Internal &&
            nodes[3].data.type != VoxelType.Internal
        ) {
            this.generateIndices( nodes, axis, position, vertices, normals, triangles );
        }
        else {

            // contour common edges in children of given voxels

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinEdgeNodes( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, axis, position, vertices, normals, triangles );
            }
        }
    }

    private void generateIndices( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Triangle> triangles ) {

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

            var materialIndex = edge.corners[0].materialIndex == MaterialIndex.Void
                ? edge.corners[1].materialIndex
                : edge.corners[0].materialIndex;

            var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( materialIndex );

            // ensure quad is indexed facing outward and reject polygons that were collapsed to an edge or point
            if( edge.corners[0].materialIndex == MaterialIndex.Void ) {
                if( voxels[0].index != voxels[1].index && voxels[1].index != voxels[2].index ) {
                    triangles.Add(
                        new(
                            voxels[0].index,
                            voxels[1].index,
                            voxels[2].index,
                            subMeshIndex
                        )
                    );
                }
                if( voxels[3].index != voxels[2].index && voxels[2].index != voxels[1].index ) {
                    triangles.Add(
                        new(
                            voxels[3].index,
                            voxels[2].index,
                            voxels[1].index,
                            subMeshIndex
                        )
                    );
                }
            }
            else {
                if( voxels[1].index != voxels[2].index && voxels[2].index != voxels[3].index ) {
                    triangles.Add(
                        new(
                            voxels[1].index,
                            voxels[2].index,
                            voxels[3].index,
                            subMeshIndex
                        )
                    );
                }
                if( voxels[2].index != voxels[1].index && voxels[1].index != voxels[0].index ) {
                    triangles.Add(
                        new(
                            voxels[2].index,
                            voxels[1].index,
                            voxels[0].index,
                            subMeshIndex
                        )
                    );
                }
            }
        }
    }

    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        throw new NotImplementedException( );
    }

}

[CustomEditor( typeof( ManifoldDualContouring ) )]
public class ManifoldDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );
    }

}
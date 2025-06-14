﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

using Axis     = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Axis;
using Position = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Position;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using ImplementationType        = SurfaceExtractor.Implementation.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

public class AdaptiveDualContouring : Voxelizer {

    public class Corner : SurfaceExtractor.Corner {

        public Vector3       position      { get; }
        public float         density       { get; set; } = float.MaxValue;
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

        public int       depth  { get; set; }
        public QEFSolver qef    { get; set; }
        public Vector3   vertex { get; set; } = Vector3.zero;
        public Vector3   normal { get; set; } = Vector3.zero;
        public float     error  { get; set; } = 0.0f;
        public int       index  { get; set; } = -1;

        public Voxel( VoxelType type, int depth, Vector3 center, Vector3 size ) {
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

    public class Quad {

        public int[] indices      = new int[6];
        public int   subMeshIndex = -1;

        public Quad( int[] indices, int subMeshIndex ) {
            this.indices      = indices;
            this.subMeshIndex = subMeshIndex;
        }

    }

    [Space( )]

    public bool simplification = false;

    [Min( 0.0f )]
    public float errorThreshold = 0.01f;

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

        // simplify octree
        // not topologically safe and i'm not gonna bother to fix it

        if( this.simplification ) {
            UnityEngine.Debug.Assert( this.errorThreshold != 0.0f );

            Octree<Voxel>.climb(
                this.octree,
                ( node ) => {
                    var voxel = node.data;

                    if( voxel.type != VoxelType.Internal ) {
                        return;
                    }

                    voxel.qef = this.qefSolverType switch {
                        QEFSolverType.Simple => new SimpleQEF ( this.minimizerIterations ),
                        QEFSolverType.SVD    => new SVDQEF    ( this.minimizerIterations ),
                        _                    => throw new Exception( "Unknown solver type specified" )
                    };

                    var averageNormal = Vector3.zero;

                    var collapsible = true;
                    foreach( var child in node.children ) {
                        if( child.data.type == VoxelType.Internal ) {
                            collapsible = false;
                        }
                        else if( child.data.qef != null ) {
                            voxel.qef.combine( child.data.qef );

                            averageNormal += child.data.normal;
                        }
                    }

                    if( !collapsible ) {
                        return;
                    }

                    if( voxel.qef.intersectionCount == 0 ) {
                        return;
                    }

                    var ( vertex, error ) = voxel.qef.solve( voxel, densityFunctions );

                    if( error > this.errorThreshold ) {
                        return;
                    }

                    voxel.type   = VoxelType.Pseudo;
                    voxel.vertex = vertex;
                    voxel.normal = Vector3.Normalize( averageNormal );

                    node.children = null;

                }
            );

        }

        // generate vertices and indices

        var vertices = new List<Vector3>( );
        var normals  = new List<Vector3>( );
        var quads    = new List<Quad>( );

        this.contourCell( this.octree, Position.Root, vertices, normals, quads );

        // build mesh
        var mesh = new Mesh {
            vertices = vertices.ToArray( ),
            normals  = normals.ToArray( )
        };

        var subMeshIndices = new Dictionary<int, List<int>>( );

        foreach( var quad in quads ) {
            var subMeshIndex = quad.subMeshIndex;
            if( !subMeshIndices.ContainsKey( subMeshIndex ) ) {
                subMeshIndices.Add( subMeshIndex, new( ) );
            }
            subMeshIndices[subMeshIndex].AddRange( quad.indices );
        }

        mesh.subMeshCount = subMeshIndices.Keys.Count;

        var subMeshCount = 0;
        foreach( var indices in subMeshIndices ) {
            mesh.SetTriangles( indices.Value, subMeshCount );
            ++subMeshCount;
        }

        // build debug information
        var voxels = Octree<Voxel>.flatten( this.octree ).Where(
            ( node ) => node.type != VoxelType.Internal
        );

        var edges = voxels.Aggregate(
            new List<SurfaceExtractor.Edge>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.edges );
                return accumulator;
            }
        ).Distinct( );

        var corners = voxels.Aggregate(
            new List<SurfaceExtractor.Corner>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.corners );
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

    private void contourCell( Octree<Voxel> node, Position position, List<Vector3> vertices, List<Vector3> normals, List<Quad> quads ) {
        if( node.data.type == VoxelType.Internal ) {

            // contour cells in children

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                this.contourCell( node.children[childIndex], position /*( Position )childIndex*/, vertices, normals, quads );
            }

            // contour common face pairs in children

            // x axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.X, position ) ) {
                this.contourFace( facePair, Axis.X, position, vertices, normals, quads );
            }

            // y axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Y, position ) ) {
                this.contourFace( facePair, Axis.Y, position, vertices, normals, quads );
            }

            // z axis faces

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, Axis.Z, position ) ) {
                this.contourFace( facePair, Axis.Z, position, vertices, normals, quads );
            }

            // contour common edges of children

            // x axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.X, position ) ) {
                this.contourEdge( edgeNodes, Axis.X, position, vertices, normals, quads );
            }

            // y axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Y, position ) ) {
                this.contourEdge( edgeNodes, Axis.Y, position, vertices, normals, quads );
            }

            // z axis edges

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinNode( node, Axis.Z, position ) ) {
                this.contourEdge( edgeNodes, Axis.Z, position, vertices, normals, quads );
            }
        }
    }

    private void contourFace( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Quad> quads ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == VoxelType.Internal || nodes[1].data.type == VoxelType.Internal ) {

            // contour common face pairs in children of given face pairs

            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinFacePairs( nodes, axis, position ) ) {
                this.contourFace( facePair, axis, position, vertices, normals, quads );
            }

            // contour common edges in children of given face pairs

            foreach( var ( edgeNodes, newAxis ) in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinFacePairs( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, newAxis, position, vertices, normals, quads );
            }
        }
    }

    private void contourEdge( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Quad> quads ) {
        UnityEngine.Debug.Assert( nodes.Length == 4 );

        if(
            nodes[0].data.type != VoxelType.Internal &&
            nodes[1].data.type != VoxelType.Internal &&
            nodes[2].data.type != VoxelType.Internal &&
            nodes[3].data.type != VoxelType.Internal
        ) {
            this.generateIndices( nodes, axis, position, vertices, normals, quads );
        }
        else {

            // contour common edges in children of given voxels

            foreach( var edgeNodes in OctreeContouringTables<Voxel>.lookupEdgeNodesWithinEdgeNodes( nodes, axis, position ) ) {
                this.contourEdge( edgeNodes, axis, position, vertices, normals, quads );
            }
        }
    }

    private void generateIndices( Octree<Voxel>[] nodes, Axis axis, Position position, List<Vector3> vertices, List<Vector3> normals, List<Quad> quads ) {

        var edge = OctreeContouringTables<Voxel>.lookupEdgeWithinEdgeNodes( nodes, axis, position );

        if( nodes.All( ( node ) => node.data.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {

            var materialIndex = edge.corners[0].materialIndex == MaterialIndex.Void
                ? edge.corners[1].materialIndex
                : edge.corners[0].materialIndex;

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
            if( edge.corners[0].materialIndex == MaterialIndex.Void ) {
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

            var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( materialIndex );

            quads.Add( new( triangles, subMeshIndex ) );
        }
    }

    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        throw new NotImplementedException( );
    }

}

[CustomEditor( typeof( AdaptiveDualContouring ) )]
public class AdaptiveDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var adaptiveDualContouring = this.target as AdaptiveDualContouring;

        // not going to bother implementing GPU solver for adaptive dual contouring
        if( adaptiveDualContouring.implementationType == ImplementationType.GPU ) {
            adaptiveDualContouring.implementationType = ImplementationType.CPU;
        }
    }

}
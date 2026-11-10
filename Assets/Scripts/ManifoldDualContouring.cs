using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

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

        public int          depth        { get; set; }
        public List<Vertex> vertices     { get; }      = new( );                                    // the vertices created in the cell
        public Vertex[]     edgeVertices { get; }      = new Vertex[SurfaceExtractor.Voxel.EdgeCount]; // in a leaf, the vertex each intersected edge is associated with
        public bool         collapsed    { get; set; } = false;                                     // whether contouring can skip the cell, no polygon comes out of it

        // a voxel has room for one vertex and a cell can hold several, which is why each vertex is a voxel of its own.
        // Seen as a voxel the cell shows the first of its vertices
        public Vector3 vertex {
            get { return this.hasFeaturePoint ? this.vertices[0].vertex : Vector3.zero; }
        }
        public Vector3 normal {
            get { return this.hasFeaturePoint ? this.vertices[0].normal : Vector3.zero; }
        }
        public bool hasFeaturePoint {
            get { return this.vertices.Count > 0; }
        }

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

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.center && this.size == other.size;
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    // a vertex of the contour. A leaf holds one for every piece of the contour that passes through it (section III),
    // and a cell above the leaves one for every group of vertices in its children that the contour connects (section
    // IV.A). The vertices form trees of their own next to the octree, each one points at the vertex it was clustered
    // into. A vertex is also a voxel, the cell it was created in with just that vertex in it, which is how the debug
    // visualization gets to see it
    public class Vertex : SurfaceExtractor.Voxel {

        public Voxel voxel { get; } // the cell the vertex was created in

        public QEFSolver qef           { get; set; }
        public Vector3   vertex        { get; set; } = Vector3.zero;
        public Vector3   normal        { get; set; } = Vector3.zero;
        public float     error         { get; set; } = 0.0f;
        public Vertex    parent        { get; set; } = null;
        public bool      collapsible   { get; set; } = true; // whether the vertex may stand in for the vertices clustered below it
        public int       index         { get; set; } = -1;
        public int[]     intersections { get; set; } = new int[SurfaceExtractor.Voxel.EdgeCount]; // number of intersections of its part of the contour with each edge of its cell
        public int       euler         { get; set; } = 0;    // euler characteristic of its part of the contour, manifold condition 1
        public bool      face          { get; set; } = true; // whether that part intersects the edges of every face of its cell 0 or 2 times, manifold condition 2

        public VoxelType type {
            get { return this.voxel.type;  }
            set { this.voxel.type = value; }
        }
        public Vector3 center {
            get { return this.voxel.center; }
        }
        public Vector3 size {
            get { return this.voxel.size; }
        }
        public Vector3 extents {
            get { return this.voxel.extents; }
        }
        public Vector3 minimum {
            get { return this.voxel.minimum; }
        }
        public Vector3 maximum {
            get { return this.voxel.maximum; }
        }
        public SurfaceExtractor.Corner[] corners {
            get { return this.voxel.corners; }
        }
        public SurfaceExtractor.Edge[] edges {
            get { return this.voxel.edges; }
        }
        public bool hasFeaturePoint {
            get { return true; }
        }

        public Vertex( Voxel voxel ) {
            this.voxel = voxel;
        }

        // the vertices of a cell share everything but themselves
        public bool Equals( SurfaceExtractor.Voxel other ) {
            return ReferenceEquals( this, other );
        }

        // the vertex at the top of the vertex tree this one is part of
        public Vertex findRoot( ) {
            var root = this;

            while( root.parent != null ) {
                root = root.parent;
            }

            return root;
        }

        // the vertex that takes the place of this one on the contour: the last collapsible one on the way up the
        // vertex tree (section IV.B)
        public Vertex findContourVertex( ) {
            var highest = this;

            var parent = this.parent;
            while( parent != null ) {
                if( parent.collapsible ) {
                    highest = parent;
                }
                parent = parent.parent;
            }

            return highest;
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

    public bool enforceManifold = true;

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

        // find contour intersections and calculate a minimizing vertex for each piece of the contour in a leaf

        Octree<Voxel>.walk(
            this.octree,
            ( node ) => {
                var voxel = node.data;

                if( voxel.type != VoxelType.Leaf ) {
                    return;
                }

                foreach( var edge in voxel.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = SurfaceExtractor.approximateIntersection ( edge, densityFunctions, this.intersectionApproximation, this.binarySearchIterations );
                    edge.normal       = SurfaceExtractor.calculateNormal         ( edge, densityFunctions );
                }

                var materialIndices = voxel.corners.Select( ( corner ) => corner.materialIndex ).ToArray( );

                var vertexEdges = DualMarchingCubes.lookupVertexEdges( materialIndices );

                if( vertexEdges.Length == 0 ) {
                    // the contour does not pass through the cell
                    return;
                }

                // the boundaries between materials that pass through the cell

                var boundaries = voxel.edges.Where( SurfaceExtractor.crossesMaterialBoundary ).ToList( );

                foreach( var boundary in boundaries ) {
                    ( boundary.intersection, boundary.normal ) = SurfaceExtractor.calculateMaterialBoundary( boundary, densityFunctions, this.intersectionApproximation, this.binarySearchIterations );
                }

                // whether one of the given edges ends in one of the two corners that a boundary passes between
                bool reaches( int[] edgeIndices, SurfaceExtractor.Edge boundary ) {
                    return edgeIndices.Any( ( edgeIndex ) => voxel.edges[edgeIndex].corners.Any( ( corner ) => boundary.corners.Contains( corner ) ) );
                }

                foreach( var edgeIndices in vertexEdges ) {
                    var vertex = new Vertex( voxel ) {
                        qef = this.createQEF( )
                    };

                    var averageNormal = Vector3.zero;

                    foreach( var edgeIndex in edgeIndices ) {
                        var edge = voxel.edges[edgeIndex];

                        vertex.qef.add( edge.intersection, edge.normal );

                        averageNormal += edge.normal;

                        // the contour around the vertex intersects each of its edges once
                        vertex.intersections[edgeIndex] = 1;

                        voxel.edgeVertices[edgeIndex] = vertex;
                    }

                    // and it is a disk, which has an euler characteristic of 1
                    vertex.euler = 1;

                    // the boundary between two materials draws a vertex to where they meet on the contour. Its normal
                    // is not one of the contour, and stays out of the normal of the vertex. In a cell with several
                    // vertices a boundary goes to those whose piece of the contour reaches a corner next to the
                    // boundary, or to all of them when none does
                    foreach( var boundary in boundaries ) {
                        if( reaches( edgeIndices, boundary ) || !vertexEdges.Any( ( other ) => reaches( other, boundary ) ) ) {
                            vertex.qef.add( boundary.intersection, boundary.normal );
                        }
                    }

                    ( vertex.vertex, vertex.error ) = vertex.qef.solve( voxel );

                    vertex.normal = Vector3.Normalize( averageNormal );

                    vertex.vertex = QEFSolver.surfaceCorrection( vertex.vertex, vertex.normal, densityFunctions, this.surfaceCorrectionIterations );

                    voxel.vertices.Add( vertex );
                }
            }
        );

        // cluster vertices

        Octree<Voxel>.climb(
            this.octree,
            ( node ) => {
                var voxel = node.data;

                if( voxel.type == VoxelType.Leaf ) {
                    return;
                }

                // the vertices at the top of the vertex trees so far are those of the children

                var members       = new List<( int childIndex, Vertex vertex )>( );
                var memberIndices = new Dictionary<Vertex, int>( );

                for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                    foreach( var vertex in node.children[childIndex].data.vertices ) {
                        memberIndices.Add( vertex, members.Count );

                        members.Add( ( childIndex, vertex ) );
                    }
                }

                // the member that stands for the cluster each member was found to belong to
                var clusters = Enumerable.Range( 0, members.Count ).ToArray( );

                int clusterOf( int memberIndex ) {
                    while( clusters[memberIndex] != memberIndex ) {
                        memberIndex = clusters[memberIndex];
                    }
                    return memberIndex;
                }

                // vertices belong to the same cluster when the contour connects them inside the cell. Below the
                // children that was taken care of on the way up, which leaves the connections through the faces
                // between the children. Where the leaves on either side of such a face share an intersected edge, the
                // polygon around that edge runs from the vertex of the edge in one leaf to its vertex in the other
                foreach( var axis in new Axis[] { Axis.X, Axis.Y, Axis.Z } ) {
                    var edgeIndices = new int[][] {
                        OctreeContouringTables<Voxel>.lookupEdgeIndicesWithinFacePair( 0, axis, Position.Root ),
                        OctreeContouringTables<Voxel>.lookupEdgeIndicesWithinFacePair( 1, axis, Position.Root )
                    };

                    foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinNode( node, axis, Position.Root ) ) {
                        this.findLeavesWithinFacePair(
                            facePair,
                            axis,
                            Position.Root,
                            ( leaves ) => {
                                for( var edge = 0; edge < edgeIndices[0].Length; ++edge ) {
                                    var vertex = leaves[0].data.edgeVertices[edgeIndices[0][edge]];
                                    var other  = leaves[1].data.edgeVertices[edgeIndices[1][edge]];

                                    // only an intersected edge has a vertex, on either side
                                    if( vertex != null ) {
                                        clusters[clusterOf( memberIndices[vertex.findRoot( )] )] = clusterOf( memberIndices[other.findRoot( )] );
                                    }
                                }
                            }
                        );
                    }
                }

                foreach( var cluster in Enumerable.Range( 0, members.Count ).GroupBy( clusterOf ) ) {
                    var vertex = new Vertex( voxel ) {
                        qef = this.createQEF( )
                    };

                    var euler         = 0;
                    var intersections = 0;
                    var averageNormal = Vector3.zero;

                    foreach( var memberIndex in cluster ) {
                        var ( childIndex, member ) = members[memberIndex];

                        member.parent = vertex;

                        vertex.qef.combine( member.qef );

                        averageNormal += member.normal;

                        // count the intersections on each edge of the cell, along which lie the edges of the children
                        // at either end of it
                        for( var edgeIndex = 0; edgeIndex < voxel.edges.Length; ++edgeIndex ) {
                            if( this.lookupExternalEdgeIndices( edgeIndex ).Contains( childIndex ) ) {
                                vertex.intersections[edgeIndex] += member.intersections[edgeIndex];
                            }
                        }

                        // calculate euler characteristic of the cluster for manifold condition 1
                        euler += member.euler;

                        foreach( var edgeIndex in this.lookupInternelEdgeIndices( childIndex ) ) {
                            intersections += member.intersections[edgeIndex];
                        }
                    }

                    // every intersection on an edge between the children was counted by each member around it: four
                    // inside the cell, two on a face of it of which each counts as a half
                    UnityEngine.Debug.Assert( intersections % 4 == 0, "Contour around an edge inside a cell is not part of a single cluster" );

                    vertex.euler = euler - ( intersections / 4 );

                    // calculate face edge intersections for manifold condition 2
                    for( var faceIndex = 0; faceIndex < SurfaceExtractor.Voxel.FaceCount && vertex.face; ++faceIndex ) {
                        var count = 0;
                        foreach( var edgeIndex in this.lookupFaceEdgeIndices( faceIndex ) ) {
                            count += vertex.intersections[edgeIndex];
                        }

                        vertex.face = count == 0 || count == 2;
                    }

                    // solve QEF for vertex cluster

                    ( vertex.vertex, vertex.error ) = vertex.qef.solve( voxel );

                    vertex.normal = Vector3.Normalize( averageNormal );

                    voxel.vertices.Add( vertex );
                }
            }
        );

        if( UnityEngine.Debug.isDebugBuild ) {
            // verify the validity of the vertex clustering
            Octree<Voxel>.walk(
                this.octree,
                ( node ) => {
                    var voxel = node.data;

                    if( voxel.type == VoxelType.Internal ) {
                        var valid = true;
                        foreach( var child in node.children ) {
                            foreach( var vertex in child.data.vertices ) {
                                if( vertex.parent == null || vertex.parent.voxel != voxel ) {
                                    valid = false;
                                    break;
                                }
                            }
                        }
                        // ensure every vertex of the child nodes was clustered into a vertex of the enclosing voxel
                        UnityEngine.Debug.Assert( valid, "Invalid octree cluster detected" );
                    }
                }
            );
        }

        // mark vertices as collapsible and cells as collapsed

        Octree<Voxel>.climb(
            this.octree,
            ( node ) => {
                var voxel = node.data;

                if( voxel.type == VoxelType.Internal ) {
                    // the vertex of a cluster is judged by itself: a cluster that is safe to collapse can sit above
                    // clusters that are not
                    foreach( var vertex in voxel.vertices ) {
                        vertex.collapsible = vertex.error < this.errorThreshold && (
                            !this.enforceManifold || ( vertex.euler == 1 && vertex.face )
                        );
                    }

                    // every polygon inside a cell collapses to an edge or a point when all of its vertices are
                    // collapsible and its children are leaves or collapsed themselves
                    var collapsed = voxel.vertices.All( ( vertex ) => vertex.collapsible );

                    foreach( var child in node.children ) {
                        if( child.data.type == VoxelType.Internal && !child.data.collapsed ) {
                            collapsed = false;
                            break;
                        }
                    }
                    voxel.collapsed = collapsed;
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

        // build debug information: the vertices on the contour, each shown in the cell it was created in
        var voxels = Octree<Voxel>.flatten( this.octree ).Where(
            ( voxel ) => voxel.type == VoxelType.Leaf
        ).SelectMany(
            ( voxel ) => voxel.vertices
        ).Select(
            ( vertex ) => vertex.findContourVertex( )
        ).Distinct( ).ToList( );

        var corners = voxels.SelectMany( ( vertex ) => vertex.corners ).Distinct( );

        var edges = voxels.SelectMany( ( vertex ) => vertex.edges ).Distinct( );

        return new Either<CPUVoxelization, GPUVoxelization>.Type0( new( ) {
            mesh    = mesh,
            corners = corners,
            edges   = edges,
            voxels  = voxels
        } );
    }

    private int[] lookupExternalEdgeIndices( int edgeIndex ) {
        return edgeIndex switch {
            0  => new int[] { 0, 1 },
            1  => new int[] { 2, 3 },
            2  => new int[] { 5, 6 },
            3  => new int[] { 4, 7 },
            4  => new int[] { 0, 5 },
            5  => new int[] { 1, 6 },
            6  => new int[] { 3, 4 },
            7  => new int[] { 2, 7 },
            8  => new int[] { 0, 3 },
            9  => new int[] { 1, 2 },
            10 => new int[] { 5, 4 },
            11 => new int[] { 6, 7 },
            _ => throw new Exception( "Unknown edge index specified" )
        };
    }

    private int[] lookupFaceEdgeIndices( int faceIndex ) {
        return faceIndex switch {
            0 => new int[] { 0, 1,  8,  9 },
            1 => new int[] { 0, 2,  4,  5 },
            2 => new int[] { 1, 3,  6,  7 },
            3 => new int[] { 4, 6,  8, 10 },
            4 => new int[] { 5, 7,  9, 11 },
            5 => new int[] { 2, 3, 10, 11 },
            _ => throw new Exception( "Unknown face index specified" )
        };
    }

    // the edges of a child that lie inside the parent or on one of its faces: all but the three that meet at the
    // corner the child shares with the parent, which run along the edges of the parent
    int[] lookupInternelEdgeIndices( int childIndex ) {
        return childIndex switch {
            0 => new int[] { 1, 2, 3, 5, 6, 7, 9, 10, 11 },
            1 => new int[] { 1, 2, 3, 4, 6, 7, 8, 10, 11 },
            2 => new int[] { 0, 2, 3, 4, 5, 6, 8, 10, 11 },
            3 => new int[] { 0, 2, 3, 4, 5, 7, 9, 10, 11 },
            4 => new int[] { 0, 1, 2, 4, 5, 7, 8,  9, 11 },
            5 => new int[] { 0, 1, 3, 5, 6, 7, 8,  9, 11 },
            6 => new int[] { 0, 1, 3, 4, 6, 7, 8,  9, 10 },
            7 => new int[] { 0, 1, 2, 4, 5, 6, 8,  9, 10 },
            _ => throw new Exception( "Unknown child index specified" )
        };
    }

    private QEFSolver createQEF( ) {
        return this.qefSolverType switch {
            QEFSolverType.Simple => new SimpleQEF ( this.minimizerIterations ),
            QEFSolverType.SVD    => new SVDQEF    ( this.minimizerIterations ),
            _                    => throw new Exception( "Unknown solver type specified" )
        };
    }

    // the vertex that the edge the four leaves around an edge have in common is associated with, in one of those leaves
    private Vertex lookupEdgeVertex( Octree<Voxel>[] nodes, int nodeIndex, Axis axis, Position position ) {
        return nodes[nodeIndex].data.edgeVertices[OctreeContouringTables<Voxel>.lookupEdgeIndexWithinEdgeNodes( nodeIndex, axis, position )];
    }

    // visits the pairs of leaves on either side of the face that two nodes have in common
    private void findLeavesWithinFacePair( Octree<Voxel>[] nodes, Axis axis, Position position, Action<Octree<Voxel>[]> visitor ) {
        UnityEngine.Debug.Assert( nodes.Length == 2 );

        if( nodes[0].data.type == VoxelType.Internal || nodes[1].data.type == VoxelType.Internal ) {
            foreach( var facePair in OctreeContouringTables<Voxel>.lookupFacePairsWithinFacePairs( nodes, axis, position ) ) {
                this.findLeavesWithinFacePair( facePair, axis, position, visitor );
            }
        }
        else {
            visitor( nodes );
        }
    }

    private void contourCell( Octree<Voxel> node, Position position, List<Vector3> vertices, List<Vector3> normals, List<Triangle> triangles ) {
        if( node.data.type == VoxelType.Internal ) {

            if( node.data.collapsed ) {
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

            if( nodes[0].data.collapsed && nodes[1].data.collapsed ) {
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

        if( edge.intersectsContour( ) ) {

            // the polygon around an edge connects the vertices that the edge is associated with in the four leaves
            // around it, or rather the vertices that take their place on the contour
            var contourVertices = new Vertex[4] {
                this.lookupEdgeVertex( nodes, 0, axis, position ).findContourVertex( ),
                this.lookupEdgeVertex( nodes, 1, axis, position ).findContourVertex( ),
                this.lookupEdgeVertex( nodes, 2, axis, position ).findContourVertex( ),
                this.lookupEdgeVertex( nodes, 3, axis, position ).findContourVertex( )
            };

            // generate vertex and normal
            foreach( var vertex in contourVertices ) {
                if( vertex.index == -1 ) {
                    vertex.index = vertices.Count;

                    vertices.Add ( vertex.vertex );
                    normals.Add  ( vertex.normal );
                }
            }

            var materialIndex = edge.corners[0].materialIndex == MaterialIndex.Void
                ? edge.corners[1].materialIndex
                : edge.corners[0].materialIndex;

            var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( materialIndex );

            int[] indices;

            // ensure quad is indexed facing outward
            if( edge.corners[0].materialIndex == MaterialIndex.Void ) {
                indices = new int[] {
                    contourVertices[0].index,
                    contourVertices[1].index,
                    contourVertices[2].index,
                    contourVertices[3].index,
                    contourVertices[2].index,
                    contourVertices[1].index
                };
            }
            else {
                indices = new int[] {
                    contourVertices[1].index,
                    contourVertices[2].index,
                    contourVertices[3].index,
                    contourVertices[2].index,
                    contourVertices[1].index,
                    contourVertices[0].index
                };
            }

            // vertices of the same cluster are one vertex on the contour, so clustering can collapse the quad to a
            // triangle, an edge or a point. Whatever is left of it are the triangles that do not use a vertex twice
            for( var triangleIndex = 0; triangleIndex < indices.Length; triangleIndex += 3 ) {
                var a = indices[triangleIndex];
                var b = indices[triangleIndex + 1];
                var c = indices[triangleIndex + 2];

                if( a != b && b != c && a != c ) {
                    triangles.Add( new( a, b, c, subMeshIndex ) );
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
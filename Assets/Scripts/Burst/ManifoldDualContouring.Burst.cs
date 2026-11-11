using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using MaterialIndex   = SurfaceExtractor.MaterialIndex;
using VoxelType       = SurfaceExtractor.Voxel.Type;
using CPUVoxelization = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class ManifoldDualContouring {

    // the vertices of every node of the octree, as the managed vertices keep them. The vertices of a node follow each
    // other: those of the leaves come first, then those of each level above them in turn, up to the root. A node above
    // the leaves is given room for as many vertices as its children have, which is how many clusters there are when no
    // two of them are connected, and uses the first of that room. What is left over between the nodes is never read
    private struct BurstVertices : IDisposable {

        public NativeList<float3>   positions;
        public NativeList<float3>   normals;
        public NativeList<BurstQEF> qefs;
        public NativeList<int>      parents;       // the vertex each one was clustered into, -1 at the top of a vertex tree
        public NativeList<bool>     collapsible;
        public NativeList<int>      eulers;
        public NativeList<int>      nodes;         // the node each vertex was created in
        public NativeList<byte>     intersections; // 12 for each vertex, one for each edge of its node

        // what clustering a node keeps of its members while it works, in the room of that node
        public NativeList<int> clusters;
        public NativeList<int> clusterIndices;
        public NativeList<int> internalIntersections;

        public BurstVertices( Allocator allocator ) {
            this.positions             = new NativeList<float3>   ( allocator );
            this.normals               = new NativeList<float3>   ( allocator );
            this.qefs                  = new NativeList<BurstQEF> ( allocator );
            this.parents               = new NativeList<int>      ( allocator );
            this.collapsible           = new NativeList<bool>     ( allocator );
            this.eulers                = new NativeList<int>      ( allocator );
            this.nodes                 = new NativeList<int>      ( allocator );
            this.intersections         = new NativeList<byte>     ( allocator );
            this.clusters              = new NativeList<int>      ( allocator );
            this.clusterIndices        = new NativeList<int>      ( allocator );
            this.internalIntersections = new NativeList<int>      ( allocator );
        }

        public int count {
            get { return this.positions.Length; }
        }

        // room for the vertices of another level. What is there already stays
        public void resize( int count ) {
            this.positions.ResizeUninitialized             ( count );
            this.normals.ResizeUninitialized               ( count );
            this.qefs.ResizeUninitialized                  ( count );
            this.parents.ResizeUninitialized               ( count );
            this.collapsible.ResizeUninitialized           ( count );
            this.eulers.ResizeUninitialized                ( count );
            this.nodes.ResizeUninitialized                 ( count );
            this.intersections.ResizeUninitialized         ( count * SurfaceExtractor.Voxel.EdgeCount );
            this.clusters.ResizeUninitialized              ( count );
            this.clusterIndices.ResizeUninitialized        ( count );
            this.internalIntersections.ResizeUninitialized ( count );
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.qefs.Dispose( );
            this.parents.Dispose( );
            this.collapsible.Dispose( );
            this.eulers.Dispose( );
            this.nodes.Dispose( );
            this.intersections.Dispose( );
            this.clusters.Dispose( );
            this.clusterIndices.Dispose( );
            this.internalIntersections.Dispose( );
        }

    }

    // which corners of every leaf are solid, a bit for each, and with that how many vertices the leaf has: one for
    // every piece of the contour that passes through it. The solid corners also tell which of those vertices each
    // intersected edge of the leaf is associated with, so that does not need to be kept for every leaf
    [BurstCompile( CompileSynchronously = true )]
    private struct CountLeafVerticesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;

        public BurstOctree octree;

        [WriteOnly] public NativeArray<byte> solidCorners;

        [NativeDisableParallelForRestriction] public NativeArray<int> vertexCounts;

        public void Execute( int local ) {
            var grid  = this.octree.grid;
            var cell  = grid.cellOf( local );
            var solid = 0;

            for( var corner = 0; corner < SurfaceExtractor.Voxel.CornerCount; ++corner ) {
                if( this.materials[grid.cornerIndex( grid.cornerOfCell( cell, corner ) )] != MaterialIndex.Void ) {
                    solid |= 1 << corner;
                }
            }

            this.solidCorners[local] = ( byte )solid;

            this.vertexCounts[this.octree.levelStart( this.octree.depth ) + local] = BurstTables.lookupVertexCount( solid );
        }

    }

    // where the vertices of every node of a level go: after those of the levels below, node after node. A leaf has
    // room for its vertices, a node above the leaves for as many as its children have between them
    [BurstCompile( CompileSynchronously = true )]
    private struct LayoutLevelJob : IJob {

        [ReadOnly] public NativeArray<int> vertexCounts;

        public BurstOctree octree;
        public int         level;

        public NativeArray<int> vertexStarts;

        // how many vertices there is room for, before and after
        public NativeArray<int> room;

        public void Execute( ) {
            var start = this.octree.levelStart( this.level );
            var end   = this.octree.levelStart( this.level + 1 );
            var next  = this.room[0];

            for( var node = start; node < end; ++node ) {
                this.vertexStarts[node] = next;

                if( this.level == this.octree.depth ) {
                    next += this.vertexCounts[node];
                }
                else {
                    var coordinates = this.octree.nodeOf( this.level, node - start );

                    for( var child = 0; child < 8; ++child ) {
                        next += this.vertexCounts[this.octree.nodeIndex( this.level + 1, this.octree.childOf( coordinates, child ) )];
                    }
                }
            }

            this.room[0] = next;
        }

    }

    // what an edge of a leaf adds to the QEFs of the leaf
    private enum Hermite {
        None,
        Contour,
        MaterialBoundary
    }

    // what the jobs share. It is a class of its own so that Burst code does not reach into the voxelizer for it
    private static class BurstManifold {

        // the hermite data of an edge of the grid of the leaves, as a leaf that the contour passes through works it out
        public static Hermite hermiteOf(
            in BurstGrid                      grid,
            int3                              start,
            int3                              end,
            in BurstSettings                  settings,
            NativeArray<BurstDensityFunction> densityFunctions,
            NativeArray<float>                densities,
            NativeArray<MaterialIndex>        materials,
            out float3                        intersection,
            out float3                        normal
        ) {
            var startIndex = grid.cornerIndex( start );
            var endIndex   = grid.cornerIndex( end );

            var startMaterial = materials[startIndex];
            var endMaterial   = materials[endIndex];

            if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                intersection = BurstSurfaceExtractor.approximateIntersection(
                    grid.positionOf( start ),
                    densities[startIndex],
                    grid.positionOf( end ),
                    densities[endIndex],
                    densityFunctions,
                    BurstSurfaceExtractor.AnyMaterial,
                    settings.intersectionApproximation,
                    settings.binarySearchIterations
                );
                normal = BurstSurfaceExtractor.calculateNormal( intersection, densityFunctions, BurstSurfaceExtractor.AnyMaterial );

                return Hermite.Contour;
            }

            if( BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) ) {
                BurstSurfaceExtractor.calculateMaterialBoundary(
                    grid.positionOf( start ),
                    startMaterial,
                    grid.positionOf( end ),
                    endMaterial,
                    densityFunctions,
                    settings.intersectionApproximation,
                    settings.binarySearchIterations,
                    out intersection,
                    out normal
                );

                return Hermite.MaterialBoundary;
            }

            intersection = float3.zero;
            normal       = float3.zero;

            return Hermite.None;
        }

        // the vertex that takes the place of a vertex on the contour: the last collapsible one on the way up the vertex
        // tree (section IV.B)
        public static int contourVertexOf( int vertex, NativeArray<int> parents, NativeArray<bool> collapsible ) {
            var highest = vertex;

            var parent = parents[vertex];
            while( parent != -1 ) {
                if( collapsible[parent] ) {
                    highest = parent;
                }
                parent = parents[parent];
            }

            return highest;
        }

    }

    // the vertices of the leaves: one for every piece of the contour in a leaf, from the QEF of the edges that piece
    // intersects, in the order the managed implementation adds them
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct SolveLeavesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<byte>                 solidCorners;
        [ReadOnly] public NativeArray<int>                  vertexStarts;
        [ReadOnly] public NativeArray<int>                  vertexCounts;

        public BurstOctree   octree;
        public BurstSettings settings;

        [NativeDisableParallelForRestriction] public NativeArray<float3>   positions;
        [NativeDisableParallelForRestriction] public NativeArray<float3>   normals;
        [NativeDisableParallelForRestriction] public NativeArray<BurstQEF> qefs;
        [NativeDisableParallelForRestriction] public NativeArray<int>      parents;
        [NativeDisableParallelForRestriction] public NativeArray<bool>     collapsible;
        [NativeDisableParallelForRestriction] public NativeArray<int>      eulers;
        [NativeDisableParallelForRestriction] public NativeArray<int>      nodes;
        [NativeDisableParallelForRestriction] public NativeArray<byte>     intersections;

        public unsafe void Execute( int local ) {
            const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

            var node  = this.octree.levelStart( this.octree.depth ) + local;
            var count = this.vertexCounts[node];

            // the contour does not pass through the cell
            if( count == 0 ) {
                return;
            }

            var grid  = this.octree.grid;
            var cell  = grid.cellOf( local );
            var solid = this.solidCorners[local];
            var first = this.vertexStarts[node];

            // the hermite data of the edges the contour intersects, and of the boundaries between materials that pass
            // through the cell
            var edgeIntersections = stackalloc float3[EdgeCount];
            var edgeNormals       = stackalloc float3[EdgeCount];
            var boundaries        = 0;

            for( var edge = 0; edge < EdgeCount; ++edge ) {
                var hermite = BurstManifold.hermiteOf(
                    grid,
                    grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) ),
                    grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) ),
                    this.settings,
                    this.densityFunctions,
                    this.densities,
                    this.materials,
                    out edgeIntersections[edge],
                    out edgeNormals[edge]
                );

                if( hermite == Hermite.MaterialBoundary ) {
                    boundaries |= 1 << edge;
                }
            }

            // the boundaries that some vertex reaches: one of its edges ends in one of the two corners that the
            // boundary passes between
            var reached = 0;

            for( var boundary = 0; boundary < EdgeCount; ++boundary ) {
                if( ( boundaries & ( 1 << boundary ) ) == 0 ) {
                    continue;
                }

                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    if( BurstTables.lookupEdgeVertex( solid, edge ) >= 0 && BurstTables.edgesShareCorner( edge, boundary ) ) {
                        reached |= 1 << boundary;
                    }
                }
            }

            for( var index = 0; index < count; ++index ) {
                var vertex        = first + index;
                var qef           = BurstQEF.create( this.settings.qefSolverType, this.settings.minimizerIterations );
                var averageNormal = float3.zero;

                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    if( BurstTables.lookupEdgeVertex( solid, edge ) != index ) {
                        this.intersections[( vertex * EdgeCount ) + edge] = 0;

                        continue;
                    }

                    qef.add( edgeIntersections[edge], edgeNormals[edge] );

                    averageNormal += edgeNormals[edge];

                    // the contour around the vertex intersects each of its edges once
                    this.intersections[( vertex * EdgeCount ) + edge] = 1;
                }

                // the boundary between two materials draws a vertex to where they meet on the contour. Its normal is
                // not one of the contour, and stays out of the normal of the vertex. In a cell with several vertices a
                // boundary goes to those whose piece of the contour reaches a corner next to the boundary, or to all
                // of them when none does
                for( var boundary = 0; boundary < EdgeCount; ++boundary ) {
                    if( ( boundaries & ( 1 << boundary ) ) == 0 ) {
                        continue;
                    }

                    var reaches = false;

                    for( var edge = 0; edge < EdgeCount; ++edge ) {
                        if( BurstTables.lookupEdgeVertex( solid, edge ) == index && BurstTables.edgesShareCorner( edge, boundary ) ) {
                            reaches = true;
                        }
                    }

                    if( reaches || ( reached & ( 1 << boundary ) ) == 0 ) {
                        qef.add( edgeIntersections[boundary], edgeNormals[boundary] );
                    }
                }

                qef.solve( grid.centerOf( cell ), out var position, out _ );

                var normal = BurstSurfaceExtractor.normalize( averageNormal );

                this.positions[vertex]   = BurstSurfaceExtractor.surfaceCorrection( position, normal, this.densityFunctions, this.settings.surfaceCorrectionIterations );
                this.normals[vertex]     = normal;
                this.qefs[vertex]        = qef;
                this.parents[vertex]     = -1;
                this.collapsible[vertex] = true;
                this.eulers[vertex]      = 1; // its piece of the contour is a disk, which has an euler characteristic of 1
                this.nodes[vertex]       = node;
            }
        }

    }

    // one level of the clustering, the managed climb for the nodes of that level, and with it whether each cluster may
    // stand in for its vertices and whether contouring can skip the node. Every node only looks at what is below it,
    // which the levels below have finished with, and writes its own vertices and the parents of those of its children
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct ClusterLevelJob : IJobParallelFor {

        [ReadOnly] public NativeArray<byte> solidCorners;
        [ReadOnly] public NativeArray<int>  vertexStarts;

        public BurstOctree   octree;
        public BurstSettings settings;
        public int           level;
        public float         errorThreshold;
        public bool          enforceManifold;

        [NativeDisableParallelForRestriction] public NativeArray<int>  vertexCounts;
        [NativeDisableParallelForRestriction] public NativeArray<bool> collapsed;

        [NativeDisableParallelForRestriction] public NativeArray<float3>   positions;
        [NativeDisableParallelForRestriction] public NativeArray<float3>   normals;
        [NativeDisableParallelForRestriction] public NativeArray<BurstQEF> qefs;
        [NativeDisableParallelForRestriction] public NativeArray<int>      parents;
        [NativeDisableParallelForRestriction] public NativeArray<bool>     collapsible;
        [NativeDisableParallelForRestriction] public NativeArray<int>      eulers;
        [NativeDisableParallelForRestriction] public NativeArray<int>      nodes;
        [NativeDisableParallelForRestriction] public NativeArray<byte>     intersections;

        [NativeDisableParallelForRestriction] public NativeArray<int> clusters;
        [NativeDisableParallelForRestriction] public NativeArray<int> clusterIndices;
        [NativeDisableParallelForRestriction] public NativeArray<int> internalIntersections;

        // set where the contour around an edge inside a node turns out not to be part of a single cluster. Every node
        // that finds one writes the same thing
        [NativeDisableParallelForRestriction] public NativeArray<bool> inconsistent;

        // the member that stands for the cluster a member was found to belong to, members being numbered from the
        // first vertex of the node
        private int clusterOf( int first, int member ) {
            while( this.clusters[first + member] != member ) {
                member = this.clusters[first + member];
            }
            return member;
        }

        // the vertex at the top of the vertex tree a vertex is part of
        private int rootOf( int vertex ) {
            while( this.parents[vertex] != -1 ) {
                vertex = this.parents[vertex];
            }
            return vertex;
        }

        public unsafe void Execute( int local ) {
            const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

            var coordinates = this.octree.nodeOf( this.level, local );
            var node        = this.octree.nodeIndex( this.level, coordinates );
            var first       = this.vertexStarts[node];
            var aboveLeaves = this.level + 1 == this.octree.depth;

            // the vertices at the top of the vertex trees so far are those of the children: the members, numbered in
            // the order of the children

            var children      = stackalloc int[8];
            var memberOffsets = stackalloc int[8];
            var memberCount   = 0;

            // contouring can skip a node, no polygon comes out of it, when all of its vertices are collapsible and its
            // children are leaves or collapsed themselves
            var isCollapsed = true;

            for( var child = 0; child < 8; ++child ) {
                children[child]      = this.octree.nodeIndex( this.level + 1, this.octree.childOf( coordinates, child ) );
                memberOffsets[child] = memberCount;

                memberCount += this.vertexCounts[children[child]];

                if( !aboveLeaves && !this.collapsed[children[child]] ) {
                    isCollapsed = false;
                }
            }

            if( memberCount == 0 ) {
                this.vertexCounts[node] = 0;
                this.collapsed[node]    = isCollapsed;

                return;
            }

            for( var member = 0; member < memberCount; ++member ) {
                this.clusters[first + member]       = member;
                this.clusterIndices[first + member] = -1;
            }

            // vertices belong to the same cluster when the contour connects them inside the node. Below the children
            // that was taken care of on the levels below, which leaves the connections through the faces between the
            // children. Where the leaves on either side of such a face share an intersected edge, the polygon around
            // that edge runs from the vertex of the edge in one leaf to its vertex in the other. The octree is full, so
            // the leaves on either side of the face between two children are two squares of leaves, one across

            var grid      = this.octree.grid;
            var leafStart = this.octree.levelStart( this.octree.depth );
            var span      = this.octree.span( this.level + 1 );

            for( var axis = 0; axis < 3; ++axis ) {
                for( var pair = 0; pair < 4; ++pair ) {
                    var child = BurstTables.lookupFacePairWithinNode( axis, pair, 0 );
                    var other = BurstTables.lookupFacePairWithinNode( axis, pair, 1 );

                    // an intersected edge has a vertex on either side
                    if( this.vertexCounts[children[child]] == 0 || this.vertexCounts[children[other]] == 0 ) {
                        continue;
                    }

                    // the leaves at the minimum of each child, the first child being the one at the minimum of the axis
                    var minimum      = this.octree.childOf( coordinates, child ) * span;
                    var otherMinimum = this.octree.childOf( coordinates, other ) * span;

                    minimum[axis] += span - 1;

                    var across = ( axis + 1 ) % 3;
                    var along  = ( axis + 2 ) % 3;

                    for( var row = 0; row < span; ++row ) {
                        for( var column = 0; column < span; ++column ) {
                            var leaf      = minimum;
                            var otherLeaf = otherMinimum;

                            leaf[across]      += row;
                            leaf[along]       += column;
                            otherLeaf[across] += row;
                            otherLeaf[along]  += column;

                            var leafIndex = grid.cellIndex( leaf );
                            var solid     = this.solidCorners[leafIndex];

                            // the contour does not pass through the leaf
                            if( solid == 0 || solid == 255 ) {
                                continue;
                            }

                            var otherLeafIndex = grid.cellIndex( otherLeaf );
                            var otherSolid     = this.solidCorners[otherLeafIndex];

                            for( var edge = 0; edge < 4; ++edge ) {
                                var vertex = BurstTables.lookupEdgeVertex( solid, BurstTables.lookupEdgeIndexWithinFacePair( axis, 0, edge ) );

                                // only an intersected edge has a vertex, on either side
                                if( vertex < 0 ) {
                                    continue;
                                }

                                var otherVertex = BurstTables.lookupEdgeVertex( otherSolid, BurstTables.lookupEdgeIndexWithinFacePair( axis, 1, edge ) );

                                var member      = memberOffsets[child] + ( this.rootOf( this.vertexStarts[leafStart + leafIndex]      + vertex      ) - this.vertexStarts[children[child]] );
                                var otherMember = memberOffsets[other] + ( this.rootOf( this.vertexStarts[leafStart + otherLeafIndex] + otherVertex ) - this.vertexStarts[children[other]] );

                                this.clusters[first + this.clusterOf( first, member )] = this.clusterOf( first, otherMember );
                            }
                        }
                    }
                }
            }

            // a vertex for every cluster, in the order of the first member of each, to which the members add what
            // they have in the order they are in

            var clusterCount = 0;

            for( var child = 0; child < 8; ++child ) {
                var childFirst = this.vertexStarts[children[child]];
                var childCount = this.vertexCounts[children[child]];

                for( var index = 0; index < childCount; ++index ) {
                    var member       = childFirst + index;
                    var cluster      = this.clusterOf( first, memberOffsets[child] + index );
                    var clusterIndex = this.clusterIndices[first + cluster];

                    if( clusterIndex == -1 ) {
                        clusterIndex = clusterCount++;

                        this.clusterIndices[first + cluster]             = clusterIndex;
                        this.internalIntersections[first + clusterIndex] = 0;

                        var created = first + clusterIndex;

                        this.qefs[created]    = BurstQEF.create( this.settings.qefSolverType, this.settings.minimizerIterations );
                        this.normals[created] = float3.zero;
                        this.eulers[created]  = 0;
                        this.parents[created] = -1;
                        this.nodes[created]   = node;

                        for( var edge = 0; edge < EdgeCount; ++edge ) {
                            this.intersections[( created * EdgeCount ) + edge] = 0;
                        }
                    }

                    var vertex = first + clusterIndex;

                    this.parents[member] = vertex;

                    var qef = this.qefs[vertex];

                    qef.combine( this.qefs[member] );

                    this.qefs[vertex] = qef;

                    this.normals[vertex] += this.normals[member];

                    // an edge of the node runs along the edges of the children at either end of it, which are the
                    // corners it runs between. The other edges of a child lie inside the node or on one of its faces
                    for( var edge = 0; edge < EdgeCount; ++edge ) {
                        var count = this.intersections[( member * EdgeCount ) + edge];

                        if( BurstTables.lookupEdgeCorner( edge, 0 ) == child || BurstTables.lookupEdgeCorner( edge, 1 ) == child ) {
                            this.intersections[( vertex * EdgeCount ) + edge] += count;
                        }
                        else {
                            this.internalIntersections[first + clusterIndex] += count;
                        }
                    }

                    // the euler characteristic of the cluster, for manifold condition 1
                    this.eulers[vertex] += this.eulers[member];
                }
            }

            for( var clusterIndex = 0; clusterIndex < clusterCount; ++clusterIndex ) {
                var vertex = first + clusterIndex;

                // every intersection on an edge between the children was counted by each member around it: four inside
                // the node, two on a face of it of which each counts as a half
                var inside = this.internalIntersections[first + clusterIndex];

                if( inside % 4 != 0 ) {
                    this.inconsistent[0] = true;
                }

                var euler = this.eulers[vertex] - ( inside / 4 );

                this.eulers[vertex] = euler;

                // manifold condition 2: its part of the contour intersects the edges of every face of the node 0 or 2
                // times
                var face = true;

                for( var faceIndex = 0; faceIndex < SurfaceExtractor.Voxel.FaceCount && face; ++faceIndex ) {
                    var count = 0;
                    for( var index = 0; index < 4; ++index ) {
                        count += this.intersections[( vertex * EdgeCount ) + BurstTables.lookupFaceEdge( faceIndex, index )];
                    }

                    face = count == 0 || count == 2;
                }

                // the QEF stays as solving it left it: that is what the level above combines

                var solved = this.qefs[vertex];

                solved.solve( this.octree.centerOf( this.level, coordinates ), out var position, out var error );

                this.qefs[vertex]      = solved;
                this.positions[vertex] = position;
                this.normals[vertex]   = BurstSurfaceExtractor.normalize( this.normals[vertex] );

                // the vertex of a cluster is judged by itself: a cluster that is safe to collapse can sit above clusters
                // that are not
                var isCollapsible = error < this.errorThreshold && ( !this.enforceManifold || ( euler == 1 && face ) );

                this.collapsible[vertex] = isCollapsible;

                if( !isCollapsible ) {
                    isCollapsed = false;
                }
            }

            this.vertexCounts[node] = clusterCount;
            this.collapsed[node]    = isCollapsed;
        }

    }

    // the check the managed implementation makes in a debug build: that every vertex of the children of a node was
    // clustered into a vertex of that node
    [BurstCompile( CompileSynchronously = true )]
    private struct ValidateJob : IJobParallelFor {

        [ReadOnly] public NativeArray<int> vertexStarts;
        [ReadOnly] public NativeArray<int> vertexCounts;
        [ReadOnly] public NativeArray<int> parents;
        [ReadOnly] public NativeArray<int> nodes;

        public BurstOctree octree;

        // every node that finds a vertex out of place writes the same thing
        [NativeDisableParallelForRestriction] public NativeArray<bool> invalid;

        // the nodes above the leaves, which come before them
        public void Execute( int node ) {
            var level       = this.octree.levelOf( node );
            var coordinates = this.octree.nodeOf( level, node - this.octree.levelStart( level ) );

            for( var child = 0; child < 8; ++child ) {
                var index = this.octree.nodeIndex( level + 1, this.octree.childOf( coordinates, child ) );

                for( var vertex = this.vertexStarts[index]; vertex < this.vertexStarts[index] + this.vertexCounts[index]; ++vertex ) {
                    if( this.parents[vertex] == -1 || this.nodes[this.parents[vertex]] != node ) {
                        this.invalid[0] = true;
                    }
                }
            }
        }

    }

    // the managed contourCell, contourFace and contourEdge, in the same order, so that vertices are created and
    // triangles come out in the order the managed implementation creates them. The octree is full: the nodes of a face
    // or around an edge are always of the same level, and a polygon is only ever made between four leaves
    [BurstCompile( CompileSynchronously = true )]
    private struct ContourJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           vertexStarts;
        [ReadOnly] public NativeArray<bool>          collapsed;
        [ReadOnly] public NativeArray<float3>        positions;
        [ReadOnly] public NativeArray<float3>        normals;
        [ReadOnly] public NativeArray<int>           parents;
        [ReadOnly] public NativeArray<bool>          collapsible;

        public BurstOctree octree;

        // how many vertices the leaves have, which come before those of the clusters
        public int leafVertexCount;

        // the index each vertex has in the mesh plus one, zero until a polygon uses it
        public NativeArray<int> meshVertices;

        public NativeList<float3>             meshPositions;
        public NativeList<float3>             meshNormals;
        public NativeList<BurstMesh.Triangle> triangles;

        // the triangles so far that have the vertex of a cluster for a corner
        public NativeHashSet<int3> clusteredTriangles;

        public void Execute( ) {
            this.contourCell( new BurstNode( 0, int3.zero ) );
        }

        private bool isLeaf( BurstNode node ) {
            return node.level == this.octree.depth;
        }

        private bool isCollapsed( BurstNode node ) {
            return this.collapsed[this.octree.nodeIndex( node.level, node.coordinates )];
        }

        private BurstNode childOf( BurstNode node, int child ) {
            return new BurstNode( node.level + 1, this.octree.childOf( node.coordinates, child ) );
        }

        private void contourCell( BurstNode node ) {
            if( this.isLeaf( node ) || this.isCollapsed( node ) ) {
                return;
            }

            for( var child = 0; child < 8; ++child ) {
                this.contourCell( this.childOf( node, child ) );
            }

            for( var axis = 0; axis < 3; ++axis ) {
                for( var pair = 0; pair < 4; ++pair ) {
                    this.contourFace(
                        this.childOf( node, BurstTables.lookupFacePairWithinNode( axis, pair, 0 ) ),
                        this.childOf( node, BurstTables.lookupFacePairWithinNode( axis, pair, 1 ) ),
                        axis
                    );
                }
            }

            for( var axis = 0; axis < 3; ++axis ) {
                for( var group = 0; group < 2; ++group ) {
                    this.contourEdge(
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 0 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 1 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 2 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 3 ) ),
                        axis
                    );
                }
            }
        }

        private void contourFace( BurstNode first, BurstNode second, int axis ) {
            if( this.isLeaf( first ) || ( this.isCollapsed( first ) && this.isCollapsed( second ) ) ) {
                return;
            }

            for( var pair = 0; pair < 4; ++pair ) {
                this.contourFace(
                    this.childOf( first,  BurstTables.lookupFacePairWithinFacePair( axis, pair, 0 ) ),
                    this.childOf( second, BurstTables.lookupFacePairWithinFacePair( axis, pair, 1 ) ),
                    axis
                );
            }

            for( var edge = 0; edge < 4; ++edge ) {
                this.contourEdge(
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 0 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 1 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 2 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 3 ),
                    BurstTables.lookupEdgeNodeWithinFacePairAxis( axis, edge )
                );
            }
        }

        // the node at the given place around one of the edges inside the face between two nodes
        private BurstNode aroundEdgeWithinFacePair( BurstNode first, BurstNode second, int axis, int edge, int place ) {
            var node = BurstTables.lookupEdgeNodeWithinFacePairNode( axis, edge, place ) == 0 ? first : second;

            return this.childOf( node, BurstTables.lookupEdgeNodeWithinFacePairChild( axis, edge, place ) );
        }

        private void contourEdge( BurstNode node0, BurstNode node1, BurstNode node2, BurstNode node3, int axis ) {
            if( this.isLeaf( node0 ) ) {
                this.generateIndices( node0, node1, node2, node3, axis );
                return;
            }

            for( var group = 0; group < 2; ++group ) {
                this.contourEdge(
                    this.childOf( node0, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 0 ) ),
                    this.childOf( node1, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 1 ) ),
                    this.childOf( node2, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 2 ) ),
                    this.childOf( node3, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 3 ) ),
                    axis
                );
            }
        }

        // the vertex that takes the place on the contour of the vertex that the edge four leaves have in common is
        // associated with in one of them
        private int contourVertexOf( BurstNode leaf, int axis, int place ) {
            var local  = this.octree.grid.cellIndex( leaf.coordinates );
            var vertex = BurstTables.lookupEdgeVertex( this.solidCorners[local], BurstTables.lookupEdgeIndexWithinEdgeNodes( axis, place ) );

            return BurstManifold.contourVertexOf(
                this.vertexStarts[this.octree.levelStart( this.octree.depth ) + local] + vertex,
                this.parents,
                this.collapsible
            );
        }

        private int meshVertexOf( int vertex ) {
            if( this.meshVertices[vertex] == 0 ) {
                this.meshPositions.Add( this.positions[vertex] );
                this.meshNormals.Add( this.normals[vertex] );

                this.meshVertices[vertex] = this.meshPositions.Length;
            }
            return this.meshVertices[vertex] - 1;
        }

        private void generateIndices( BurstNode node0, BurstNode node1, BurstNode node2, BurstNode node3, int axis ) {
            // the edge the four leaves have in common, as the first of them has it
            var edge = BurstTables.lookupEdgeIndexWithinEdgeNodes( axis, 0 );

            var grid          = this.octree.grid;
            var startMaterial = this.materials[grid.cornerIndex( grid.cornerOfCell( node0.coordinates, BurstTables.lookupEdgeCorner( edge, 0 ) ) )];
            var endMaterial   = this.materials[grid.cornerIndex( grid.cornerOfCell( node0.coordinates, BurstTables.lookupEdgeCorner( edge, 1 ) ) )];

            if( !BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                return;
            }

            // the polygon around an edge connects the vertices that the edge is associated with in the four leaves
            // around it, or rather the vertices that take their place on the contour
            var contour0 = this.contourVertexOf( node0, axis, 0 );
            var contour1 = this.contourVertexOf( node1, axis, 1 );
            var contour2 = this.contourVertexOf( node2, axis, 2 );
            var contour3 = this.contourVertexOf( node3, axis, 3 );

            var v0 = this.meshVertexOf( contour0 );
            var v1 = this.meshVertexOf( contour1 );
            var v2 = this.meshVertexOf( contour2 );
            var v3 = this.meshVertexOf( contour3 );

            var materialIndex = startMaterial == MaterialIndex.Void ? endMaterial : startMaterial;
            var subMeshIndex  = BurstSurfaceExtractor.findHighestMaterialBit( materialIndex );

            // the same triangle can only come out of two polygons when clustering collapsed both onto it. Polygons
            // between the vertices of four leaves have at most two of those in common
            var clustered = contour0 >= this.leafVertexCount || contour1 >= this.leafVertexCount || contour2 >= this.leafVertexCount || contour3 >= this.leafVertexCount;

            // ensure quad is indexed facing outward
            if( startMaterial == MaterialIndex.Void ) {
                this.addTriangle( v0, v1, v2, subMeshIndex, clustered );
                this.addTriangle( v3, v2, v1, subMeshIndex, clustered );
            }
            else {
                this.addTriangle( v1, v2, v3, subMeshIndex, clustered );
                this.addTriangle( v2, v1, v0, subMeshIndex, clustered );
            }
        }

        // vertices of the same cluster are one vertex on the contour, so clustering can collapse the quad to a triangle,
        // an edge or a point. Whatever is left of it are the triangles that do not use a vertex twice, and of the
        // triangles that are the same, corner for corner, the first one
        private void addTriangle( int a, int b, int c, int subMeshIndex, bool clustered ) {
            if( a == b || b == c || a == c ) {
                return;
            }
            if( clustered && !this.clusteredTriangles.Add( new int3( a, b, c ) ) ) {
                return;
            }

            this.triangles.Add( new BurstMesh.Triangle( a, b, c, subMeshIndex ) );
        }

    }

    // the debug view of the managed implementation: the vertices on the contour, each shown as a voxel in the node it
    // was created in, in the order the leaves of the managed octree come to them, and the corners and edges of those
    // nodes, each the first time a node has it. Only the leaves work out the hermite data of their edges
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct DebugInformationJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<int>                  vertexStarts;
        [ReadOnly] public NativeArray<int>                  vertexCounts;
        [ReadOnly] public NativeArray<float3>               positions;
        [ReadOnly] public NativeArray<float3>               normals;
        [ReadOnly] public NativeArray<int>                  parents;
        [ReadOnly] public NativeArray<bool>                 collapsible;
        [ReadOnly] public NativeArray<int>                  nodes;

        public BurstOctree   octree;
        public BurstSettings settings;

        public NativeList<BurstDebugInformation.CornerData> corners;
        public NativeList<BurstDebugInformation.EdgeData>   edges;
        public NativeList<BurstDebugInformation.VoxelData>  voxels;

        public NativeHashMap<int, int>  cornerIndices;
        public NativeHashMap<long, int> edgeIndices;
        public NativeHashSet<int>       shown;

        public void Execute( ) {
            this.visit( new BurstNode( 0, int3.zero ) );
        }

        private int cornerOf( int3 position ) {
            var grid  = this.octree.grid;
            var index = grid.cornerIndex( position );

            if( !this.cornerIndices.TryGetValue( index, out var corner ) ) {
                corner = this.corners.Length;

                this.corners.Add( new BurstDebugInformation.CornerData {
                    position      = grid.positionOf( position ),
                    density       = this.densities[index],
                    materialIndex = this.materials[index]
                } );

                this.cornerIndices.Add( index, corner );
            }

            return corner;
        }

        private int edgeOf( int3 start, int3 end, bool ofLeaf ) {
            var grid = this.octree.grid;
            var key  = ( ( long )grid.cornerIndex( start ) * grid.cornerCount ) + grid.cornerIndex( end );

            if( this.edgeIndices.TryGetValue( key, out var edge ) ) {
                return edge;
            }

            var data = new BurstDebugInformation.EdgeData {
                corner0 = this.cornerOf( start ),
                corner1 = this.cornerOf( end )
            };

            if( ofLeaf ) {
                BurstManifold.hermiteOf( grid, start, end, this.settings, this.densityFunctions, this.densities, this.materials, out data.intersection, out data.normal );
            }

            edge = this.edges.Length;

            this.edges.Add( data );
            this.edgeIndices.Add( key, edge );

            return edge;
        }

        private void visit( BurstNode node ) {
            var index = this.octree.nodeIndex( node.level, node.coordinates );

            // a node without vertices has no leaves with vertices below it
            if( this.vertexCounts[index] == 0 ) {
                return;
            }

            if( node.level < this.octree.depth ) {
                for( var child = 0; child < 8; ++child ) {
                    this.visit( new BurstNode( node.level + 1, this.octree.childOf( node.coordinates, child ) ) );
                }
                return;
            }

            for( var vertex = this.vertexStarts[index]; vertex < this.vertexStarts[index] + this.vertexCounts[index]; ++vertex ) {
                var contourVertex = BurstManifold.contourVertexOf( vertex, this.parents, this.collapsible );

                if( this.shown.Add( contourVertex ) ) {
                    this.show( contourVertex );
                }
            }
        }

        private void show( int vertex ) {
            var index       = this.nodes[vertex];
            var level       = this.octree.levelOf( index );
            var coordinates = this.octree.nodeOf( level, index - this.octree.levelStart( level ) );
            var ofLeaf      = level == this.octree.depth;

            var data = new BurstDebugInformation.VoxelData {
                type            = ofLeaf ? VoxelType.Leaf : VoxelType.Internal,
                center          = this.octree.centerOf( level, coordinates ),
                size            = new float3( this.octree.sizeOf( level ) ),
                vertex          = this.positions[vertex],
                normal          = this.normals[vertex],
                hasFeaturePoint = true
            };

            for( var corner = 0; corner < SurfaceExtractor.Voxel.CornerCount; ++corner ) {
                data.setCorner( corner, this.cornerOf( this.octree.cornerOfNode( level, coordinates, corner ) ) );
            }

            for( var edge = 0; edge < SurfaceExtractor.Voxel.EdgeCount; ++edge ) {
                data.setEdge(
                    edge,
                    this.edgeOf(
                        this.octree.cornerOfNode( level, coordinates, BurstTables.lookupEdgeCorner( edge, 0 ) ),
                        this.octree.cornerOfNode( level, coordinates, BurstTables.lookupEdgeCorner( edge, 1 ) ),
                        ofLeaf
                    )
                );
            }

            this.voxels.Add( data );
        }

    }

    private static readonly ProfilerMarker burstSampleCornersMarker = new( "ManifoldDualContouring.Burst.SampleCorners" );
    private static readonly ProfilerMarker burstSolveVerticesMarker = new( "ManifoldDualContouring.Burst.SolveVertices" );
    private static readonly ProfilerMarker burstClusterMarker       = new( "ManifoldDualContouring.Burst.Cluster" );
    private static readonly ProfilerMarker burstValidateMarker      = new( "ManifoldDualContouring.Burst.Validate" );
    private static readonly ProfilerMarker burstContourMarker       = new( "ManifoldDualContouring.Burst.Contour" );
    private static readonly ProfilerMarker burstBuildMeshMarker     = new( "ManifoldDualContouring.Burst.BuildMesh" );
    private static readonly ProfilerMarker burstDebugMarker         = new( "ManifoldDualContouring.Burst.DebugInformation" );

    private Either<CPUVoxelization, GPUVoxelization> voxelizeBurst( IEnumerable<DensityFunction> densityFunctions ) {
        // the octree has a depth equal to the resolution, the leaves are at depth resolution - 1
        var octree = new BurstOctree( this.resolution - 1 );
        var grid   = octree.grid;

        var settings = new BurstSettings {
            resolution                  = grid.resolution,
            minimizerIterations         = this.minimizerIterations,
            binarySearchIterations      = this.binarySearchIterations,
            surfaceCorrectionIterations = this.surfaceCorrectionIterations,
            qefSolverType               = this.qefSolverType,
            intersectionApproximation   = this.intersectionApproximation
        };

        using var functions = BurstDensityFunction.snapshot( densityFunctions, Allocator.TempJob );
        using var densities = new NativeArray<float>         ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var materials = new NativeArray<MaterialIndex> ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        using var solidCorners = new NativeArray<byte> ( grid.cellCount,   Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var vertexStarts = new NativeArray<int>  ( octree.nodeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var vertexCounts = new NativeArray<int>  ( octree.nodeCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var collapsed    = new NativeArray<bool> ( octree.nodeCount, Allocator.TempJob );
        using var room         = new NativeArray<int>  ( 1, Allocator.TempJob );
        using var vertices     = new BurstVertices( Allocator.TempJob );

        using( burstSampleCornersMarker.Auto( ) ) {
            new BurstSampleCornersJob {
                densityFunctions = functions,
                grid             = grid,
                densities        = densities,
                materials        = materials
            }.Schedule( grid.cornerCount, 256 ).Complete( );
        }

        // the next level of the octree gets room for its vertices after those of the levels so far
        void layout( int level ) {
            new LayoutLevelJob {
                vertexCounts = vertexCounts,
                octree       = octree,
                level        = level,
                vertexStarts = vertexStarts,
                room         = room
            }.Run( );

            vertices.resize( room[0] );
        }

        using( burstSolveVerticesMarker.Auto( ) ) {
            new CountLeafVerticesJob {
                materials    = materials,
                octree       = octree,
                solidCorners = solidCorners,
                vertexCounts = vertexCounts
            }.Schedule( grid.cellCount, 1024 ).Complete( );

            layout( octree.depth );

            new SolveLeavesJob {
                densityFunctions = functions,
                densities        = densities,
                materials        = materials,
                solidCorners     = solidCorners,
                vertexStarts     = vertexStarts,
                vertexCounts     = vertexCounts,
                octree           = octree,
                settings         = settings,
                positions        = vertices.positions.AsArray( ),
                normals          = vertices.normals.AsArray( ),
                qefs             = vertices.qefs.AsArray( ),
                parents          = vertices.parents.AsArray( ),
                collapsible      = vertices.collapsible.AsArray( ),
                eulers           = vertices.eulers.AsArray( ),
                nodes            = vertices.nodes.AsArray( ),
                intersections    = vertices.intersections.AsArray( )
            }.Schedule( grid.cellCount, 64 ).Complete( );
        }

        var leafVertexCount = vertices.count;

        using( burstClusterMarker.Auto( ) ) {
            using var inconsistent = new NativeArray<bool>( 1, Allocator.TempJob );

            for( var level = octree.depth - 1; level >= 0; --level ) {
                layout( level );

                new ClusterLevelJob {
                    solidCorners          = solidCorners,
                    vertexStarts          = vertexStarts,
                    octree                = octree,
                    settings              = settings,
                    level                 = level,
                    errorThreshold        = this.errorThreshold,
                    enforceManifold       = this.enforceManifold,
                    vertexCounts          = vertexCounts,
                    collapsed             = collapsed,
                    positions             = vertices.positions.AsArray( ),
                    normals               = vertices.normals.AsArray( ),
                    qefs                  = vertices.qefs.AsArray( ),
                    parents               = vertices.parents.AsArray( ),
                    collapsible           = vertices.collapsible.AsArray( ),
                    eulers                = vertices.eulers.AsArray( ),
                    nodes                 = vertices.nodes.AsArray( ),
                    intersections         = vertices.intersections.AsArray( ),
                    clusters              = vertices.clusters.AsArray( ),
                    clusterIndices        = vertices.clusterIndices.AsArray( ),
                    internalIntersections = vertices.internalIntersections.AsArray( ),
                    inconsistent          = inconsistent
                }.Schedule( 1 << ( 3 * level ), level > 3 ? 16 : 1 ).Complete( );
            }

            UnityEngine.Debug.Assert( !inconsistent[0], "Contour around an edge inside a cell is not part of a single cluster" );
        }

        using( burstValidateMarker.Auto( ) ) {
            if( UnityEngine.Debug.isDebugBuild ) {
                using var invalid = new NativeArray<bool>( 1, Allocator.TempJob );

                // verify the validity of the vertex clustering
                new ValidateJob {
                    vertexStarts = vertexStarts,
                    vertexCounts = vertexCounts,
                    parents      = vertices.parents.AsArray( ),
                    nodes        = vertices.nodes.AsArray( ),
                    octree       = octree,
                    invalid      = invalid
                }.Schedule( octree.levelStart( octree.depth ), 256 ).Complete( );

                // ensure every vertex of the child nodes was clustered into a vertex of the enclosing voxel
                UnityEngine.Debug.Assert( !invalid[0], "Invalid octree cluster detected" );
            }
        }

        using var meshVertices       = new NativeArray<int>( vertices.count, Allocator.TempJob );
        using var meshPositions      = new NativeList<float3>( Allocator.TempJob );
        using var meshNormals        = new NativeList<float3>( Allocator.TempJob );
        using var triangles          = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
        using var clusteredTriangles = new NativeHashSet<int3>( 1024, Allocator.TempJob );

        using( burstContourMarker.Auto( ) ) {
            new ContourJob {
                materials          = materials,
                solidCorners       = solidCorners,
                vertexStarts       = vertexStarts,
                collapsed          = collapsed,
                positions          = vertices.positions.AsArray( ),
                normals            = vertices.normals.AsArray( ),
                parents            = vertices.parents.AsArray( ),
                collapsible        = vertices.collapsible.AsArray( ),
                octree             = octree,
                leafVertexCount    = leafVertexCount,
                meshVertices       = meshVertices,
                meshPositions      = meshPositions,
                meshNormals        = meshNormals,
                triangles          = triangles,
                clusteredTriangles = clusteredTriangles
            }.Run( );
        }

        UnityEngine.Mesh mesh;

        using( burstBuildMeshMarker.Auto( ) ) {
            mesh = BurstMesh.createMesh( meshPositions.AsArray( ), meshNormals.AsArray( ), triangles.AsArray( ) );
        }

        var voxelization = new CPUVoxelization {
            mesh    = mesh,
            corners = Array.Empty<SurfaceExtractor.Corner>( ),
            edges   = Array.Empty<SurfaceExtractor.Edge>( ),
            voxels  = Array.Empty<SurfaceExtractor.Voxel>( )
        };

        if( this.debugInformationWanted ) {
            using( burstDebugMarker.Auto( ) ) {
                using var debugCorners  = new NativeList<BurstDebugInformation.CornerData> ( Allocator.TempJob );
                using var debugEdges    = new NativeList<BurstDebugInformation.EdgeData>   ( Allocator.TempJob );
                using var debugVoxels   = new NativeList<BurstDebugInformation.VoxelData>  ( Allocator.TempJob );
                using var cornerIndices = new NativeHashMap<int, int>  ( 1024, Allocator.TempJob );
                using var edgeIndices   = new NativeHashMap<long, int> ( 1024, Allocator.TempJob );
                using var shown         = new NativeHashSet<int>       ( 1024, Allocator.TempJob );

                new DebugInformationJob {
                    densityFunctions = functions,
                    densities        = densities,
                    materials        = materials,
                    vertexStarts     = vertexStarts,
                    vertexCounts     = vertexCounts,
                    positions        = vertices.positions.AsArray( ),
                    normals          = vertices.normals.AsArray( ),
                    parents          = vertices.parents.AsArray( ),
                    collapsible      = vertices.collapsible.AsArray( ),
                    nodes            = vertices.nodes.AsArray( ),
                    octree           = octree,
                    settings         = settings,
                    corners          = debugCorners,
                    edges            = debugEdges,
                    voxels           = debugVoxels,
                    cornerIndices    = cornerIndices,
                    edgeIndices      = edgeIndices,
                    shown            = shown
                }.Run( );

                var ( corners, edges, voxels ) = BurstDebugInformation.convert( debugCorners.AsArray( ), debugEdges.AsArray( ), debugVoxels.AsArray( ) );

                voxelization.corners = corners;
                voxelization.edges   = edges;
                voxelization.voxels  = voxels;
            }
        }

        return new Either<CPUVoxelization, GPUVoxelization>.Type0( voxelization );
    }

}

using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Profiling;
using UnityEngine;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using Implementation            = SurfaceExtractor.Implementation;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

public partial class UniformDualContouring {

    private class ManagedImplementation : Implementation {

        private static readonly ProfilerMarker buildGridMarker      = new( "UniformDualContouring.Managed.BuildGrid" );
        private static readonly ProfilerMarker sampleCornersMarker  = new( "UniformDualContouring.Managed.SampleCorners" );
        private static readonly ProfilerMarker solveVerticesMarker  = new( "UniformDualContouring.Managed.SolveVertices" );
        private static readonly ProfilerMarker contourMarker        = new( "UniformDualContouring.Managed.Contour" );
        private static readonly ProfilerMarker buildMeshMarker      = new( "UniformDualContouring.Managed.BuildMesh" );
        private static readonly ProfilerMarker debugMarker          = new( "UniformDualContouring.Managed.DebugInformation" );

        private Voxel[,,] grid;

        public Either<CPUVoxelization, GPUVoxelization> voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            QEFSolverType                qefSolverType,
            IntersectionApproximation    intersectionApproximation,
            VertexNormals                vertexNormals
        ) {

            // volumes work out where they are from their transforms on every sample
            densityFunctions = DensityFunction.snapshot( densityFunctions );

            // create uniformly subdivided voxel grid

            buildGridMarker.Begin( );

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

            buildGridMarker.End( );

            // sample corner densities

            sampleCornersMarker.Begin( );

            foreach( var voxel in this.grid ) {
                foreach( var corner in voxel.corners ) {
                    corner.density       = SurfaceExtractor.calculateDensity  ( corner.position, densityFunctions );
                    corner.materialIndex = SurfaceExtractor.calculateMaterial ( corner.position, densityFunctions );
                }
            }

            sampleCornersMarker.End( );

            // find contour intersections and calculate minimizing vertices

            solveVerticesMarker.Begin( );

            foreach( var voxel in this.grid ) {

                QEFSolver qef = qefSolverType switch {
                    QEFSolverType.Simple => new SimpleQEF ( minimizerIterations ),
                    QEFSolverType.SVD    => new SVDQEF    ( minimizerIterations ),
                    _                    => throw new Exception( "Unknown solver type specified" )
                };

                var averageNormal = Vector3.zero;

                if(
                    voxel.corners.All( ( corner ) => corner.materialIndex == MaterialIndex.Void      ) ||
                    voxel.corners.All( ( corner ) => corner.materialIndex >= MaterialIndex.Material0 )
                ) {
                    // cell is either fully inside or outside the volume, skip
                    continue;
                }

                foreach( var edge in voxel.edges ) {
                    if( edge.intersectsContour( ) ) {
                        edge.intersection = SurfaceExtractor.approximateIntersection ( edge, densityFunctions, intersectionApproximation, binarySearchIterations );
                        edge.normal       = SurfaceExtractor.calculateNormal         ( edge, densityFunctions );

                        qef.add( edge.intersection, edge.normal );

                        averageNormal += edge.normal;
                    }
                    else if( SurfaceExtractor.crossesMaterialBoundary( edge ) ) {
                        // the boundary between two materials draws the vertex to where they meet on the contour. Its
                        // normal is not one of the contour, and stays out of the normal of the vertex
                        ( edge.intersection, edge.normal ) = SurfaceExtractor.calculateMaterialBoundary( edge, densityFunctions, intersectionApproximation, binarySearchIterations );

                        qef.add( edge.intersection, edge.normal );
                    }
                }

                if( qef.intersectionCount > 0 ) {
                    ( voxel.vertex, _ ) = QEFSolver.placeVertex( qef, voxel );

                    voxel.normal = Vector3.Normalize( averageNormal );

                    voxel.vertex = QEFSolver.surfaceCorrection( voxel.vertex, voxel.normal, densityFunctions, surfaceCorrectionIterations );

                    voxel.hasFeaturePoint = true;
                }
            }

            solveVerticesMarker.End( );

            // generate vertices and indices

            contourMarker.Begin( );

            var vertices = new List<Vector3>( );
            var normals  = new List<Vector3>( );
            var quads    = new List<Quad>( );

            for( var x = 0; x < this.grid.GetLength( 0 ); ++x ) {
                for( var y = 0; y < this.grid.GetLength( 1 ); ++y ) {
                    for( var z = 0; z < this.grid.GetLength( 2 ); ++z ) {
                        var voxel = this.grid[x, y, z];

                        if( !voxel.hasFeaturePoint ) {
                            continue;
                        }

                        // on every positive axis, generate indices using 4 voxel surrounding a common edge

                        // x axis
                        if( y + 1 < this.grid.GetLength( 1 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                            var cluster = new Voxel[] {
                                voxel,
                                this.grid[x, y,     z + 1],
                                this.grid[x, y + 1, z    ],
                                this.grid[x, y + 1, z + 1]
                            };
                            var edge = voxel.edges[3]; // common edge surrounded by all 4 voxels, refer to edge layout diagram

                            this.contour( cluster, edge, vertices, normals, quads );
                        }

                        // y axis
                        if( x + 1 < this.grid.GetLength( 0 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                            var cluster = new Voxel[] {
                                voxel,
                                this.grid[x + 1, y, z    ],
                                this.grid[x,     y, z + 1],
                                this.grid[x + 1, y, z + 1],
                            };
                            var edge = voxel.edges[7];

                            this.contour( cluster, edge, vertices, normals, quads );
                        }

                        // z axis
                        if( x + 1 < this.grid.GetLength( 0 ) && y + 1 < this.grid.GetLength( 1 ) ) {
                            var cluster = new Voxel[] {
                                voxel,
                                this.grid[x,     y + 1, z],
                                this.grid[x + 1, y,     z],
                                this.grid[x + 1, y + 1, z]
                            };
                            var edge = voxel.edges[11];

                            this.contour( cluster, edge, vertices, normals, quads );
                        }
                    }
                }
            }

            contourMarker.End( );

            // build mesh

            buildMeshMarker.Begin( );

            var mesh = SurfaceExtractor.createMesh(
                vertices.ToArray( ),
                normals.ToArray( ),
                quads.Select( ( quad ) => ( quad.subMeshIndex, quad.indices ) )
            );

            buildMeshMarker.End( );

            // build debug information

            debugMarker.Begin( );

            var voxels = this.grid.flatten( );

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

            debugMarker.End( );

            return new Either<CPUVoxelization, GPUVoxelization>.Type0( new( ) {
                mesh    = mesh,
                corners = corners,
                edges   = edges,
                voxels  = voxels
            } );
        }

        private void contour( Voxel[] voxels, SurfaceExtractor.Edge edge, List<Vector3> vertices, List<Vector3> normals, List<Quad> quads ) {
            if( voxels.All( ( voxel ) => voxel.hasFeaturePoint ) && edge.intersectsContour( ) ) {

                // indices should only be generated from void - solid intersections
                UnityEngine.Debug.Assert(
                    ( edge.corners[0].materialIndex == MaterialIndex.Void && edge.corners[1].materialIndex >= MaterialIndex.Material0 ) ||
                    ( edge.corners[1].materialIndex == MaterialIndex.Void && edge.corners[0].materialIndex >= MaterialIndex.Material0 )
                );

                var materialIndex = edge.corners[0].materialIndex == MaterialIndex.Void
                    ? edge.corners[1].materialIndex
                    : edge.corners[0].materialIndex;

                // generate vertex and normal
                foreach( var voxel in voxels ) {
                    if( voxel.index == -1 ) {
                        voxel.index = vertices.Count;

                        vertices.Add ( voxel.vertex );
                        normals.Add  ( voxel.normal );
                    }
                }

                int[] indices = new int[6];

                // ensure quad is indexed facing outward
                if( edge.corners[0].materialIndex == MaterialIndex.Void ) {
                    indices[0] = voxels[0].index;
                    indices[1] = voxels[1].index;
                    indices[2] = voxels[2].index;
                    indices[3] = voxels[3].index;
                    indices[4] = voxels[2].index;
                    indices[5] = voxels[1].index;
                }
                else {
                    indices[0] = voxels[1].index;
                    indices[1] = voxels[2].index;
                    indices[2] = voxels[3].index;
                    indices[3] = voxels[2].index;
                    indices[4] = voxels[1].index;
                    indices[5] = voxels[0].index;
                }

                var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( materialIndex );

                quads.Add( new( indices, subMeshIndex ) );

            }
        }

    }

}

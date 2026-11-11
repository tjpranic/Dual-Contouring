using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using FixedDensityFunction      = BurstTestSupport.FixedDensityFunction;

// the Burst port of the density functions and of SurfaceExtractor does the same float operations in the same order as
// the managed code. Mono keeps the intermediate results of an expression at a higher precision than Burst, so the two
// agree to the last bits, which is what these tests hold the Burst code to on random shapes, points and edges
public class BurstSurfaceExtractorTest {

    // relative to the size of the value: a few units in the last place of a float
    private const float densityTolerance = 1e-6f;

    // a normal is the difference of densities 2e-4 apart, which turns the last bits of the densities into the fourth
    // decimal of the normal. That is well inside the 0.1 degrees HermiteNormalTest holds the normals to. Normals are only
    // taken on the surface, at intersections. Far from it the densities are larger and so are their last bits, which
    // is why normals are compared within the distance of a cell or two of the surface they are the normal of
    private const float normalTolerance = 1e-3f;
    private const float normalDistance  = 0.1f;

    // where along an edge the density changes sign, and where a vertex ends up after surface correction
    private const float positionTolerance = 1e-5f;

    // closer to the surface of a shape than this a corner can be solid on one side and empty on the other
    private const float surfaceTolerance = 1e-5f;

    private const int shapeSets      = 24;
    private const int pointsPerSet   = 400;
    private const int edgesPerSet    = 400;
    private const int materialMasks  = 4;     // the first four material bits, each by itself
    private const float edgeLength   = 1.0f / 16.0f;

    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct PointJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float3>               points;

        public NativeArray<float>         densities;
        public NativeArray<float>         maskedDensities;
        public NativeArray<MaterialIndex> materials;
        public NativeArray<float3>        normals;
        public NativeArray<float3>        maskedNormals;
        public NativeArray<bool>          managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var point = 0; point < this.points.Length; ++point ) {
                var position = this.points[point];

                this.densities[point] = BurstSurfaceExtractor.calculateDensity  ( position, this.densityFunctions );
                this.materials[point] = BurstSurfaceExtractor.calculateMaterial ( position, this.densityFunctions );
                this.normals[point]   = BurstSurfaceExtractor.calculateNormal   ( position, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial );

                for( var mask = 0; mask < materialMasks; ++mask ) {
                    this.maskedDensities[( point * materialMasks ) + mask] = BurstSurfaceExtractor.calculateDensity ( position, this.densityFunctions, ( MaterialIndex )( 1 << mask ) );
                    this.maskedNormals[( point * materialMasks ) + mask]   = BurstSurfaceExtractor.calculateNormal  ( position, this.densityFunctions, ( MaterialIndex )( 1 << mask ) );
                }
            }
        }

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct EdgeJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float3>               starts;
        [ReadOnly] public NativeArray<float3>               ends;
        [ReadOnly] public NativeArray<float3>               correctionNormals;

        public NativeArray<float3> binarySearches;
        public NativeArray<float3> linearInterpolations;
        public NativeArray<float3> boundaryIntersections;
        public NativeArray<float3> boundaryNormals;
        public NativeArray<float3> corrections;
        public NativeArray<bool>   managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var edge = 0; edge < this.starts.Length; ++edge ) {
                var start = this.starts[edge];
                var end   = this.ends[edge];

                var startDensity = BurstSurfaceExtractor.calculateDensity( start, this.densityFunctions );
                var endDensity   = BurstSurfaceExtractor.calculateDensity( end,   this.densityFunctions );

                this.binarySearches[edge]       = BurstSurfaceExtractor.approximateIntersection( start, startDensity, end, endDensity, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial, IntersectionApproximation.BinarySearch,        6 );
                this.linearInterpolations[edge] = BurstSurfaceExtractor.approximateIntersection( start, startDensity, end, endDensity, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial, IntersectionApproximation.LinearInterpolation, 6 );

                var startMaterial = BurstSurfaceExtractor.calculateMaterial( start, this.densityFunctions );
                var endMaterial   = BurstSurfaceExtractor.calculateMaterial( end,   this.densityFunctions );

                if( BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) ) {
                    BurstSurfaceExtractor.calculateMaterialBoundary(
                        start, startMaterial, end, endMaterial, this.densityFunctions, IntersectionApproximation.BinarySearch, 6,
                        out var intersection, out var normal
                    );

                    this.boundaryIntersections[edge] = intersection;
                    this.boundaryNormals[edge]       = normal;
                }

                this.corrections[edge] = BurstSurfaceExtractor.surfaceCorrection( start, this.correctionNormals[edge], this.densityFunctions, 6 );
            }
        }

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct MaterialJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;

        public NativeArray<int>  highestBits;
        public NativeArray<bool> intersections;
        public NativeArray<bool> boundaries;
        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var material = 0; material < this.materials.Length; ++material ) {
                this.highestBits[material] = BurstSurfaceExtractor.findHighestMaterialBit( this.materials[material] );

                for( var other = 0; other < this.materials.Length; ++other ) {
                    this.intersections[( material * this.materials.Length ) + other] = BurstSurfaceExtractor.intersectsContour       ( this.materials[material], this.materials[other] );
                    this.boundaries[( material * this.materials.Length ) + other]    = BurstSurfaceExtractor.crossesMaterialBoundary ( this.materials[material], this.materials[other] );
                }
            }
        }

    }

    private static Vector3[] toVectors( NativeArray<float3> vectors ) {
        return vectors.Select( ( vector ) => ( Vector3 )vector ).ToArray( );
    }

    // points all over the grid and a little past it, the origin of every shape, and points just off those origins,
    // where an ellipsoid takes care not to divide by zero
    private static Vector3[] pointsFor( System.Random random, FixedDensityFunction[] shapes ) {
        return Enumerable.Range( 0, pointsPerSet ).Select(
            ( _ ) => BurstTestSupport.range( random, -Vector3.one * 0.6f, Vector3.one * 0.6f )
        ).Concat(
            shapes.Select( ( shape ) => shape.origin )
        ).Concat(
            shapes.Select( ( shape ) => shape.origin + ( BurstTestSupport.direction( random ) * 5e-6f ) )
        ).Concat(
            shapes.Select( ( shape ) => shape.origin + ( BurstTestSupport.direction( random ) * 2e-5f ) )
        ).ToArray( );
    }

    [Test]
    public void densitiesMaterialsAndNormalsAreTheManagedOnes( ) {
        var random = new System.Random( 1 );

        var nearSurfaces = 0;

        for( var set = 0; set < shapeSets; ++set ) {
            var shapes = BurstTestSupport.randomShapes( random, 1 + random.Next( 5 ) );
            var points = pointsFor( random, shapes );

            using var densityFunctions = BurstTestSupport.toNative( shapes );
            using var nativePoints     = new NativeArray<float3>( points.Select( ( point ) => ( float3 )point ).ToArray( ), Allocator.TempJob );
            using var densities        = new NativeArray<float>( points.Length, Allocator.TempJob );
            using var maskedDensities  = new NativeArray<float>( points.Length * materialMasks, Allocator.TempJob );
            using var materials        = new NativeArray<MaterialIndex>( points.Length, Allocator.TempJob );
            using var normals          = new NativeArray<float3>( points.Length, Allocator.TempJob );
            using var maskedNormals    = new NativeArray<float3>( points.Length * materialMasks, Allocator.TempJob );
            using var managed          = new NativeArray<bool>( 1, Allocator.TempJob );

            new PointJob {
                densityFunctions = densityFunctions,
                points           = nativePoints,
                densities        = densities,
                maskedDensities  = maskedDensities,
                materials        = materials,
                normals          = normals,
                maskedNormals    = maskedNormals,
                managed          = managed
            }.Run( );

            BurstTestSupport.assertRanUnderBurst( managed );

            BurstTestSupport.assertClose( "densities", points.Select( ( point ) => SurfaceExtractor.calculateDensity( point, shapes ) ).ToArray( ), densities.ToArray( ), densityTolerance );
            var nearSurface = Enumerable.Range( 0, points.Length ).Where( ( point ) => Mathf.Abs( SurfaceExtractor.calculateDensity( points[point], shapes ) ) < normalDistance ).ToArray( );

            BurstTestSupport.assertClose(
                "normals near the surface",
                nearSurface.Select( ( point ) => SurfaceExtractor.calculateNormal( points[point], shapes ) ).ToArray( ),
                nearSurface.Select( ( point ) => ( Vector3 )normals[point] ).ToArray( ),
                normalTolerance
            );

            // the material of a point is exact, except right at the surface of a shape, where the last bits of the
            // density decide on which side of it the point is
            var expectedMaterials = points.Select( ( point ) => SurfaceExtractor.calculateMaterial( point, shapes ) ).ToArray( );
            var awayFromSurfaces  = Enumerable.Range( 0, points.Length ).Where( ( point ) => shapes.All( ( shape ) => Mathf.Abs( shape.sample( points[point] ) ) > surfaceTolerance ) ).ToArray( );

            BurstTestSupport.assertSame( "materials away from the surfaces", awayFromSurfaces.Select( ( point ) => expectedMaterials[point] ).ToArray( ), awayFromSurfaces.Select( ( point ) => materials[point] ).ToArray( ) );

            // the managed code gives the shapes of one material by themselves to the same functions
            var ofMaterial = Enumerable.Range( 0, materialMasks ).Select(
                ( mask ) => shapes.Where( ( shape ) => ( shape.materialIndex & ( MaterialIndex )( 1 << mask ) ) != 0 ).ToList( )
            ).ToArray( );

            BurstTestSupport.assertClose(
                "densities of one material",
                points.SelectMany( ( point ) => ofMaterial.Select( ( material ) => SurfaceExtractor.calculateDensity( point, material ) ) ).ToArray( ),
                maskedDensities.ToArray( ),
                densityTolerance
            );
            var nearMaterialSurface = Enumerable.Range( 0, points.Length * materialMasks ).Where(
                ( index ) => Mathf.Abs( SurfaceExtractor.calculateDensity( points[index / materialMasks], ofMaterial[index % materialMasks] ) ) < normalDistance
            ).ToArray( );

            BurstTestSupport.assertClose(
                "normals of one material near its surface",
                nearMaterialSurface.Select( ( index ) => SurfaceExtractor.calculateNormal( points[index / materialMasks], ofMaterial[index % materialMasks] ) ).ToArray( ),
                nearMaterialSurface.Select( ( index ) => ( Vector3 )maskedNormals[index] ).ToArray( ),
                normalTolerance
            );

            nearSurfaces += nearSurface.Length + nearMaterialSurface.Length;
        }

        Assert.Greater( nearSurfaces, 1000, "few points were close enough to a surface to compare their normals" );
    }

    [Test]
    public void intersectionsBoundariesAndCorrectionsAreTheManagedOnes( ) {
        var random = new System.Random( 2 );

        var crossings  = 0;
        var boundaries = 0;

        for( var set = 0; set < shapeSets; ++set ) {
            // two overlapping volumes of different materials, which puts a boundary between materials inside the
            // contour, and more shapes on top of them
            var origin = BurstTestSupport.range( random, -Vector3.one * 0.2f, Vector3.one * 0.2f );

            var shapes = new[] {
                new FixedDensityFunction( DensityFunction.Type.Cuboid,    origin,                                                                    BurstTestSupport.range( random, Vector3.one * 0.15f, Vector3.one * 0.3f ), DensityFunction.Combination.Union, MaterialIndex.Material0 ),
                new FixedDensityFunction( DensityFunction.Type.Ellipsoid, origin + BurstTestSupport.range( random, -Vector3.one * 0.1f, Vector3.one * 0.1f ), BurstTestSupport.range( random, Vector3.one * 0.1f, Vector3.one * 0.25f ), DensityFunction.Combination.Union, MaterialIndex.Material1 )
            }.Concat( BurstTestSupport.randomShapes( random, random.Next( 3 ) ) ).ToArray( );

            // edges of a grid of 16^3 cells, along every axis
            var starts            = Enumerable.Range( 0, edgesPerSet ).Select( ( _ ) => BurstTestSupport.range( random, -Vector3.one * 0.5f, Vector3.one * 0.5f ) ).ToArray( );
            var axes              = new[] { Vector3.right, Vector3.up, Vector3.forward };
            var ends              = starts.Select( ( start ) => start + ( axes[random.Next( 3 )] * edgeLength ) ).ToArray( );
            var correctionNormals = starts.Select( ( _ ) => BurstTestSupport.direction( random ) ).ToArray( );

            using var densityFunctions      = BurstTestSupport.toNative( shapes );
            using var nativeStarts          = new NativeArray<float3>( starts.Select( ( point ) => ( float3 )point ).ToArray( ), Allocator.TempJob );
            using var nativeEnds            = new NativeArray<float3>( ends.Select( ( point ) => ( float3 )point ).ToArray( ), Allocator.TempJob );
            using var nativeNormals         = new NativeArray<float3>( correctionNormals.Select( ( normal ) => ( float3 )normal ).ToArray( ), Allocator.TempJob );
            using var binarySearches        = new NativeArray<float3>( starts.Length, Allocator.TempJob );
            using var linearInterpolations  = new NativeArray<float3>( starts.Length, Allocator.TempJob );
            using var boundaryIntersections = new NativeArray<float3>( starts.Length, Allocator.TempJob );
            using var boundaryNormals       = new NativeArray<float3>( starts.Length, Allocator.TempJob );
            using var corrections           = new NativeArray<float3>( starts.Length, Allocator.TempJob );
            using var managed               = new NativeArray<bool>( 1, Allocator.TempJob );

            new EdgeJob {
                densityFunctions      = densityFunctions,
                starts                = nativeStarts,
                ends                  = nativeEnds,
                correctionNormals     = nativeNormals,
                binarySearches        = binarySearches,
                linearInterpolations  = linearInterpolations,
                boundaryIntersections = boundaryIntersections,
                boundaryNormals       = boundaryNormals,
                corrections           = corrections,
                managed               = managed
            }.Run( );

            BurstTestSupport.assertRanUnderBurst( managed );

            var expectedBinarySearches = new Vector3[starts.Length];
            var expectedInterpolations = new Vector3[starts.Length];
            var expectedIntersections  = new Vector3[starts.Length];
            var expectedNormals        = new Vector3[starts.Length];
            var expectedCorrections    = new Vector3[starts.Length];

            // intersections are only looked for on edges where the density changes sign. Elsewhere linear
            // interpolation extrapolates far past the edge, or divides by zero
            var crossing = new List<int>( );

            // and the boundaries between materials on edges whose ends are solid and of different materials
            var boundary = new List<int>( );

            for( var edge = 0; edge < starts.Length; ++edge ) {
                var start = ( starts[edge], SurfaceExtractor.calculateDensity( starts[edge], shapes ) );
                var end   = ( ends[edge],   SurfaceExtractor.calculateDensity( ends[edge],   shapes ) );

                if( ( start.Item2 < 0.0f ) != ( end.Item2 < 0.0f ) ) {
                    crossing.Add( edge );
                }

                expectedBinarySearches[edge] = SurfaceExtractor.approximateIntersection( start, end, shapes, IntersectionApproximation.BinarySearch,        6 );
                expectedInterpolations[edge] = SurfaceExtractor.approximateIntersection( start, end, shapes, IntersectionApproximation.LinearInterpolation, 6 );

                var corners = new SurfaceExtractor.Corner[] {
                    new AdaptiveDualContouring.Corner( starts[edge] ) { materialIndex = SurfaceExtractor.calculateMaterial( starts[edge], shapes ) },
                    new AdaptiveDualContouring.Corner( ends[edge] )   { materialIndex = SurfaceExtractor.calculateMaterial( ends[edge],   shapes ) }
                };
                var managedEdge = new AdaptiveDualContouring.Edge( corners );

                if( SurfaceExtractor.crossesMaterialBoundary( managedEdge ) ) {
                    ( expectedIntersections[edge], expectedNormals[edge] ) = SurfaceExtractor.calculateMaterialBoundary( managedEdge, shapes, IntersectionApproximation.BinarySearch, 6 );

                    boundary.Add( edge );
                }

                expectedCorrections[edge] = QEFSolver.surfaceCorrection( starts[edge], correctionNormals[edge], shapes, 6 );
            }

            Vector3[] at( List<int> edges, Vector3[] values ) {
                return edges.Select( ( edge ) => values[edge] ).ToArray( );
            }

            BurstTestSupport.assertClose( "binary searches",                 at( crossing, expectedBinarySearches ), at( crossing, toVectors( binarySearches ) ),        positionTolerance );
            BurstTestSupport.assertClose( "linear interpolations",           at( crossing, expectedInterpolations ), at( crossing, toVectors( linearInterpolations ) ),  positionTolerance );
            BurstTestSupport.assertClose( "material boundary intersections", at( boundary, expectedIntersections ),  at( boundary, toVectors( boundaryIntersections ) ), positionTolerance );
            BurstTestSupport.assertClose( "material boundary normals",       at( boundary, expectedNormals ),        at( boundary, toVectors( boundaryNormals ) ),       normalTolerance );
            BurstTestSupport.assertClose( "surface corrections",             expectedCorrections,                    toVectors( corrections ),                          positionTolerance );

            crossings  += crossing.Count;
            boundaries += boundary.Count;
        }

        Assert.Greater( crossings,  100, "few edges crossed the surface" );
        Assert.Greater( boundaries, 10,  "few edges crossed a material boundary" );
    }

    [Test]
    public void materialBitsAreReadAsTheManagedCodeReadsThem( ) {
        // every combination of the first four materials, and the highest material there is
        var materials = Enumerable.Range( 0, 16 ).Select( ( bits ) => ( MaterialIndex )bits ).Append( MaterialIndex.Material9 ).Append( MaterialIndex.Material9 | MaterialIndex.Material2 ).ToArray( );

        using var nativeMaterials = new NativeArray<MaterialIndex>( materials, Allocator.TempJob );
        using var highestBits     = new NativeArray<int>( materials.Length, Allocator.TempJob );
        using var intersections   = new NativeArray<bool>( materials.Length * materials.Length, Allocator.TempJob );
        using var boundaries      = new NativeArray<bool>( materials.Length * materials.Length, Allocator.TempJob );
        using var managed         = new NativeArray<bool>( 1, Allocator.TempJob );

        new MaterialJob {
            materials     = nativeMaterials,
            highestBits   = highestBits,
            intersections = intersections,
            boundaries    = boundaries,
            managed       = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        // the managed code logs when it is asked for the highest bit of no material
        BurstTestSupport.assertSame( "highest bits", materials.Skip( 1 ).Select( SurfaceExtractor.findHighestMaterialBit ).Prepend( 0 ).ToArray( ), highestBits.ToArray( ) );

        var expectedIntersections = new List<bool>( );
        var expectedBoundaries    = new List<bool>( );

        foreach( var material in materials ) {
            foreach( var other in materials ) {
                var corners = new SurfaceExtractor.Corner[] {
                    new AdaptiveDualContouring.Corner( Vector3.zero )  { materialIndex = material },
                    new AdaptiveDualContouring.Corner( Vector3.right ) { materialIndex = other }
                };
                var edge = new AdaptiveDualContouring.Edge( corners );

                expectedIntersections.Add( edge.intersectsContour( ) );
                expectedBoundaries.Add( SurfaceExtractor.crossesMaterialBoundary( edge ) );
            }
        }

        BurstTestSupport.assertSame( "edges crossing the contour",         expectedIntersections.ToArray( ), intersections.ToArray( ) );
        BurstTestSupport.assertSame( "edges crossing a material boundary", expectedBoundaries.ToArray( ),    boundaries.ToArray( ) );
    }

}

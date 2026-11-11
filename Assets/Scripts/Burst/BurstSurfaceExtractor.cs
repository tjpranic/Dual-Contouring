using Unity.Collections;
using Unity.Mathematics;

using Combination               = DensityFunction.Combination;
using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;

// the static functions of SurfaceExtractor and QEFSolver for Burst code. They do the same operations in the same order
// as the managed ones, including the quirks of Unity's math: Mathf.Min and Mathf.Max, and Vector3.Normalize leaving
// vectors shorter than 1e-5 at zero
public static class BurstSurfaceExtractor {

    // in place of a material: the density functions of every material, as opposed to those of one of them
    public const MaterialIndex AnyMaterial = ( MaterialIndex )( -1 );

    // Mathf.Min and Mathf.Max, which unlike math.min and math.max do nothing special about NaN
    public static float min( float a, float b ) {
        return a < b ? a : b;
    }

    public static float max( float a, float b ) {
        return a > b ? a : b;
    }

    // Vector3.Magnitude
    public static float magnitude( float3 vector ) {
        return math.sqrt( ( vector.x * vector.x ) + ( vector.y * vector.y ) + ( vector.z * vector.z ) );
    }

    // Vector3.Normalize
    public static float3 normalize( float3 vector ) {
        var length = magnitude( vector );

        return length > 1e-5f ? vector / length : float3.zero;
    }

    // Vector3.Dot
    public static float dot( float3 vector, float3 other ) {
        return ( vector.x * other.x ) + ( vector.y * other.y ) + ( vector.z * other.z );
    }

    public static float calculateDensity( float3 point, NativeArray<BurstDensityFunction> densityFunctions ) {
        return calculateDensity( point, densityFunctions, AnyMaterial );
    }

    // the density of the density functions of the given material taken by themselves, or of all of them
    public static float calculateDensity( float3 point, NativeArray<BurstDensityFunction> densityFunctions, MaterialIndex materialIndex ) {
        var density = float.MaxValue;

        for( var index = 0; index < densityFunctions.Length; ++index ) {
            var densityFunction = densityFunctions[index];

            if( materialIndex != AnyMaterial && ( densityFunction.materialIndex & materialIndex ) == 0 ) {
                continue;
            }

            switch( densityFunction.combination ) {
                case Combination.Union:
                    density = min( density, densityFunction.sample( point ) );
                    break;
                case Combination.Intersection:
                    density = max( density, densityFunction.sample( point ) );
                    break;
                case Combination.Subtraction:
                    density = max( density, -densityFunction.sample( point ) );
                    break;
            }
        }

        return density;
    }

    public static MaterialIndex calculateMaterial( float3 position, NativeArray<BurstDensityFunction> densityFunctions ) {
        var materialIndex = MaterialIndex.Void;

        for( var index = 0; index < densityFunctions.Length; ++index ) {
            var densityFunction = densityFunctions[index];

            var density = densityFunction.sample( position );

            // set material bit if the corner is inside of the shape
            if( densityFunction.combination == Combination.Union && density < 0.0f ) {
                materialIndex |= densityFunction.materialIndex;
            }
            // unset material bit if the corner is outside of the shape
            if( densityFunction.combination == Combination.Intersection && density > 0.0f ) {
                materialIndex &= ~densityFunction.materialIndex;
            }
            // unset material bit if the corner is inside of the shape
            if( densityFunction.combination == Combination.Subtraction && density < 0.0f ) {
                materialIndex &= ~densityFunction.materialIndex;
            }
        }

        return materialIndex;
    }

    // calculateDensity and calculateMaterial at once, which takes every density function's sample for both instead of
    // sampling it twice. The results are those of the two functions by themselves
    public static float calculateDensityAndMaterial( float3 point, NativeArray<BurstDensityFunction> densityFunctions, out MaterialIndex materialIndex ) {
        var density = float.MaxValue;

        materialIndex = MaterialIndex.Void;

        for( var index = 0; index < densityFunctions.Length; ++index ) {
            var densityFunction = densityFunctions[index];

            var sample = densityFunction.sample( point );

            switch( densityFunction.combination ) {
                case Combination.Union:
                    density = min( density, sample );
                    if( sample < 0.0f ) {
                        materialIndex |= densityFunction.materialIndex;
                    }
                    break;
                case Combination.Intersection:
                    density = max( density, sample );
                    if( sample > 0.0f ) {
                        materialIndex &= ~densityFunction.materialIndex;
                    }
                    break;
                case Combination.Subtraction:
                    density = max( density, -sample );
                    if( sample < 0.0f ) {
                        materialIndex &= ~densityFunction.materialIndex;
                    }
                    break;
            }
        }

        return density;
    }

    // where the density of the given material, or of all of them, changes sign between two points
    public static float3 approximateIntersection(
        float3                             position,
        float                              density,
        float3                             otherPosition,
        float                              otherDensity,
        NativeArray<BurstDensityFunction>  densityFunctions,
        MaterialIndex                      materialIndex,
        IntersectionApproximation          intersectionApproximation,
        int                                binarySearchIterations
    ) {
        if( density == 0.0f || otherDensity == 0.0f ) {
            // one of the corners is at the exact intersection
            return density == 0.0f ? position : otherPosition;
        }

        if( intersectionApproximation == IntersectionApproximation.BinarySearch ) {
            float3 start;
            float3 end;

            if( density < otherDensity ) {
                start = position;
                end   = otherPosition;
            }
            else {
                start = otherPosition;
                end   = position;
            }

            var intersection = float3.zero;
            for( var binarySearchIteration = 0; binarySearchIteration < binarySearchIterations; ++binarySearchIteration ) {
                intersection = start + ( 0.5f * ( end - start ) );

                var intersectionDensity = calculateDensity( intersection, densityFunctions, materialIndex );

                if( intersectionDensity < 0.0f ) {
                    start = intersection;
                }
                else if( intersectionDensity > 0.0f ) {
                    end = intersection;
                }
                else if( intersectionDensity == 0.0f ) {
                    break;
                }
            }

            return intersection;
        }

        // linear interpolation
        return position + ( ( -density ) * ( otherPosition - position ) / ( otherDensity - density ) );
    }

    // the normal of the surface of the given material, or of all of them, from the difference of the density around
    // the position. The step is the same as SurfaceExtractor.calculateNormal's, far smaller than a voxel
    public static float3 calculateNormal( float3 position, NativeArray<BurstDensityFunction> densityFunctions, MaterialIndex materialIndex ) {
        const float step = 1e-4f;

        var positive = new float3(
            calculateDensity( position + new float3( step, 0.0f, 0.0f ), densityFunctions, materialIndex ),
            calculateDensity( position + new float3( 0.0f, step, 0.0f ), densityFunctions, materialIndex ),
            calculateDensity( position + new float3( 0.0f, 0.0f, step ), densityFunctions, materialIndex )
        );

        var negative = new float3(
            calculateDensity( position - new float3( step, 0.0f, 0.0f ), densityFunctions, materialIndex ),
            calculateDensity( position - new float3( 0.0f, step, 0.0f ), densityFunctions, materialIndex ),
            calculateDensity( position - new float3( 0.0f, 0.0f, step ), densityFunctions, materialIndex )
        );

        return normalize( positive - negative );
    }

    // SurfaceExtractor.calculateMaterialBoundary: the boundary between the materials at the ends of an edge is the
    // surface of the higher of the two, taken by itself
    public static void calculateMaterialBoundary(
        float3                            position,
        MaterialIndex                     materialIndex,
        float3                            otherPosition,
        MaterialIndex                     otherMaterialIndex,
        NativeArray<BurstDensityFunction> densityFunctions,
        IntersectionApproximation         intersectionApproximation,
        int                               binarySearchIterations,
        out float3                        intersection,
        out float3                        normal
    ) {
        var material = ( MaterialIndex )( 1 << math.max( findHighestMaterialBit( materialIndex ), findHighestMaterialBit( otherMaterialIndex ) ) );

        intersection = approximateIntersection(
            position,
            calculateDensity( position, densityFunctions, material ),
            otherPosition,
            calculateDensity( otherPosition, densityFunctions, material ),
            densityFunctions,
            material,
            intersectionApproximation,
            binarySearchIterations
        );

        normal = calculateNormal( intersection, densityFunctions, material );
    }

    // the index of the highest material bit, 0 when there is none
    public static int findHighestMaterialBit( MaterialIndex materialIndex ) {
        return ( int )materialIndex <= 0 ? 0 : 31 - math.lzcnt( ( int )materialIndex );
    }

    // whether an edge between corners of these materials runs between empty space and solid
    public static bool intersectsContour( MaterialIndex materialIndex, MaterialIndex otherMaterialIndex ) {
        return (
            materialIndex == MaterialIndex.Void && otherMaterialIndex >= MaterialIndex.Material0
        ) || (
            otherMaterialIndex == MaterialIndex.Void && materialIndex >= MaterialIndex.Material0
        );
    }

    // whether an edge between corners of these materials crosses the boundary between two of them
    public static bool crossesMaterialBoundary( MaterialIndex materialIndex, MaterialIndex otherMaterialIndex ) {
        return materialIndex != MaterialIndex.Void && otherMaterialIndex != MaterialIndex.Void && findHighestMaterialBit( materialIndex ) != findHighestMaterialBit( otherMaterialIndex );
    }

    // QEFSolver.surfaceCorrection
    public static float3 surfaceCorrection( float3 vertex, float3 normal, NativeArray<BurstDensityFunction> densityFunctions, int surfaceCorrectionIterations ) {
        for( var surfaceCorrectionIteration = 0; surfaceCorrectionIteration < surfaceCorrectionIterations; ++surfaceCorrectionIteration ) {
            var density = calculateDensity( vertex, densityFunctions );
            if( density == 0.0f ) {
                // vertex is at the surface
                break;
            }
            vertex -= normal * density;
        }
        return vertex;
    }

}

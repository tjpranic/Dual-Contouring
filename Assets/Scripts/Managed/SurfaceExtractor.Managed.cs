using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public partial interface SurfaceExtractor {

    public static MaterialIndex calculateMaterial( Vector3 position, IEnumerable<DensityFunction> densityFunctions ) {
        return densityFunctions.Aggregate(
            MaterialIndex.Void,
            ( materialIndex, densityFunction ) => {
                var density = densityFunction.sample( position );

                // set material bit if the corner is inside of the shape
                if( densityFunction.combination == DensityFunction.Combination.Union && density < 0.0f ) {
                    materialIndex |= densityFunction.materialIndex;
                }
                // unset material bit if the corner is outside of the shape
                if( densityFunction.combination == DensityFunction.Combination.Intersection && density > 0.0f ) {
                    materialIndex &= ~densityFunction.materialIndex;
                }
                // unset material bit if the corner is inside of the shape
                if( densityFunction.combination == DensityFunction.Combination.Subtraction && density < 0.0f ) {
                    materialIndex &= ~densityFunction.materialIndex;
                }

                return materialIndex;
            }
        );
    }

    public static float calculateDensity( Vector3 point, IEnumerable<DensityFunction> densityFunctions ) {
        return densityFunctions.Aggregate(
            float.MaxValue,
            ( density, densityFunction ) => densityFunction.combination switch {
                DensityFunction.Combination.Union        => Mathf.Min( density,  densityFunction.sample( point ) ),
                DensityFunction.Combination.Intersection => Mathf.Max( density,  densityFunction.sample( point ) ),
                DensityFunction.Combination.Subtraction  => Mathf.Max( density, -densityFunction.sample( point ) ),
                _                                        => throw new Exception( "Unknown combination mode specified" ),
            }
        );
    }

    public static Vector3 approximateIntersection( Edge edge, IEnumerable<DensityFunction> densityFunctions, IntersectionApproximation intersectionApproximation, int binarySearchIterations ) {
        return approximateIntersection(
            ( edge.corners[0].position, edge.corners[0].density ),
            ( edge.corners[1].position, edge.corners[1].density ),
            densityFunctions,
            intersectionApproximation,
            binarySearchIterations
        );
    }

    // where the density changes sign between two points, given the density at each of them
    public static Vector3 approximateIntersection(
        ( Vector3 position, float density ) corner,
        ( Vector3 position, float density ) other,
        IEnumerable<DensityFunction>        densityFunctions,
        IntersectionApproximation           intersectionApproximation,
        int                                 binarySearchIterations
    ) {
        if( corner.density == 0.0f || other.density == 0.0f ) {
            // one of the corners is at the exact intersection
            return corner.density == 0.0f ? corner.position : other.position;
        }
        if( intersectionApproximation == IntersectionApproximation.BinarySearch ) {
            var ( start, end ) = corner.density < other.density
                ? ( corner.position, other.position )
                : ( other.position,  corner.position );

            var intersection = Vector3.zero;
            for( var binarySearchIteration = 0; binarySearchIteration < binarySearchIterations; ++binarySearchIteration ) {
                intersection = start + ( 0.5f * ( end - start ) );

                var density = calculateDensity( intersection, densityFunctions );

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
        else if( intersectionApproximation == IntersectionApproximation.LinearInterpolation ) {
            return corner.position + ( ( -corner.density ) * ( other.position - corner.position ) / ( other.density - corner.density ) );
        }
        throw new Exception( "Unknown intersection approximation mode specified" );
    }

    public static Vector3 calculateNormal( Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
        return calculateNormal( edge.intersection, densityFunctions );
    }

    public static Vector3 calculateNormal( Vector3 position, IEnumerable<DensityFunction> densityFunctions ) {
        // the step has to be far smaller than a voxel, or the samples reach across sharp edges of the surface and the
        // normal becomes a blend of the faces on either side. It also has to stay well above the precision of a float,
        // or the difference is mostly rounding error
        var step = 1e-4f;

        // sample surrounding x, y, z locations and take the difference

        var positive = Vector3.positiveInfinity;

        positive.x = calculateDensity( position + new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        positive.y = calculateDensity( position + new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        positive.z = calculateDensity( position + new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        var negative = Vector3.positiveInfinity;

        negative.x = calculateDensity( position - new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        negative.y = calculateDensity( position - new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        negative.z = calculateDensity( position - new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        return Vector3.Normalize( positive - negative );
    }

    // Dual Contouring of Hermite Data, section 2.3: with several materials every edge whose ends differ in material
    // is tagged with an intersection and a normal, not only the edges between solid and empty. An edge between two
    // solid corners crosses the boundary between their materials. The material of a corner is the highest one it has,
    // the one that is drawn
    public static bool crossesMaterialBoundary( Edge edge ) {
        var materialIndex = edge.corners[0].materialIndex;
        var other         = edge.corners[1].materialIndex;

        return materialIndex != MaterialIndex.Void && other != MaterialIndex.Void && findHighestMaterialBit( materialIndex ) != findHighestMaterialBit( other );
    }

    // the intersection of an edge with the boundary between the materials at its ends, and the normal of that boundary
    // there. A volume only ever sets or clears its own material, so a material is present exactly where the volumes
    // of that material are solid when they are combined by themselves. And of the two materials at the ends of the
    // edge the higher one is absent from the other end, or it would have been the material there as well. The
    // boundary is therefore the surface of the higher material taken by itself
    public static ( Vector3 intersection, Vector3 normal ) calculateMaterialBoundary(
        Edge                         edge,
        IEnumerable<DensityFunction> densityFunctions,
        IntersectionApproximation    intersectionApproximation,
        int                          binarySearchIterations
    ) {
        var materialIndex = ( MaterialIndex )( 1 << Mathf.Max(
            findHighestMaterialBit( edge.corners[0].materialIndex ),
            findHighestMaterialBit( edge.corners[1].materialIndex )
        ) );

        var material = densityFunctions.Where( ( densityFunction ) => ( densityFunction.materialIndex & materialIndex ) != 0 ).ToList( );

        var intersection = approximateIntersection(
            ( edge.corners[0].position, calculateDensity( edge.corners[0].position, material ) ),
            ( edge.corners[1].position, calculateDensity( edge.corners[1].position, material ) ),
            material,
            intersectionApproximation,
            binarySearchIterations
        );

        return ( intersection, calculateNormal( intersection, material ) );
    }

    public static int findHighestMaterialBit( MaterialIndex materialIndex ) {
        // return index of highest set bit in material index
        for( var bitIndex = ( sizeof( int ) * 8 ) - 1; bitIndex >= 0; --bitIndex ) {
            if( ( ( int )materialIndex >> bitIndex ) > 0 ) {
                return bitIndex;
            }
        }

        Debug.Log( "Unable to find highest bit, no material set." );

        return 0;
    }

}

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public interface SurfaceExtractor {

    public interface Implementation {

        public enum Type {
            CPU,
            GPU
        }

        public (
            Mesh                mesh,
            IEnumerable<Corner> corners,
            IEnumerable<Edge>   edges,
            IEnumerable<Voxel>  voxels
        ) voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            QEFSolver.Type               qefSolverType,
            IntersectionApproximation    intersectionApproximation
        );

    }

    public enum IntersectionApproximation {
        BinarySearch,
        LinearInterpolation
    }

    [Flags]
    public enum MaterialIndex {
        Void      = 0,
        Material1 = 1,
        Material2 = 2,
        Material3 = 4,
        Material4 = 8,
        Material5 = 16,
        Material6 = 32,
        Material7 = 64,
        Material8 = 128,
        Material9 = 256
    }

    public interface Corner : IEquatable<Corner> {

        public Vector3       position      { get; }
        public float         density       { get; set; }
        public MaterialIndex materialIndex { get; set; }

    }

    public interface Edge : IEquatable<Edge> {

        public Vector3  intersection { get; set; }
        public Vector3  normal       { get; set; }
        public Corner[] corners      { get; }

        public bool intersectsContour( );

    }

    public interface Voxel : IEquatable<Voxel> {

        public enum Type {
            None     = -1,
            Internal =  0,
            Leaf     =  1,
            Pseudo   =  2
        }

        public Type     type    { get; set; }
        public Vector3  center  { get; }
        public Vector3  size    { get; }
        public Vector3  vertex  { get; }
        public Vector3  normal  { get; }
        public Corner[] corners { get; }
        public Edge[]   edges   { get; }

        public bool hasFeaturePoint( );

    }

    public int                       resolution                  { get; set; }
    public int                       minimizerIterations         { get; set; }
    public int                       binarySearchIterations      { get; set; }
    public int                       surfaceCorrectionIterations { get; set; }
    public Implementation.Type       implementationType          { get; set; }
    public QEFSolver.Type            qefSolverType               { get; set; }
    public IntersectionApproximation intersectionApproximation   { get; set; }

    public (
        Mesh                mesh,
        IEnumerable<Corner> corners,
        IEnumerable<Edge>   edges,
        IEnumerable<Voxel>  voxels
    ) voxelize( IEnumerable<DensityFunction> densityFunctions );

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
        if( edge.corners[0].density == 0.0f || edge.corners[1].density == 0.0f ) {
            // one of the corners is at the exact intersection
            return edge.corners[0].density == 0.0f ? edge.corners[0].position : edge.corners[1].position;
        }
        if( intersectionApproximation == IntersectionApproximation.BinarySearch ) {
            var ( start, end ) = edge.corners[0].density < edge.corners[1].density
                ? ( edge.corners[0].position, edge.corners[1].position )
                : ( edge.corners[1].position, edge.corners[0].position );

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
            return edge.corners[0].position + ( ( -edge.corners[0].density ) * ( edge.corners[1].position - edge.corners[0].position ) / ( edge.corners[1].density - edge.corners[0].density ) );
        }
        throw new Exception( "Unknown intersection approximation mode specified" );
    }

    public static Vector3 calculateNormal( Edge edge, IEnumerable<DensityFunction> densityFunctions ) {
        var step = 0.1f;

        // sample surrounding x, y, z locations and take the difference

        var positive = Vector3.positiveInfinity;

        positive.x = calculateDensity( edge.intersection + new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        positive.y = calculateDensity( edge.intersection + new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        positive.z = calculateDensity( edge.intersection + new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        var negative = Vector3.positiveInfinity;

        negative.x = calculateDensity( edge.intersection - new Vector3( step, 0.0f, 0.0f ), densityFunctions );
        negative.y = calculateDensity( edge.intersection - new Vector3( 0.0f, step, 0.0f ), densityFunctions );
        negative.z = calculateDensity( edge.intersection - new Vector3( 0.0f, 0.0f, step ), densityFunctions );

        return Vector3.Normalize( positive - negative );
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
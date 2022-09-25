using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public interface SurfaceExtractor {

    public enum Implementation {
        CPU,
        GPU
    }

    public enum IntersectionApproximationMode {
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

    public int minimizerIterations         { get; set; }
    public int binarySearchIterations      { get; set; }
    public int surfaceCorrectionIterations { get; set; }

    public Implementation                implementation                { get; set; }
    public QEFSolver.Type                qefSolver                     { get; set; }
    public IntersectionApproximationMode intersectionApproximationMode { get; set; }

    public IEnumerable<Corner> corners { get; }
    public IEnumerable<Edge>   edges   { get; }
    public IEnumerable<Voxel>  voxels  { get; }

    public Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions );

    public static ( float density, MaterialIndex material ) calculateDensityAndMaterial( Corner corner, DensityFunction densityFunction ) {
        var density  = densityFunction.sample( corner.position );
        var material = corner.materialIndex;

        // set material bit if the corner is inside of the shape
        if( densityFunction.combinationMode == DensityFunction.CombinationMode.Union && density < 0.0f ) {
            material |= densityFunction.materialIndex;
        }
        // unset material bit if the corner is outside of the shape
        if( densityFunction.combinationMode == DensityFunction.CombinationMode.Intersection && density > 0.0f ) {
            material &= ~densityFunction.materialIndex;
        }
        // unset material bit if the corner is inside of the shape
        if( densityFunction.combinationMode == DensityFunction.CombinationMode.Subtraction && density < 0.0f ) {
            material &= ~densityFunction.materialIndex;
        }

        density = densityFunction.combinationMode switch {
            DensityFunction.CombinationMode.Union        => Mathf.Min( corner.density,  density ),
            DensityFunction.CombinationMode.Intersection => Mathf.Max( corner.density,  density ),
            DensityFunction.CombinationMode.Subtraction  => Mathf.Max( corner.density, -density ),
            _                                            => throw new Exception( "Unknown combination mode specified" ),
        };

        return ( density, material );
    }

    public static float calculateDensity( Vector3 point, IEnumerable<DensityFunction> densityFunctions ) {
        return densityFunctions.Aggregate(
            float.MaxValue,
            ( density, densityFunction ) => densityFunction.combinationMode switch {
                DensityFunction.CombinationMode.Union        => Mathf.Min( density,  densityFunction.sample( point ) ),
                DensityFunction.CombinationMode.Intersection => Mathf.Max( density,  densityFunction.sample( point ) ),
                DensityFunction.CombinationMode.Subtraction  => Mathf.Max( density, -densityFunction.sample( point ) ),
                _                                            => throw new Exception( "Unknown combination mode specified" ),
            }
        );
    }

    public static Vector3 approximateIntersection( Edge edge, IEnumerable<DensityFunction> densityFunctions, IntersectionApproximationMode intersectionApproximationMode, int binarySearchIterations ) {
        if( edge.corners[0].density == 0.0f || edge.corners[1].density == 0.0f ) {
            // one of the corners is at the exact intersection
            return edge.corners[0].density == 0.0f ? edge.corners[0].position : edge.corners[1].position;
        }
        if( intersectionApproximationMode == IntersectionApproximationMode.BinarySearch ) {
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
        else if( intersectionApproximationMode == IntersectionApproximationMode.LinearInterpolation ) {
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

    public static int findHighestMaterialBit( Edge edge ) {
        var materialIndex = edge.corners[0].materialIndex == MaterialIndex.Void
            ? edge.corners[1].materialIndex
            : edge.corners[0].materialIndex;

        // return index of highest set bit in material index
        for( var bitIndex = ( sizeof( int ) * 8 ) - 1; bitIndex >= 0; --bitIndex ) {
            if( ( ( int )materialIndex >> bitIndex ) > 0 ) {
                return bitIndex;
            }
        }

        throw new Exception( "Unable to calculate sub mesh index, no material set" );
    }

}
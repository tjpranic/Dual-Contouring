using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public interface SurfaceExtractor {

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
        public MaterialIndex materialIndex { get; }

    }

    public interface Edge : IEquatable<Edge> {

        public Corner[] corners { get; }

        public bool intersectsContour( );

    }

    public interface Voxel : IEquatable<Voxel> {

        public Vector3 center { get; }
        public Vector3 size   { get; }
        public Vector3 vertex { get; }
        public Vector3 normal { get; }

        public bool hasFeaturePoint( );

    }

    public IEnumerable<Corner> corners { get; }
    public IEnumerable<Edge>   edges   { get; }
    public IEnumerable<Voxel>  voxels  { get; }

    public Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions );

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

}
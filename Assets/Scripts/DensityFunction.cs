using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public interface DensityFunction {

    public struct Data {

        public Type          type;
        public Combination   combination;
        public MaterialIndex materialIndex;
        public Vector3       origin;
        public Vector3       extents;

        public Data( DensityFunction densityFunction ) {
            this.type          = densityFunction.type;
            this.combination   = densityFunction.combination;
            this.materialIndex = densityFunction.materialIndex;
            this.origin        = densityFunction.origin;
            this.extents       = densityFunction.extents;
        }

    }

    public enum Type {
        Cuboid,
        Ellipsoid
    }

    public enum Combination {
        Union,
        Intersection,
        Subtraction
    }

    // a density function as it stands, with its origin and extents worked out once. A volume works them out from its
    // transform and its voxelizer's on every sample, which costs far more than the sample itself
    public sealed class Snapshot : DensityFunction {

        public DensityFunction.Type type          { get; }
        public Vector3              origin        { get; }
        public Vector3              extents       { get; }
        public Combination          combination   { get; set; }
        public MaterialIndex        materialIndex { get; set; }

        public Snapshot( DensityFunction densityFunction ) {
            this.type          = densityFunction.type;
            this.origin        = densityFunction.origin;
            this.extents       = densityFunction.extents;
            this.combination   = densityFunction.combination;
            this.materialIndex = densityFunction.materialIndex;
        }

        public float sample( Vector3 position ) {
            return this.type switch {
                DensityFunction.Type.Cuboid    => Cuboid.sample    ( position, this.origin, this.extents ),
                DensityFunction.Type.Ellipsoid => Ellipsoid.sample ( position, this.origin, this.extents ),
                _                              => throw new Exception( "Unknown density function type specified" )
            };
        }

    }

    // the given density functions in the same order, with every volume among them replaced by a snapshot of it. Other
    // density functions do not depend on transforms and are kept as they are
    public static List<DensityFunction> snapshot( IEnumerable<DensityFunction> densityFunctions ) {
        return densityFunctions.Select(
            ( densityFunction ) => densityFunction is Volume ? new Snapshot( densityFunction ) : densityFunction
        ).ToList( );
    }

    public Type          type          { get; }
    public Vector3       origin        { get; }
    public Vector3       extents       { get; }
    public Combination   combination   { get; set; }
    public MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

}
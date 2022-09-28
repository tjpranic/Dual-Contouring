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

    public Type          type          { get; }
    public Vector3       origin        { get; }
    public Vector3       extents       { get; }
    public Combination   combination   { get; set; }
    public MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

}
using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public partial interface DensityFunction {

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

}

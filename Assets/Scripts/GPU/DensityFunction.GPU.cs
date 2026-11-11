using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public partial interface DensityFunction {

    // a density function as the compute shader reads it. The fields and their order are those of struct DensityFunction
    // in UniformDualContouring.compute, and the buffer's stride is the size of this struct
    public struct Data {

        public Type          type;
        public Combination   combination;
        public MaterialIndex materialIndex;
        public Vector3       origin;
        public Vector3       extents;

        // the shape's frame, copied from Frame: the rows of the inverse rotation. The shader takes every shape into its
        // frame, so it needs no flag for whether there is a rotation to undo
        public Vector3 xAxis;
        public Vector3 yAxis;
        public Vector3 zAxis;

        public Data( DensityFunction densityFunction ) {
            var frame = new Frame( densityFunction );

            this.type          = densityFunction.type;
            this.combination   = densityFunction.combination;
            this.materialIndex = densityFunction.materialIndex;
            this.origin        = frame.origin;
            this.extents       = densityFunction.extents;
            this.xAxis         = frame.xAxis;
            this.yAxis         = frame.yAxis;
            this.zAxis         = frame.zAxis;
        }

    }

}

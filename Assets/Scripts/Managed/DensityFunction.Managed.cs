using System.Collections.Generic;
using System.Linq;
using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public partial interface DensityFunction {

    // a density function as it stands, with its origin, rotation and extents worked out once, and its frame with them.
    // A volume works them out from its transform and its voxelizer's on every sample, which costs far more than the
    // sample itself
    public sealed class Snapshot : DensityFunction {

        public DensityFunction.Type type          { get; }
        public Vector3              origin        { get; }
        public Quaternion           rotation      { get; }
        public Vector3              extents       { get; }
        public Combination          combination   { get; set; }
        public MaterialIndex        materialIndex { get; set; }

        private readonly Frame frame;

        public Snapshot( DensityFunction densityFunction ) {
            this.type          = densityFunction.type;
            this.origin        = densityFunction.origin;
            this.rotation      = densityFunction.rotation;
            this.extents       = densityFunction.extents;
            this.combination   = densityFunction.combination;
            this.materialIndex = densityFunction.materialIndex;

            this.frame = new Frame( this.origin, this.rotation );
        }

        public float sample( Vector3 position ) {
            return DensityFunction.sample( this.type, this.frame.local( position ), this.extents );
        }

    }

    // the given density functions in the same order, with every volume among them replaced by a snapshot of it. Other
    // density functions do not depend on transforms and are kept as they are
    public static List<DensityFunction> snapshot( IEnumerable<DensityFunction> densityFunctions ) {
        return densityFunctions.Select(
            ( densityFunction ) => densityFunction is Volume ? new Snapshot( densityFunction ) : densityFunction
        ).ToList( );
    }

}

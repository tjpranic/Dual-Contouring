using System.Collections.Generic;
using UnityEngine;

public class AdaptiveDualContouring : Voxelizer {

    // TODO

    public override IEnumerable<SurfaceExtractor.Corner> corners {
        get { throw new System.NotImplementedException( ); }
    }

    public override IEnumerable<SurfaceExtractor.Edge> edges {
        get { throw new System.NotImplementedException( ); }
    }

    public override IEnumerable<SurfaceExtractor.Voxel> voxels {
        get { throw new System.NotImplementedException( ); }
    }

    public override Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions ) {
        throw new System.NotImplementedException( );
    }
}
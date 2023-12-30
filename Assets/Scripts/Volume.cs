using UnityEngine;

using DensityFunctionType = DensityFunction.Type;
using Combination         = DensityFunction.Combination;
using MaterialIndex       = SurfaceExtractor.MaterialIndex;

public abstract class Volume : MonoBehaviour, DensityFunction {

    public void Start( ) {
        Debug.Assert( this.GetComponentInParent<Voxelizer>( ) != null );
    }

    public abstract DensityFunctionType type { get; }

    // these 2 accessors transform to voxelizer object space because volumes are voxelized in voxelizer object space

    public Vector3 origin {
        get {
            return this.transform.localPosition.divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale );
        }
    }

    public Vector3 extents {
        get {
            return this.transform.localScale.divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale ) / 2;
        }
    }

    [SerializeField( )]
    private Combination _combination = Combination.Union;
    public Combination combination {
        get { return this._combination;  }
        set { this._combination = value; }
    }

    [SerializeField( )]
    private MaterialIndex _materialIndex = MaterialIndex.Void;
    public MaterialIndex materialIndex {
        get { return this._materialIndex;  }
        set { this._materialIndex = value; }
    }

    public abstract float sample( Vector3 position );

}
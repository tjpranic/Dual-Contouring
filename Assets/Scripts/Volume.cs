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

    // read the same way as the origin, off the local transform. Dividing by the voxelizer's scale keeps the angles of a
    // turned volume only when that scale is the same on every axis. Under any other scale the volume would be sheared,
    // which a rotation cannot describe, so the rotation is taken as it is
    public Quaternion rotation {
        get {
            return this.transform.localRotation;
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
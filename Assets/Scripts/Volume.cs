using UnityEngine;

public abstract class Volume : MonoBehaviour, DensityFunction {

    public void Start( ) {
        Debug.Assert( this.GetComponentInParent<Voxelizer>( ) != null );
    }

    // these 2 accessors transform to voxelizer object space because volumes are voxelized in voxelizer object space

    public Vector3 origin {
        get { return this.transform.localPosition.divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale ); }
    }

    public Vector3 extents {
        get { return this.transform.localScale.divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale ) / 2; }
    }

    [SerializeField( )]
    private DensityFunction.CombinationMode _combinationMode = DensityFunction.CombinationMode.Union;
    public DensityFunction.CombinationMode combinationMode {
        get { return this._combinationMode;  }
        set { this._combinationMode = value; }
    }

    [SerializeField( )]
    private SurfaceExtractor.MaterialIndex _materialIndex = SurfaceExtractor.MaterialIndex.Void;
    public SurfaceExtractor.MaterialIndex materialIndex {
        get { return this._materialIndex;  }
        set { this._materialIndex = value; }
    }

    public abstract float sample( Vector3 position );

}
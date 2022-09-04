using UnityEngine;

public abstract class Volume : MonoBehaviour, DensityFunction {

    public DensityFunction.CombinationMode combinationMode = DensityFunction.CombinationMode.Union;

    public void Start( ) {
        Debug.Assert( this.GetComponentInParent<Voxelizer>( ) != null );
    }

    // these 2 methods transform to voxelizer object space because volumes are voxelized in voxelizer object space

    public Vector3 getOrigin( ) {
        return this.transform.localPosition.Divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale );
    }

    public Vector3 getExtents( ) {
        return this.transform.localScale.Divide( this.GetComponentInParent<Voxelizer>( ).transform.localScale ) / 2;
    }

    public DensityFunction.CombinationMode getCombinationMode( ) {
        return combinationMode;
    }

    public abstract float sample( Vector3 position );

}
using System.Collections.Generic;
using UnityEngine;

public interface QEFSolver<T> where T : QEFSolver<T> {

    public Vector3 vertex { get; }
    public Vector3 normal { get; }
    public float   error  { get; }

    public void add( Vector3 intersection, Vector3 normal );

    public void combine( T other );

    public void solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions );

}
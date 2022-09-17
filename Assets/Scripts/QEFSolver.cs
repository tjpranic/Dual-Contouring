using System.Collections.Generic;
using UnityEngine;

public interface QEFSolver {

    public Vector3 vertex { get; }
    public Vector3 normal { get; }
    public float   error  { get; }

    public void add( Vector3 intersection, Vector3 normal );

    public void solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions );

}
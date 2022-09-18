using System.Collections.Generic;
using UnityEngine;

public interface QEFSolver<T> where T : QEFSolver<T> {

    public bool empty { get; }

    public void add( Vector3 intersection, Vector3 normal );

    public void combine( T other );

    public ( Vector3 vertex, Vector3 normal, float error ) solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions );

}
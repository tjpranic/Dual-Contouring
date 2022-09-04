using System;
using System.Collections.Generic;
using UnityEngine;

public interface SurfaceExtractor {

    public interface Corner : IEquatable<Corner> {

        public enum Sign {
            None,
            Inside,
            Outside
        }

        public Vector3 getPosition( );

        public Sign getSign( );

    }

    public interface Edge : IEquatable<Edge> {

        public Corner[] getCorners( );

    }

    public interface Voxel : IEquatable<Voxel> {

        public Vector3 getCenter( );

        public Vector3 getSize( );

        public Vector3 getVertex( );

        public Vector3 getNormal( );

        public bool intersectsContour( );

    }

    public IEnumerable<Corner> getCorners( );

    public IEnumerable<Edge> getEdges( );

    public IEnumerable<Voxel> getVoxels( );

    public Mesh voxelize( int resolution, IEnumerable<DensityFunction> densityFunctions );

}
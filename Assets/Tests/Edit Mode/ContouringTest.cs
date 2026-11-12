using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using QEFSolverType      = QEFSolver.Type;

// shared setup for tests that contour a few volumes: builds a voxelizer with its volumes and cleans up after the test
public abstract class ContouringTest {

    // a volume in voxelizer object space, where the voxel grid spans -0.5 to 0.5 on every axis
    public readonly struct Shape {

        public readonly DensityFunction.Type type;
        public readonly Vector3              origin;
        public readonly Quaternion           rotation;
        public readonly Vector3              extents;
        public readonly Combination          combination;
        public readonly MaterialIndex        materialIndex;

        public Shape( DensityFunction.Type type, Vector3 origin, Vector3 extents, Combination combination, MaterialIndex materialIndex )
            : this( type, origin, Quaternion.identity, extents, combination, materialIndex ) {
        }

        public Shape( DensityFunction.Type type, Vector3 origin, Quaternion rotation, Vector3 extents, Combination combination, MaterialIndex materialIndex ) {
            this.type          = type;
            this.origin        = origin;
            this.rotation      = rotation;
            this.extents       = extents;
            this.combination   = combination;
            this.materialIndex = materialIndex;
        }

        // the same shape with the given rotation
        public Shape rotated( Quaternion rotation ) {
            return new( this.type, this.origin, rotation, this.extents, this.combination, this.materialIndex );
        }

    }

    private readonly List<Object>    created    = new( );
    private readonly List<Voxelizer> voxelizers = new( );

    // the implementation the voxelizers of a fixture use, which a fixture that runs for several of them sets
    protected ImplementationType implementation = ImplementationType.Managed;

    protected static Shape ellipsoid( Vector3 origin, Vector3 extents, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
        return new( DensityFunction.Type.Ellipsoid, origin, extents, combination, materialIndex );
    }

    protected static Shape sphere( Vector3 origin, float radius, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
        return ellipsoid( origin, Vector3.one * radius, combination, materialIndex );
    }

    protected static Shape cuboid( Vector3 origin, Vector3 extents, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
        return new( DensityFunction.Type.Cuboid, origin, extents, combination, materialIndex );
    }

    protected static Shape cube( Vector3 origin, float extent, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
        return cuboid( origin, Vector3.one * extent, combination, materialIndex );
    }

    [TearDown]
    public void tearDown( ) {
        // OnDestroy releases what the implementations hold outside of managed memory, such as the compute buffers of
        // the GPU implementation, and edit mode does not call it by itself
        foreach( var voxelizer in this.voxelizers ) {
            voxelizer.OnDestroy( );
        }
        this.voxelizers.Clear( );

        foreach( var item in this.created ) {
            Object.DestroyImmediate( item );
        }
        this.created.Clear( );
    }

    // creates a voxelizer of unit scale with a child volume for every shape. Nothing is saved to or shown in the open
    // scene, and everything is destroyed again when the test ends
    protected T createVoxelizer<T>( int resolution, QEFSolverType solver, IEnumerable<Shape> shapes ) where T : Voxelizer {
        var gameObject = new GameObject( typeof( T ).Name ) { hideFlags = HideFlags.HideAndDontSave };

        this.created.Add( gameObject );

        var voxelizer = gameObject.AddComponent<T>( );

        this.voxelizers.Add( voxelizer );

        // the tests look at the corners, edges and voxels of the contours, which the Burst implementations only copy out
        // when asked to
        voxelizer.keepDebugInformation = true;

        voxelizer.implementationType = this.implementation;

        voxelizer.resolution    = resolution;
        voxelizer.qefSolverType = solver;

        // vertices stay where the solver puts them, as in the papers
        voxelizer.surfaceCorrectionIterations = 0;

        foreach( var shape in shapes ) {
            var child = new GameObject( shape.type.ToString( ) ) { hideFlags = HideFlags.HideAndDontSave };

            // a volume reads its origin, rotation and extents off its transform, relative to the scale of the voxelizer
            child.transform.SetParent( gameObject.transform, false );
            child.transform.localPosition = shape.origin;
            child.transform.localRotation = shape.rotation;
            child.transform.localScale    = shape.extents * 2;

            var volume = ( Volume )child.AddComponent( Volume.componentOf( shape.type ) );

            volume.combination   = shape.combination;
            volume.materialIndex = shape.materialIndex;
        }

        return voxelizer;
    }

    protected UniformDualContouring createUniformVoxelizer( int resolution, ImplementationType implementation, QEFSolverType solver, IEnumerable<Shape> shapes ) {
        var voxelizer = this.createVoxelizer<UniformDualContouring>( resolution, solver, shapes );

        voxelizer.implementationType = implementation;

        // Start creates the CPU and GPU implementations, and with testing set it does not go on to contour
        voxelizer.testing = true;
        voxelizer.Start( );

        return voxelizer;
    }

    // contours the volumes below the voxelizer with its current settings, the same way Voxelizer.Start does
    protected CPUVoxelization voxelize( Voxelizer voxelizer ) {
        return this.voxelize( voxelizer, voxelizer.GetComponentsInChildren<Volume>( ) );
    }

    // contours the given density functions with the settings of the voxelizer, whatever volumes are below it
    protected CPUVoxelization voxelize( Voxelizer voxelizer, IEnumerable<DensityFunction> densityFunctions ) {
        var voxelization = voxelizer.voxelize( densityFunctions ).visit(
            ( cpu ) => cpu,
            ( gpu ) => voxelizer.convert( gpu )
        );

        this.created.Add( voxelization.mesh );

        return voxelization;
    }

}

using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public partial interface DensityFunction {

    // the GPU reads these as numbers, so new types go at the end
    public enum Type {
        Cuboid,
        Ellipsoid,
        Cylinder,
        Cone,
        Capsule,
        Torus,
        Pyramid,
        Octahedron,
        HexagonalPrism
    }

    public enum Combination {
        Union,
        Intersection,
        Subtraction
    }

    // the shape sits at its origin, turned by its rotation about it, and its extents lie along its own axes. All three are
    // in voxelizer object space
    public Type          type          { get; }
    public Vector3       origin        { get; }
    public Quaternion    rotation      { get; }
    public Vector3       extents       { get; }
    public Combination   combination   { get; set; }
    public MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

    // a shape of the type, centred on the origin of its own frame, at a position in that frame. Every managed density
    // function samples through this once it has taken the position into its frame
    public static float sample( Type type, Vector3 position, Vector3 extents ) {
        return type switch {
            Type.Cuboid         => Cuboid.sample         ( position, extents ),
            Type.Ellipsoid      => Ellipsoid.sample      ( position, extents ),
            Type.Cylinder       => Cylinder.sample       ( position, extents ),
            Type.Cone           => Cone.sample           ( position, extents ),
            Type.Capsule        => Capsule.sample        ( position, extents ),
            Type.Torus          => Torus.sample          ( position, extents ),
            Type.Pyramid        => Pyramid.sample        ( position, extents ),
            Type.Octahedron     => Octahedron.sample     ( position, extents ),
            Type.HexagonalPrism => HexagonalPrism.sample ( position, extents ),
            _                   => throw new System.Exception( "Unknown density function type specified" )
        };
    }

    // a shape's own frame: centred on its origin and turned with it, so that its extents lie along the axes. A shape is
    // sampled by taking the position into its frame and applying its formula there. The rotation is undone with the rows
    // of its inverse, worked out here once, so that every implementation undoes it with the same numbers. A shape that is
    // not rotated keeps the offset from its origin as it is, and samples exactly as it did before shapes could rotate
    public readonly struct Frame {

        public readonly Vector3 origin;

        // the rows of the inverse rotation, which are the shape's own axes in voxelizer object space
        public readonly Vector3 xAxis;
        public readonly Vector3 yAxis;
        public readonly Vector3 zAxis;

        // whether the rotation turns anything. It does not when its rows are exactly the axes, as a rotation by no angle
        // and a zero quaternion give
        public readonly bool rotated;

        public Frame( Vector3 origin, Quaternion rotation ) {
            // a quaternion that is not of unit length would scale the shape as well. Normalize turns a zero quaternion,
            // which a density function that never sets a rotation would give, into the identity
            rotation = Quaternion.Normalize( rotation );

            this.origin = origin;

            this.xAxis = rotation * Vector3.right;
            this.yAxis = rotation * Vector3.up;
            this.zAxis = rotation * Vector3.forward;

            // Equals compares exactly, == only to within 1e-5
            this.rotated = !( this.xAxis.Equals( Vector3.right ) && this.yAxis.Equals( Vector3.up ) && this.zAxis.Equals( Vector3.forward ) );
        }

        public Frame( DensityFunction densityFunction ) : this( densityFunction.origin, densityFunction.rotation ) {
        }

        // the position relative to the origin, along the shape's own axes
        public Vector3 local( Vector3 position ) {
            var offset = position - this.origin;

            if( !this.rotated ) {
                return offset;
            }

            return new Vector3(
                ( this.xAxis.x * offset.x ) + ( this.xAxis.y * offset.y ) + ( this.xAxis.z * offset.z ),
                ( this.yAxis.x * offset.x ) + ( this.yAxis.y * offset.y ) + ( this.yAxis.z * offset.z ),
                ( this.zAxis.x * offset.x ) + ( this.zAxis.y * offset.y ) + ( this.zAxis.z * offset.z )
            );
        }

    }

}

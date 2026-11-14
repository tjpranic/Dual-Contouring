using System;
using System.Collections.Generic;
using UnityEngine;

public partial interface SurfaceExtractor {

    public interface Implementation {

        public class CPU {

            public record Voxelization {
                public Mesh                mesh;
                public IEnumerable<Corner> corners;
                public IEnumerable<Edge>   edges;
                public IEnumerable<Voxel>  voxels;
            }

        }

        public class GPU {

            public record Voxelization {
                public ComputeBuffer vertices;
                public ComputeBuffer normals;
                public ComputeBuffer quads;
                public ComputeBuffer corners;
                public ComputeBuffer edges;
                public ComputeBuffer voxels;
                public ComputeBuffer counts;
                public ComputeBuffer arguments;
            }

        }

        // the values are what scenes save, so they stay as they are
        public enum Type {
            Managed = 0,
            GPU     = 1,
            Burst   = 2
        }

        public Either<CPU.Voxelization, GPU.Voxelization> voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            QEFSolver.Type               qefSolverType,
            IntersectionApproximation    intersectionApproximation,
            ContourVertices              contourVertices,
            float                        creaseAngle,
            SmoothNormals                smoothNormals
        );

    }

    public enum IntersectionApproximation {
        BinarySearch,
        LinearInterpolation
    }

    // the normals of the mesh a voxelizer shows: creased by angle (BurstCreases), smooth across curved surfaces and
    // broken at sharp edges, or split for flat shading. Creased has the value Shared had, which it replaced, so a
    // scene saved with Shared vertex normals has creased ones
    public enum VertexNormals {
        Creased,
        Split
    }

    // what an implementation is asked to give: vertices that its triangles share, with the contour's normals, which the
    // voxelizer simplifies afterwards; vertices split for flat shading; or creased vertices. The GPU implementations
    // split and crease themselves, and the others give shared vertices for the voxelizer to split or crease
    public enum ContourVertices {
        Shared,
        Split,
        Creased
    }

    // the normal a creased vertex takes when its triangles all meet within the crease angle (BurstCreases): the normal
    // the contour gave it, where that normal is within the angle of every one of its triangles, or the normal of its
    // triangles
    public enum SmoothNormals {
        Contour,
        Faces
    }

    [Flags]
    public enum MaterialIndex {
        Void      = 0,
        Material0 = 1,
        Material1 = 2,
        Material2 = 4,
        Material3 = 8,
        Material4 = 16,
        Material5 = 32,
        Material6 = 64,
        Material7 = 128,
        Material8 = 256,
        Material9 = 512
    }

    public interface Corner : IEquatable<Corner> {

        public Vector3       position      { get; }
        public float         density       { get; set; }
        public MaterialIndex materialIndex { get; set; }

    }

    public interface Edge : IEquatable<Edge> {

        public Vector3  intersection { get; set; }
        public Vector3  normal       { get; set; }
        public Corner[] corners      { get; }

        public bool intersectsContour( );

    }

    public interface Voxel : IEquatable<Voxel> {

        public const int FaceCount   = 6;
        public const int CornerCount = 8;
        public const int EdgeCount   = 12;
        
        public enum Type {
            None     = -1,
            Internal =  0,
            Leaf     =  1,
            Pseudo   =  2
        }

        public Type     type    { get; set; }
        public Vector3  center  { get; }
        public Vector3  size    { get; }
        public Vector3  extents { get; }
        public Vector3  minimum { get; }
        public Vector3  maximum { get; }
        public Vector3  vertex  { get; }
        public Vector3  normal  { get; }
        public Corner[] corners { get; }
        public Edge[]   edges   { get; }

        // whether a minimizing vertex was placed in the voxel. This is kept separately from the vertex and normal
        // because any value of those, including zero, is a valid result
        public bool hasFeaturePoint { get; }

    }

    public int                       resolution                  { get; set; }
    public int                       minimizerIterations         { get; set; }
    public int                       binarySearchIterations      { get; set; }
    public int                       surfaceCorrectionIterations { get; set; }
    public Implementation.Type       implementationType          { get; set; }
    public QEFSolver.Type            qefSolverType               { get; set; }
    public IntersectionApproximation intersectionApproximation   { get; set; }
    public VertexNormals             vertexNormals               { get; set; }

    public Either<Implementation.CPU.Voxelization, Implementation.GPU.Voxelization> voxelize( IEnumerable<DensityFunction> densityFunctions );

    // the mesh of a contour. Sub-mesh n holds the polygons on the surface of material bit n, which the mesh renderer
    // draws with its n-th material and the GPU with its n-th colour, so a material that is not on the surface leaves its
    // sub-mesh empty. Past 65,535 vertices the indices need 32 bits
    public static Mesh createMesh( Vector3[] vertices, Vector3[] normals, IEnumerable<( int subMeshIndex, int[] indices )> polygons ) {
        var mesh = new Mesh {
            indexFormat = vertices.Length > ushort.MaxValue ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16,
            vertices    = vertices,
            normals     = normals
        };

        var subMeshes = new List<List<int>>( );

        foreach( var ( subMeshIndex, indices ) in polygons ) {
            while( subMeshes.Count <= subMeshIndex ) {
                subMeshes.Add( new( ) );
            }
            subMeshes[subMeshIndex].AddRange( indices );
        }

        mesh.subMeshCount = subMeshes.Count;

        for( var subMeshIndex = 0; subMeshIndex < subMeshes.Count; ++subMeshIndex ) {
            mesh.SetTriangles( subMeshes[subMeshIndex], subMeshIndex );
        }

        return mesh;
    }

    // a hash of a position that tells the points of a grid apart, for corners, which are equal when their positions are.
    // Vector3.GetHashCode shifts the hashes of the coordinates by a couple of bits and XORs them, which on the few
    // mantissa bits of the coordinates of a grid gives one hash to many points
    public static int hashOf( Vector3 position ) {
        return HashCode.Combine( position.x, position.y, position.z );
    }

}
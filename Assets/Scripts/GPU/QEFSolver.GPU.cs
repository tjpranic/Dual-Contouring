public partial interface QEFSolver {

    // the solver a compute shader is given for the type, as QEF.hlsl numbers them. The shaders have no simple solver, and
    // solve with SVD in its place
    public static int shaderSolverOf( Type type ) {
        return ( int )( type == Type.Probabilistic ? Type.Probabilistic : Type.SVD );
    }

}

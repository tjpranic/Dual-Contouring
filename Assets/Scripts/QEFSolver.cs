public partial interface QEFSolver {

    // the values are what scenes save, so they stay as they are
    public enum Type {
        Simple,
        SVD,
        Probabilistic
    }

}

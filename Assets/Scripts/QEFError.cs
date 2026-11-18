// the sums of a QEF a second time, in doubles, for its error alone: E[x] = xT AtA x - 2 xT AtB + BtB, the sum over the
// hermite samples of the squared distance from x to the plane of the sample. Close to the surface that is a small
// difference of three large terms, which grow with the number of samples and with their distance from the origin. In
// floats their rounding is larger than the error itself: the octant of a cube of 0.6 at the origin, at a resolution of 8,
// has an error of exactly zero, and of 1.2e-4 in floats, which a threshold of 1e-5 refuses. A cube of 0.5 has nothing
// but powers of two in it and rounds to zero. In doubles the rounding is some 1e9 times smaller. The solvers still solve
// with their float sums, so the vertices stay what they were, and the same as the GPU's, which has no doubles. A plain
// struct, which managed code and Burst code both keep, and work out operation for operation the same
public struct QEFError {

    // AtA, of which only the upper triangle is kept, AtB and BtB
    public double ata00, ata01, ata02, ata11, ata12, ata22;
    public double atb0, atb1, atb2;
    public double btb;

    // the sums of the given float ones, which have already been rounded
    public static QEFError of( float ata00, float ata01, float ata02, float ata11, float ata12, float ata22, float atb0, float atb1, float atb2, float btb ) {
        return new QEFError {
            ata00 = ata00, ata01 = ata01, ata02 = ata02, ata11 = ata11, ata12 = ata12, ata22 = ata22,
            atb0  = atb0,  atb1  = atb1,  atb2  = atb2,
            btb   = btb
        };
    }

    public void add( float intersectionX, float intersectionY, float intersectionZ, float normalX, float normalY, float normalZ ) {
        double nx = normalX;
        double ny = normalY;
        double nz = normalZ;

        var dot = ( intersectionX * nx ) + ( intersectionY * ny ) + ( intersectionZ * nz );

        this.ata00 += nx * nx;
        this.ata01 += nx * ny;
        this.ata02 += nx * nz;
        this.ata11 += ny * ny;
        this.ata12 += ny * nz;
        this.ata22 += nz * nz;

        this.atb0 += dot * nx;
        this.atb1 += dot * ny;
        this.atb2 += dot * nz;

        this.btb += dot * dot;
    }

    public void combine( in QEFError other ) {
        this.ata00 += other.ata00;
        this.ata01 += other.ata01;
        this.ata02 += other.ata02;
        this.ata11 += other.ata11;
        this.ata12 += other.ata12;
        this.ata22 += other.ata22;

        this.atb0 += other.atb0;
        this.atb1 += other.atb1;
        this.atb2 += other.atb2;

        this.btb += other.btb;
    }

    // the value of the QEF at x. Rounding can still leave it just below zero, where it is zero
    public readonly float valueAt( float x, float y, float z ) {
        double vx = x;
        double vy = y;
        double vz = z;

        var ax = ( this.ata00 * vx ) + ( this.ata01 * vy ) + ( this.ata02 * vz );
        var ay = ( this.ata01 * vx ) + ( this.ata11 * vy ) + ( this.ata12 * vz );
        var az = ( this.ata02 * vx ) + ( this.ata12 * vy ) + ( this.ata22 * vz );

        var error = ( ( vx * ax ) + ( vy * ay ) + ( vz * az ) ) - ( 2.0 * ( ( vx * this.atb0 ) + ( vy * this.atb1 ) + ( vz * this.atb2 ) ) ) + this.btb;

        return error > 0.0 ? ( float )error : 0.0f;
    }

}

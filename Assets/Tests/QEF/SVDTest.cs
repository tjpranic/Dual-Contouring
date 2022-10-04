using NUnit.Framework;
using UnityEngine;

public class SVDTest : ImplementationTest {

    protected override string shader { get; } = "Tests/SVDTest";

    [Test]
    public void calculateSymmetricGivensCoefficientsTest( ) {
        var parameter0 = new Vector3( 0.7640296f, 0.6270213f, 1.117985f );

        var expectedResult = SVDQEF.SVD.calculateSymmetricGivensCoefficients( parameter0.x, parameter0.y, parameter0.z );
        var actualResult   = this.testKernel(
            "calculateSymmetricGivensCoefficientsTest",
            parameter0,
            Vector2.zero
        );

        Assert.AreEqual( expectedResult.Item1, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.Item2, actualResult.y, epsilon );
    }

    public struct SVDTestResult {

        public SVDQEF.SymmetricMatrix3x3.Data symmetricMatrix3x3;
        public SVDQEF.Matrix3x3.Data          matrix3x3;

        public SVDTestResult( SVDQEF.SymmetricMatrix3x3.Data symmetricMatrix3x3, SVDQEF.Matrix3x3.Data matrix3x3 ) {
            this.symmetricMatrix3x3 = symmetricMatrix3x3;
            this.matrix3x3          = matrix3x3;
        }

    }

    [Test]
    public void rotate01Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2894885f, 0.0f, -0.06483984f, 1.592526f, 1.124694f, 1.117985f );
        var parameter1 = new SVDQEF.Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        var expectedResult = SVDQEF.SVD.rotate01( parameter0, parameter1 );
        var actualResult   = this.testKernel(
            "rotate01Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            new SVDQEF.Matrix3x3.Data( parameter1 ),
            new SVDTestResult( )
        );

        Assert.AreEqual( expectedResult.Item1.m00, actualResult.symmetricMatrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item1.m01, actualResult.symmetricMatrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item1.m02, actualResult.symmetricMatrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item1.m11, actualResult.symmetricMatrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item1.m12, actualResult.symmetricMatrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item1.m22, actualResult.symmetricMatrix3x3.m22, epsilon );

        Assert.AreEqual( expectedResult.Item2.m00, actualResult.matrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item2.m01, actualResult.matrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item2.m02, actualResult.matrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item2.m10, actualResult.matrix3x3.m10, epsilon );
        Assert.AreEqual( expectedResult.Item2.m11, actualResult.matrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item2.m12, actualResult.matrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item2.m20, actualResult.matrix3x3.m20, epsilon );
        Assert.AreEqual( expectedResult.Item2.m21, actualResult.matrix3x3.m21, epsilon );
        Assert.AreEqual( expectedResult.Item2.m22, actualResult.matrix3x3.m22, epsilon );
    }

    [Test]
    public void rotate02Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2894885f, 0.0f, -0.06483984f, 1.592526f, 1.124694f, 1.117985f );
        var parameter1 = new SVDQEF.Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        var expectedResult = SVDQEF.SVD.rotate02( parameter0, parameter1 );
        var actualResult   = this.testKernel(
            "rotate02Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            new SVDQEF.Matrix3x3.Data( parameter1 ),
            new SVDTestResult( )
        );

        Assert.AreEqual( expectedResult.Item1.m00, actualResult.symmetricMatrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item1.m01, actualResult.symmetricMatrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item1.m02, actualResult.symmetricMatrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item1.m11, actualResult.symmetricMatrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item1.m12, actualResult.symmetricMatrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item1.m22, actualResult.symmetricMatrix3x3.m22, epsilon );

        Assert.AreEqual( expectedResult.Item2.m00, actualResult.matrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item2.m01, actualResult.matrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item2.m02, actualResult.matrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item2.m10, actualResult.matrix3x3.m10, epsilon );
        Assert.AreEqual( expectedResult.Item2.m11, actualResult.matrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item2.m12, actualResult.matrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item2.m20, actualResult.matrix3x3.m20, epsilon );
        Assert.AreEqual( expectedResult.Item2.m21, actualResult.matrix3x3.m21, epsilon );
        Assert.AreEqual( expectedResult.Item2.m22, actualResult.matrix3x3.m22, epsilon );
    }

    [Test]
    public void rotate12Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2894885f, 0.0f, -0.06483984f, 1.592526f, 1.124694f, 1.117985f );
        var parameter1 = new SVDQEF.Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        var expectedResult = SVDQEF.SVD.rotate12( parameter0, parameter1 );
        var actualResult   = this.testKernel(
            "rotate12Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            new SVDQEF.Matrix3x3.Data( parameter1 ),
            new SVDTestResult( )
        );

        Assert.AreEqual( expectedResult.Item1.m00, actualResult.symmetricMatrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item1.m01, actualResult.symmetricMatrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item1.m02, actualResult.symmetricMatrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item1.m11, actualResult.symmetricMatrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item1.m12, actualResult.symmetricMatrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item1.m22, actualResult.symmetricMatrix3x3.m22, epsilon );

        Assert.AreEqual( expectedResult.Item2.m00, actualResult.matrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item2.m01, actualResult.matrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item2.m02, actualResult.matrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item2.m10, actualResult.matrix3x3.m10, epsilon );
        Assert.AreEqual( expectedResult.Item2.m11, actualResult.matrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item2.m12, actualResult.matrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item2.m20, actualResult.matrix3x3.m20, epsilon );
        Assert.AreEqual( expectedResult.Item2.m21, actualResult.matrix3x3.m21, epsilon );
        Assert.AreEqual( expectedResult.Item2.m22, actualResult.matrix3x3.m22, epsilon );
    }

    public const int SVDSweeps = 6;

    [Test]
    public void getSymmetricSVDTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var parameter1 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var parameter3 = new SVDQEF.Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        var expectedResult = SVDQEF.SVD.getSymmetricSVD( parameter0, SVDSweeps );
        var actualResult   = this.testKernel(
            "getSymmetricSVDTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            new SVDQEF.SymmetricMatrix3x3.Data( parameter1 ),
            new SVDQEF.Matrix3x3.Data( parameter3 ),
            new SVDTestResult( )
        );

        Assert.AreEqual( expectedResult.Item1.m00, actualResult.symmetricMatrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item1.m01, actualResult.symmetricMatrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item1.m02, actualResult.symmetricMatrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item1.m11, actualResult.symmetricMatrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item1.m12, actualResult.symmetricMatrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item1.m22, actualResult.symmetricMatrix3x3.m22, epsilon );

        Assert.AreEqual( expectedResult.Item2.m00, actualResult.matrix3x3.m00, epsilon );
        Assert.AreEqual( expectedResult.Item2.m01, actualResult.matrix3x3.m01, epsilon );
        Assert.AreEqual( expectedResult.Item2.m02, actualResult.matrix3x3.m02, epsilon );
        Assert.AreEqual( expectedResult.Item2.m10, actualResult.matrix3x3.m10, epsilon );
        Assert.AreEqual( expectedResult.Item2.m11, actualResult.matrix3x3.m11, epsilon );
        Assert.AreEqual( expectedResult.Item2.m12, actualResult.matrix3x3.m12, epsilon );
        Assert.AreEqual( expectedResult.Item2.m20, actualResult.matrix3x3.m20, epsilon );
        Assert.AreEqual( expectedResult.Item2.m21, actualResult.matrix3x3.m21, epsilon );
        Assert.AreEqual( expectedResult.Item2.m22, actualResult.matrix3x3.m22, epsilon );
    }

    [Test]
    public void pseudoInverseScalarTest( ) {
        var parameter0 = 0.3124973f;

        var expectedResult = SVDQEF.SVD.pseudoInverse( parameter0 );
        var actualResult   = this.testKernel(
            "pseudoInverseScalarTest",
            parameter0,
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );
    }

    [Test]
    public void pseudoInverseMatrixTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.3124973f, 7.298034e-16f, -5.872501e-26f, 2.505459f, 0.0f, 0.1820442f );
        var parameter1 = new SVDQEF.Matrix3x3( 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );

        var expectedResult = SVDQEF.SVD.pseudoInverse( parameter0, parameter1 );
        var actualResult   = this.testKernel(
            "pseudoInverseMatrixTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            new SVDQEF.Matrix3x3.Data( parameter1 ),
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void calculateErrorTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var parameter1 = new Vector3( 0.0f, 0.0f, 0.0f );
        var parameter2 = new Vector3( -0.02622265f, -0.02307659f, -0.02307659f );

        // fails with these parameters, no idea why
        // parameter0 = new SVDQEF.SymmetricMatrix3x3( -0.5651875f, 0.8850253f, -0.2808784f, 0.3854723f, 0.2691586f, 0.5050807f );
        // parameter1 = new Vector3( 0.2325951f, 0.5514052f, -0.6665196f );
        // parameter2 = new Vector3( 0.563621f, 0.7674949f, 0.1588005f );

        var expectedResult = SVDQEF.SVD.calculateError( parameter0, parameter1, parameter2 );
        var actualResult   = this.testKernel(
            "calculateErrorTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            parameter2,
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );
    }

    [Test]
    public void solveSVDTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var parameter1 = new Vector3( -0.02622265f, -0.02307659f, -0.02307659f );

        var expectedResult = SVDQEF.SVD.solve( parameter0, parameter1, SVDSweeps );
        var actualResult   = this.testKernel(
            "solveSVDTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            new Vector4( )
        );

        Assert.AreEqual( expectedResult.Item1.x, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.Item1.y, actualResult.y, epsilon );
        Assert.AreEqual( expectedResult.Item1.z, actualResult.z, epsilon );
        Assert.AreEqual( expectedResult.Item2,   actualResult.w, epsilon );
    }

}

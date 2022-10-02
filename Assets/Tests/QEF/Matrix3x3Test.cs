using NUnit.Framework;
using UnityEngine;

public class Matrix3x3Test : ImplementationTest {

    protected override string shader { get; } = "Tests/Matrix3x3Test";

    [Test]
    public void multiplyVectorTest( ) {
        var matrix3x3 = new SVDQEF.Matrix3x3( 0.7640296f, 0.0f, 0.0f, 0.0f, 1.117985f, 0.0f, 0.0f, 0.0f, 1.117985f );
        var vector    = new Vector3( -0.02622265f, -0.02307659f, -0.02307659f );

        var expectedResult = matrix3x3.multiplyVector( vector );
        var actualResult   = this.testKernel(
            "multiplyVectorTest",
            new SVDQEF.Matrix3x3.Data( matrix3x3 ),
            vector,
            Vector3.zero
        );

        Assert.AreEqual( expectedResult.x, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.y, actualResult.y, epsilon );
        Assert.AreEqual( expectedResult.z, actualResult.z, epsilon );
    }

    [Test]
    public void rotate01PostTest( ) {
        var matrix3x3 = SVDQEF.Matrix3x3.identity;
        var cs        = Vector2.zero;

        var expectedResult = matrix3x3.rotate01Post( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate01PostTest",
            new SVDQEF.Matrix3x3.Data( matrix3x3 ),
            cs,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate02PostTest( ) {
        var matrix3x3 = SVDQEF.Matrix3x3.identity;
        var cs        = Vector2.zero;

        var expectedResult = matrix3x3.rotate02Post( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate02PostTest",
            new SVDQEF.Matrix3x3.Data( matrix3x3 ),
            cs,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate12PostTest( ) {
        var matrix3x3 = SVDQEF.Matrix3x3.identity;
        var cs        = Vector2.zero;

        var expectedResult = matrix3x3.rotate12Post( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate12PostTest",
            new SVDQEF.Matrix3x3.Data( matrix3x3 ),
            cs,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

}
